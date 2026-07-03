#include "server/routes_camera.hpp"

#include <exception>
#include <optional>
#include <regex>
#include <string>
#include <unordered_map>

#include "producer/camera_settings.hpp"
#include "server/camera_repository.hpp"
#include "server/camera_supervisor.hpp"
#include "server/route_helpers.hpp"

namespace gw::server {

namespace {

// Pull the reprojection-error number we appended to the camchain in
// KalibrJob's augment_with_quality(). The camchain YAML is small and the
// regex is tight, so doing this on every list_all() is comfortably fast.
// Returns nullopt for legacy calibrations (uploaded before this feature) or
// when Kalibr's results.txt was missing at calibration time.
std::optional<double> extract_reproj_error_px(const std::string& yaml) {
    static const std::regex re(
        R"(reprojection_error_px:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?))");
    std::smatch m;
    if (!std::regex_search(yaml, m, re)) return std::nullopt;
    try { return std::stod(m[1].str()); } catch (...) { return std::nullopt; }
}

crow::json::wvalue camera_to_json(const Camera&                              c,
                                  bool                                       online,
                                  const std::optional<gw::VideoModeOption>&  current) {
    crow::json::wvalue j;
    j["id"]         = c.id;
    j["name"]       = c.name;
    j["serial"]     = c.serial;
    j["focal_length_mm"] = c.focal_length_mm;
    j["calibrated_at"] = c.calibrated_at ? crow::json::wvalue(*c.calibrated_at)
                                         : crow::json::wvalue(nullptr);
    // Quality score for the stored calibration, in pixels (RMS of the per-
    // axis reprojection-error sigma). null when uncalibrated or when the
    // stored YAML predates the guesswork_meta enrichment.
    if (c.calibration_json) {
        const auto err = extract_reproj_error_px(*c.calibration_json);
        if (err) j["reprojection_error_px"] = *err;
        else     j["reprojection_error_px"] = nullptr;
    } else {
        j["reprojection_error_px"] = nullptr;
    }
    if (c.mode) j["mode"] = *c.mode;
    else        j["mode"] = nullptr;
    if (current && current->width)   j["mode_width"]   = *current->width;
    else                              j["mode_width"]   = nullptr;
    if (current && current->height)  j["mode_height"]  = *current->height;
    else                              j["mode_height"]  = nullptr;
    if (current && current->max_fps) j["mode_max_fps"] = *current->max_fps;
    else                              j["mode_max_fps"] = nullptr;
    put_opt(j, "gain_auto",     c.gain_auto);
    put_opt(j, "gain",          c.gain);
    put_opt(j, "exposure_auto", c.exposure_auto);
    put_opt(j, "exposure",      c.exposure);
    j["hardware_sync_enabled"] = c.hardware_sync_enabled;
    put_opt(j, "trigger_output_pin", c.trigger_output_pin);
    put_opt(j, "role", c.role);
    j["orientation"] = c.orientation;
    j["extrinsics_calibrated_at"] =
        c.extrinsics_calibrated_at ? crow::json::wvalue(*c.extrinsics_calibrated_at)
                                   : crow::json::wvalue(nullptr);
    j["online"]     = online;
    j["created_at"] = c.created_at;
    return j;
}

crow::json::wvalue range_to_json(const gw::CameraSettingRange& r) {
    crow::json::wvalue j;
    j["min"]  = r.min;
    j["max"]  = r.max;
    j["unit"] = r.unit;
    return j;
}

crow::json::wvalue limits_to_json(const gw::CameraSettingsLimits& l) {
    crow::json::wvalue j;
    if (l.gain)     j["gain"]     = range_to_json(*l.gain);     else j["gain"]     = nullptr;
    if (l.exposure) j["exposure"] = range_to_json(*l.exposure); else j["exposure"] = nullptr;
    return j;
}

crow::json::wvalue modes_to_json(const gw::VideoModeList& list) {
    crow::json::wvalue j;
    j["supported"] = list.supported;
    if (list.current) j["current"] = *list.current;
    else              j["current"] = nullptr;
    crow::json::wvalue::list opts;
    opts.reserve(list.options.size());
    for (const auto& opt : list.options) {
        crow::json::wvalue o;
        o["name"]         = opt.name;
        o["display_name"] = opt.display_name;
        o["description"]  = opt.description;
        if (opt.width)   o["width"]   = *opt.width;   else o["width"]   = nullptr;
        if (opt.height)  o["height"]  = *opt.height;  else o["height"]  = nullptr;
        if (opt.max_fps) o["max_fps"] = *opt.max_fps; else o["max_fps"] = nullptr;
        opts.emplace_back(std::move(o));
    }
    j["options"] = std::move(opts);
    return j;
}

std::unordered_map<int64_t, bool> online_map_from_supervisor(CameraSupervisor& supervisor) {
    std::unordered_map<int64_t, bool> out;
    for (const auto& s : supervisor.snapshot_all()) {
        out[s.id] = s.online;
    }
    return out;
}

// Reads a non-empty string field. On failure, populates `err` with the 400
// response and returns false; on success, writes to `out` and returns true.
bool parse_required_string(const crow::json::rvalue& body, const char* field,
                           std::string& out, crow::response& err) {
    if (!body.has(field) || body[field].t() != crow::json::type::String) {
        err = error_response(400, std::string("missing string field: ") + field);
        return false;
    }
    out = std::string(body[field].s());
    if (out.empty()) {
        err = error_response(400, std::string(field) + " must be non-empty");
        return false;
    }
    return true;
}

// If the field is present, it must be a non-empty string. If absent, `out`
// stays nullopt and the call succeeds. Returns false on type error.
bool parse_optional_string(const crow::json::rvalue& body, const char* field,
                           std::optional<std::string>& out, crow::response& err) {
    if (!body.has(field)) {
        out = std::nullopt;
        return true;
    }
    if (body[field].t() != crow::json::type::String) {
        err = error_response(400, std::string("field must be a string: ") + field);
        return false;
    }
    std::string v(body[field].s());
    if (v.empty()) {
        err = error_response(400, std::string(field) + " must be non-empty");
        return false;
    }
    out = std::move(v);
    return true;
}

bool parse_optional_bool(const crow::json::rvalue& body, const char* field,
                         std::optional<bool>& out, crow::response& err) {
    if (!body.has(field)) { out = std::nullopt; return true; }
    const auto t = body[field].t();
    if (t == crow::json::type::True)  { out = true;  return true; }
    if (t == crow::json::type::False) { out = false; return true; }
    err = error_response(400, std::string("field must be a boolean: ") + field);
    return false;
}

bool parse_optional_number(const crow::json::rvalue& body, const char* field,
                           std::optional<double>& out, crow::response& err) {
    if (!body.has(field)) { out = std::nullopt; return true; }
    const auto t = body[field].t();
    if (t == crow::json::type::Number) { out = body[field].d(); return true; }
    err = error_response(400, std::string("field must be a number: ") + field);
    return false;
}

// Like parse_optional_number, but a JSON `null` is treated as "explicitly
// clear". The outer optional models "present in body"; the inner optional
// models the value (nullopt = clear to SQL NULL). Used by trigger_output_pin
// where clearing on PUT is meaningful (hw-sync being disabled).
bool parse_optional_nullable_int(const crow::json::rvalue& body, const char* field,
                                 std::optional<std::optional<int64_t>>& out,
                                 crow::response& err) {
    if (!body.has(field)) { out = std::nullopt; return true; }
    const auto t = body[field].t();
    if (t == crow::json::type::Null) { out = std::optional<int64_t>(std::nullopt); return true; }
    if (t == crow::json::type::Number) {
        out = std::optional<int64_t>(static_cast<int64_t>(body[field].i()));
        return true;
    }
    err = error_response(400, std::string("field must be an integer or null: ") + field);
    return false;
}

bool is_valid_trigger_output_pin(int64_t v) { return v >= 1 && v <= 6; }

bool is_valid_role(const std::string& v) {
    return v == "apriltag" || v == "vio_left" || v == "vio_right";
}

bool is_valid_orientation(int64_t v) {
    return v == 0 || v == 90 || v == 180 || v == 270;
}

// String-or-null sibling of parse_optional_nullable_int — used by `role`
// where a JSON null clears the column.
bool parse_optional_nullable_string(const crow::json::rvalue& body, const char* field,
                                    std::optional<std::optional<std::string>>& out,
                                    crow::response& err) {
    if (!body.has(field)) { out = std::nullopt; return true; }
    const auto t = body[field].t();
    if (t == crow::json::type::Null) {
        out = std::optional<std::string>(std::nullopt);
        return true;
    }
    if (t == crow::json::type::String) {
        out = std::optional<std::string>(std::string(body[field].s()));
        return true;
    }
    err = error_response(400, std::string("field must be a string or null: ") + field);
    return false;
}

// Lens focal length in mm — must be positive and within a wide sanity
// envelope. Typical machine-vision lenses are 1.4–35 mm; we accept up to
// 1000 mm to leave headroom without inviting bogus inputs.
bool is_valid_focal_length_mm(double v) {
    return v > 0.0 && v < 1000.0;
}

// Reads a number field as a double. Same shape as parse_required_string but
// for numeric fields where the value is mandatory.
bool parse_required_number(const crow::json::rvalue& body, const char* field,
                           double& out, crow::response& err) {
    if (!body.has(field) || body[field].t() != crow::json::type::Number) {
        err = error_response(400, std::string("missing number field: ") + field);
        return false;
    }
    out = body[field].d();
    return true;
}

struct CreateBody {
    std::string                name;
    std::string                serial;
    double                     focal_length_mm = 0.0;
    std::optional<std::string> mode;
    bool                       hardware_sync_enabled = false;
    std::optional<int64_t>     trigger_output_pin;
    std::optional<std::string> role;         // applied post-create via update
    std::optional<int64_t>     orientation;  // "
    crow::response             error;
    bool                       ok = false;
};

CreateBody parse_create_body(const crow::request& req) {
    CreateBody r;
    const auto body = crow::json::load(req.body);
    if (!body) {
        r.error = error_response(400, "invalid JSON body");
        return r;
    }
    if (!parse_required_string(body, "name", r.name, r.error))                       return r;
    if (!parse_required_string(body, "serial", r.serial, r.error))                   return r;
    if (!parse_required_number(body, "focal_length_mm", r.focal_length_mm, r.error)) return r;
    if (!is_valid_focal_length_mm(r.focal_length_mm)) {
        r.error = error_response(400, "focal_length_mm must be a positive number in millimeters");
        return r;
    }
    if (!parse_optional_string(body, "mode", r.mode, r.error))                       return r;

    std::optional<bool> hw_sync;
    if (!parse_optional_bool(body, "hardware_sync_enabled", hw_sync, r.error)) return r;
    if (hw_sync) r.hardware_sync_enabled = *hw_sync;

    std::optional<std::optional<int64_t>> pin;
    if (!parse_optional_nullable_int(body, "trigger_output_pin", pin, r.error)) return r;
    if (pin && pin->has_value()) {
        if (!is_valid_trigger_output_pin(**pin)) {
            r.error = error_response(400, "trigger_output_pin must be 1..6");
            return r;
        }
        r.trigger_output_pin = **pin;
    }
    if (r.hardware_sync_enabled && !r.trigger_output_pin) {
        r.error = error_response(400,
            "trigger_output_pin is required when hardware_sync_enabled is true");
        return r;
    }

    // Optional role/orientation so a camera can be fully configured in one
    // POST instead of create-then-edit. Same validation as PUT.
    if (!parse_optional_string(body, "role", r.role, r.error)) return r;
    if (r.role && !is_valid_role(*r.role)) {
        r.error = error_response(
            400, "role must be 'apriltag', 'vio_left', or 'vio_right'");
        return r;
    }
    std::optional<double> orientation;
    if (!parse_optional_number(body, "orientation", orientation, r.error)) return r;
    if (orientation) {
        const auto v = static_cast<int64_t>(*orientation);
        if (static_cast<double>(v) != *orientation || !is_valid_orientation(v)) {
            r.error = error_response(400, "orientation must be 0, 90, 180, or 270");
            return r;
        }
        r.orientation = v;
    }
    r.ok = true;
    return r;
}

struct UpdateBody {
    std::optional<std::string> name;
    std::optional<double>      focal_length_mm;
    std::optional<std::string> mode;
    std::optional<bool>        gain_auto;
    std::optional<double>      gain;
    std::optional<bool>        exposure_auto;
    std::optional<double>      exposure;
    std::optional<bool>        hardware_sync_enabled;
    // Outer = present in body; inner = value (nullopt → JSON null, clear).
    std::optional<std::optional<int64_t>> trigger_output_pin;
    std::optional<std::optional<std::string>> role;
    std::optional<int64_t>     orientation;
    crow::response             error;
    bool                       ok = false;

    bool has_settings() const {
        return gain_auto || gain || exposure_auto || exposure;
    }
    bool empty() const {
        return !name && !focal_length_mm && !mode && !has_settings()
            && !hardware_sync_enabled && !trigger_output_pin && !role
            && !orientation;
    }
};

UpdateBody parse_update_body(const crow::request& req) {
    UpdateBody r;
    const auto body = crow::json::load(req.body);
    if (!body) {
        r.error = error_response(400, "invalid JSON body");
        return r;
    }
    if (!parse_optional_string(body, "name",            r.name,            r.error)) return r;
    if (!parse_optional_number(body, "focal_length_mm", r.focal_length_mm, r.error)) return r;
    if (r.focal_length_mm && !is_valid_focal_length_mm(*r.focal_length_mm)) {
        r.error = error_response(400, "focal_length_mm must be a positive number in millimeters");
        return r;
    }
    if (!parse_optional_string(body, "mode", r.mode, r.error)) return r;
    if (!parse_optional_bool  (body, "gain_auto",     r.gain_auto,     r.error)) return r;
    if (!parse_optional_number(body, "gain",          r.gain,          r.error)) return r;
    if (!parse_optional_bool  (body, "exposure_auto", r.exposure_auto, r.error)) return r;
    if (!parse_optional_number(body, "exposure",      r.exposure,      r.error)) return r;
    if (!parse_optional_bool         (body, "hardware_sync_enabled",
                                      r.hardware_sync_enabled, r.error)) return r;
    if (!parse_optional_nullable_int (body, "trigger_output_pin",
                                      r.trigger_output_pin,    r.error)) return r;
    if (r.trigger_output_pin && r.trigger_output_pin->has_value()) {
        if (!is_valid_trigger_output_pin(**r.trigger_output_pin)) {
            r.error = error_response(400, "trigger_output_pin must be 1..6");
            return r;
        }
    }
    if (!parse_optional_nullable_string(body, "role", r.role, r.error)) return r;
    if (r.role && r.role->has_value() && !is_valid_role(**r.role)) {
        r.error = error_response(400,
            "role must be 'apriltag', 'vio_left', 'vio_right', or null");
        return r;
    }
    std::optional<double> orientation;
    if (!parse_optional_number(body, "orientation", orientation, r.error)) return r;
    if (orientation) {
        const auto v = static_cast<int64_t>(*orientation);
        if (static_cast<double>(v) != *orientation || !is_valid_orientation(v)) {
            r.error = error_response(400, "orientation must be 0, 90, 180, or 270");
            return r;
        }
        r.orientation = v;
    }
    // Disabling hw-sync implicitly clears the pin; preserve any explicit value
    // the caller already gave us if it matches that intent.
    if (r.hardware_sync_enabled && !*r.hardware_sync_enabled) {
        r.trigger_output_pin = std::optional<int64_t>(std::nullopt);
    }
    // Enabling hw-sync without a pin (and no prior pin) is rejected in the
    // route handler — we don't have access to the existing row here.
    if (r.empty()) {
        r.error = error_response(400, "PUT body must include at least one updatable field");
        return r;
    }
    r.ok = true;
    return r;
}

gw::CameraSettingsPatch patch_from_body(const UpdateBody& b) {
    gw::CameraSettingsPatch p;
    p.gain_auto     = b.gain_auto;
    p.gain          = b.gain;
    p.exposure_auto = b.exposure_auto;
    p.exposure      = b.exposure;
    return p;
}

}  // namespace

void register_camera_routes(crow::SimpleApp&  app,
                            CameraRepository& repo,
                            CameraSupervisor& supervisor) {
    // GET /api/cameras/available — connected Spinnaker cameras not in the DB.
    // Registered BEFORE the /<int> route so "available" isn't matched as an int.
    CROW_ROUTE(app, "/api/cameras/available").methods("GET"_method)
    ([&supervisor] {
        try {
            const auto avail = supervisor.list_unmapped_connected();
            crow::json::wvalue::list items;
            items.reserve(avail.size());
            for (const auto& a : avail) {
                crow::json::wvalue j;
                j["serial"] = a.serial;
                j["model"]  = a.model;
                j["vendor"] = a.vendor;
                items.emplace_back(std::move(j));
            }
            return json_response(200, crow::json::wvalue(std::move(items)));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/cameras").methods("GET"_method)
    ([&repo, &supervisor] {
        try {
            const auto rows   = repo.list_all();
            const auto online = online_map_from_supervisor(supervisor);
            crow::json::wvalue::list items;
            items.reserve(rows.size());
            for (const auto& c : rows) {
                const auto it       = online.find(c.id);
                const bool is_on    = it != online.end() && it->second;
                auto       mode_opt = is_on ? supervisor.current_mode_for(c.id)
                                            : std::nullopt;
                items.emplace_back(camera_to_json(c, is_on, mode_opt));
            }
            return json_response(200, crow::json::wvalue(std::move(items)));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/cameras").methods("POST"_method)
    ([&repo, &supervisor](const crow::request& req) {
        auto parsed = parse_create_body(req);
        if (!parsed.ok) return std::move(parsed.error);
        try {
            // vio roles are exclusive (same route-layer check as PUT) —
            // reject BEFORE creating so a 409 leaves no half-made camera.
            if (parsed.role && *parsed.role != "apriltag") {
                for (const auto& other : repo.list_all()) {
                    if (other.role && *other.role == *parsed.role) {
                        return error_response(409,
                            "role '" + *parsed.role +
                            "' is already assigned to camera '" + other.name +
                            "'");
                    }
                }
            }
            const std::optional<std::string_view> mode_view =
                parsed.mode ? std::optional<std::string_view>(*parsed.mode)
                            : std::nullopt;
            auto c = repo.create(parsed.name, parsed.serial,
                                 parsed.focal_length_mm, mode_view,
                                 parsed.hardware_sync_enabled,
                                 parsed.trigger_output_pin);
            if (parsed.role || parsed.orientation) {
                CameraUpdate upd;
                if (parsed.role) upd.role = std::optional<std::string>(*parsed.role);
                upd.orientation = parsed.orientation;
                if (const auto updated = repo.update(c.id, upd)) c = *updated;
            }
            supervisor.on_camera_added(c.id);
            const bool on = supervisor.is_online(c.id);
            return json_response(
                201,
                camera_to_json(c, on,
                               on ? supervisor.current_mode_for(c.id) : std::nullopt));
        } catch (const DuplicateNameError& e) {
            return error_response(409, e.what());
        } catch (const DuplicateSerialError& e) {
            return error_response(409, e.what());
        } catch (const DuplicateTriggerOutputPinError& e) {
            return error_response(409, e.what());
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/cameras/<int>").methods("GET"_method)
    ([&repo, &supervisor](int64_t id) {
        try {
            const auto c = repo.get(id);
            if (!c) return error_response(404, "camera not found");
            const bool on = supervisor.is_online(c->id);
            return json_response(
                200,
                camera_to_json(*c, on,
                               on ? supervisor.current_mode_for(c->id) : std::nullopt));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/cameras/<int>").methods("PUT"_method)
    ([&repo, &supervisor](const crow::request& req, int64_t id) {
        auto parsed = parse_update_body(req);
        if (!parsed.ok) return std::move(parsed.error);
        try {
            // Enabling hw-sync requires a pin: either provided in this body, or
            // already persisted on the row. Reject early so we don't half-apply.
            if (parsed.hardware_sync_enabled && *parsed.hardware_sync_enabled) {
                const bool incoming_pin =
                    parsed.trigger_output_pin && parsed.trigger_output_pin->has_value();
                if (!incoming_pin) {
                    const auto existing = repo.get(id);
                    if (!existing) return error_response(404, "camera not found");
                    if (!existing->trigger_output_pin) {
                        return error_response(400,
                            "trigger_output_pin is required when enabling hardware_sync_enabled");
                    }
                }
            }

            // vio_left/vio_right are exclusive: exactly one camera each. The
            // cameras table can't express partial uniqueness, so check here.
            if (parsed.role && parsed.role->has_value() && **parsed.role != "apriltag") {
                for (const auto& other : repo.list_all()) {
                    if (other.id != id && other.role && *other.role == **parsed.role) {
                        return error_response(409,
                            "role '" + **parsed.role + "' is already assigned to camera '" +
                            other.name + "'");
                    }
                }
            }

            // Push settings to the live producer first so we can persist the
            // actually-applied values (post-clamp, post-quantization) instead
            // of the raw request body. Skipped when the camera is offline —
            // the DB write below still takes effect on next start().
            if (parsed.has_settings() && supervisor.is_online(id)) {
                const auto applied = supervisor.apply_settings_live(
                    id, patch_from_body(parsed));
                parsed.gain_auto     = applied.gain_auto;
                parsed.gain          = applied.gain;
                parsed.exposure_auto = applied.exposure_auto;
                parsed.exposure      = applied.exposure;
            }

            CameraUpdate upd;
            upd.name            = parsed.name;
            upd.focal_length_mm = parsed.focal_length_mm;
            upd.mode            = parsed.mode;
            upd.gain_auto     = parsed.gain_auto;
            upd.gain          = parsed.gain;
            upd.exposure_auto = parsed.exposure_auto;
            upd.exposure      = parsed.exposure;
            upd.hardware_sync_enabled = parsed.hardware_sync_enabled;
            upd.trigger_output_pin    = parsed.trigger_output_pin;
            upd.role                  = parsed.role;
            upd.orientation           = parsed.orientation;

            const auto c = repo.update(id, upd);
            if (!c) return error_response(404, "camera not found");

            supervisor.on_camera_updated(c->id);

            const bool on = supervisor.is_online(c->id);
            return json_response(
                200,
                camera_to_json(*c, on,
                               on ? supervisor.current_mode_for(c->id) : std::nullopt));
        } catch (const DuplicateNameError& e) {
            return error_response(409, e.what());
        } catch (const DuplicateTriggerOutputPinError& e) {
            return error_response(409, e.what());
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/cameras/<int>/settings/limits").methods("GET"_method)
    ([&repo, &supervisor](int64_t id) {
        try {
            if (!repo.get(id))                  return error_response(404, "camera not found");
            auto lim = supervisor.settings_limits_for_id(id);
            if (!lim)                           return error_response(409, "camera is offline");
            return json_response(200, limits_to_json(*lim));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    // Lazy per-camera mode enumeration for a DB-mapped (online) camera.
    CROW_ROUTE(app, "/api/cameras/<int>/modes").methods("GET"_method)
    ([&repo, &supervisor](int64_t id) {
        try {
            const auto row = repo.get(id);
            if (!row) return error_response(404, "camera not found");
            const auto list = supervisor.list_video_modes_for_id(id);
            if (!list) return error_response(409, "camera is offline");
            return json_response(200, modes_to_json(*list));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    // Lazy mode enumeration for a connected-but-not-yet-mapped serial. Used by
    // the Add modal before the row exists.
    CROW_ROUTE(app, "/api/cameras/available/<string>/modes").methods("GET"_method)
    ([&supervisor](const std::string& serial) {
        try {
            const auto list = supervisor.list_video_modes_for_serial(serial);
            if (!list) return error_response(404, "serial not connected");
            return json_response(200, modes_to_json(*list));
        } catch (const std::exception& e) {
            // Spinnaker Init failures surface here; map to 503.
            return error_response(503, e.what());
        }
    });

    CROW_ROUTE(app, "/api/cameras/<int>").methods("DELETE"_method)
    ([&repo, &supervisor](int64_t id) {
        try {
            // Stop the producer first so the row removal can't race with
            // an in-flight frame referencing the slot.
            supervisor.on_camera_removed(id);
            if (!repo.remove(id)) return error_response(404, "camera not found");
            return with_no_store(crow::response(204));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });
}

}  // namespace gw::server
