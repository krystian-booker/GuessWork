#include "server/config_snapshot.hpp"

#include <chrono>
#include <exception>
#include <map>
#include <optional>
#include <unordered_set>

#include "apriltag/field_layout.hpp"
#include "server/camera_repository.hpp"
#include "server/can_config_repository.hpp"
#include "server/field_layout_repository.hpp"
#include "server/fusion_config_repository.hpp"
#include "server/imu_config_repository.hpp"
#include "server/trigger_group_repository.hpp"
#include "server/vio_config_repository.hpp"

namespace gw::server {

namespace {

template <typename T>
void put_opt(crow::json::wvalue& j, const char* key, const std::optional<T>& v) {
    if (v) j[key] = *v;
    else   j[key] = nullptr;
}

// --- rvalue extraction helpers (absent or null → nullopt) -------------------

bool has_value(const crow::json::rvalue& obj, const char* key) {
    return obj.has(key) && obj[key].t() != crow::json::type::Null;
}

std::optional<std::string> opt_string(const crow::json::rvalue& obj,
                                      const char* key) {
    if (!has_value(obj, key)) return std::nullopt;
    return std::string(obj[key].s());
}

std::optional<double> opt_double(const crow::json::rvalue& obj, const char* key) {
    if (!has_value(obj, key)) return std::nullopt;
    return obj[key].d();
}

std::optional<int64_t> opt_int(const crow::json::rvalue& obj, const char* key) {
    if (!has_value(obj, key)) return std::nullopt;
    return obj[key].i();
}

std::optional<bool> opt_bool(const crow::json::rvalue& obj, const char* key) {
    if (!has_value(obj, key)) return std::nullopt;
    return obj[key].b();
}

}  // namespace

// ---------------------------------------------------------------------------
// export

crow::json::wvalue export_snapshot(CameraRepository&       cameras,
                                   TriggerGroupRepository& trigger_groups,
                                   FieldLayoutRepository&  field_layouts,
                                   ImuConfigRepository&    imu_config,
                                   VioConfigRepository&    vio_config,
                                   CanConfigRepository&    can_config,
                                   FusionConfigRepository& fusion_config) {
    crow::json::wvalue j;
    j["snapshot_version"] = kSnapshotVersion;
    j["exported_at"] = std::chrono::duration_cast<std::chrono::seconds>(
                           std::chrono::system_clock::now().time_since_epoch())
                           .count();

    crow::json::wvalue::list cams;
    for (const auto& c : cameras.list_all()) {
        crow::json::wvalue cj;
        cj["name"]            = c.name;
        cj["serial"]          = c.serial;
        cj["focal_length_mm"] = c.focal_length_mm;
        put_opt(cj, "mode", c.mode);
        put_opt(cj, "gain_auto", c.gain_auto);
        put_opt(cj, "gain", c.gain);
        put_opt(cj, "exposure_auto", c.exposure_auto);
        put_opt(cj, "exposure", c.exposure);
        cj["hardware_sync_enabled"] = c.hardware_sync_enabled;
        put_opt(cj, "trigger_output_pin", c.trigger_output_pin);
        put_opt(cj, "role", c.role);
        put_opt(cj, "calibration_json", c.calibration_json);
        put_opt(cj, "calibrated_at", c.calibrated_at);  // informational only
        put_opt(cj, "imu_extrinsics_json", c.imu_extrinsics_json);
        put_opt(cj, "extrinsics_calibrated_at", c.extrinsics_calibrated_at);
        cams.emplace_back(std::move(cj));
    }
    j["cameras"] = std::move(cams);

    crow::json::wvalue::list groups;
    for (const auto& g : trigger_groups.list_all()) {
        crow::json::wvalue gj;
        gj["name"] = g.name;
        gj["fps"]  = g.fps;
        crow::json::wvalue::list pins;
        for (uint8_t p : g.output_pins) pins.emplace_back(static_cast<int>(p));
        gj["output_pins"] = std::move(pins);
        groups.emplace_back(std::move(gj));
    }
    j["trigger_groups"] = std::move(groups);

    crow::json::wvalue::list layouts;
    for (const auto& l : field_layouts.list_all()) {
        crow::json::wvalue lj;
        lj["name"]   = l.name;
        lj["json"]   = l.json;  // verbatim document string
        lj["active"] = l.active;
        layouts.emplace_back(std::move(lj));
    }
    j["field_layouts"] = std::move(layouts);

    {
        const auto c = imu_config.get();
        crow::json::wvalue cj;
        cj["rate_hz"]             = c.rate_hz;
        cj["accel_noise_density"] = c.accel_noise_density;
        cj["accel_random_walk"]   = c.accel_random_walk;
        cj["gyro_noise_density"]  = c.gyro_noise_density;
        cj["gyro_random_walk"]    = c.gyro_random_walk;
        put_opt(cj, "t_imu_robot_json", c.t_imu_robot_json);
        j["imu_config"] = std::move(cj);
    }
    {
        const auto c = vio_config.get();
        crow::json::wvalue cj;
        cj["enabled"]              = c.enabled;
        cj["num_pts"]              = c.num_pts;
        cj["fast_threshold"]       = c.fast_threshold;
        cj["downsample"]           = c.downsample;
        cj["max_reproj_std_px"]    = c.max_reproj_std_px;
        cj["auto_reinit"]          = c.auto_reinit;
        cj["reinit_min_features"]  = c.reinit_min_features;
        cj["reinit_window_frames"] = c.reinit_window_frames;
        cj["reinit_max_pos_std_m"] = c.reinit_max_pos_std_m;
        j["vio_config"] = std::move(cj);
    }
    {
        crow::json::wvalue cj;
        cj["mode"]      = can_config.get().mode;
        j["can_config"] = std::move(cj);
    }
    {
        const auto c = fusion_config.get();
        crow::json::wvalue cj;
        cj["enabled"]              = c.enabled;
        cj["lag_s"]                = c.lag_s;
        cj["min_state_dt_ms"]      = c.min_state_dt_ms;
        cj["output_hz"]            = c.output_hz;
        cj["max_extrapolation_ms"] = c.max_extrapolation_ms;
        cj["tag_gate_chi2"]        = c.tag_gate_chi2;
        cj["tag_huber_k"]          = c.tag_huber_k;
        cj["vio_huber_k"]          = c.vio_huber_k;
        cj["odom_cauchy_k"]        = c.odom_cauchy_k;
        cj["odom_sigma_vx"]        = c.odom_sigma_vx;
        cj["odom_sigma_vy"]        = c.odom_sigma_vy;
        cj["odom_sigma_omega"]     = c.odom_sigma_omega;
        cj["vio_sigma_rot"]        = c.vio_sigma_rot;
        cj["vio_sigma_trans"]      = c.vio_sigma_trans;
        cj["collision_inflation"]  = c.collision_inflation;
        cj["collision_window"]     = c.collision_window;
        cj["reinit_pos_std_m"]     = c.reinit_pos_std_m;
        j["fusion_config"] = std::move(cj);
    }
    return j;
}

// ---------------------------------------------------------------------------
// import

namespace {

void import_cameras(const crow::json::rvalue& snap, CameraRepository& cameras,
                    ImportReport& rep) {
    if (!snap.has("cameras")) return;
    const auto& arr = snap["cameras"];

    // Pre-clear pass: when a snapshot camera wants a pin currently held by a
    // DIFFERENT serial-matched camera that the snapshot also reassigns, clear
    // it first so swaps/rotations among snapshot cameras can't deadlock on
    // the UNIQUE constraint. Pins held by non-snapshot cameras are never
    // stolen — those imports fail with a per-camera error instead.
    std::unordered_set<std::string> snapshot_serials;
    std::map<int64_t, std::string>  desired_pin_owner;  // pin → serial
    for (const auto& cj : arr) {
        if (!cj.has("serial")) continue;
        snapshot_serials.insert(std::string(cj["serial"].s()));
        if (const auto pin = opt_int(cj, "trigger_output_pin")) {
            desired_pin_owner[*pin] = std::string(cj["serial"].s());
        }
    }
    for (const auto& existing : cameras.list_all()) {
        if (!existing.trigger_output_pin) continue;
        const auto it = desired_pin_owner.find(*existing.trigger_output_pin);
        if (it == desired_pin_owner.end()) continue;
        if (it->second == existing.serial) continue;  // keeps its pin
        if (!snapshot_serials.count(existing.serial)) continue;  // not ours to touch
        CameraUpdate clear;
        clear.trigger_output_pin = std::optional<int64_t>{};  // → SQL NULL
        cameras.update(existing.id, clear);
    }

    for (const auto& cj : arr) {
        std::string serial;
        try {
            if (!cj.has("serial") || !cj.has("name") ||
                !cj.has("focal_length_mm")) {
                rep.cameras.errors.emplace_back(
                    "camera entry missing serial/name/focal_length_mm");
                continue;
            }
            serial = std::string(cj["serial"].s());
            const std::string name(cj["name"].s());
            const double      focal = cj["focal_length_mm"].d();

            auto existing = cameras.find_by_serial(serial);
            bool created  = false;
            if (!existing) {
                existing = cameras.create(name, serial, focal);
                created  = true;
            }

            CameraUpdate patch;
            patch.name            = name;
            patch.focal_length_mm = focal;
            if (has_value(cj, "mode")) patch.mode = std::string(cj["mode"].s());
            patch.gain_auto     = opt_bool(cj, "gain_auto");
            patch.gain          = opt_double(cj, "gain");
            patch.exposure_auto = opt_bool(cj, "exposure_auto");
            patch.exposure      = opt_double(cj, "exposure");
            if (cj.has("hardware_sync_enabled")) {
                patch.hardware_sync_enabled = cj["hardware_sync_enabled"].b();
            }
            if (cj.has("trigger_output_pin")) {
                // Snapshot-null clears the pin; snapshot-absent leaves it.
                patch.trigger_output_pin = opt_int(cj, "trigger_output_pin");
            }
            if (cj.has("role")) {
                patch.role = opt_string(cj, "role");
            }
            cameras.update(existing->id, patch);

            // Blobs: set when present, never cleared by absence.
            if (const auto cal = opt_string(cj, "calibration_json")) {
                cameras.set_calibration(existing->id, *cal);
            }
            if (const auto ext = opt_string(cj, "imu_extrinsics_json")) {
                cameras.set_imu_extrinsics(existing->id, *ext);
            }

            if (created) {
                ++rep.cameras.created;
                rep.camera_ids_created.push_back(existing->id);
            } else {
                ++rep.cameras.updated;
                rep.camera_ids_updated.push_back(existing->id);
            }
        } catch (const std::exception& e) {
            rep.cameras.errors.emplace_back("camera " + serial + ": " + e.what());
        }
    }
}

void import_trigger_groups(const crow::json::rvalue& snap,
                           TriggerGroupRepository& groups, ImportReport& rep) {
    if (!snap.has("trigger_groups")) return;
    const auto& arr = snap["trigger_groups"];

    // Group identity is purely {name, fps, pins} (TeensyManager re-pushes by
    // value), and pins can't be staged through an empty set — so matched
    // names are removed first, then everything recreated.
    std::unordered_set<std::string> existing_names;
    for (const auto& g : groups.list_all()) {
        existing_names.insert(g.name);
    }
    std::unordered_set<std::string> snapshot_names;
    for (const auto& gj : arr) {
        if (gj.has("name")) snapshot_names.insert(std::string(gj["name"].s()));
    }
    for (const auto& g : groups.list_all()) {
        if (snapshot_names.count(g.name)) groups.remove(g.id);
    }

    for (const auto& gj : arr) {
        std::string name;
        try {
            name = std::string(gj["name"].s());
            const double fps = gj["fps"].d();
            std::vector<uint8_t> pins;
            for (const auto& p : gj["output_pins"]) {
                pins.push_back(static_cast<uint8_t>(p.i()));
            }
            groups.create(name, fps, pins);
            if (existing_names.count(name)) ++rep.trigger_groups.updated;
            else ++rep.trigger_groups.created;
        } catch (const std::exception& e) {
            rep.trigger_groups.errors.emplace_back("group " + name + ": " +
                                                   e.what());
        }
    }
}

void import_field_layouts(const crow::json::rvalue& snap,
                          FieldLayoutRepository& layouts, ImportReport& rep) {
    if (!snap.has("field_layouts")) return;
    const auto& arr = snap["field_layouts"];

    std::optional<std::string> active_name;
    for (const auto& lj : arr) {
        std::string name;
        try {
            name = std::string(lj["name"].s());
            const std::string json(lj["json"].s());
            // A bad layout silently breaks tag→field math — validate first.
            gw::apriltag::parse_field_layout_json(json);

            if (lj.has("active") && lj["active"].b()) active_name = name;

            std::optional<int64_t> existing_id;
            for (const auto& l : layouts.list_all()) {
                if (l.name == name) {
                    existing_id = l.id;
                    break;
                }
            }
            if (existing_id) {
                layouts.update_json(*existing_id, json);
                ++rep.field_layouts.updated;
            } else {
                layouts.create(name, json);
                ++rep.field_layouts.created;
            }
        } catch (const std::exception& e) {
            rep.field_layouts.errors.emplace_back("layout " + name + ": " +
                                                  e.what());
        }
    }

    // Activate last — after every snapshot row exists.
    if (active_name) {
        for (const auto& l : layouts.list_all()) {
            if (l.name == *active_name) {
                layouts.activate(l.id);
                break;
            }
        }
    }
}

}  // namespace

ImportReport import_snapshot(const crow::json::rvalue& snap,
                             CameraRepository&         cameras,
                             TriggerGroupRepository&   trigger_groups,
                             FieldLayoutRepository&    field_layouts,
                             ImuConfigRepository&      imu_config,
                             VioConfigRepository&      vio_config,
                             CanConfigRepository&      can_config,
                             FusionConfigRepository&   fusion_config) {
    if (snap.t() != crow::json::type::Object) {
        throw std::runtime_error("snapshot must be a JSON object");
    }
    if (!snap.has("snapshot_version") ||
        snap["snapshot_version"].i() != kSnapshotVersion) {
        throw std::runtime_error("unsupported snapshot_version (expected " +
                                 std::to_string(kSnapshotVersion) + ")");
    }

    ImportReport rep;
    import_cameras(snap, cameras, rep);
    import_trigger_groups(snap, trigger_groups, rep);
    import_field_layouts(snap, field_layouts, rep);

    if (snap.has("imu_config")) {
        try {
            const auto&     cj = snap["imu_config"];
            ImuConfigUpdate patch;
            patch.rate_hz             = opt_double(cj, "rate_hz");
            patch.accel_noise_density = opt_double(cj, "accel_noise_density");
            patch.accel_random_walk   = opt_double(cj, "accel_random_walk");
            patch.gyro_noise_density  = opt_double(cj, "gyro_noise_density");
            patch.gyro_random_walk    = opt_double(cj, "gyro_random_walk");
            if (const auto t = opt_string(cj, "t_imu_robot_json")) {
                patch.t_imu_robot_json = std::optional<std::string>{*t};
            }
            if (!patch.empty()) {
                imu_config.update(patch);
                ++rep.imu_config.updated;
            }
        } catch (const std::exception& e) {
            rep.imu_config.errors.emplace_back(e.what());
        }
    }
    if (snap.has("vio_config")) {
        try {
            const auto&     cj = snap["vio_config"];
            VioConfigUpdate patch;
            patch.enabled              = opt_bool(cj, "enabled");
            patch.num_pts              = opt_int(cj, "num_pts");
            patch.fast_threshold       = opt_int(cj, "fast_threshold");
            patch.downsample           = opt_bool(cj, "downsample");
            patch.max_reproj_std_px    = opt_double(cj, "max_reproj_std_px");
            patch.auto_reinit          = opt_bool(cj, "auto_reinit");
            patch.reinit_min_features  = opt_int(cj, "reinit_min_features");
            patch.reinit_window_frames = opt_int(cj, "reinit_window_frames");
            patch.reinit_max_pos_std_m = opt_double(cj, "reinit_max_pos_std_m");
            if (!patch.empty()) {
                vio_config.update(patch);
                ++rep.vio_config.updated;
            }
        } catch (const std::exception& e) {
            rep.vio_config.errors.emplace_back(e.what());
        }
    }
    if (snap.has("can_config")) {
        try {
            CanConfigUpdate patch;
            patch.mode = opt_string(snap["can_config"], "mode");
            if (!patch.empty()) {
                can_config.update(patch);
                ++rep.can_config.updated;
            }
        } catch (const std::exception& e) {
            rep.can_config.errors.emplace_back(e.what());
        }
    }
    if (snap.has("fusion_config")) {
        try {
            const auto&        cj = snap["fusion_config"];
            FusionConfigUpdate patch;
            patch.enabled              = opt_bool(cj, "enabled");
            patch.lag_s                = opt_double(cj, "lag_s");
            patch.min_state_dt_ms      = opt_int(cj, "min_state_dt_ms");
            patch.output_hz            = opt_int(cj, "output_hz");
            patch.max_extrapolation_ms = opt_int(cj, "max_extrapolation_ms");
            patch.tag_gate_chi2        = opt_double(cj, "tag_gate_chi2");
            patch.tag_huber_k          = opt_double(cj, "tag_huber_k");
            patch.vio_huber_k          = opt_double(cj, "vio_huber_k");
            patch.odom_cauchy_k        = opt_double(cj, "odom_cauchy_k");
            patch.odom_sigma_vx        = opt_double(cj, "odom_sigma_vx");
            patch.odom_sigma_vy        = opt_double(cj, "odom_sigma_vy");
            patch.odom_sigma_omega     = opt_double(cj, "odom_sigma_omega");
            patch.vio_sigma_rot        = opt_double(cj, "vio_sigma_rot");
            patch.vio_sigma_trans      = opt_double(cj, "vio_sigma_trans");
            patch.collision_inflation  = opt_double(cj, "collision_inflation");
            patch.collision_window     = opt_int(cj, "collision_window");
            patch.reinit_pos_std_m     = opt_double(cj, "reinit_pos_std_m");
            if (!patch.empty()) {
                fusion_config.update(patch);
                ++rep.fusion_config.updated;
            }
        } catch (const std::exception& e) {
            rep.fusion_config.errors.emplace_back(e.what());
        }
    }
    return rep;
}

crow::json::wvalue ImportReport::to_json() const {
    crow::json::wvalue j;
    const auto section = [](const SectionReport& s) {
        crow::json::wvalue v;
        v["created"] = s.created;
        v["updated"] = s.updated;
        crow::json::wvalue::list errs;
        for (const auto& e : s.errors) errs.emplace_back(e);
        v["errors"] = std::move(errs);
        return v;
    };
    j["cameras"]        = section(cameras);
    j["trigger_groups"] = section(trigger_groups);
    j["field_layouts"]  = section(field_layouts);
    j["imu_config"]     = section(imu_config);
    j["vio_config"]     = section(vio_config);
    j["can_config"]     = section(can_config);
    j["fusion_config"]  = section(fusion_config);
    return j;
}

}  // namespace gw::server
