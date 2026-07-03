#include "server/routes_hardware_sync.hpp"

#include <exception>
#include <optional>
#include <string>
#include <vector>

#include "server/route_helpers.hpp"
#include "server/teensy_manager.hpp"
#include "server/trigger_group_repository.hpp"

namespace gw::server {

namespace {

crow::json::wvalue group_to_json(const TriggerGroup& g) {
    crow::json::wvalue j;
    j["id"]   = g.id;
    j["name"] = g.name;
    j["fps"]  = g.fps;
    crow::json::wvalue::list pins;
    pins.reserve(g.output_pins.size());
    for (auto p : g.output_pins) pins.emplace_back(static_cast<int>(p));
    j["output_pins"] = std::move(pins);
    j["created_at"]  = g.created_at;
    return j;
}

bool parse_pins_array(const crow::json::rvalue& body, const char* field,
                      std::optional<std::vector<uint8_t>>& out,
                      crow::response& err) {
    if (!body.has(field)) { out = std::nullopt; return true; }
    const auto& v = body[field];
    if (v.t() != crow::json::type::List) {
        err = error_response(400, std::string("field must be an array: ") + field);
        return false;
    }
    std::vector<uint8_t> pins;
    pins.reserve(v.size());
    for (const auto& e : v) {
        if (e.t() != crow::json::type::Number) {
            err = error_response(400, std::string("array must contain numbers: ") + field);
            return false;
        }
        const int64_t n = e.i();
        if (n < 1 || n > 6) {
            err = error_response(400, "output_pins entries must be 1..6");
            return false;
        }
        pins.push_back(static_cast<uint8_t>(n));
    }
    out = std::move(pins);
    return true;
}

crow::response map_repo_exception_to_http(const std::exception& e) {
    if (dynamic_cast<const DuplicateTriggerGroupNameError*>(&e)) {
        return error_response(409, e.what());
    }
    if (dynamic_cast<const TriggerOutputPinConflictError*>(&e)) {
        return error_response(409, e.what());
    }
    if (dynamic_cast<const InvalidTriggerOutputsError*>(&e)) {
        return error_response(400, e.what());
    }
    return error_response(500, e.what());
}

}  // namespace

void register_hardware_sync_routes(crow::SimpleApp&        app,
                                   TriggerGroupRepository& repo,
                                   TeensyManager&          teensy) {
    CROW_ROUTE(app, "/api/hardware-sync/status").methods("GET"_method)
    ([&teensy] {
        const auto s = teensy.status();
        crow::json::wvalue j;
        j["connected"]    = s.connected;
        j["armed"]        = s.armed;
        j["total_pulses"] = s.total_pulses;
        put_opt(j, "port",              s.port);
        put_opt(j, "last_pulse_age_ms", s.last_pulse_age_ms);
        put_opt(j, "last_error",        s.last_error);
        return json_response(200, std::move(j));
    });

    CROW_ROUTE(app, "/api/hardware-sync/groups").methods("GET"_method)
    ([&repo] {
        try {
            const auto rows = repo.list_all();
            crow::json::wvalue::list items;
            items.reserve(rows.size());
            for (const auto& g : rows) items.emplace_back(group_to_json(g));
            return json_response(200, crow::json::wvalue(std::move(items)));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/hardware-sync/groups").methods("POST"_method)
    ([&repo](const crow::request& req) {
        const auto body = crow::json::load(req.body);
        if (!body) return error_response(400, "invalid JSON body");
        if (!body.has("name") || body["name"].t() != crow::json::type::String) {
            return error_response(400, "missing string field: name");
        }
        if (!body.has("fps") || body["fps"].t() != crow::json::type::Number) {
            return error_response(400, "missing number field: fps");
        }
        std::string                         name(body["name"].s());
        if (name.empty()) return error_response(400, "name must be non-empty");
        const double                        fps = body["fps"].d();
        std::optional<std::vector<uint8_t>> pins;
        crow::response                      err;
        if (!parse_pins_array(body, "output_pins", pins, err)) return err;
        if (!pins || pins->empty()) {
            return error_response(400, "output_pins is required and must be non-empty");
        }
        try {
            const auto g = repo.create(name, fps, *pins);
            return json_response(201, group_to_json(g));
        } catch (const std::exception& e) {
            return map_repo_exception_to_http(e);
        }
    });

    CROW_ROUTE(app, "/api/hardware-sync/groups/<int>").methods("PUT"_method)
    ([&repo](const crow::request& req, int64_t id) {
        const auto body = crow::json::load(req.body);
        if (!body) return error_response(400, "invalid JSON body");

        TriggerGroupUpdate patch;
        if (body.has("name")) {
            if (body["name"].t() != crow::json::type::String) {
                return error_response(400, "name must be a string");
            }
            std::string v(body["name"].s());
            if (v.empty()) return error_response(400, "name must be non-empty");
            patch.name = std::move(v);
        }
        if (body.has("fps")) {
            if (body["fps"].t() != crow::json::type::Number) {
                return error_response(400, "fps must be a number");
            }
            patch.fps = body["fps"].d();
        }
        std::optional<std::vector<uint8_t>> pins;
        crow::response                      err;
        if (!parse_pins_array(body, "output_pins", pins, err)) return err;
        if (pins) patch.output_pins = std::move(*pins);

        if (patch.empty()) {
            return error_response(400, "PUT body must include at least one updatable field");
        }
        try {
            const auto g = repo.update(id, patch);
            if (!g) return error_response(404, "trigger group not found");
            return json_response(200, group_to_json(*g));
        } catch (const std::exception& e) {
            return map_repo_exception_to_http(e);
        }
    });

    CROW_ROUTE(app, "/api/hardware-sync/groups/<int>").methods("DELETE"_method)
    ([&repo](int64_t id) {
        try {
            if (!repo.remove(id)) return error_response(404, "trigger group not found");
            return with_no_store(crow::response(204));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/hardware-sync/arm").methods("POST"_method)
    ([&repo, &teensy] {
        try {
            const auto groups = repo.list_all();
            if (groups.empty()) {
                return error_response(409, "no trigger groups configured");
            }
            // Intent first: even if the push below fails, the desired state
            // is persisted (survives reboots) and TeensyManager's retry
            // resync converges the device onto it.
            repo.set_armed(true);
            std::vector<TeensyManager::GroupConfig> cfg;
            cfg.reserve(groups.size());
            for (const auto& g : groups) {
                cfg.push_back({g.name, g.fps, g.output_pins});
            }
            std::string err;
            const bool pushed = teensy.push_config(cfg, err);
            crow::json::wvalue j;
            j["armed_desired"] = true;
            j["pushed"]        = pushed;
            if (pushed) j["push_error"] = nullptr;
            else        j["push_error"] = err.empty() ? "arm push failed (retrying)" : err + " (retrying)";
            return json_response(200, std::move(j));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/hardware-sync/stop").methods("POST"_method)
    ([&repo, &teensy] {
        try {
            repo.set_armed(false);  // intent first, mirror of arm
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
        std::string err;
        if (!teensy.stop_outputs(err)) {
            return error_response(503, err.empty() ? "stop failed" : err);
        }
        return json_response(200, crow::json::wvalue{});
    });
}

}  // namespace gw::server
