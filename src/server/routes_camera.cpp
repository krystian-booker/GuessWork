#include "server/routes_camera.hpp"

#include <exception>
#include <string>
#include <unordered_map>

#include "server/camera_repository.hpp"
#include "server/camera_supervisor.hpp"

namespace gw::server {

namespace {

crow::response with_no_store(crow::response res) {
    res.add_header("Cache-Control", "no-store");
    return res;
}

crow::response json_response(int status, crow::json::wvalue body) {
    crow::response res(status, body);
    return with_no_store(std::move(res));
}

crow::response error_response(int status, const std::string& message) {
    crow::json::wvalue body;
    body["error"] = message;
    return json_response(status, std::move(body));
}

crow::json::wvalue camera_to_json(const Camera& c, bool online) {
    crow::json::wvalue j;
    j["id"]         = c.id;
    j["name"]       = c.name;
    j["serial"]     = c.serial;
    j["online"]     = online;
    j["created_at"] = c.created_at;
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

struct CreateBody {
    std::string    name;
    std::string    serial;
    crow::response error;
    bool           ok = false;
};

CreateBody parse_create_body(const crow::request& req) {
    CreateBody r;
    const auto body = crow::json::load(req.body);
    if (!body) {
        r.error = error_response(400, "invalid JSON body");
        return r;
    }
    if (!parse_required_string(body, "name", r.name, r.error))     return r;
    if (!parse_required_string(body, "serial", r.serial, r.error)) return r;
    r.ok = true;
    return r;
}

struct UpdateBody {
    std::string    name;
    crow::response error;
    bool           ok = false;
};

UpdateBody parse_update_body(const crow::request& req) {
    UpdateBody r;
    const auto body = crow::json::load(req.body);
    if (!body) {
        r.error = error_response(400, "invalid JSON body");
        return r;
    }
    if (!parse_required_string(body, "name", r.name, r.error)) return r;
    r.ok = true;
    return r;
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
                const auto it = online.find(c.id);
                items.emplace_back(camera_to_json(c, it != online.end() && it->second));
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
            const auto c = repo.create(parsed.name, parsed.serial);
            supervisor.on_camera_added(c.id);
            return json_response(201, camera_to_json(c, supervisor.is_online(c.id)));
        } catch (const DuplicateNameError& e) {
            return error_response(409, e.what());
        } catch (const DuplicateSerialError& e) {
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
            return json_response(200, camera_to_json(*c, supervisor.is_online(c->id)));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/cameras/<int>").methods("PUT"_method)
    ([&repo, &supervisor](const crow::request& req, int64_t id) {
        auto parsed = parse_update_body(req);
        if (!parsed.ok) return std::move(parsed.error);
        try {
            const auto c = repo.update(id, parsed.name);
            if (!c) return error_response(404, "camera not found");
            return json_response(200, camera_to_json(*c, supervisor.is_online(c->id)));
        } catch (const DuplicateNameError& e) {
            return error_response(409, e.what());
        } catch (const std::exception& e) {
            return error_response(500, e.what());
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
