#include "server/routes_camera.hpp"

#include <exception>
#include <string>

#include "server/camera_repository.hpp"

namespace gw::server {

namespace {

crow::json::wvalue camera_to_json(const Camera& c) {
    crow::json::wvalue j;
    j["id"]         = c.id;
    j["name"]       = c.name;
    j["created_at"] = c.created_at;
    return j;
}

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

// Reads a non-empty string "name" from a JSON body. On failure populates
// `err` with a 400 response and returns std::nullopt.
struct NameParseResult {
    std::string    name;
    crow::response error;
    bool           ok = false;
};

NameParseResult parse_name(const crow::request& req) {
    NameParseResult r;
    const auto body = crow::json::load(req.body);
    if (!body) {
        r.error = error_response(400, "invalid JSON body");
        return r;
    }
    if (!body.has("name") || body["name"].t() != crow::json::type::String) {
        r.error = error_response(400, "missing string field: name");
        return r;
    }
    std::string name = body["name"].s();
    if (name.empty()) {
        r.error = error_response(400, "name must be non-empty");
        return r;
    }
    r.name = std::move(name);
    r.ok   = true;
    return r;
}

}  // namespace

void register_camera_routes(crow::SimpleApp& app, CameraRepository& repo) {
    CROW_ROUTE(app, "/api/cameras").methods("GET"_method)
    ([&repo] {
        try {
            const auto rows = repo.list_all();
            crow::json::wvalue::list items;
            items.reserve(rows.size());
            for (const auto& c : rows) items.emplace_back(camera_to_json(c));
            return json_response(200, crow::json::wvalue(std::move(items)));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/cameras").methods("POST"_method)
    ([&repo](const crow::request& req) {
        auto parsed = parse_name(req);
        if (!parsed.ok) return std::move(parsed.error);
        try {
            const auto c = repo.create(parsed.name);
            return json_response(201, camera_to_json(c));
        } catch (const DuplicateNameError& e) {
            return error_response(409, e.what());
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/cameras/<int>").methods("GET"_method)
    ([&repo](int64_t id) {
        try {
            const auto c = repo.get(id);
            if (!c) return error_response(404, "camera not found");
            return json_response(200, camera_to_json(*c));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/cameras/<int>").methods("PUT"_method)
    ([&repo](const crow::request& req, int64_t id) {
        auto parsed = parse_name(req);
        if (!parsed.ok) return std::move(parsed.error);
        try {
            const auto c = repo.update(id, parsed.name);
            if (!c) return error_response(404, "camera not found");
            return json_response(200, camera_to_json(*c));
        } catch (const DuplicateNameError& e) {
            return error_response(409, e.what());
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/cameras/<int>").methods("DELETE"_method)
    ([&repo](int64_t id) {
        try {
            if (!repo.remove(id)) return error_response(404, "camera not found");
            return with_no_store(crow::response(204));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });
}

}  // namespace gw::server
