#include "server/routes_calibration.hpp"

#include <exception>
#include <string>

#include "server/calibration_supervisor.hpp"
#include "server/camera_repository.hpp"
#include "server/route_helpers.hpp"

namespace gw::server {

namespace {

crow::json::wvalue status_to_json(const CalibrationSessionStatus& s) {
    crow::json::wvalue j;
    j["session_id"]     = s.session_id;
    j["path"]           = s.path.string();
    j["frames_written"] = s.frames_written;
    j["frames_dropped"] = s.frames_dropped;
    j["elapsed_ms"]     = s.elapsed_ms;
    return j;
}

crow::json::wvalue result_to_json(const CalibrationSessionResult& r) {
    crow::json::wvalue j;
    j["session_id"]        = r.session_id;
    j["path"]              = r.path.string();
    j["frames_written"]    = r.frames_written;
    j["frames_dropped"]    = r.frames_dropped;
    j["elapsed_ms"]        = r.elapsed_ms;
    j["suggested_command"] = r.suggested_command;
    return j;
}

crow::json::wvalue calibration_summary_to_json(const Camera& c) {
    crow::json::wvalue j;
    j["camera_id"]     = c.id;
    j["calibrated_at"] = c.calibrated_at ? crow::json::wvalue(*c.calibrated_at)
                                         : crow::json::wvalue(nullptr);
    // calibration_json now holds raw Kalibr camchain YAML. The frontend
    // parses it with js-yaml; we just pass the string through.
    j["calibration"]   = c.calibration_json ? crow::json::wvalue(*c.calibration_json)
                                            : crow::json::wvalue(nullptr);
    return j;
}

}  // namespace

void register_calibration_routes(crow::SimpleApp&       app,
                                 CameraRepository&      repo,
                                 CalibrationSupervisor& calib) {
    // ---- Recording session ----

    CROW_ROUTE(app, "/api/cameras/<int>/calibration/recording").methods("POST"_method)
    ([&repo, &calib](int64_t id) {
        try {
            if (!repo.get(id)) return error_response(404, "camera not found");
            const auto status = calib.start(id);
            return json_response(201, status_to_json(status));
        } catch (const CalibrationError& e) {
            return error_response(409, e.what());
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/cameras/<int>/calibration/recording").methods("GET"_method)
    ([&repo, &calib](int64_t id) {
        try {
            if (!repo.get(id)) return error_response(404, "camera not found");
            const auto st = calib.status(id);
            if (!st) return error_response(404, "no active session");
            return json_response(200, status_to_json(*st));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/cameras/<int>/calibration/recording").methods("DELETE"_method)
    ([&repo, &calib](int64_t id) {
        try {
            if (!repo.get(id)) return error_response(404, "camera not found");
            const auto r = calib.stop(id);
            return json_response(200, result_to_json(r));
        } catch (const CalibrationError& e) {
            return error_response(404, e.what());
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    // ---- Stored calibration (Kalibr camchain YAML) ----

    CROW_ROUTE(app, "/api/cameras/<int>/calibration").methods("GET"_method)
    ([&repo](int64_t id) {
        try {
            const auto c = repo.get(id);
            if (!c) return error_response(404, "camera not found");
            if (!c->calibration_json) return error_response(404, "camera is not calibrated");
            return json_response(200, calibration_summary_to_json(*c));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    // PUT body is Kalibr's camchain YAML.
    //
    // Content-Type negotiation: text/yaml or application/x-yaml (preferred,
    // and what the web UI sends) stores req.body verbatim. application/json
    // accepts a transport wrapper `{ "yaml": "<camchain text>" }` so curl
    // users on JSON-only clients aren't surprised.
    //
    // Shape check: Kalibr camchains always start with a top-level `cam0:`
    // map. We grep for that substring rather than parsing YAML server-side
    // (yaml-cpp would be the only consumer of a YAML parser here).
    CROW_ROUTE(app, "/api/cameras/<int>/calibration").methods("PUT"_method)
    ([&repo](const crow::request& req, int64_t id) {
        try {
            std::string yaml_text;
            const auto  ctype = req.get_header_value("Content-Type");
            const bool  looks_json =
                ctype.find("application/json") != std::string::npos;
            if (looks_json) {
                const auto body = crow::json::load(req.body);
                if (!body || !body.has("yaml") ||
                    body["yaml"].t() != crow::json::type::String) {
                    return error_response(400,
                        "JSON body must contain string field \"yaml\"");
                }
                yaml_text = body["yaml"].s();
            } else {
                yaml_text = req.body;
            }

            if (yaml_text.find("cam0:") == std::string::npos) {
                return error_response(400,
                    "calibration YAML must contain a top-level 'cam0:' key (Kalibr camchain)");
            }
            const auto c = repo.set_calibration(id, yaml_text);
            if (!c) return error_response(404, "camera not found");
            return json_response(200, calibration_summary_to_json(*c));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/cameras/<int>/calibration").methods("DELETE"_method)
    ([&repo](int64_t id) {
        try {
            if (!repo.clear_calibration(id)) return error_response(404, "camera not found");
            return with_no_store(crow::response(204));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });
}

}  // namespace gw::server
