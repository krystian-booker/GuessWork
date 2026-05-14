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
    j["camera_id"]        = c.id;
    j["calibrated_at"]    = c.calibrated_at ? crow::json::wvalue(*c.calibrated_at)
                                            : crow::json::wvalue(nullptr);
    // Re-parse the stored JSON so the client receives a structured object,
    // not a JSON-encoded string. If parsing fails (shouldn't, we wrote it),
    // fall back to the raw string.
    if (c.calibration_json) {
        const auto parsed = crow::json::load(*c.calibration_json);
        if (parsed) {
            j["calibration"] = crow::json::wvalue(parsed);
        } else {
            j["calibration"] = *c.calibration_json;
        }
    } else {
        j["calibration"] = nullptr;
    }
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

    // ---- Stored calibration (intrinsics JSON) ----

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

    // PUT body is the parsed contents of basalt_calibrate's calibration.json.
    // We only enforce a minimal shape check ("value" or "intrinsics" must be
    // present at top level) — Basalt's exact schema can drift; we'll tighten
    // when we link the library for VIO.
    CROW_ROUTE(app, "/api/cameras/<int>/calibration").methods("PUT"_method)
    ([&repo](const crow::request& req, int64_t id) {
        try {
            const auto body = crow::json::load(req.body);
            if (!body) {
                return error_response(400, "invalid JSON body");
            }
            // Basalt's calibration.json wraps everything under a top-level
            // "value0" key (cereal serialization). Accept either that or a
            // bare intrinsics shape so the front-end can be permissive.
            if (!body.has("value0") && !body.has("intrinsics") && !body.has("T_imu_cam")) {
                return error_response(400,
                    "calibration JSON must contain 'value0', 'intrinsics', or 'T_imu_cam'");
            }
            const auto c = repo.set_calibration(id, req.body);
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
