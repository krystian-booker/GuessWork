#include "server/routes_calibration.hpp"

#include <chrono>
#include <exception>
#include <string>

#include "server/calibration_supervisor.hpp"
#include "server/camera_repository.hpp"
#include "server/kalibr_job.hpp"
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

crow::json::wvalue job_status_to_json(const CalibrationJobStatus& s) {
    crow::json::wvalue j;
    j["state"]              = to_string(s.state);
    j["model"]              = s.model;
    j["exit_code"]          = s.exit_code;
    j["started_at_ms"]      = s.started_at_ms;
    j["ended_at_ms"]        = s.ended_at_ms;
    j["log_bytes"]          = static_cast<int64_t>(s.log_bytes);
    j["calibration_stored"] = s.calibration_stored;
    j["upload_error"]       = s.upload_error.empty()
                                  ? crow::json::wvalue(nullptr)
                                  : crow::json::wvalue(s.upload_error);
    return j;
}

// Per the SSE spec, every newline inside a data block becomes the boundary
// between two `data:` lines (each starts a new line of payload that the
// browser concatenates with '\n'). The event ends on a blank line. Splitting
// here avoids EventSource parsing the source code's literal '\n' as event
// separators when the subprocess output contains them.
std::string sse_data_event(const std::string& payload) {
    std::string out;
    out.reserve(payload.size() + 16);
    size_t start = 0;
    while (start <= payload.size()) {
        const auto nl = payload.find('\n', start);
        const auto end = (nl == std::string::npos) ? payload.size() : nl;
        out.append("data: ");
        out.append(payload, start, end - start);
        out.push_back('\n');
        if (nl == std::string::npos) break;
        start = nl + 1;
    }
    out.push_back('\n');  // blank line terminates the event
    return out;
}

std::string sse_done_event(const CalibrationJobStatus& s) {
    crow::json::wvalue done;
    done["state"]              = to_string(s.state);
    done["exit_code"]          = s.exit_code;
    done["calibration_stored"] = s.calibration_stored;
    done["upload_error"]       = s.upload_error.empty()
                                     ? crow::json::wvalue(nullptr)
                                     : crow::json::wvalue(s.upload_error);
    return "event: done\ndata: " + done.dump() + "\n\n";
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
            const auto cam = repo.get(id);
            if (!cam) return error_response(404, "camera not found");
            const auto r = calib.stop(id);

            // Recording succeeded — don't roll back if the Kalibr launch
            // races another running job. Surface it as 409 with the
            // recording_result still attached so the UI keeps the dataset.
            crow::json::wvalue body;
            body["recording_result"] = result_to_json(r);
            try {
                const auto job = calib.start_kalibr_job(id, r.path, cam->focal_length_mm);
                body["job"] = job_status_to_json(job);
            } catch (const CalibrationError& e) {
                body["job"]       = crow::json::wvalue(nullptr);
                body["job_error"] = e.what();
                return json_response(409, std::move(body));
            }
            return json_response(200, std::move(body));
        } catch (const CalibrationError& e) {
            return error_response(404, e.what());
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    // ---- Kalibr job (post-recording, automatic) ----
    //
    // The job is started by DELETE /recording above; these endpoints expose
    // status, the live log stream, and a cancel handle.

    CROW_ROUTE(app, "/api/cameras/<int>/calibration/job").methods("GET"_method)
    ([&calib](int64_t id) {
        try {
            const auto st = calib.kalibr_job_status(id);
            if (!st) return error_response(404, "no active job for this camera");
            return json_response(200, job_status_to_json(*st));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/cameras/<int>/calibration/job").methods("DELETE"_method)
    ([&calib](int64_t id) {
        try {
            if (!calib.kalibr_job_cancel(id)) {
                return error_response(404, "no running job for this camera");
            }
            return with_no_store(crow::response(204));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    // SSE log stream. Returns text/event-stream and keeps the connection open
    // until the subprocess reaches a terminal state. The handler thread is a
    // Crow worker — multithreaded mode is already on (see http_server.cpp),
    // so blocking here doesn't stall other requests.
    CROW_ROUTE(app, "/api/cameras/<int>/calibration/job/log").methods("GET"_method)
    ([&calib](const crow::request& /*req*/, crow::response& res, int64_t id) {
        const auto job = calib.kalibr_job_handle(id);
        if (!job) {
            res.code = 404;
            res.set_header("Content-Type", "application/json");
            res.write(R"({"error":"no active job for this camera"})");
            res.end();
            return;
        }

        res.set_header("Content-Type", "text/event-stream");
        res.set_header("Cache-Control", "no-store");
        res.set_header("Connection",    "keep-alive");

        size_t off = 0;
        // Flush whatever bytes have buffered up before we subscribed so a
        // late connector still sees the start of the run.
        const std::string initial = job->log_snapshot();
        if (!initial.empty()) {
            res.write(sse_data_event(initial));
            off = initial.size();
        }

        // Drain new bytes as they arrive. wait_for_log returns either when
        // log_bytes_ > off, the job reaches a terminal state, or the timeout
        // fires — the latter lets us periodically check job state without
        // sleeping forever if the subprocess goes quiet.
        for (;;) {
            const size_t now_bytes =
                job->wait_for_log(off, std::chrono::milliseconds(1000));
            if (now_bytes > off) {
                const std::string chunk = job->log_slice(off, now_bytes - off);
                res.write(sse_data_event(chunk));
                off = now_bytes;
            }
            const auto st = job->status();
            if (st.state != SubprocessState::Running) {
                res.write(sse_done_event(st));
                break;
            }
        }
        res.end();
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
