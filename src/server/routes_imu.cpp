#include "server/routes_imu.hpp"

#include "server/imu_attitude_service.hpp"

#include <exception>
#include <optional>
#include <string>

#include "calibration/calibration_store.hpp"
#include "server/apriltag_supervisor.hpp"
#include "server/fusion_supervisor.hpp"
#include "server/imu_allan_service.hpp"
#include "server/imu_config_repository.hpp"
#include "server/route_helpers.hpp"
#include "server/sync_controller_manager.hpp"

namespace gw::server {

namespace {

crow::json::wvalue config_to_json(const ImuConfig& c) {
    crow::json::wvalue j;
    j["rate_hz"]             = c.rate_hz;
    j["accel_noise_density"] = c.accel_noise_density;
    j["accel_random_walk"]   = c.accel_random_walk;
    j["gyro_noise_density"]  = c.gyro_noise_density;
    j["gyro_random_walk"]    = c.gyro_random_walk;
    if (c.t_imu_robot_json) {
        // Stored as a JSON document — re-emit it as structured JSON rather
        // than a quoted string.
        j["t_imu_robot"] = crow::json::load(*c.t_imu_robot_json);
    } else {
        j["t_imu_robot"] = nullptr;
    }
    j["updated_at"] = c.updated_at;
    return j;
}

bool parse_positive_number(const crow::json::rvalue& body, const char* field,
                           std::optional<double>& out, crow::response& err) {
    if (!body.has(field)) { out = std::nullopt; return true; }
    if (body[field].t() != crow::json::type::Number) {
        err = error_response(400, std::string(field) + " must be a number");
        return false;
    }
    const double v = body[field].d();
    if (!(v > 0.0)) {
        err = error_response(400, std::string(field) + " must be > 0");
        return false;
    }
    out = v;
    return true;
}

}  // namespace

void register_imu_routes(crow::SimpleApp&     app,
                         ImuConfigRepository& imu_config,
                         SyncControllerManager& controller,
                         ApriltagSupervisor&  apriltag,
                         FusionSupervisor&    fusion,
                         ImuAllanService&     allan,
                         ImuAttitudeService&  attitude) {
    CROW_ROUTE(app, "/api/imu/attitude").methods("GET"_method)
    ([&attitude] {
        const auto st = attitude.status();
        crow::json::wvalue j;
        j["initialized"] = st.attitude.initialized;
        j["rate_hz"]     = st.rate_hz;
        if (st.last_age_ms >= 0) j["last_age_ms"] = st.last_age_ms;
        else                     j["last_age_ms"] = nullptr;
        crow::json::wvalue q;
        q["w"] = st.attitude.q[0];
        q["x"] = st.attitude.q[1];
        q["y"] = st.attitude.q[2];
        q["z"] = st.attitude.q[3];
        j["q"] = std::move(q);
        crow::json::wvalue euler;
        euler["roll_deg"]  = st.attitude.roll_deg;
        euler["pitch_deg"] = st.attitude.pitch_deg;
        euler["yaw_deg"]   = st.attitude.yaw_deg;
        j["euler"] = std::move(euler);
        crow::json::wvalue accel, gyro;
        accel["x"] = st.accel[0]; accel["y"] = st.accel[1]; accel["z"] = st.accel[2];
        gyro["x"]  = st.gyro[0];  gyro["y"]  = st.gyro[1];  gyro["z"]  = st.gyro[2];
        j["accel_mps2"]   = std::move(accel);
        j["gyro_radps"]   = std::move(gyro);
        return json_response(200, std::move(j));
    });

    CROW_ROUTE(app, "/api/imu/attitude/zero-yaw").methods("POST"_method)
    ([&attitude] {
        attitude.zero_yaw();
        crow::json::wvalue j;
        j["ok"] = true;
        return json_response(200, std::move(j));
    });

    CROW_ROUTE(app, "/api/imu/status").methods("GET"_method)
    ([&controller] {
        const auto s = controller.status();
        crow::json::wvalue j;
        j["controller_connected"] = s.connected;
        j["imu_ok"]               = s.imu_ok;
        j["rate_hz"]              = s.imu_rate_hz;
        j["samples"]              = s.imu_samples;
        j["fw_drops"]             = s.imu_fw_drops;
        j["crc_errors"]           = s.imu_crc_errors;
        j["usb_errors"]           = s.usb_errors;
        put_opt(j, "last_sample_age_ms", s.imu_last_sample_age_ms);
        put_opt(j, "firmware_version",   s.firmware_version);
        put_opt(j, "protocol_version",   s.protocol_version);
        put_opt(j, "board",              s.board);
        return json_response(200, std::move(j));
    });

    CROW_ROUTE(app, "/api/imu/config").methods("GET"_method)
    ([&imu_config] {
        try {
            return json_response(200, config_to_json(imu_config.get()));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/imu/config").methods("PUT"_method)
    ([&imu_config, &apriltag, &fusion](const crow::request& req) {
        const auto body = crow::json::load(req.body);
        if (!body) return error_response(400, "invalid JSON body");

        ImuConfigUpdate patch;
        crow::response  err;
        if (!parse_positive_number(body, "rate_hz", patch.rate_hz, err)) return err;
        if (!parse_positive_number(body, "accel_noise_density",
                                   patch.accel_noise_density, err)) return err;
        if (!parse_positive_number(body, "accel_random_walk",
                                   patch.accel_random_walk, err)) return err;
        if (!parse_positive_number(body, "gyro_noise_density",
                                   patch.gyro_noise_density, err)) return err;
        if (!parse_positive_number(body, "gyro_random_walk",
                                   patch.gyro_random_walk, err)) return err;

        if (body.has("t_imu_robot")) {
            const auto& v = body["t_imu_robot"];
            if (v.t() == crow::json::type::Null) {
                patch.t_imu_robot_json = std::optional<std::string>{};  // clear
            } else if (v.t() == crow::json::type::Object) {
                const std::string json = crow::json::wvalue(v).dump();
                // Validate the transform's shape before storing: a bad
                // T_robot_imu silently corrupts every published robot pose.
                try {
                    gw::calib::parse_t_robot_imu(json);
                } catch (const std::exception& e) {
                    return error_response(400, e.what());
                }
                patch.t_imu_robot_json = std::optional<std::string>{json};
            } else {
                return error_response(400, "t_imu_robot must be an object or null");
            }
        }

        if (patch.empty()) {
            return error_response(400, "PUT body must include at least one updatable field");
        }
        try {
            auto resp = json_response(200, config_to_json(imu_config.update(patch)));
            if (patch.t_imu_robot_json) apriltag.reload_shared();
            // T_robot_imu and noise changes both matter to fusion.
            fusion.reload();
            return resp;
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    // --- Allan-variance refinement -----------------------------------------

    const auto analysis_to_json = [](const ImuAllanService::Analysis& a) {
        crow::json::wvalue j;
        j["file"]       = a.file;
        j["samples"]    = a.samples;
        j["duration_s"] = a.duration_s;
        j["rate_hz"]    = a.rate_hz;

        crow::json::wvalue sug;
        sug["accel_noise_density"] = a.accel_noise_density;
        sug["accel_random_walk"]   = a.accel_random_walk;
        sug["gyro_noise_density"]  = a.gyro_noise_density;
        sug["gyro_random_walk"]    = a.gyro_random_walk;
        j["suggested"] = std::move(sug);

        static const char* kAxisNames[6] = {"accel_x", "accel_y", "accel_z",
                                            "gyro_x",  "gyro_y",  "gyro_z"};
        crow::json::wvalue axes;
        for (int i = 0; i < 6; ++i) {
            crow::json::wvalue ax;
            ax["noise_density"]    = a.axes[i].noise_density;
            ax["random_walk"]      = a.axes[i].random_walk;
            ax["noise_density_ok"] = a.axes[i].noise_density_ok;
            ax["random_walk_ok"]   = a.axes[i].random_walk_ok;
            ax["fit_quality"]      = a.axes[i].fit_quality;
            axes[kAxisNames[i]] = std::move(ax);
        }
        j["axes"] = std::move(axes);

        crow::json::wvalue::list warns;
        for (const auto& w : a.warnings) warns.emplace_back(w);
        j["warnings"]    = std::move(warns);
        j["analyzed_at"] = a.analyzed_at;
        return j;
    };

    CROW_ROUTE(app, "/api/imu/allan/recording").methods("POST"_method)
    ([&allan](const crow::request& req) {
        const auto body = crow::json::load(req.body);
        if (!body || !body.has("duration_s") ||
            body["duration_s"].t() != crow::json::type::Number) {
            return error_response(400, "body must carry numeric duration_s");
        }
        const int64_t duration_s = body["duration_s"].i();
        std::string   err;
        if (!allan.start_recording(duration_s, err)) {
            const int code =
                err.find("in progress") != std::string::npos ? 409 : 400;
            return error_response(code, err);
        }
        const auto st = allan.recording_status();
        crow::json::wvalue j;
        j["recording"]  = true;
        put_opt(j, "file", st.file);
        j["duration_s"] = duration_s;
        return json_response(200, std::move(j));
    });

    CROW_ROUTE(app, "/api/imu/allan/recording").methods("DELETE"_method)
    ([&allan] {
        const auto before = allan.recording_status();
        if (!allan.stop_recording()) {
            return error_response(409, "no recording in progress");
        }
        crow::json::wvalue j;
        j["stopped"] = true;
        j["samples"] = before.samples;
        put_opt(j, "file", before.file);
        return json_response(200, std::move(j));
    });

    CROW_ROUTE(app, "/api/imu/allan/status").methods("GET"_method)
    ([&allan, analysis_to_json] {
        const auto st = allan.recording_status();
        crow::json::wvalue rec;
        rec["recording"] = st.recording;
        put_opt(rec, "file", st.file);
        rec["samples"]     = st.samples;
        rec["bytes"]       = st.bytes;
        rec["rate_hz"]     = st.rate_hz;
        rec["remaining_s"] = st.remaining_s;

        crow::json::wvalue j;
        j["recording"] = std::move(rec);
        if (const auto last = allan.last_analysis()) {
            j["last_analysis"] = analysis_to_json(*last);
        } else {
            j["last_analysis"] = nullptr;
        }
        return json_response(200, std::move(j));
    });

    CROW_ROUTE(app, "/api/imu/allan/analyze").methods("POST"_method)
    ([&allan, analysis_to_json](const crow::request& req) {
        std::string file;
        if (!req.body.empty()) {
            const auto body = crow::json::load(req.body);
            if (!body) return error_response(400, "invalid JSON body");
            if (body.has("file")) {
                if (body["file"].t() != crow::json::type::String) {
                    return error_response(400, "file must be a string");
                }
                file = body["file"].s();
            }
        }
        try {
            return json_response(200, analysis_to_json(allan.analyze(file)));
        } catch (const std::exception& e) {
            const std::string msg = e.what();
            const int code = msg.find("no IMU logs") != std::string::npos ? 404 : 400;
            return error_response(code, msg);
        }
    });

    CROW_ROUTE(app, "/api/imu/allan/apply").methods("POST"_method)
    ([&allan, &imu_config, &fusion] {
        std::string err;
        if (!allan.apply(err)) {
            const int code =
                err.find("no analysis") != std::string::npos ? 409 : 500;
            return error_response(code, err);
        }
        // Noise changes matter to fusion (and VIO picks them up on its next
        // reload, per the imu-config PUT convention).
        const bool restarted = fusion.reload();
        auto j = config_to_json(imu_config.get());
        j["restarted"] = restarted;
        return json_response(200, std::move(j));
    });
}

}  // namespace gw::server
