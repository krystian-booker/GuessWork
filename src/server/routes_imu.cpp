#include "server/routes_imu.hpp"

#include <exception>
#include <optional>
#include <string>

#include "server/imu_config_repository.hpp"
#include "server/route_helpers.hpp"
#include "server/teensy_manager.hpp"

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
                         TeensyManager&       teensy) {
    CROW_ROUTE(app, "/api/imu/status").methods("GET"_method)
    ([&teensy] {
        const auto s = teensy.status();
        crow::json::wvalue j;
        j["teensy_connected"]    = s.connected;
        j["telemetry_connected"] = s.telemetry_connected;
        j["imu_ok"]              = s.imu_ok;
        j["rate_hz"]             = s.imu_rate_hz;
        j["samples"]             = s.imu_samples;
        j["fw_drops"]            = s.imu_fw_drops;
        j["crc_errors"]          = s.imu_crc_errors;
        put_opt(j, "last_sample_age_ms", s.imu_last_sample_age_ms);
        put_opt(j, "fw_version",         s.fw_version);
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
    ([&imu_config](const crow::request& req) {
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
                patch.t_imu_robot_json =
                    std::optional<std::string>{crow::json::wvalue(v).dump()};
            } else {
                return error_response(400, "t_imu_robot must be an object or null");
            }
        }

        if (patch.empty()) {
            return error_response(400, "PUT body must include at least one updatable field");
        }
        try {
            return json_response(200, config_to_json(imu_config.update(patch)));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });
}

}  // namespace gw::server
