#include "server/routes_can.hpp"

#include <exception>
#include <string>

#include "server/can_config_repository.hpp"
#include "server/route_helpers.hpp"
#include "server/teensy_manager.hpp"

namespace gw::server {

namespace {

crow::json::wvalue config_to_json(const CanConfig& c) {
    crow::json::wvalue j;
    j["mode"]       = c.mode;
    j["updated_at"] = c.updated_at;
    return j;
}

// DB mode string → firmware mode. Validated before this is called.
CanMode mode_from_db(const std::string& mode) {
    if (mode == "roborio") return CanMode::Classic;
    if (mode == "systemcore") return CanMode::Fd;
    return CanMode::Off;
}

const char* fw_mode_name(int fw_mode) {
    switch (fw_mode) {
        case 0: return "off";
        case 1: return "classic";
        case 2: return "fd";
        default: return nullptr;  // unknown (no fw=3 heartbeat yet)
    }
}

}  // namespace

void register_can_routes(crow::SimpleApp&     app,
                         CanConfigRepository& can_config,
                         TeensyManager&       teensy) {
    CROW_ROUTE(app, "/api/can/config").methods("GET"_method)
    ([&can_config] {
        try {
            return json_response(200, config_to_json(can_config.get()));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/can/config").methods("PUT"_method)
    ([&can_config, &teensy](const crow::request& req) {
        const auto body = crow::json::load(req.body);
        if (!body) return error_response(400, "invalid JSON body");
        if (!body.has("mode") || body["mode"].t() != crow::json::type::String) {
            return error_response(400, "mode must be a string");
        }
        const std::string mode = body["mode"].s();
        if (mode != "off" && mode != "roborio" && mode != "systemcore") {
            return error_response(400,
                                  "mode must be 'off', 'roborio' or 'systemcore'");
        }

        CanConfig updated;
        try {
            CanConfigUpdate patch;
            patch.mode = mode;
            updated    = can_config.update(patch);
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }

        // Push to the Teensy. A failed push is not an error response — the
        // DB row is the source of truth and reconnect resync delivers it.
        std::string push_err;
        const bool pushed = teensy.set_can_mode(mode_from_db(mode), push_err);

        auto j = config_to_json(updated);
        j["pushed"] = pushed;
        if (pushed) j["push_error"] = nullptr;
        else        j["push_error"] = push_err;
        return json_response(200, std::move(j));
    });

    CROW_ROUTE(app, "/api/can/status").methods("GET"_method)
    ([&can_config, &teensy] {
        const auto s = teensy.status();
        crow::json::wvalue j;
        try {
            j["mode"] = can_config.get().mode;
        } catch (...) {
            j["mode"] = nullptr;
        }
        if (const char* fw_mode = fw_mode_name(s.can_mode_fw)) {
            j["fw_mode"] = fw_mode;
        } else {
            j["fw_mode"] = nullptr;
        }
        j["teensy_connected"]    = s.connected;
        j["telemetry_connected"] = s.telemetry_connected;
        put_opt(j, "fw_version", s.fw_version);
        j["can_ok"] = s.can_ok;

        crow::json::wvalue odom;
        odom["rate_hz"] = s.odom_rate_hz;
        odom["packets"] = s.odom_packets;
        put_opt(odom, "last_age_ms", s.odom_last_age_ms);
        if (s.odom_last) {
            crow::json::wvalue last;
            last["t_ns"]         = s.odom_last->t_ns;
            last["t_arrival_ns"] = s.odom_last->t_arrival_ns;
            last["rio_time_us"]  = s.odom_last->rio_time_us;
            last["vx_mps"]       = s.odom_last->vx_mps;
            last["vy_mps"]       = s.odom_last->vy_mps;
            last["omega_radps"]  = s.odom_last->omega_radps;
            last["status_flags"] = s.odom_last->status_flags;
            last["counter"]      = s.odom_last->counter;
            odom["last"] = std::move(last);
        } else {
            odom["last"] = nullptr;
        }
        j["odom"] = std::move(odom);

        crow::json::wvalue counters;
        counters["can_rx"]           = s.can_rx;
        counters["can_rx_drops"]     = s.can_rx_drops;
        counters["odom_tx_fw_drops"] = s.odom_tx_fw_drops;
        counters["odom_crc_errors"]  = s.imu_crc_errors;  // shared telemetry CRC counter
        counters["pose_tx_fw"]       = s.pose_tx_fw;
        counters["pose_sent"]        = s.pose_sent;
        counters["pose_send_errors"] = s.pose_send_errors;
        j["counters"] = std::move(counters);

        crow::json::wvalue sync;
        sync["healthy"]   = s.sync_healthy;
        sync["offset_us"] = s.sync_offset_us;
        sync["drift_ppm"] = s.sync_drift_ppm;
        sync["samples"]   = s.sync_samples;
        sync["resets"]    = s.sync_resets;
        j["clock_sync"] = std::move(sync);

        return json_response(200, std::move(j));
    });

    // Bench/debug downlink. Phase 6's fusion graph will call
    // TeensyManager::send_pose directly; this exercises the same path with a
    // hand-supplied pose.
    CROW_ROUTE(app, "/api/can/pose").methods("POST"_method)
    ([&teensy](const crow::request& req) {
        const auto body = crow::json::load(req.body);
        if (!body) return error_response(400, "invalid JSON body");
        for (const char* f : {"x", "y", "theta"}) {
            if (!body.has(f) || body[f].t() != crow::json::type::Number) {
                return error_response(400, std::string(f) + " must be a number");
            }
        }
        gw::FusedPose pose;
        pose.x_m       = static_cast<float>(body["x"].d());
        pose.y_m       = static_cast<float>(body["y"].d());
        pose.theta_rad = static_cast<float>(body["theta"].d());
        pose.quality   = 255;
        if (body.has("quality")) {
            if (body["quality"].t() != crow::json::type::Number) {
                return error_response(400, "quality must be a number");
            }
            const auto q = body["quality"].i();
            if (q < 0 || q > 255) {
                return error_response(400, "quality must be 0..255");
            }
            pose.quality = static_cast<uint8_t>(q);
        }
        // Bench poses have no fusion timestamp; use the freshest Teensy-domain
        // time we know (the last odom arrival) so the RIO mapping is sane.
        const auto s = teensy.status();
        pose.t_ns    = s.odom_last ? s.odom_last->t_arrival_ns : 0;

        std::string err;
        if (!teensy.send_pose(pose, err)) {
            return error_response(503, err);
        }
        crow::json::wvalue j;
        j["sent"] = true;
        return json_response(200, std::move(j));
    });
}

}  // namespace gw::server
