#include "server/routes_robot.hpp"

#include <exception>
#include <string>

#include "core/clock.hpp"
#include "net/robot_link.hpp"
#include "net/udp_payloads.h"
#include "server/net_config_repository.hpp"
#include "server/route_helpers.hpp"
#include "server/teensy_manager.hpp"

namespace gw::server {

namespace {

crow::json::wvalue config_to_json(const NetConfig& c) {
    crow::json::wvalue j;
    j["enabled"]    = c.enabled;
    j["bind_port"]  = c.bind_port;
    j["robot_port"] = c.robot_port;
    j["robot_ip"]   = c.robot_ip;  // "" = learn from inbound packets
    j["updated_at"] = c.updated_at;
    return j;
}

gw::net::RobotLink::Config link_config(const NetConfig& c) {
    gw::net::RobotLink::Config lc;
    lc.enabled    = c.enabled;
    lc.bind_port  = static_cast<uint16_t>(c.bind_port);
    lc.robot_port = static_cast<uint16_t>(c.robot_port);
    lc.robot_ip   = c.robot_ip;
    return lc;
}

const char* mode_name(uint8_t mode) {
    switch (mode) {
        case gw::udpp::kModeNominal:       return "nominal";
        case gw::udpp::kModeNoVio:         return "no_vio";
        case gw::udpp::kModeNoOdom:        return "no_odom";
        case gw::udpp::kModeTagsOnly:      return "tags_only";
        case gw::udpp::kModeDeadReckoning: return "dead_reckoning";
        case gw::udpp::kModeCollision:     return "collision";
        default:                           return "uninitialized";
    }
}

}  // namespace

void register_robot_routes(crow::SimpleApp&     app,
                           NetConfigRepository& net_config,
                           gw::net::RobotLink&  robot,
                           TeensyManager&       teensy) {
    CROW_ROUTE(app, "/api/robot/config").methods("GET"_method)
    ([&net_config] {
        try {
            return json_response(200, config_to_json(net_config.get()));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/robot/config").methods("PUT"_method)
    ([&net_config, &robot](const crow::request& req) {
        const auto body = crow::json::load(req.body);
        if (!body) return error_response(400, "invalid JSON body");

        NetConfigUpdate patch;
        if (body.has("enabled")) {
            const auto& v = body["enabled"];
            if (v.t() != crow::json::type::True &&
                v.t() != crow::json::type::False) {
                return error_response(400, "enabled must be a boolean");
            }
            patch.enabled = v.b();
        }
        for (const char* f : {"bind_port", "robot_port"}) {
            if (!body.has(f)) continue;
            if (body[f].t() != crow::json::type::Number) {
                return error_response(400, std::string(f) + " must be a number");
            }
            const int64_t port = body[f].i();
            if (port < 1024 || port > 65535) {
                return error_response(400,
                                      std::string(f) + " must be 1024..65535");
            }
            if (std::string(f) == "bind_port") patch.bind_port = port;
            else                                patch.robot_port = port;
        }
        if (body.has("robot_ip")) {
            if (body["robot_ip"].t() != crow::json::type::String) {
                return error_response(400, "robot_ip must be a string");
            }
            patch.robot_ip = std::string(body["robot_ip"].s());
        }

        NetConfig updated;
        try {
            updated = net_config.update(patch);
        } catch (const std::exception& e) {
            // CHECK-constraint violations land here.
            return error_response(400, e.what());
        }

        // Rebind the link. A failed rebind is not an error response — the DB
        // row is the source of truth; the response says what happened.
        auto j = config_to_json(updated);
        try {
            robot.reconfigure(link_config(updated));
            j["restarted"]     = true;
            j["restart_error"] = nullptr;
        } catch (const std::exception& e) {
            j["restarted"]     = false;
            j["restart_error"] = std::string(e.what());
        }
        return json_response(200, std::move(j));
    });

    CROW_ROUTE(app, "/api/robot/status").methods("GET"_method)
    ([&robot, &teensy] {
        const auto st = robot.status();
        const auto ts = teensy.status();

        crow::json::wvalue j;
        j["running"]   = st.running;
        j["bind_port"] = st.bind_port;
        if (st.robot_addr) j["robot_addr"] = *st.robot_addr;
        else               j["robot_addr"] = nullptr;

        crow::json::wvalue odom;
        odom["rate_hz"]      = st.odom_rate_hz;
        odom["packets"]      = st.rx_packets;
        odom["rejected"]     = st.rx_rejected;
        odom["counter_gaps"] = st.rx_counter_gaps;
        if (st.odom_last_age_ms) odom["last_age_ms"] = *st.odom_last_age_ms;
        else                     odom["last_age_ms"] = nullptr;
        if (st.odom_last) {
            crow::json::wvalue last;
            last["vx_mps"]       = st.odom_last->vx_mps;
            last["vy_mps"]       = st.odom_last->vy_mps;
            last["omega_radps"]  = st.odom_last->omega_radps;
            last["rio_time_us"]  = st.odom_last->rio_time_us;
            last["status_flags"] = st.odom_last->status_flags;
            last["t_ns"]         = st.odom_last->t_ns;
            odom["last"]         = std::move(last);
        } else {
            odom["last"] = nullptr;
        }
        j["odom"] = std::move(odom);

        crow::json::wvalue pose;
        pose["sent"]        = st.pose_sent;
        pose["send_errors"] = st.pose_send_errors;
        pose["no_dest"]     = st.pose_no_dest;
        j["pose"] = std::move(pose);

        // Both hops of the timestamp chain (docs/ethernet-protocol.md §time
        // sync). Chain healthy = both hops healthy.
        crow::json::wvalue rio_host;
        rio_host["healthy"]   = st.sync_healthy;
        rio_host["offset_us"] = st.sync_offset_us;
        rio_host["drift_ppm"] = st.sync_drift_ppm;
        rio_host["samples"]   = st.sync_samples;
        rio_host["resets"]    = st.sync_resets;

        crow::json::wvalue host_teensy;
        host_teensy["healthy"]   = ts.sync_healthy;
        host_teensy["offset_us"] = ts.sync_offset_us;
        host_teensy["drift_ppm"] = ts.sync_drift_ppm;
        host_teensy["samples"]   = ts.sync_samples;
        host_teensy["resets"]    = ts.sync_resets;

        crow::json::wvalue sync;
        sync["healthy"]     = st.sync_healthy && ts.sync_healthy;
        sync["rio_host"]    = std::move(rio_host);
        sync["host_teensy"] = std::move(host_teensy);
        j["clock_sync"] = std::move(sync);

        return json_response(200, std::move(j));
    });

    CROW_ROUTE(app, "/api/robot/pose").methods("POST"_method)
    ([&robot, &teensy](const crow::request& req) {
        const auto body = crow::json::load(req.body);
        if (!body) return error_response(400, "invalid JSON body");
        for (const char* f : {"x", "y", "theta"}) {
            if (!body.has(f) || body[f].t() != crow::json::type::Number) {
                return error_response(400, std::string(f) + " must be a number");
            }
        }
        gw::net::RobotLink::PoseSend pose;
        pose.x_m       = static_cast<float>(body["x"].d());
        pose.y_m       = static_cast<float>(body["y"].d());
        pose.theta_rad = static_cast<float>(body["theta"].d());
        pose.quality   = 255;
        pose.mode      = gw::udpp::kModeNominal;
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
        // Bench poses have no fusion timestamp; use Teensy-now via the
        // host↔Teensy mapping so the RIO stamp on the wire is sane.
        pose.t_ns = teensy.host_to_teensy_ns(gw::Clock::now_ns()).value_or(0);

        if (!robot.send_pose(pose)) {
            return error_response(503,
                                  "pose not sent (link down or no robot "
                                  "address learned yet)");
        }
        crow::json::wvalue j;
        j["sent"] = true;
        j["mode"] = mode_name(pose.mode);
        return json_response(200, std::move(j));
    });
}

}  // namespace gw::server
