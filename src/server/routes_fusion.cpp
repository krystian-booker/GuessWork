#include "server/routes_fusion.hpp"

#include <cmath>
#include <exception>
#include <optional>
#include <string>

#include "server/fusion_config_repository.hpp"
#include "server/fusion_supervisor.hpp"
#include "server/route_helpers.hpp"

namespace gw::server {

namespace {

crow::json::wvalue config_to_json(const FusionConfig& c) {
    crow::json::wvalue j;
    j["enabled"]              = c.enabled;
    j["lag_s"]                = c.lag_s;
    j["min_state_dt_ms"]      = c.min_state_dt_ms;
    j["output_hz"]            = c.output_hz;
    j["max_extrapolation_ms"] = c.max_extrapolation_ms;
    j["tag_gate_chi2"]        = c.tag_gate_chi2;
    j["tag_huber_k"]          = c.tag_huber_k;
    j["vio_huber_k"]          = c.vio_huber_k;
    j["odom_cauchy_k"]        = c.odom_cauchy_k;
    j["odom_sigma_vx"]        = c.odom_sigma_vx;
    j["odom_sigma_vy"]        = c.odom_sigma_vy;
    j["odom_sigma_omega"]     = c.odom_sigma_omega;
    j["vio_sigma_rot"]        = c.vio_sigma_rot;
    j["vio_sigma_trans"]      = c.vio_sigma_trans;
    j["collision_inflation"]  = c.collision_inflation;
    j["collision_window"]     = c.collision_window;
    j["reinit_pos_std_m"]     = c.reinit_pos_std_m;
    j["updated_at"]           = c.updated_at;
    return j;
}

crow::json::wvalue mat4_to_json(const gw::apriltag::Mat4& T) {
    crow::json::wvalue::list rows;
    for (int r = 0; r < 4; ++r) {
        crow::json::wvalue::list row;
        for (int c = 0; c < 4; ++c) row.emplace_back(T[r][c]);
        rows.emplace_back(std::move(row));
    }
    return crow::json::wvalue(std::move(rows));
}

bool parse_opt_bool(const crow::json::rvalue& body, const char* field,
                    std::optional<bool>& out, crow::response& err) {
    if (!body.has(field)) return true;
    const auto t = body[field].t();
    if (t == crow::json::type::True || t == crow::json::type::False) {
        out = body[field].b();
        return true;
    }
    err = error_response(400, std::string(field) + " must be a boolean");
    return false;
}

bool parse_opt_int(const crow::json::rvalue& body, const char* field,
                   std::optional<int64_t>& out, crow::response& err) {
    if (!body.has(field)) return true;
    if (body[field].t() != crow::json::type::Number) {
        err = error_response(400, std::string(field) + " must be a number");
        return false;
    }
    out = body[field].i();
    return true;
}

bool parse_opt_double(const crow::json::rvalue& body, const char* field,
                      std::optional<double>& out, crow::response& err) {
    if (!body.has(field)) return true;
    if (body[field].t() != crow::json::type::Number) {
        err = error_response(400, std::string(field) + " must be a number");
        return false;
    }
    out = body[field].d();
    return true;
}

}  // namespace

void register_fusion_routes(crow::SimpleApp&        app,
                            FusionSupervisor&       fusion,
                            FusionConfigRepository& fusion_config) {
    CROW_ROUTE(app, "/api/fusion/status").methods("GET"_method)
    ([&fusion] {
        try {
            const auto st = fusion.status();
            crow::json::wvalue j;
            j["enabled"]        = st.enabled;
            j["reason"]         = st.reason;
            j["initialized"]    = st.state.initialized;
            j["collision_mode"] = st.state.collision_mode;
            j["reinits"]        = st.counters.reinits;
            j["quality"]        = st.state.quality;

            if (st.state.initialized) {
                const auto& T = st.state.T_field_robot;
                crow::json::wvalue pose;
                pose["x_m"]       = T[0][3];
                pose["y_m"]       = T[1][3];
                pose["theta_rad"] = std::atan2(T[1][0], T[0][0]);
                pose["t_ns"]      = st.state.t_ns;
                j["pose"]          = std::move(pose);
                j["T_field_robot"] = mat4_to_json(T);
            } else {
                j["pose"]          = nullptr;
                j["T_field_robot"] = nullptr;
            }

            crow::json::wvalue sources;
            {
                crow::json::wvalue tag;
                tag["rate_hz"] = st.tag.rate_hz;
                put_opt(tag, "last_age_ms", st.tag.last_age_ms);
                tag["accepted"]       = st.counters.tag_accepted;
                tag["rejected_gate"]  = st.counters.tag_rejected_gate;
                tag["rejected_clock"] = st.counters.tag_rejected_clock;
                tag["rejected_stale"] = st.counters.tag_rejected_stale;
                tag["bus_dropped"]    = st.tag.bus_dropped;
                sources["tag"] = std::move(tag);
            }
            {
                crow::json::wvalue vio;
                vio["enabled"] = st.vio_enabled;
                if (st.vio_enabled) vio["reason"] = nullptr;
                else                vio["reason"] = st.vio_reason;
                vio["rate_hz"] = st.vio.rate_hz;
                put_opt(vio, "last_age_ms", st.vio.last_age_ms);
                vio["fused_intervals"]   = st.counters.vio_fused_intervals;
                vio["skipped_epoch"]     = st.counters.vio_skipped_epoch;
                vio["skipped_unhealthy"] = st.counters.vio_skipped_unhealthy;
                vio["bus_dropped"]       = st.vio.bus_dropped;
                sources["vio"] = std::move(vio);
            }
            {
                crow::json::wvalue odom;
                odom["rate_hz"] = st.odom.rate_hz;
                put_opt(odom, "last_age_ms", st.odom.last_age_ms);
                odom["fused_intervals"] = st.counters.odom_fused_intervals;
                odom["stale"]           = st.counters.odom_stale;
                odom["slip"]            = st.counters.odom_slip;
                odom["bus_dropped"]     = st.odom.bus_dropped;
                sources["odom"] = std::move(odom);
            }
            j["sources"] = std::move(sources);

            crow::json::wvalue solve;
            solve["last"] = st.counters.solve_ms_last;
            solve["p95"]  = st.counters.solve_ms_p95;
            j["solve_ms"] = std::move(solve);

            crow::json::wvalue lag;
            lag["states"]       = st.counters.lag_states;
            lag["lag_s"]        = st.lag_s;
            lag["oldest_age_s"] = st.counters.oldest_state_age_s;
            j["lag"] = std::move(lag);

            crow::json::wvalue tnow;
            tnow["healthy"]   = st.teensy_now_healthy;
            tnow["offset_ms"] = st.teensy_now_offset_ms;
            j["teensy_now"] = std::move(tnow);

            crow::json::wvalue output;
            output["sent"]              = st.output_sent;
            output["send_errors"]       = st.output_send_errors;
            output["queue_dropped"]     = st.queue_dropped;
            output["bridge_factors"]    = st.counters.bridge_factors;
            output["gate_reopens"]      = st.counters.gate_reopens;
            output["update_exceptions"] = st.counters.update_exceptions;
            j["output"] = std::move(output);

            return json_response(200, std::move(j));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/fusion/config").methods("GET"_method)
    ([&fusion_config] {
        try {
            return json_response(200, config_to_json(fusion_config.get()));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/fusion/config").methods("PUT"_method)
    ([&fusion_config, &fusion](const crow::request& req) {
        const auto body = crow::json::load(req.body);
        if (!body) return error_response(400, "invalid JSON body");

        FusionConfigUpdate patch;
        crow::response     err;
        if (!parse_opt_bool(body, "enabled", patch.enabled, err)) return err;
        if (!parse_opt_double(body, "lag_s", patch.lag_s, err)) return err;
        if (!parse_opt_int(body, "min_state_dt_ms", patch.min_state_dt_ms, err)) return err;
        if (!parse_opt_int(body, "output_hz", patch.output_hz, err)) return err;
        if (!parse_opt_int(body, "max_extrapolation_ms",
                           patch.max_extrapolation_ms, err)) return err;
        if (!parse_opt_double(body, "tag_gate_chi2", patch.tag_gate_chi2, err)) return err;
        if (!parse_opt_double(body, "tag_huber_k", patch.tag_huber_k, err)) return err;
        if (!parse_opt_double(body, "vio_huber_k", patch.vio_huber_k, err)) return err;
        if (!parse_opt_double(body, "odom_cauchy_k", patch.odom_cauchy_k, err)) return err;
        if (!parse_opt_double(body, "odom_sigma_vx", patch.odom_sigma_vx, err)) return err;
        if (!parse_opt_double(body, "odom_sigma_vy", patch.odom_sigma_vy, err)) return err;
        if (!parse_opt_double(body, "odom_sigma_omega",
                              patch.odom_sigma_omega, err)) return err;
        if (!parse_opt_double(body, "vio_sigma_rot", patch.vio_sigma_rot, err)) return err;
        if (!parse_opt_double(body, "vio_sigma_trans",
                              patch.vio_sigma_trans, err)) return err;
        if (!parse_opt_double(body, "collision_inflation",
                              patch.collision_inflation, err)) return err;
        if (!parse_opt_int(body, "collision_window",
                           patch.collision_window, err)) return err;
        if (!parse_opt_double(body, "reinit_pos_std_m",
                              patch.reinit_pos_std_m, err)) return err;

        if (patch.empty()) {
            return error_response(
                400, "PUT body must include at least one updatable field");
        }
        try {
            const auto updated = fusion_config.update(patch);
            const bool restarted = fusion.reload();
            auto j = config_to_json(updated);
            j["restarted"] = restarted;
            return json_response(200, std::move(j));
        } catch (const std::exception& e) {
            // SQLite CHECK violations land here — they're client errors.
            return error_response(400, e.what());
        }
    });

    CROW_ROUTE(app, "/api/fusion/reset").methods("POST"_method)
    ([&fusion] {
        try {
            fusion.reset();
            crow::json::wvalue j;
            j["reinits"] = fusion.status().counters.reinits;
            return json_response(200, std::move(j));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });
}

}  // namespace gw::server
