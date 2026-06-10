#include "server/routes_vio.hpp"

#include <exception>
#include <optional>
#include <string>

#include "server/route_helpers.hpp"
#include "server/vio_config_repository.hpp"
#include "server/vio_supervisor.hpp"

namespace gw::server {

namespace {

crow::json::wvalue config_to_json(const VioConfig& c) {
    crow::json::wvalue j;
    j["enabled"]              = c.enabled;
    j["num_pts"]              = c.num_pts;
    j["fast_threshold"]       = c.fast_threshold;
    j["downsample"]           = c.downsample;
    j["max_reproj_std_px"]    = c.max_reproj_std_px;
    j["auto_reinit"]          = c.auto_reinit;
    j["reinit_min_features"]  = c.reinit_min_features;
    j["reinit_window_frames"] = c.reinit_window_frames;
    j["reinit_max_pos_std_m"] = c.reinit_max_pos_std_m;
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

void register_vio_routes(crow::SimpleApp&     app,
                         VioSupervisor&       vio,
                         VioConfigRepository& vio_config) {
    CROW_ROUTE(app, "/api/vio/status").methods("GET"_method)
    ([&vio] {
        try {
            const auto st = vio.status();
            crow::json::wvalue j;
            j["enabled"]          = st.enabled;
            j["reason"]           = st.reason;
            j["running"]          = st.running;
            j["initialized"]      = st.initialized;
            j["phase"]            = st.phase;
            j["epoch"]            = st.epoch;
            j["reinits"]          = st.reinits;
            j["freq_hz"]          = st.freq_hz;
            j["tracked_features"] = st.tracked_features;
            j["cov_pos_std_m"]    = st.cov_pos_std_m;
            j["imu_rate_hz"]      = st.imu_rate_hz;

            crow::json::wvalue counters;
            counters["paired"]             = st.pair_counters.paired;
            counters["dropped_zero_ts"]    = st.pair_counters.dropped_zero_ts;
            counters["dropped_unmatched"]  = st.pair_counters.dropped_unmatched;
            counters["dropped_pair_queue"] = st.pair_counters.dropped_pair_queue;
            counters["frames_fed"]         = st.frames_fed;
            counters["imu_fed"]            = st.imu_fed;
            counters["imu_bus_dropped"]    = st.imu_bus_dropped;
            j["counters"] = std::move(counters);

            if (st.last) {
                crow::json::wvalue lp;
                lp["t_ns"]       = st.last->t_ns;
                lp["epoch"]      = st.last->epoch;
                lp["T_odom_imu"] = mat4_to_json(st.last->T_odom_imu);
                j["last_pose"]   = std::move(lp);
            } else {
                j["last_pose"] = nullptr;
            }

            crow::json::wvalue::list cams;
            for (const auto& c : st.cameras) {
                crow::json::wvalue cj;
                cj["camera_id"]      = c.camera_id;
                cj["name"]           = c.name;
                cj["role"]           = c.role;
                cj["feeder_running"] = c.feeder_running;
                put_opt(cj, "reproj_std_px", c.reproj_std_px);
                cams.emplace_back(std::move(cj));
            }
            j["cameras"] = std::move(cams);
            return json_response(200, std::move(j));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/vio/config").methods("GET"_method)
    ([&vio_config] {
        try {
            return json_response(200, config_to_json(vio_config.get()));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });

    CROW_ROUTE(app, "/api/vio/config").methods("PUT"_method)
    ([&vio_config, &vio](const crow::request& req) {
        const auto body = crow::json::load(req.body);
        if (!body) return error_response(400, "invalid JSON body");

        VioConfigUpdate patch;
        crow::response  err;
        if (!parse_opt_bool  (body, "enabled",              patch.enabled, err))              return err;
        if (!parse_opt_int   (body, "num_pts",              patch.num_pts, err))              return err;
        if (!parse_opt_int   (body, "fast_threshold",       patch.fast_threshold, err))       return err;
        if (!parse_opt_bool  (body, "downsample",           patch.downsample, err))           return err;
        if (!parse_opt_double(body, "max_reproj_std_px",    patch.max_reproj_std_px, err))    return err;
        if (!parse_opt_bool  (body, "auto_reinit",          patch.auto_reinit, err))          return err;
        if (!parse_opt_int   (body, "reinit_min_features",  patch.reinit_min_features, err))  return err;
        if (!parse_opt_int   (body, "reinit_window_frames", patch.reinit_window_frames, err)) return err;
        if (!parse_opt_double(body, "reinit_max_pos_std_m", patch.reinit_max_pos_std_m, err)) return err;

        if (patch.empty()) {
            return error_response(400, "PUT body must include at least one updatable field");
        }
        try {
            auto updated = vio_config.update(patch);
            vio.reload();  // fingerprint change ⇒ runner rebuild ⇒ reinit
            auto j           = config_to_json(updated);
            j["restarted"]   = true;
            return json_response(200, std::move(j));
        } catch (const std::exception& e) {
            // SQLite CHECK violations land here.
            return error_response(400, e.what());
        }
    });

    CROW_ROUTE(app, "/api/vio/restart").methods("POST"_method)
    ([&vio] {
        try {
            vio.restart();
            const auto st = vio.status();
            crow::json::wvalue j;
            j["epoch"]   = st.epoch;
            j["running"] = st.running;
            return json_response(200, std::move(j));
        } catch (const std::exception& e) {
            return error_response(500, e.what());
        }
    });
}

}  // namespace gw::server
