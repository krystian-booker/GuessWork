#include "server/fusion_config_repository.hpp"

#include <sqlite3.h>

#include <stdexcept>
#include <string>

#include "server/database.hpp"

namespace gw::server {

namespace {

struct StmtGuard {
    sqlite3_stmt* stmt = nullptr;
    ~StmtGuard() { if (stmt) sqlite3_finalize(stmt); }
};

[[noreturn]] void throw_sqlite(sqlite3* h, const char* what) {
    throw std::runtime_error(std::string(what) + ": " + sqlite3_errmsg(h));
}

constexpr const char* kSelect =
    "SELECT enabled, lag_s, min_state_dt_ms, output_hz, max_extrapolation_ms,"
    "       tag_gate_chi2, tag_huber_k, vio_huber_k, odom_cauchy_k,"
    "       odom_sigma_vx, odom_sigma_vy, odom_sigma_omega,"
    "       vio_sigma_rot, vio_sigma_trans,"
    "       collision_inflation, collision_window, reinit_pos_std_m,"
    "       updated_at"
    "  FROM fusion_config WHERE id = 1;";

FusionConfig read_row(sqlite3_stmt* stmt) {
    FusionConfig c;
    c.enabled              = sqlite3_column_int(stmt, 0) != 0;
    c.lag_s                = sqlite3_column_double(stmt, 1);
    c.min_state_dt_ms      = sqlite3_column_int64(stmt, 2);
    c.output_hz            = sqlite3_column_int64(stmt, 3);
    c.max_extrapolation_ms = sqlite3_column_int64(stmt, 4);
    c.tag_gate_chi2        = sqlite3_column_double(stmt, 5);
    c.tag_huber_k          = sqlite3_column_double(stmt, 6);
    c.vio_huber_k          = sqlite3_column_double(stmt, 7);
    c.odom_cauchy_k        = sqlite3_column_double(stmt, 8);
    c.odom_sigma_vx        = sqlite3_column_double(stmt, 9);
    c.odom_sigma_vy        = sqlite3_column_double(stmt, 10);
    c.odom_sigma_omega     = sqlite3_column_double(stmt, 11);
    c.vio_sigma_rot        = sqlite3_column_double(stmt, 12);
    c.vio_sigma_trans      = sqlite3_column_double(stmt, 13);
    c.collision_inflation  = sqlite3_column_double(stmt, 14);
    c.collision_window     = sqlite3_column_int64(stmt, 15);
    c.reinit_pos_std_m     = sqlite3_column_double(stmt, 16);
    c.updated_at           = sqlite3_column_int64(stmt, 17);
    return c;
}

}  // namespace

FusionConfig FusionConfigRepository::get() {
    return db_.with_handle([&](sqlite3* h) {
        StmtGuard g;
        if (sqlite3_prepare_v2(h, kSelect, -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "fusion_config get: prepare");
        }
        if (sqlite3_step(g.stmt) != SQLITE_ROW) {
            throw std::runtime_error(
                "fusion_config row missing (schema seed failed?)");
        }
        return read_row(g.stmt);
    });
}

FusionConfig FusionConfigRepository::update(const FusionConfigUpdate& patch) {
    if (patch.empty()) return get();

    return db_.with_handle([&](sqlite3* h) {
        std::string sql =
            "UPDATE fusion_config SET updated_at = strftime('%s','now')";
        if (patch.enabled)              sql += ", enabled = ?1";
        if (patch.lag_s)                sql += ", lag_s = ?2";
        if (patch.min_state_dt_ms)      sql += ", min_state_dt_ms = ?3";
        if (patch.output_hz)            sql += ", output_hz = ?4";
        if (patch.max_extrapolation_ms) sql += ", max_extrapolation_ms = ?5";
        if (patch.tag_gate_chi2)        sql += ", tag_gate_chi2 = ?6";
        if (patch.tag_huber_k)          sql += ", tag_huber_k = ?7";
        if (patch.vio_huber_k)          sql += ", vio_huber_k = ?8";
        if (patch.odom_cauchy_k)        sql += ", odom_cauchy_k = ?9";
        if (patch.odom_sigma_vx)        sql += ", odom_sigma_vx = ?10";
        if (patch.odom_sigma_vy)        sql += ", odom_sigma_vy = ?11";
        if (patch.odom_sigma_omega)     sql += ", odom_sigma_omega = ?12";
        if (patch.vio_sigma_rot)        sql += ", vio_sigma_rot = ?13";
        if (patch.vio_sigma_trans)      sql += ", vio_sigma_trans = ?14";
        if (patch.collision_inflation)  sql += ", collision_inflation = ?15";
        if (patch.collision_window)     sql += ", collision_window = ?16";
        if (patch.reinit_pos_std_m)     sql += ", reinit_pos_std_m = ?17";
        sql += " WHERE id = 1;";

        StmtGuard g;
        if (sqlite3_prepare_v2(h, sql.c_str(), -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "fusion_config update: prepare");
        }
        if (patch.enabled)              sqlite3_bind_int   (g.stmt, 1, *patch.enabled ? 1 : 0);
        if (patch.lag_s)                sqlite3_bind_double(g.stmt, 2, *patch.lag_s);
        if (patch.min_state_dt_ms)      sqlite3_bind_int64 (g.stmt, 3, *patch.min_state_dt_ms);
        if (patch.output_hz)            sqlite3_bind_int64 (g.stmt, 4, *patch.output_hz);
        if (patch.max_extrapolation_ms) sqlite3_bind_int64 (g.stmt, 5, *patch.max_extrapolation_ms);
        if (patch.tag_gate_chi2)        sqlite3_bind_double(g.stmt, 6, *patch.tag_gate_chi2);
        if (patch.tag_huber_k)          sqlite3_bind_double(g.stmt, 7, *patch.tag_huber_k);
        if (patch.vio_huber_k)          sqlite3_bind_double(g.stmt, 8, *patch.vio_huber_k);
        if (patch.odom_cauchy_k)        sqlite3_bind_double(g.stmt, 9, *patch.odom_cauchy_k);
        if (patch.odom_sigma_vx)        sqlite3_bind_double(g.stmt, 10, *patch.odom_sigma_vx);
        if (patch.odom_sigma_vy)        sqlite3_bind_double(g.stmt, 11, *patch.odom_sigma_vy);
        if (patch.odom_sigma_omega)     sqlite3_bind_double(g.stmt, 12, *patch.odom_sigma_omega);
        if (patch.vio_sigma_rot)        sqlite3_bind_double(g.stmt, 13, *patch.vio_sigma_rot);
        if (patch.vio_sigma_trans)      sqlite3_bind_double(g.stmt, 14, *patch.vio_sigma_trans);
        if (patch.collision_inflation)  sqlite3_bind_double(g.stmt, 15, *patch.collision_inflation);
        if (patch.collision_window)     sqlite3_bind_int64 (g.stmt, 16, *patch.collision_window);
        if (patch.reinit_pos_std_m)     sqlite3_bind_double(g.stmt, 17, *patch.reinit_pos_std_m);

        if (sqlite3_step(g.stmt) != SQLITE_DONE) {
            throw_sqlite(h, "fusion_config update: step");
        }

        StmtGuard sel;
        if (sqlite3_prepare_v2(h, kSelect, -1, &sel.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "fusion_config update: reselect");
        }
        if (sqlite3_step(sel.stmt) != SQLITE_ROW) {
            throw std::runtime_error("fusion_config row missing after update");
        }
        return read_row(sel.stmt);
    });
}

}  // namespace gw::server
