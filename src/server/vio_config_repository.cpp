#include "server/vio_config_repository.hpp"

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
    "SELECT enabled, num_pts, fast_threshold, downsample, max_reproj_std_px,"
    "       auto_reinit, reinit_min_features, reinit_window_frames,"
    "       reinit_max_pos_std_m, updated_at"
    "  FROM vio_config WHERE id = 1;";

VioConfig read_row(sqlite3_stmt* stmt) {
    VioConfig c;
    c.enabled              = sqlite3_column_int(stmt, 0) != 0;
    c.num_pts              = sqlite3_column_int64(stmt, 1);
    c.fast_threshold       = sqlite3_column_int64(stmt, 2);
    c.downsample           = sqlite3_column_int(stmt, 3) != 0;
    c.max_reproj_std_px    = sqlite3_column_double(stmt, 4);
    c.auto_reinit          = sqlite3_column_int(stmt, 5) != 0;
    c.reinit_min_features  = sqlite3_column_int64(stmt, 6);
    c.reinit_window_frames = sqlite3_column_int64(stmt, 7);
    c.reinit_max_pos_std_m = sqlite3_column_double(stmt, 8);
    c.updated_at           = sqlite3_column_int64(stmt, 9);
    return c;
}

}  // namespace

VioConfig VioConfigRepository::get() {
    return db_.with_handle([&](sqlite3* h) {
        StmtGuard g;
        if (sqlite3_prepare_v2(h, kSelect, -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "vio_config get: prepare");
        }
        if (sqlite3_step(g.stmt) != SQLITE_ROW) {
            throw std::runtime_error("vio_config row missing (schema seed failed?)");
        }
        return read_row(g.stmt);
    });
}

VioConfig VioConfigRepository::update(const VioConfigUpdate& patch) {
    if (patch.empty()) return get();

    return db_.with_handle([&](sqlite3* h) {
        std::string sql = "UPDATE vio_config SET updated_at = strftime('%s','now')";
        if (patch.enabled)              sql += ", enabled = ?1";
        if (patch.num_pts)              sql += ", num_pts = ?2";
        if (patch.fast_threshold)       sql += ", fast_threshold = ?3";
        if (patch.downsample)           sql += ", downsample = ?4";
        if (patch.max_reproj_std_px)    sql += ", max_reproj_std_px = ?5";
        if (patch.auto_reinit)          sql += ", auto_reinit = ?6";
        if (patch.reinit_min_features)  sql += ", reinit_min_features = ?7";
        if (patch.reinit_window_frames) sql += ", reinit_window_frames = ?8";
        if (patch.reinit_max_pos_std_m) sql += ", reinit_max_pos_std_m = ?9";
        sql += " WHERE id = 1;";

        StmtGuard g;
        if (sqlite3_prepare_v2(h, sql.c_str(), -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "vio_config update: prepare");
        }
        if (patch.enabled)              sqlite3_bind_int   (g.stmt, 1, *patch.enabled ? 1 : 0);
        if (patch.num_pts)              sqlite3_bind_int64 (g.stmt, 2, *patch.num_pts);
        if (patch.fast_threshold)       sqlite3_bind_int64 (g.stmt, 3, *patch.fast_threshold);
        if (patch.downsample)           sqlite3_bind_int   (g.stmt, 4, *patch.downsample ? 1 : 0);
        if (patch.max_reproj_std_px)    sqlite3_bind_double(g.stmt, 5, *patch.max_reproj_std_px);
        if (patch.auto_reinit)          sqlite3_bind_int   (g.stmt, 6, *patch.auto_reinit ? 1 : 0);
        if (patch.reinit_min_features)  sqlite3_bind_int64 (g.stmt, 7, *patch.reinit_min_features);
        if (patch.reinit_window_frames) sqlite3_bind_int64 (g.stmt, 8, *patch.reinit_window_frames);
        if (patch.reinit_max_pos_std_m) sqlite3_bind_double(g.stmt, 9, *patch.reinit_max_pos_std_m);

        if (sqlite3_step(g.stmt) != SQLITE_DONE) {
            throw_sqlite(h, "vio_config update: step");
        }

        StmtGuard sel;
        if (sqlite3_prepare_v2(h, kSelect, -1, &sel.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "vio_config update: reselect");
        }
        if (sqlite3_step(sel.stmt) != SQLITE_ROW) {
            throw std::runtime_error("vio_config row missing after update");
        }
        return read_row(sel.stmt);
    });
}

}  // namespace gw::server
