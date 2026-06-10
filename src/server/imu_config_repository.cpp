#include "server/imu_config_repository.hpp"

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

ImuConfig read_row(sqlite3_stmt* stmt) {
    ImuConfig c;
    c.rate_hz             = sqlite3_column_double(stmt, 0);
    c.accel_noise_density = sqlite3_column_double(stmt, 1);
    c.accel_random_walk   = sqlite3_column_double(stmt, 2);
    c.gyro_noise_density  = sqlite3_column_double(stmt, 3);
    c.gyro_random_walk    = sqlite3_column_double(stmt, 4);
    if (sqlite3_column_type(stmt, 5) != SQLITE_NULL) {
        c.t_imu_robot_json =
            reinterpret_cast<const char*>(sqlite3_column_text(stmt, 5));
    }
    c.updated_at = sqlite3_column_int64(stmt, 6);
    return c;
}

constexpr const char* kSelect =
    "SELECT rate_hz, accel_noise_density, accel_random_walk,"
    "       gyro_noise_density, gyro_random_walk, t_imu_robot_json, updated_at"
    "  FROM imu_config WHERE id = 1;";

}  // namespace

ImuConfig ImuConfigRepository::get() {
    return db_.with_handle([&](sqlite3* h) {
        StmtGuard g;
        if (sqlite3_prepare_v2(h, kSelect, -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "imu_config get: prepare");
        }
        if (sqlite3_step(g.stmt) != SQLITE_ROW) {
            throw std::runtime_error("imu_config row missing (schema seed failed?)");
        }
        return read_row(g.stmt);
    });
}

ImuConfig ImuConfigRepository::update(const ImuConfigUpdate& patch) {
    if (patch.empty()) return get();

    return db_.with_handle([&](sqlite3* h) {
        std::string sql = "UPDATE imu_config SET updated_at = strftime('%s','now')";
        if (patch.rate_hz)             sql += ", rate_hz = ?1";
        if (patch.accel_noise_density) sql += ", accel_noise_density = ?2";
        if (patch.accel_random_walk)   sql += ", accel_random_walk = ?3";
        if (patch.gyro_noise_density)  sql += ", gyro_noise_density = ?4";
        if (patch.gyro_random_walk)    sql += ", gyro_random_walk = ?5";
        if (patch.t_imu_robot_json)    sql += ", t_imu_robot_json = ?6";
        sql += " WHERE id = 1;";

        StmtGuard g;
        if (sqlite3_prepare_v2(h, sql.c_str(), -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "imu_config update: prepare");
        }
        if (patch.rate_hz)             sqlite3_bind_double(g.stmt, 1, *patch.rate_hz);
        if (patch.accel_noise_density) sqlite3_bind_double(g.stmt, 2, *patch.accel_noise_density);
        if (patch.accel_random_walk)   sqlite3_bind_double(g.stmt, 3, *patch.accel_random_walk);
        if (patch.gyro_noise_density)  sqlite3_bind_double(g.stmt, 4, *patch.gyro_noise_density);
        if (patch.gyro_random_walk)    sqlite3_bind_double(g.stmt, 5, *patch.gyro_random_walk);
        if (patch.t_imu_robot_json) {
            if (patch.t_imu_robot_json->has_value()) {
                sqlite3_bind_text(g.stmt, 6, (*patch.t_imu_robot_json)->c_str(),
                                  -1, SQLITE_TRANSIENT);
            } else {
                sqlite3_bind_null(g.stmt, 6);
            }
        }
        if (sqlite3_step(g.stmt) != SQLITE_DONE) {
            throw_sqlite(h, "imu_config update: step");
        }

        StmtGuard sel;
        if (sqlite3_prepare_v2(h, kSelect, -1, &sel.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "imu_config update: reselect");
        }
        if (sqlite3_step(sel.stmt) != SQLITE_ROW) {
            throw std::runtime_error("imu_config row missing after update");
        }
        return read_row(sel.stmt);
    });
}

}  // namespace gw::server
