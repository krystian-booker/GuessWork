#include "server/net_config_repository.hpp"

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
    "SELECT enabled, bind_port, robot_port, robot_ip, updated_at "
    "FROM net_config WHERE id = 1;";

NetConfig read_row(sqlite3_stmt* stmt) {
    NetConfig c;
    c.enabled    = sqlite3_column_int64(stmt, 0) != 0;
    c.bind_port  = sqlite3_column_int64(stmt, 1);
    c.robot_port = sqlite3_column_int64(stmt, 2);
    if (const auto* ip = sqlite3_column_text(stmt, 3)) {
        c.robot_ip = reinterpret_cast<const char*>(ip);
    }
    c.updated_at = sqlite3_column_int64(stmt, 4);
    return c;
}

}  // namespace

NetConfig NetConfigRepository::get() {
    return db_.with_handle([&](sqlite3* h) {
        StmtGuard g;
        if (sqlite3_prepare_v2(h, kSelect, -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "net_config get: prepare");
        }
        if (sqlite3_step(g.stmt) != SQLITE_ROW) {
            throw std::runtime_error("net_config row missing (schema seed failed?)");
        }
        return read_row(g.stmt);
    });
}

NetConfig NetConfigRepository::update(const NetConfigUpdate& patch) {
    if (patch.empty()) return get();

    return db_.with_handle([&](sqlite3* h) {
        std::string sql = "UPDATE net_config SET updated_at = strftime('%s','now')";
        if (patch.enabled)    sql += ", enabled = ?1";
        if (patch.bind_port)  sql += ", bind_port = ?2";
        if (patch.robot_port) sql += ", robot_port = ?3";
        if (patch.robot_ip)   sql += ", robot_ip = ?4";
        sql += " WHERE id = 1;";

        StmtGuard g;
        if (sqlite3_prepare_v2(h, sql.c_str(), -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "net_config update: prepare");
        }
        if (patch.enabled) {
            sqlite3_bind_int64(g.stmt, 1, *patch.enabled ? 1 : 0);
        }
        if (patch.bind_port) sqlite3_bind_int64(g.stmt, 2, *patch.bind_port);
        if (patch.robot_port) sqlite3_bind_int64(g.stmt, 3, *patch.robot_port);
        if (patch.robot_ip) {
            sqlite3_bind_text(g.stmt, 4, patch.robot_ip->c_str(), -1,
                              SQLITE_TRANSIENT);
        }
        if (sqlite3_step(g.stmt) != SQLITE_DONE) {
            throw_sqlite(h, "net_config update: step");
        }

        StmtGuard sel;
        if (sqlite3_prepare_v2(h, kSelect, -1, &sel.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "net_config update: reselect");
        }
        if (sqlite3_step(sel.stmt) != SQLITE_ROW) {
            throw std::runtime_error("net_config row missing after update");
        }
        return read_row(sel.stmt);
    });
}

}  // namespace gw::server
