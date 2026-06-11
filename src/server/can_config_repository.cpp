#include "server/can_config_repository.hpp"

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
    "SELECT mode, updated_at FROM can_config WHERE id = 1;";

CanConfig read_row(sqlite3_stmt* stmt) {
    CanConfig c;
    c.mode = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
    c.updated_at = sqlite3_column_int64(stmt, 1);
    return c;
}

}  // namespace

CanConfig CanConfigRepository::get() {
    return db_.with_handle([&](sqlite3* h) {
        StmtGuard g;
        if (sqlite3_prepare_v2(h, kSelect, -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "can_config get: prepare");
        }
        if (sqlite3_step(g.stmt) != SQLITE_ROW) {
            throw std::runtime_error("can_config row missing (schema seed failed?)");
        }
        return read_row(g.stmt);
    });
}

CanConfig CanConfigRepository::update(const CanConfigUpdate& patch) {
    if (patch.empty()) return get();

    return db_.with_handle([&](sqlite3* h) {
        std::string sql = "UPDATE can_config SET updated_at = strftime('%s','now')";
        if (patch.mode) sql += ", mode = ?1";
        sql += " WHERE id = 1;";

        StmtGuard g;
        if (sqlite3_prepare_v2(h, sql.c_str(), -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "can_config update: prepare");
        }
        if (patch.mode) {
            sqlite3_bind_text(g.stmt, 1, patch.mode->c_str(), -1, SQLITE_TRANSIENT);
        }
        if (sqlite3_step(g.stmt) != SQLITE_DONE) {
            throw_sqlite(h, "can_config update: step");
        }

        StmtGuard sel;
        if (sqlite3_prepare_v2(h, kSelect, -1, &sel.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "can_config update: reselect");
        }
        if (sqlite3_step(sel.stmt) != SQLITE_ROW) {
            throw std::runtime_error("can_config row missing after update");
        }
        return read_row(sel.stmt);
    });
}

}  // namespace gw::server
