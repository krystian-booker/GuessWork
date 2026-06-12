#include "server/field_layout_repository.hpp"

#include <sqlite3.h>

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

constexpr const char* kSelectColumns = "id, name, json, active, created_at";

FieldLayoutRow read_row(sqlite3_stmt* stmt) {
    FieldLayoutRow r;
    r.id = sqlite3_column_int64(stmt, 0);
    const auto* name = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
    r.name = name ? name : "";
    const auto* json = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
    r.json       = json ? json : "";
    r.active     = sqlite3_column_int(stmt, 3) != 0;
    r.created_at = sqlite3_column_int64(stmt, 4);
    return r;
}

void exec_or_throw(sqlite3* h, const char* sql) {
    char* err = nullptr;
    if (sqlite3_exec(h, sql, nullptr, nullptr, &err) != SQLITE_OK) {
        const std::string msg = err ? err : "unknown sqlite error";
        sqlite3_free(err);
        throw std::runtime_error(std::string("sqlite3_exec failed: ") + msg);
    }
}

}  // namespace

std::vector<FieldLayoutRow> FieldLayoutRepository::list_all() {
    return db_.with_handle([](sqlite3* h) {
        StmtGuard g;
        const std::string sql =
            std::string("SELECT ") + kSelectColumns + " FROM field_layouts ORDER BY id;";
        if (sqlite3_prepare_v2(h, sql.c_str(), -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "field_layouts list: prepare");
        }
        std::vector<FieldLayoutRow> out;
        for (;;) {
            const int rc = sqlite3_step(g.stmt);
            if (rc == SQLITE_DONE) break;
            if (rc != SQLITE_ROW) throw_sqlite(h, "field_layouts list: step");
            out.push_back(read_row(g.stmt));
        }
        return out;
    });
}

std::optional<FieldLayoutRow> FieldLayoutRepository::get(int64_t id) {
    return db_.with_handle([id](sqlite3* h) -> std::optional<FieldLayoutRow> {
        StmtGuard g;
        const std::string sql =
            std::string("SELECT ") + kSelectColumns + " FROM field_layouts WHERE id = ?;";
        if (sqlite3_prepare_v2(h, sql.c_str(), -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "field_layouts get: prepare");
        }
        sqlite3_bind_int64(g.stmt, 1, id);
        const int rc = sqlite3_step(g.stmt);
        if (rc == SQLITE_ROW) return read_row(g.stmt);
        if (rc == SQLITE_DONE) return std::nullopt;
        throw_sqlite(h, "field_layouts get: step");
    });
}

std::optional<FieldLayoutRow> FieldLayoutRepository::get_active() {
    return db_.with_handle([](sqlite3* h) -> std::optional<FieldLayoutRow> {
        StmtGuard g;
        const std::string sql = std::string("SELECT ") + kSelectColumns +
                                " FROM field_layouts WHERE active = 1 LIMIT 1;";
        if (sqlite3_prepare_v2(h, sql.c_str(), -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "field_layouts get_active: prepare");
        }
        const int rc = sqlite3_step(g.stmt);
        if (rc == SQLITE_ROW) return read_row(g.stmt);
        if (rc == SQLITE_DONE) return std::nullopt;
        throw_sqlite(h, "field_layouts get_active: step");
    });
}

FieldLayoutRow FieldLayoutRepository::create(std::string_view name,
                                             std::string_view json) {
    const std::string name_str(name);
    const std::string json_str(json);
    return db_.with_handle([&](sqlite3* h) {
        // First row ever auto-activates so a freshly seeded table is usable
        // with no further operator action.
        StmtGuard count_g;
        if (sqlite3_prepare_v2(h, "SELECT COUNT(*) FROM field_layouts;", -1,
                               &count_g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "field_layouts create: count");
        }
        if (sqlite3_step(count_g.stmt) != SQLITE_ROW) {
            throw_sqlite(h, "field_layouts create: count step");
        }
        const bool first = sqlite3_column_int64(count_g.stmt, 0) == 0;

        StmtGuard g;
        const std::string sql =
            std::string("INSERT INTO field_layouts (name, json, active) "
                        "VALUES (?, ?, ?) RETURNING ") + kSelectColumns + ";";
        if (sqlite3_prepare_v2(h, sql.c_str(), -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "field_layouts create: prepare");
        }
        sqlite3_bind_text(g.stmt, 1, name_str.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(g.stmt, 2, json_str.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_int (g.stmt, 3, first ? 1 : 0);

        const int rc = sqlite3_step(g.stmt);
        if (rc == SQLITE_ROW) return read_row(g.stmt);
        if (rc == SQLITE_CONSTRAINT || rc == SQLITE_CONSTRAINT_UNIQUE) {
            throw DuplicateLayoutNameError(name_str);
        }
        throw_sqlite(h, "field_layouts create: step");
    });
}

std::optional<FieldLayoutRow> FieldLayoutRepository::update_json(
        int64_t id, std::string_view json) {
    const std::string json_str(json);
    return db_.with_handle([&](sqlite3* h) -> std::optional<FieldLayoutRow> {
        StmtGuard g;
        const std::string sql =
            std::string("UPDATE field_layouts SET json = ? WHERE id = ? "
                        "RETURNING ") + kSelectColumns + ";";
        if (sqlite3_prepare_v2(h, sql.c_str(), -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "field_layouts update_json: prepare");
        }
        sqlite3_bind_text (g.stmt, 1, json_str.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_int64(g.stmt, 2, id);
        const int rc = sqlite3_step(g.stmt);
        if (rc == SQLITE_ROW) return read_row(g.stmt);
        if (rc == SQLITE_DONE) return std::nullopt;
        throw_sqlite(h, "field_layouts update_json: step");
    });
}

bool FieldLayoutRepository::activate(int64_t id) {
    return db_.with_handle([id](sqlite3* h) {
        // Single transaction keeps "exactly one active" invariant atomic.
        exec_or_throw(h, "BEGIN IMMEDIATE;");
        try {
            StmtGuard g;
            if (sqlite3_prepare_v2(h,
                    "UPDATE field_layouts SET active = (id = ?);",
                    -1, &g.stmt, nullptr) != SQLITE_OK) {
                throw_sqlite(h, "field_layouts activate: prepare");
            }
            sqlite3_bind_int64(g.stmt, 1, id);
            if (sqlite3_step(g.stmt) != SQLITE_DONE) {
                throw_sqlite(h, "field_layouts activate: step");
            }

            // Did the id exist? (The UPDATE always touches every row.)
            StmtGuard chk;
            if (sqlite3_prepare_v2(h,
                    "SELECT COUNT(*) FROM field_layouts WHERE id = ? AND active = 1;",
                    -1, &chk.stmt, nullptr) != SQLITE_OK) {
                throw_sqlite(h, "field_layouts activate: check");
            }
            sqlite3_bind_int64(chk.stmt, 1, id);
            if (sqlite3_step(chk.stmt) != SQLITE_ROW) {
                throw_sqlite(h, "field_layouts activate: check step");
            }
            const bool found = sqlite3_column_int64(chk.stmt, 0) == 1;
            if (!found) {
                exec_or_throw(h, "ROLLBACK;");
                return false;
            }
            exec_or_throw(h, "COMMIT;");
            return true;
        } catch (...) {
            exec_or_throw(h, "ROLLBACK;");
            throw;
        }
    });
}

bool FieldLayoutRepository::remove(int64_t id) {
    return db_.with_handle([id](sqlite3* h) {
        StmtGuard chk;
        if (sqlite3_prepare_v2(h, "SELECT active FROM field_layouts WHERE id = ?;",
                               -1, &chk.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "field_layouts remove: check");
        }
        sqlite3_bind_int64(chk.stmt, 1, id);
        const int rc = sqlite3_step(chk.stmt);
        if (rc == SQLITE_DONE) return false;
        if (rc != SQLITE_ROW) throw_sqlite(h, "field_layouts remove: check step");
        if (sqlite3_column_int(chk.stmt, 0) != 0) throw ActiveLayoutDeleteError();

        StmtGuard g;
        if (sqlite3_prepare_v2(h, "DELETE FROM field_layouts WHERE id = ?;", -1,
                               &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "field_layouts remove: prepare");
        }
        sqlite3_bind_int64(g.stmt, 1, id);
        if (sqlite3_step(g.stmt) != SQLITE_DONE) {
            throw_sqlite(h, "field_layouts remove: step");
        }
        return sqlite3_changes(h) > 0;
    });
}

bool FieldLayoutRepository::seed_if_empty(std::string_view name,
                                          std::string_view json) {
    const bool empty = db_.with_handle([](sqlite3* h) {
        StmtGuard g;
        if (sqlite3_prepare_v2(h, "SELECT COUNT(*) FROM field_layouts;", -1,
                               &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "field_layouts seed: count");
        }
        if (sqlite3_step(g.stmt) != SQLITE_ROW) {
            throw_sqlite(h, "field_layouts seed: count step");
        }
        return sqlite3_column_int64(g.stmt, 0) == 0;
    });
    if (!empty) return false;
    create(name, json);  // first row auto-activates
    return true;
}

}  // namespace gw::server
