#include "server/camera_repository.hpp"

#include <sqlite3.h>

#include <stdexcept>
#include <string>
#include <string_view>

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

Camera read_row(sqlite3_stmt* stmt) {
    Camera c;
    c.id         = sqlite3_column_int64(stmt, 0);
    const auto* name_text = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
    c.name       = name_text ? name_text : "";
    const auto* serial_text = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
    c.serial     = serial_text ? serial_text : "";
    if (sqlite3_column_type(stmt, 3) == SQLITE_NULL) {
        c.mode = std::nullopt;
    } else {
        const auto* mode_text = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3));
        c.mode = mode_text ? std::string(mode_text) : std::string();
    }
    c.created_at = sqlite3_column_int64(stmt, 4);
    return c;
}

bool is_unique_violation(int rc) {
    return rc == SQLITE_CONSTRAINT_UNIQUE || rc == SQLITE_CONSTRAINT;
}

// SQLite's error message for a UNIQUE violation names the offending column,
// e.g. "UNIQUE constraint failed: cameras.name". We use that to map back to
// either DuplicateNameError or DuplicateSerialError.
bool err_mentions(sqlite3* h, std::string_view needle) {
    const char* msg = sqlite3_errmsg(h);
    return msg && std::string_view(msg).find(needle) != std::string_view::npos;
}

}  // namespace

std::vector<Camera> CameraRepository::list_all() {
    return db_.with_handle([](sqlite3* h) {
        StmtGuard g;
        if (sqlite3_prepare_v2(h,
                               "SELECT id, name, serial, mode, created_at FROM cameras ORDER BY id;",
                               -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "list_all: prepare");
        }

        std::vector<Camera> out;
        for (;;) {
            const int rc = sqlite3_step(g.stmt);
            if (rc == SQLITE_DONE) break;
            if (rc != SQLITE_ROW) throw_sqlite(h, "list_all: step");
            out.push_back(read_row(g.stmt));
        }
        return out;
    });
}

std::optional<Camera> CameraRepository::get(int64_t id) {
    return db_.with_handle([id](sqlite3* h) -> std::optional<Camera> {
        StmtGuard g;
        if (sqlite3_prepare_v2(h,
                               "SELECT id, name, serial, mode, created_at FROM cameras WHERE id = ?;",
                               -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "get: prepare");
        }
        sqlite3_bind_int64(g.stmt, 1, id);

        const int rc = sqlite3_step(g.stmt);
        if (rc == SQLITE_ROW) return read_row(g.stmt);
        if (rc == SQLITE_DONE) return std::nullopt;
        throw_sqlite(h, "get: step");
    });
}

std::optional<Camera> CameraRepository::find_by_serial(std::string_view serial) {
    const std::string serial_str(serial);
    return db_.with_handle([&](sqlite3* h) -> std::optional<Camera> {
        StmtGuard g;
        if (sqlite3_prepare_v2(
                h,
                "SELECT id, name, serial, mode, created_at FROM cameras WHERE serial = ?;",
                -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "find_by_serial: prepare");
        }
        sqlite3_bind_text(g.stmt, 1, serial_str.c_str(), -1, SQLITE_TRANSIENT);

        const int rc = sqlite3_step(g.stmt);
        if (rc == SQLITE_ROW) return read_row(g.stmt);
        if (rc == SQLITE_DONE) return std::nullopt;
        throw_sqlite(h, "find_by_serial: step");
    });
}

Camera CameraRepository::create(std::string_view                name,
                                std::string_view                serial,
                                std::optional<std::string_view> mode) {
    const std::string name_str(name);
    const std::string serial_str(serial);
    const std::optional<std::string> mode_str =
        mode ? std::optional<std::string>(std::string(*mode)) : std::nullopt;
    return db_.with_handle([&](sqlite3* h) {
        StmtGuard g;
        if (sqlite3_prepare_v2(
                h,
                "INSERT INTO cameras (name, serial, mode) VALUES (?, ?, ?) "
                "RETURNING id, name, serial, mode, created_at;",
                -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "create: prepare");
        }
        sqlite3_bind_text(g.stmt, 1, name_str.c_str(),   -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(g.stmt, 2, serial_str.c_str(), -1, SQLITE_TRANSIENT);
        if (mode_str) {
            sqlite3_bind_text(g.stmt, 3, mode_str->c_str(), -1, SQLITE_TRANSIENT);
        } else {
            sqlite3_bind_null(g.stmt, 3);
        }

        const int rc = sqlite3_step(g.stmt);
        if (rc == SQLITE_ROW) return read_row(g.stmt);
        if (is_unique_violation(rc)) {
            if (err_mentions(h, "serial")) throw DuplicateSerialError(serial_str);
            throw DuplicateNameError(name_str);
        }
        throw_sqlite(h, "create: step");
    });
}

std::optional<Camera> CameraRepository::update(int64_t                         id,
                                               std::optional<std::string_view> name,
                                               std::optional<std::string_view> mode) {
    // Both nullopt → no-op; return the current row (or nullopt if id missing).
    if (!name && !mode) return get(id);

    const std::optional<std::string> name_str =
        name ? std::optional<std::string>(std::string(*name)) : std::nullopt;
    const std::optional<std::string> mode_str =
        mode ? std::optional<std::string>(std::string(*mode)) : std::nullopt;

    return db_.with_handle([&](sqlite3* h) -> std::optional<Camera> {
        StmtGuard g;
        // Build SET clause based on which fields are present. Mode allows
        // explicit empty string == NULL (caller decides) — we treat any value
        // with has_value() as an explicit set.
        std::string sql = "UPDATE cameras SET ";
        bool first = true;
        if (name_str) { sql += "name = ?"; first = false; }
        if (mode_str) { if (!first) sql += ", "; sql += "mode = ?"; }
        sql += " WHERE id = ? RETURNING id, name, serial, mode, created_at;";

        if (sqlite3_prepare_v2(h, sql.c_str(), -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "update: prepare");
        }
        int idx = 1;
        if (name_str) {
            sqlite3_bind_text(g.stmt, idx++, name_str->c_str(), -1, SQLITE_TRANSIENT);
        }
        if (mode_str) {
            // Convention: empty string means "clear the mode" (NULL); any other
            // string is an explicit set.
            if (mode_str->empty()) sqlite3_bind_null(g.stmt, idx++);
            else sqlite3_bind_text(g.stmt, idx++, mode_str->c_str(), -1, SQLITE_TRANSIENT);
        }
        sqlite3_bind_int64(g.stmt, idx, id);

        const int rc = sqlite3_step(g.stmt);
        if (rc == SQLITE_ROW) return read_row(g.stmt);
        if (rc == SQLITE_DONE) return std::nullopt;
        if (is_unique_violation(rc) && name_str) throw DuplicateNameError(*name_str);
        throw_sqlite(h, "update: step");
    });
}

bool CameraRepository::remove(int64_t id) {
    return db_.with_handle([id](sqlite3* h) {
        StmtGuard g;
        if (sqlite3_prepare_v2(h, "DELETE FROM cameras WHERE id = ?;", -1, &g.stmt, nullptr) !=
            SQLITE_OK) {
            throw_sqlite(h, "remove: prepare");
        }
        sqlite3_bind_int64(g.stmt, 1, id);

        const int rc = sqlite3_step(g.stmt);
        if (rc != SQLITE_DONE) throw_sqlite(h, "remove: step");
        return sqlite3_changes(h) > 0;
    });
}

}  // namespace gw::server
