#include "server/camera_repository.hpp"

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

Camera read_row(sqlite3_stmt* stmt) {
    Camera c;
    c.id         = sqlite3_column_int64(stmt, 0);
    const auto* name_text = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
    c.name       = name_text ? name_text : "";
    c.created_at = sqlite3_column_int64(stmt, 2);
    return c;
}

bool is_unique_violation(int rc) {
    return rc == SQLITE_CONSTRAINT_UNIQUE || rc == SQLITE_CONSTRAINT;
}

}  // namespace

std::vector<Camera> CameraRepository::list_all() {
    return db_.with_handle([](sqlite3* h) {
        StmtGuard g;
        if (sqlite3_prepare_v2(h, "SELECT id, name, created_at FROM cameras ORDER BY id;", -1,
                               &g.stmt, nullptr) != SQLITE_OK) {
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
        if (sqlite3_prepare_v2(h, "SELECT id, name, created_at FROM cameras WHERE id = ?;", -1,
                               &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "get: prepare");
        }
        sqlite3_bind_int64(g.stmt, 1, id);

        const int rc = sqlite3_step(g.stmt);
        if (rc == SQLITE_ROW) return read_row(g.stmt);
        if (rc == SQLITE_DONE) return std::nullopt;
        throw_sqlite(h, "get: step");
    });
}

Camera CameraRepository::create(std::string_view name) {
    const std::string name_str(name);
    return db_.with_handle([&](sqlite3* h) {
        StmtGuard g;
        if (sqlite3_prepare_v2(
                h,
                "INSERT INTO cameras (name) VALUES (?) RETURNING id, name, created_at;",
                -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "create: prepare");
        }
        sqlite3_bind_text(g.stmt, 1, name_str.c_str(), -1, SQLITE_TRANSIENT);

        const int rc = sqlite3_step(g.stmt);
        if (rc == SQLITE_ROW) return read_row(g.stmt);
        if (is_unique_violation(rc)) throw DuplicateNameError(name_str);
        throw_sqlite(h, "create: step");
    });
}

std::optional<Camera> CameraRepository::update(int64_t id, std::string_view name) {
    const std::string name_str(name);
    return db_.with_handle([&](sqlite3* h) -> std::optional<Camera> {
        StmtGuard g;
        if (sqlite3_prepare_v2(
                h,
                "UPDATE cameras SET name = ? WHERE id = ? RETURNING id, name, created_at;",
                -1, &g.stmt, nullptr) != SQLITE_OK) {
            throw_sqlite(h, "update: prepare");
        }
        sqlite3_bind_text(g.stmt, 1, name_str.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_int64(g.stmt, 2, id);

        const int rc = sqlite3_step(g.stmt);
        if (rc == SQLITE_ROW) return read_row(g.stmt);
        if (rc == SQLITE_DONE) return std::nullopt;
        if (is_unique_violation(rc)) throw DuplicateNameError(name_str);
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
