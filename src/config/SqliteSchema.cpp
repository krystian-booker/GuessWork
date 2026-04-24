#include "posest/config/SqliteSchema.h"

#include <stdexcept>
#include <string>

#include <sqlite3.h>

namespace posest::config {

namespace {

void exec(sqlite3* db, const char* sql) {
    char* err = nullptr;
    if (sqlite3_exec(db, sql, nullptr, nullptr, &err) != SQLITE_OK) {
        std::string msg = err ? err : "unknown SQLite error";
        sqlite3_free(err);
        throw std::runtime_error("SQLite schema migration failed: " + msg);
    }
}

int userVersion(sqlite3* db) {
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db, "PRAGMA user_version", -1, &stmt, nullptr) != SQLITE_OK) {
        throw std::runtime_error("Failed to read SQLite user_version");
    }
    const int rc = sqlite3_step(stmt);
    if (rc != SQLITE_ROW) {
        sqlite3_finalize(stmt);
        throw std::runtime_error("SQLite user_version returned no row");
    }
    const int version = sqlite3_column_int(stmt, 0);
    sqlite3_finalize(stmt);
    return version;
}

const char* migration1Sql() {
    return R"sql(
BEGIN;

CREATE TABLE IF NOT EXISTS cameras (
    id TEXT PRIMARY KEY NOT NULL,
    backend_type TEXT NOT NULL,
    device TEXT NOT NULL,
    enabled INTEGER NOT NULL DEFAULT 1,
    width INTEGER NOT NULL,
    height INTEGER NOT NULL,
    fps REAL NOT NULL,
    pixel_format TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS camera_controls (
    camera_id TEXT NOT NULL,
    name TEXT NOT NULL,
    value INTEGER NOT NULL,
    PRIMARY KEY (camera_id, name),
    FOREIGN KEY (camera_id) REFERENCES cameras(id) ON DELETE CASCADE
);

CREATE TABLE IF NOT EXISTS pipelines (
    id TEXT PRIMARY KEY NOT NULL,
    type TEXT NOT NULL,
    enabled INTEGER NOT NULL DEFAULT 1,
    parameters_json TEXT NOT NULL DEFAULT '{}'
);

CREATE TABLE IF NOT EXISTS camera_pipeline_bindings (
    camera_id TEXT NOT NULL,
    pipeline_id TEXT NOT NULL,
    PRIMARY KEY (camera_id, pipeline_id),
    FOREIGN KEY (camera_id) REFERENCES cameras(id) ON DELETE CASCADE,
    FOREIGN KEY (pipeline_id) REFERENCES pipelines(id) ON DELETE CASCADE
);

CREATE TABLE IF NOT EXISTS calibrations (
    camera_id TEXT NOT NULL,
    file_path TEXT NOT NULL,
    version TEXT NOT NULL,
    created_at TEXT NOT NULL,
    PRIMARY KEY (camera_id, version),
    FOREIGN KEY (camera_id) REFERENCES cameras(id) ON DELETE CASCADE
);

CREATE TABLE IF NOT EXISTS teensy_config (
    id INTEGER PRIMARY KEY CHECK (id = 1),
    serial_port TEXT NOT NULL,
    fused_pose_can_id INTEGER NOT NULL,
    status_can_id INTEGER NOT NULL,
    pose_publish_hz REAL NOT NULL
);

PRAGMA user_version = 1;
COMMIT;
)sql";
}

const char* migration2Sql() {
    return R"sql(
BEGIN;

ALTER TABLE teensy_config
    ADD COLUMN baud_rate INTEGER NOT NULL DEFAULT 115200;

ALTER TABLE teensy_config
    ADD COLUMN reconnect_interval_ms INTEGER NOT NULL DEFAULT 1000;

ALTER TABLE teensy_config
    ADD COLUMN read_timeout_ms INTEGER NOT NULL DEFAULT 20;

CREATE TABLE IF NOT EXISTS camera_triggers (
    camera_id TEXT PRIMARY KEY NOT NULL,
    enabled INTEGER NOT NULL DEFAULT 1,
    teensy_pin INTEGER NOT NULL,
    rate_hz REAL NOT NULL,
    pulse_width_us INTEGER NOT NULL DEFAULT 1000,
    phase_offset_us INTEGER NOT NULL DEFAULT 0,
    FOREIGN KEY (camera_id) REFERENCES cameras(id) ON DELETE CASCADE
);

PRAGMA user_version = 2;
COMMIT;
)sql";
}

}  // namespace

int currentSchemaVersion() {
    return 2;
}

void applyMigrations(sqlite3* db) {
    if (!db) {
        throw std::invalid_argument("applyMigrations called with null sqlite3 handle");
    }

    const int version = userVersion(db);
    if (version > currentSchemaVersion()) {
        throw std::runtime_error("SQLite config database schema is newer than this binary");
    }
    int migrated_version = version;
    if (migrated_version == 0) {
        exec(db, migration1Sql());
        migrated_version = 1;
    }
    if (migrated_version == 1) {
        exec(db, migration2Sql());
    }
}

}  // namespace posest::config
