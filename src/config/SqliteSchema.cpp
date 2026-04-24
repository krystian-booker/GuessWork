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

const char* migration3Sql() {
    return R"sql(
BEGIN;

ALTER TABLE calibrations RENAME TO calibrations_v2;

CREATE TABLE calibrations (
    camera_id TEXT NOT NULL,
    version TEXT NOT NULL,
    active INTEGER NOT NULL DEFAULT 0,
    source_file_path TEXT NOT NULL,
    created_at TEXT NOT NULL,
    image_width INTEGER NOT NULL,
    image_height INTEGER NOT NULL,
    camera_model TEXT NOT NULL,
    distortion_model TEXT NOT NULL,
    fx REAL NOT NULL,
    fy REAL NOT NULL,
    cx REAL NOT NULL,
    cy REAL NOT NULL,
    distortion_coefficients_json TEXT NOT NULL DEFAULT '[]',
    PRIMARY KEY (camera_id, version),
    FOREIGN KEY (camera_id) REFERENCES cameras(id) ON DELETE CASCADE
);

INSERT INTO calibrations (
    camera_id, version, active, source_file_path, created_at, image_width, image_height,
    camera_model, distortion_model, fx, fy, cx, cy, distortion_coefficients_json
)
SELECT
    camera_id, version, 0, file_path, created_at, 1, 1,
    'legacy-unparsed', 'legacy-unparsed', 1.0, 1.0, 0.0, 0.0, '[]'
FROM calibrations_v2;

DROP TABLE calibrations_v2;

CREATE UNIQUE INDEX calibrations_one_active_per_camera
    ON calibrations(camera_id)
    WHERE active = 1;

CREATE TABLE IF NOT EXISTS camera_extrinsics (
    camera_id TEXT NOT NULL,
    version TEXT NOT NULL,
    tx_m REAL NOT NULL,
    ty_m REAL NOT NULL,
    tz_m REAL NOT NULL,
    roll_rad REAL NOT NULL,
    pitch_rad REAL NOT NULL,
    yaw_rad REAL NOT NULL,
    PRIMARY KEY (camera_id, version),
    FOREIGN KEY (camera_id, version) REFERENCES calibrations(camera_id, version) ON DELETE CASCADE
);

CREATE TABLE IF NOT EXISTS field_layouts (
    id TEXT PRIMARY KEY NOT NULL,
    name TEXT NOT NULL,
    source_file_path TEXT NOT NULL,
    field_length_m REAL NOT NULL,
    field_width_m REAL NOT NULL,
    active INTEGER NOT NULL DEFAULT 0
);

CREATE UNIQUE INDEX field_layouts_one_active
    ON field_layouts(active)
    WHERE active = 1;

CREATE TABLE IF NOT EXISTS field_tags (
    layout_id TEXT NOT NULL,
    tag_id INTEGER NOT NULL,
    tx_m REAL NOT NULL,
    ty_m REAL NOT NULL,
    tz_m REAL NOT NULL,
    roll_rad REAL NOT NULL,
    pitch_rad REAL NOT NULL,
    yaw_rad REAL NOT NULL,
    PRIMARY KEY (layout_id, tag_id),
    FOREIGN KEY (layout_id) REFERENCES field_layouts(id) ON DELETE CASCADE
);

PRAGMA user_version = 3;
COMMIT;
)sql";
}

}  // namespace

int currentSchemaVersion() {
    return 3;
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
        migrated_version = 2;
    }
    if (migrated_version == 2) {
        exec(db, migration3Sql());
    }
}

}  // namespace posest::config
