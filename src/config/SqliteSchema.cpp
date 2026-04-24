#include "posest/config/SqliteSchema.h"

namespace posest::config {

const char* sqliteSchemaSql() {
    return R"sql(
CREATE TABLE IF NOT EXISTS cameras (
    id TEXT PRIMARY KEY NOT NULL,
    backend_type TEXT NOT NULL,
    device TEXT NOT NULL,
    width INTEGER NOT NULL,
    height INTEGER NOT NULL,
    fps REAL NOT NULL,
    pixel_format TEXT NOT NULL,
    enabled INTEGER NOT NULL DEFAULT 1
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
)sql";
}

}  // namespace posest::config
