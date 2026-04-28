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

const char* migration5Sql() {
    return R"sql(
BEGIN;

ALTER TABLE cameras
    ADD COLUMN trigger_mode TEXT NOT NULL DEFAULT 'free_run';

ALTER TABLE cameras
    ADD COLUMN reconnect_interval_ms INTEGER NOT NULL DEFAULT 1000;

ALTER TABLE cameras
    ADD COLUMN reconnect_max_attempts INTEGER NOT NULL DEFAULT 0;

PRAGMA user_version = 5;
COMMIT;
)sql";
}

const char* migration4Sql() {
    return R"sql(
BEGIN;

CREATE TABLE IF NOT EXISTS calibration_tool_config (
    id INTEGER PRIMARY KEY CHECK (id = 1),
    docker_image TEXT NOT NULL DEFAULT 'kalibr:latest'
);

INSERT OR IGNORE INTO calibration_tool_config (id, docker_image)
VALUES (1, 'kalibr:latest');

CREATE TABLE IF NOT EXISTS kalibr_datasets (
    id TEXT PRIMARY KEY NOT NULL,
    path TEXT NOT NULL,
    created_at TEXT NOT NULL,
    duration_s REAL NOT NULL,
    camera_ids_json TEXT NOT NULL DEFAULT '[]'
);

CREATE TABLE IF NOT EXISTS camera_imu_calibrations (
    camera_id TEXT NOT NULL,
    version TEXT NOT NULL,
    active INTEGER NOT NULL DEFAULT 0,
    source_file_path TEXT NOT NULL,
    created_at TEXT NOT NULL,
    cam_imu_tx_m REAL NOT NULL,
    cam_imu_ty_m REAL NOT NULL,
    cam_imu_tz_m REAL NOT NULL,
    cam_imu_roll_rad REAL NOT NULL,
    cam_imu_pitch_rad REAL NOT NULL,
    cam_imu_yaw_rad REAL NOT NULL,
    imu_cam_tx_m REAL NOT NULL,
    imu_cam_ty_m REAL NOT NULL,
    imu_cam_tz_m REAL NOT NULL,
    imu_cam_roll_rad REAL NOT NULL,
    imu_cam_pitch_rad REAL NOT NULL,
    imu_cam_yaw_rad REAL NOT NULL,
    time_shift_s REAL NOT NULL DEFAULT 0.0,
    PRIMARY KEY (camera_id, version),
    FOREIGN KEY (camera_id) REFERENCES cameras(id) ON DELETE CASCADE
);

CREATE UNIQUE INDEX camera_imu_one_active_per_camera
    ON camera_imu_calibrations(camera_id)
    WHERE active = 1;

PRAGMA user_version = 4;
COMMIT;
)sql";
}

const char* migration6Sql() {
    return R"sql(
BEGIN;

CREATE TABLE IF NOT EXISTS imu_config (
    id INTEGER PRIMARY KEY CHECK (id = 1),
    accel_range_g INTEGER NOT NULL DEFAULT 24,
    accel_odr_hz INTEGER NOT NULL DEFAULT 1000,
    accel_bandwidth_code INTEGER NOT NULL DEFAULT 2,
    gyro_range_dps INTEGER NOT NULL DEFAULT 2000,
    gyro_bandwidth_code INTEGER NOT NULL DEFAULT 2,
    data_sync_rate_hz INTEGER NOT NULL DEFAULT 1000,
    run_selftest_on_boot INTEGER NOT NULL DEFAULT 1
);

CREATE TABLE IF NOT EXISTS can_config (
    id INTEGER PRIMARY KEY CHECK (id = 1),
    enabled INTEGER NOT NULL DEFAULT 0,
    nominal_bitrate_bps INTEGER NOT NULL DEFAULT 1000000,
    data_bitrate_bps INTEGER NOT NULL DEFAULT 2000000,
    pose_publish_hz INTEGER NOT NULL DEFAULT 100,
    rio_offset_stale_ms INTEGER NOT NULL DEFAULT 1000,
    rio_pose_can_id INTEGER NOT NULL DEFAULT 256,
    rio_time_sync_can_id INTEGER NOT NULL DEFAULT 257,
    teensy_pose_can_id INTEGER NOT NULL DEFAULT 384
);

ALTER TABLE teensy_config
    ADD COLUMN time_sync_interval_ms INTEGER NOT NULL DEFAULT 1000;

PRAGMA user_version = 6;
COMMIT;
)sql";
}

const char* migration7Sql() {
    return R"sql(
BEGIN;

UPDATE teensy_config SET baud_rate = 921600 WHERE baud_rate = 115200;

PRAGMA user_version = 7;
COMMIT;
)sql";
}

const char* migration8Sql() {
    // Single-row VIO workflow config table. ON DELETE SET NULL on the camera
    // FK so deleting the bound camera doesn't cascade-erase the row; the
    // validator catches the resulting null vio_camera_id when enabled=1.
    return R"sql(
BEGIN;

CREATE TABLE IF NOT EXISTS vio_config (
    id INTEGER PRIMARY KEY CHECK (id = 1),
    enabled INTEGER NOT NULL DEFAULT 0,
    vio_camera_id TEXT REFERENCES cameras(id) ON DELETE SET NULL,
    vio_slot_index INTEGER NOT NULL DEFAULT 0,
    ir_led_enabled INTEGER NOT NULL DEFAULT 1,
    ir_led_pulse_width_us INTEGER NOT NULL DEFAULT 400,
    tof_enabled INTEGER NOT NULL DEFAULT 1,
    tof_i2c_address INTEGER NOT NULL DEFAULT 41,
    tof_timing_budget_ms INTEGER NOT NULL DEFAULT 10,
    tof_intermeasurement_period_ms INTEGER NOT NULL DEFAULT 20,
    tof_offset_after_flash_us INTEGER NOT NULL DEFAULT 500,
    tof_divisor INTEGER NOT NULL DEFAULT 1,
    tof_mounting_offset_mm INTEGER NOT NULL DEFAULT 0,
    tof_expected_min_mm INTEGER NOT NULL DEFAULT 50,
    tof_expected_max_mm INTEGER NOT NULL DEFAULT 4000
);

INSERT OR IGNORE INTO vio_config (id) VALUES (1);

PRAGMA user_version = 8;
COMMIT;
)sql";
}

const char* migration9Sql() {
    // Single-row GTSAM fusion-graph tunables. Six-element sigma arrays use
    // gtsam::Pose3 tangent ordering (rx, ry, rz, tx, ty, tz). All Phase A/B/C
    // fields are persisted up front so feature flags toggle without further
    // migrations. See docs/features/fusion-service.md §3 and §6.
    return R"sql(
BEGIN;

CREATE TABLE IF NOT EXISTS fusion_config (
    id INTEGER PRIMARY KEY CHECK (id = 1),
    chassis_sigma_rx REAL NOT NULL DEFAULT 0.05,
    chassis_sigma_ry REAL NOT NULL DEFAULT 0.05,
    chassis_sigma_rz REAL NOT NULL DEFAULT 0.05,
    chassis_sigma_tx REAL NOT NULL DEFAULT 0.02,
    chassis_sigma_ty REAL NOT NULL DEFAULT 0.02,
    chassis_sigma_tz REAL NOT NULL DEFAULT 0.02,
    origin_prior_sigma_rx REAL NOT NULL DEFAULT 10.0,
    origin_prior_sigma_ry REAL NOT NULL DEFAULT 10.0,
    origin_prior_sigma_rz REAL NOT NULL DEFAULT 3.14,
    origin_prior_sigma_tx REAL NOT NULL DEFAULT 10.0,
    origin_prior_sigma_ty REAL NOT NULL DEFAULT 10.0,
    origin_prior_sigma_tz REAL NOT NULL DEFAULT 10.0,
    shock_threshold_mps2 REAL NOT NULL DEFAULT 50.0,
    freefall_threshold_mps2 REAL NOT NULL DEFAULT 3.0,
    shock_inflation_factor REAL NOT NULL DEFAULT 100.0,
    imu_window_seconds REAL NOT NULL DEFAULT 0.05,
    max_chassis_dt_seconds REAL NOT NULL DEFAULT 0.5,
    gravity_local_x REAL NOT NULL DEFAULT 0.0,
    gravity_local_y REAL NOT NULL DEFAULT 0.0,
    gravity_local_z REAL NOT NULL DEFAULT 9.80665,
    enable_vio INTEGER NOT NULL DEFAULT 0,
    vio_sigma_rx REAL NOT NULL DEFAULT 0.05,
    vio_sigma_ry REAL NOT NULL DEFAULT 0.05,
    vio_sigma_rz REAL NOT NULL DEFAULT 0.05,
    vio_sigma_tx REAL NOT NULL DEFAULT 0.02,
    vio_sigma_ty REAL NOT NULL DEFAULT 0.02,
    vio_sigma_tz REAL NOT NULL DEFAULT 0.02,
    huber_k REAL NOT NULL DEFAULT 1.5,
    enable_imu_preintegration INTEGER NOT NULL DEFAULT 0,
    imu_extrinsic_tx_m REAL NOT NULL DEFAULT 0.0,
    imu_extrinsic_ty_m REAL NOT NULL DEFAULT 0.0,
    imu_extrinsic_tz_m REAL NOT NULL DEFAULT 0.0,
    imu_extrinsic_roll_rad REAL NOT NULL DEFAULT 0.0,
    imu_extrinsic_pitch_rad REAL NOT NULL DEFAULT 0.0,
    imu_extrinsic_yaw_rad REAL NOT NULL DEFAULT 0.0,
    accel_noise_sigma REAL NOT NULL DEFAULT 0.05,
    gyro_noise_sigma REAL NOT NULL DEFAULT 0.001,
    accel_bias_rw_sigma REAL NOT NULL DEFAULT 0.0001,
    gyro_bias_rw_sigma REAL NOT NULL DEFAULT 0.00001,
    integration_cov_sigma REAL NOT NULL DEFAULT 0.00000001,
    persisted_bias_ax REAL NOT NULL DEFAULT 0.0,
    persisted_bias_ay REAL NOT NULL DEFAULT 0.0,
    persisted_bias_az REAL NOT NULL DEFAULT 0.0,
    persisted_bias_gx REAL NOT NULL DEFAULT 0.0,
    persisted_bias_gy REAL NOT NULL DEFAULT 0.0,
    persisted_bias_gz REAL NOT NULL DEFAULT 0.0,
    bias_calibration_seconds REAL NOT NULL DEFAULT 1.5,
    bias_calibration_chassis_threshold REAL NOT NULL DEFAULT 0.02,
    max_keyframe_dt_seconds REAL NOT NULL DEFAULT 0.020,
    max_imu_gap_seconds REAL NOT NULL DEFAULT 0.100,
    marginalize_keyframe_window INTEGER NOT NULL DEFAULT 500,
    slip_disagreement_mps REAL NOT NULL DEFAULT 1.0
);

INSERT OR IGNORE INTO fusion_config (id) VALUES (1);

PRAGMA user_version = 9;
COMMIT;
)sql";
}

const char* migration10Sql() {
    // F-2 floor-constraint prior on every new pose key, plus F-3 max chassis
    // speed gate. Both default-on so an unmigrated DB picks up the platform
    // invariants immediately. Sigmas are [σ_z, σ_roll, σ_pitch] in meters /
    // radians; max_chassis_speed_mps is 5.2 m/s × 1.25 margin.
    return R"sql(
BEGIN;

ALTER TABLE fusion_config
    ADD COLUMN enable_floor_constraint INTEGER NOT NULL DEFAULT 1;

ALTER TABLE fusion_config
    ADD COLUMN floor_constraint_sigma_z REAL NOT NULL DEFAULT 0.01;

ALTER TABLE fusion_config
    ADD COLUMN floor_constraint_sigma_roll REAL NOT NULL DEFAULT 0.0087;

ALTER TABLE fusion_config
    ADD COLUMN floor_constraint_sigma_pitch REAL NOT NULL DEFAULT 0.0087;

ALTER TABLE fusion_config
    ADD COLUMN max_chassis_speed_mps REAL NOT NULL DEFAULT 6.5;

PRAGMA user_version = 10;
COMMIT;
)sql";
}

const char* migration11Sql() {
    // Calibration target catalog. Operators pick a row by id when starting a
    // calibration run; the daemon materializes a Kalibr-shaped target.yaml on
    // demand. The default AprilGrid 6x6 row matches Kalibr's bundled board so
    // a fresh install can run intrinsic calibration without manual setup.
    return R"sql(
BEGIN;

CREATE TABLE IF NOT EXISTS calibration_targets (
    id TEXT PRIMARY KEY NOT NULL,
    type TEXT NOT NULL,
    rows INTEGER NOT NULL,
    cols INTEGER NOT NULL,
    tag_size_m REAL NOT NULL DEFAULT 0.0,
    tag_spacing_ratio REAL NOT NULL DEFAULT 0.0,
    square_size_m REAL NOT NULL DEFAULT 0.0,
    tag_family TEXT NOT NULL DEFAULT 'tag36h11',
    notes TEXT NOT NULL DEFAULT ''
);

INSERT OR IGNORE INTO calibration_targets
    (id, type, rows, cols, tag_size_m, tag_spacing_ratio, tag_family)
    VALUES ('default_aprilgrid_6x6', 'aprilgrid', 6, 6, 0.088, 0.3, 'tag36h11');

PRAGMA user_version = 11;
COMMIT;
)sql";
}

const char* migration12Sql() {
    // Quality metrics from Kalibr's results files plus the gate thresholds
    // that govern persistence. Defaults of 0.0 are intentional — an
    // unmigrated row predates the gate and must not be confused with
    // "0 px reprojection error".
    return R"sql(
BEGIN;

ALTER TABLE calibrations
    ADD COLUMN reprojection_rms_px REAL NOT NULL DEFAULT 0.0;
ALTER TABLE calibrations
    ADD COLUMN observation_count INTEGER NOT NULL DEFAULT 0;
ALTER TABLE calibrations
    ADD COLUMN coverage_score REAL NOT NULL DEFAULT 0.0;
ALTER TABLE calibrations
    ADD COLUMN report_path TEXT NOT NULL DEFAULT '';

ALTER TABLE camera_imu_calibrations
    ADD COLUMN reprojection_rms_px REAL NOT NULL DEFAULT 0.0;
ALTER TABLE camera_imu_calibrations
    ADD COLUMN gyro_rms_radps REAL NOT NULL DEFAULT 0.0;
ALTER TABLE camera_imu_calibrations
    ADD COLUMN accel_rms_mps2 REAL NOT NULL DEFAULT 0.0;
ALTER TABLE camera_imu_calibrations
    ADD COLUMN report_path TEXT NOT NULL DEFAULT '';

ALTER TABLE calibration_tool_config
    ADD COLUMN max_reprojection_rms_px REAL NOT NULL DEFAULT 1.0;
ALTER TABLE calibration_tool_config
    ADD COLUMN max_camera_imu_rms_px REAL NOT NULL DEFAULT 1.5;

PRAGMA user_version = 12;
COMMIT;
)sql";
}

}  // namespace

int currentSchemaVersion() {
    return 12;
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
        migrated_version = 3;
    }
    if (migrated_version == 3) {
        exec(db, migration4Sql());
        migrated_version = 4;
    }
    if (migrated_version == 4) {
        exec(db, migration5Sql());
        migrated_version = 5;
    }
    if (migrated_version == 5) {
        exec(db, migration6Sql());
        migrated_version = 6;
    }
    if (migrated_version == 6) {
        exec(db, migration7Sql());
        migrated_version = 7;
    }
    if (migrated_version == 7) {
        exec(db, migration8Sql());
        migrated_version = 8;
    }
    if (migrated_version == 8) {
        exec(db, migration9Sql());
        migrated_version = 9;
    }
    if (migrated_version == 9) {
        exec(db, migration10Sql());
        migrated_version = 10;
    }
    if (migrated_version == 10) {
        exec(db, migration11Sql());
        migrated_version = 11;
    }
    if (migrated_version == 11) {
        exec(db, migration12Sql());
    }
}

}  // namespace posest::config
