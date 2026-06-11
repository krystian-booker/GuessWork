#include "server/database.hpp"

#include <sqlite3.h>

#include <cstdlib>
#include <stdexcept>
#include <string>

namespace gw::server {

namespace {

void exec_or_throw(sqlite3* db, const char* sql) {
    char* err = nullptr;
    if (sqlite3_exec(db, sql, nullptr, nullptr, &err) != SQLITE_OK) {
        const std::string msg = err ? err : "unknown sqlite error";
        sqlite3_free(err);
        throw std::runtime_error(std::string("sqlite3_exec failed: ") + msg + " (sql: " + sql + ")");
    }
}

// Schema is the source of truth. While the project is in early development we
// don't write migrations - bump the schema here and run `guesswork --reset-db`
// (or call Database::remove_files) to start over.
constexpr const char* kSchemaCameras =
    "CREATE TABLE IF NOT EXISTS cameras ("
    "  id                    INTEGER PRIMARY KEY AUTOINCREMENT,"
    "  name                  TEXT    NOT NULL UNIQUE,"
    "  serial                TEXT    NOT NULL UNIQUE,"
    "  focal_length_mm       REAL    NOT NULL,"   // lens focal length, drives Kalibr focal hint + model
    "  mode                  TEXT,"
    "  gain_auto             INTEGER,"
    "  gain                  REAL,"
    "  exposure_auto         INTEGER,"
    "  exposure              REAL,"
    "  calibration_json      TEXT,"      // Kalibr camchain YAML, NULL = uncalibrated
    "  calibrated_at         INTEGER,"   // unix seconds when calibration was uploaded
    "  hardware_sync_enabled INTEGER NOT NULL DEFAULT 0,"
    // 1..6 when hw-sync is on; NULL when freerun. Drives both the GenICam
    // TriggerSource on the camera and the host-side pulse-event routing.
    "  trigger_output_pin    INTEGER UNIQUE,"
    // Pipeline role: which consumer attaches to this camera's channel.
    // vio_left/vio_right are unique across cameras (enforced in the route
    // layer — partial-unique can't be expressed as a column constraint).
    "  role                  TEXT CHECK (role IN ('apriltag','vio_left','vio_right')),"
    // The camera's block of a Kalibr camchain-imucam result (T_cam_imu,
    // timeshift, T_cn_cnm1 where present), re-keyed to cam0. NULL until an
    // extrinsics job stores it.
    "  imu_extrinsics_json      TEXT,"
    "  extrinsics_calibrated_at INTEGER,"
    "  created_at            INTEGER NOT NULL DEFAULT (strftime('%s', 'now'))"
    ");";

// Single-row source of truth for Teensy trigger groups. `outputs_bitmask` is a
// 6-bit field where bit (n-1) is set iff Teensy output `n` belongs to this
// group. Uniqueness of pin assignment across groups is enforced in
// TriggerGroupRepository (no SQL constraint can express it cleanly).
constexpr const char* kSchemaTriggerGroups =
    "CREATE TABLE IF NOT EXISTS trigger_groups ("
    "  id              INTEGER PRIMARY KEY AUTOINCREMENT,"
    "  name            TEXT    NOT NULL UNIQUE,"
    "  fps             REAL    NOT NULL CHECK (fps > 0),"
    "  outputs_bitmask INTEGER NOT NULL CHECK (outputs_bitmask > 0 AND outputs_bitmask < 64),"
    "  created_at      INTEGER NOT NULL DEFAULT (strftime('%s', 'now'))"
    ");";

// Single-row IMU configuration. Noise defaults are BMI088 datasheet values
// in Kalibr's continuous-time units; the calibration layer applies the
// customary ×5–10 inflation when generating imu.yaml (datasheet figures are
// optimistic vs. a sensor bolted to a robot). t_imu_robot_json is the
// CAD-derived IMU→robot transform consumed when chaining camera extrinsics
// into the robot frame.
// WPILib AprilTagFieldLayout documents, stored verbatim. Exactly one row is
// active at a time — enforced in FieldLayoutRepository::activate() (a
// partial-unique constraint can't be expressed as a column constraint).
// Seeded with the bundled season layout on first boot (additive table — no
// --reset-db needed when this arrived).
constexpr const char* kSchemaFieldLayouts =
    "CREATE TABLE IF NOT EXISTS field_layouts ("
    "  id         INTEGER PRIMARY KEY AUTOINCREMENT,"
    "  name       TEXT    NOT NULL UNIQUE,"
    "  json       TEXT    NOT NULL,"
    "  active     INTEGER NOT NULL DEFAULT 0 CHECK (active IN (0,1)),"
    "  created_at INTEGER NOT NULL DEFAULT (strftime('%s', 'now'))"
    ");";

constexpr const char* kSchemaImuConfig =
    "CREATE TABLE IF NOT EXISTS imu_config ("
    "  id                  INTEGER PRIMARY KEY CHECK (id = 1),"
    "  rate_hz             REAL NOT NULL DEFAULT 400 CHECK (rate_hz > 0),"
    "  accel_noise_density REAL NOT NULL DEFAULT 1.7e-3,"   // m/s²/√Hz
    "  accel_random_walk   REAL NOT NULL DEFAULT 4.4e-4,"   // m/s³/√Hz
    "  gyro_noise_density  REAL NOT NULL DEFAULT 2.4e-4,"   // rad/s/√Hz
    "  gyro_random_walk    REAL NOT NULL DEFAULT 2.7e-5,"   // rad/s²/√Hz
    "  t_imu_robot_json    TEXT,"                            // NULL until configured
    "  updated_at          INTEGER NOT NULL DEFAULT (strftime('%s', 'now'))"
    ");"
    "INSERT OR IGNORE INTO imu_config (id) VALUES (1);";

// Single-row VIO (OpenVINS) configuration. Tunables only — which cameras
// participate is driven by cameras.role ('vio_left'/'vio_right'), and the
// noise model comes from imu_config.
constexpr const char* kSchemaVioConfig =
    "CREATE TABLE IF NOT EXISTS vio_config ("
    "  id                   INTEGER PRIMARY KEY CHECK (id = 1),"
    "  enabled              INTEGER NOT NULL DEFAULT 1 CHECK (enabled IN (0,1)),"
    "  num_pts              INTEGER NOT NULL DEFAULT 150 CHECK (num_pts BETWEEN 50 AND 400),"
    "  fast_threshold       INTEGER NOT NULL DEFAULT 20 CHECK (fast_threshold BETWEEN 5 AND 100),"
    // CPU lever: pyrDown the images + halve intrinsics before tracking.
    "  downsample           INTEGER NOT NULL DEFAULT 1 CHECK (downsample IN (0,1)),"
    // Calibration-quality gate: reject extrinsics whose Kalibr per-camera
    // reprojection std exceeds this (bad calibration silently destroys VIO).
    "  max_reproj_std_px    REAL NOT NULL DEFAULT 1.0 CHECK (max_reproj_std_px > 0),"
    "  auto_reinit          INTEGER NOT NULL DEFAULT 1 CHECK (auto_reinit IN (0,1)),"
    "  reinit_min_features  INTEGER NOT NULL DEFAULT 15 CHECK (reinit_min_features > 0),"
    "  reinit_window_frames INTEGER NOT NULL DEFAULT 15 CHECK (reinit_window_frames > 0),"
    "  reinit_max_pos_std_m REAL NOT NULL DEFAULT 2.0 CHECK (reinit_max_pos_std_m > 0),"
    "  updated_at           INTEGER NOT NULL DEFAULT (strftime('%s', 'now'))"
    ");"
    "INSERT OR IGNORE INTO vio_config (id) VALUES (1);";

// Single-row CAN bridge configuration. 'roborio' = classic CAN 2.0 @ 1 Mbps,
// 'systemcore' = CAN FD 1M/4M; the host pushes the matching CAN_MODE command
// to the Teensy on update and on every reconnect (docs/can-protocol.md).
constexpr const char* kSchemaCanConfig =
    "CREATE TABLE IF NOT EXISTS can_config ("
    "  id         INTEGER PRIMARY KEY CHECK (id = 1),"
    "  mode       TEXT NOT NULL DEFAULT 'off'"
    "             CHECK (mode IN ('off','roborio','systemcore')),"
    "  updated_at INTEGER NOT NULL DEFAULT (strftime('%s', 'now'))"
    ");"
    "INSERT OR IGNORE INTO can_config (id) VALUES (1);";

}  // namespace

Database::Database(const std::filesystem::path& db_path) {
    if (db_path.has_parent_path()) {
        std::filesystem::create_directories(db_path.parent_path());
    }

    if (sqlite3_open(db_path.string().c_str(), &db_) != SQLITE_OK) {
        const std::string msg = db_ ? sqlite3_errmsg(db_) : "sqlite3_open failed";
        sqlite3_close(db_);
        db_ = nullptr;
        throw std::runtime_error("Database: open failed: " + msg);
    }

    exec_or_throw(db_, "PRAGMA journal_mode = WAL;");
    exec_or_throw(db_, "PRAGMA foreign_keys = ON;");
    exec_or_throw(db_, "PRAGMA busy_timeout = 2000;");
    exec_or_throw(db_, kSchemaCameras);
    exec_or_throw(db_, kSchemaTriggerGroups);
    exec_or_throw(db_, kSchemaFieldLayouts);
    exec_or_throw(db_, kSchemaImuConfig);
    exec_or_throw(db_, kSchemaVioConfig);
    exec_or_throw(db_, kSchemaCanConfig);
}

void Database::remove_files(const std::filesystem::path& db_path) {
    std::error_code ec;
    std::filesystem::remove(db_path, ec);
    std::filesystem::remove(db_path.string() + "-wal", ec);
    std::filesystem::remove(db_path.string() + "-shm", ec);
}

Database::~Database() {
    if (db_) {
        sqlite3_close(db_);
    }
}

std::filesystem::path Database::default_path() {
    return data_dir() / "guesswork.db";
}

std::filesystem::path Database::data_dir() {
    const char* home = std::getenv("HOME");
    if (!home || !*home) {
        throw std::runtime_error("Database::data_dir: $HOME is not set");
    }
    return std::filesystem::path(home) / ".guesswork";
}

}  // namespace gw::server
