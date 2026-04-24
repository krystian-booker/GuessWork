#include "posest/config/SqliteConfigStore.h"

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include <sqlite3.h>

#include "posest/config/ConfigValidator.h"
#include "posest/config/SqliteSchema.h"

namespace posest::config {

namespace {

std::string sqliteError(sqlite3* db) {
    return db ? sqlite3_errmsg(db) : "null SQLite handle";
}

void exec(sqlite3* db, const char* sql) {
    char* err = nullptr;
    if (sqlite3_exec(db, sql, nullptr, nullptr, &err) != SQLITE_OK) {
        std::string msg = err ? err : sqliteError(db);
        sqlite3_free(err);
        throw std::runtime_error("SQLite exec failed: " + msg);
    }
}

class Statement final {
public:
    Statement(sqlite3* db, const char* sql) : db_(db) {
        if (sqlite3_prepare_v2(db_, sql, -1, &stmt_, nullptr) != SQLITE_OK) {
            throw std::runtime_error("SQLite prepare failed: " + sqliteError(db_));
        }
    }

    ~Statement() {
        if (stmt_) {
            sqlite3_finalize(stmt_);
        }
    }

    Statement(const Statement&) = delete;
    Statement& operator=(const Statement&) = delete;

    void bindText(int index, const std::string& value) {
        if (sqlite3_bind_text(stmt_, index, value.c_str(), -1, SQLITE_TRANSIENT) != SQLITE_OK) {
            throw std::runtime_error("SQLite text bind failed: " + sqliteError(db_));
        }
    }

    void bindInt(int index, int value) {
        if (sqlite3_bind_int(stmt_, index, value) != SQLITE_OK) {
            throw std::runtime_error("SQLite int bind failed: " + sqliteError(db_));
        }
    }

    void bindInt64(int index, sqlite3_int64 value) {
        if (sqlite3_bind_int64(stmt_, index, value) != SQLITE_OK) {
            throw std::runtime_error("SQLite int64 bind failed: " + sqliteError(db_));
        }
    }

    void bindDouble(int index, double value) {
        if (sqlite3_bind_double(stmt_, index, value) != SQLITE_OK) {
            throw std::runtime_error("SQLite double bind failed: " + sqliteError(db_));
        }
    }

    bool stepRow() {
        const int rc = sqlite3_step(stmt_);
        if (rc == SQLITE_ROW) {
            return true;
        }
        if (rc == SQLITE_DONE) {
            return false;
        }
        throw std::runtime_error("SQLite step failed: " + sqliteError(db_));
    }

    void stepDone() {
        const int rc = sqlite3_step(stmt_);
        if (rc != SQLITE_DONE) {
            throw std::runtime_error("SQLite statement did not finish: " + sqliteError(db_));
        }
    }

    std::string columnText(int index) const {
        const auto* text = sqlite3_column_text(stmt_, index);
        return text ? reinterpret_cast<const char*>(text) : "";
    }

    int columnInt(int index) const {
        return sqlite3_column_int(stmt_, index);
    }

    double columnDouble(int index) const {
        return sqlite3_column_double(stmt_, index);
    }

    sqlite3_int64 columnInt64(int index) const {
        return sqlite3_column_int64(stmt_, index);
    }

private:
    sqlite3* db_{nullptr};
    sqlite3_stmt* stmt_{nullptr};
};

class Transaction final {
public:
    explicit Transaction(sqlite3* db) : db_(db) {
        exec(db_, "BEGIN IMMEDIATE");
    }

    ~Transaction() {
        if (!committed_) {
            sqlite3_exec(db_, "ROLLBACK", nullptr, nullptr, nullptr);
        }
    }

    Transaction(const Transaction&) = delete;
    Transaction& operator=(const Transaction&) = delete;

    void commit() {
        exec(db_, "COMMIT");
        committed_ = true;
    }

private:
    sqlite3* db_{nullptr};
    bool committed_{false};
};

std::uint32_t checkedUint32(sqlite3_int64 value, const char* field_name) {
    if (value < 0 || value > std::numeric_limits<std::uint32_t>::max()) {
        throw std::runtime_error(std::string("SQLite field out of uint32 range: ") + field_name);
    }
    return static_cast<std::uint32_t>(value);
}

}  // namespace

SqliteConfigStore::SqliteConfigStore(std::filesystem::path db_path)
    : db_path_(std::move(db_path)) {
    sqlite3* opened = nullptr;
    if (sqlite3_open(db_path_.string().c_str(), &opened) != SQLITE_OK) {
        std::string msg = sqliteError(opened);
        if (opened) {
            sqlite3_close(opened);
        }
        throw std::runtime_error("Failed to open SQLite config DB: " + msg);
    }
    db_ = opened;
    exec(db_, "PRAGMA foreign_keys = ON");
    applyMigrations(db_);
}

SqliteConfigStore::~SqliteConfigStore() {
    if (db_) {
        sqlite3_close(db_);
        db_ = nullptr;
    }
}

runtime::RuntimeConfig SqliteConfigStore::loadRuntimeConfig() const {
    std::lock_guard<std::mutex> g(mu_);

    runtime::RuntimeConfig config;
    std::unordered_map<std::string, std::size_t> camera_index;

    {
        Statement stmt(
            db_,
            "SELECT id, backend_type, device, enabled, width, height, fps, pixel_format "
            "FROM cameras ORDER BY id");
        while (stmt.stepRow()) {
            CameraConfig camera;
            camera.id = stmt.columnText(0);
            camera.type = stmt.columnText(1);
            camera.device = stmt.columnText(2);
            camera.enabled = stmt.columnInt(3) != 0;
            camera.format.width = stmt.columnInt(4);
            camera.format.height = stmt.columnInt(5);
            camera.format.fps = stmt.columnDouble(6);
            camera.format.pixel_format = stmt.columnText(7);
            camera_index.emplace(camera.id, config.cameras.size());
            config.cameras.push_back(std::move(camera));
        }
    }

    {
        Statement stmt(
            db_,
            "SELECT camera_id, name, value FROM camera_controls "
            "ORDER BY camera_id, name");
        while (stmt.stepRow()) {
            const std::string camera_id = stmt.columnText(0);
            const auto it = camera_index.find(camera_id);
            if (it == camera_index.end()) {
                throw std::runtime_error("SQLite camera_controls row references unknown camera");
            }
            config.cameras[it->second].controls.push_back({
                stmt.columnText(1),
                static_cast<std::int32_t>(stmt.columnInt(2)),
            });
        }
    }

    {
        Statement stmt(
            db_,
            "SELECT id, type, enabled, parameters_json FROM pipelines ORDER BY id");
        while (stmt.stepRow()) {
            runtime::PipelineConfig pipeline;
            pipeline.id = stmt.columnText(0);
            pipeline.type = stmt.columnText(1);
            pipeline.enabled = stmt.columnInt(2) != 0;
            pipeline.parameters_json = stmt.columnText(3);
            config.pipelines.push_back(std::move(pipeline));
        }
    }

    {
        Statement stmt(
            db_,
            "SELECT camera_id, pipeline_id FROM camera_pipeline_bindings "
            "ORDER BY camera_id, pipeline_id");
        while (stmt.stepRow()) {
            config.bindings.push_back({
                stmt.columnText(0),
                stmt.columnText(1),
            });
        }
    }

    {
        Statement stmt(
            db_,
            "SELECT camera_id, file_path, version, created_at FROM calibrations "
            "ORDER BY camera_id, version");
        while (stmt.stepRow()) {
            config.calibrations.push_back({
                stmt.columnText(0),
                stmt.columnText(1),
                stmt.columnText(2),
                stmt.columnText(3),
            });
        }
    }

    {
        Statement stmt(
            db_,
            "SELECT camera_id, enabled, teensy_pin, rate_hz, pulse_width_us, phase_offset_us "
            "FROM camera_triggers ORDER BY camera_id");
        while (stmt.stepRow()) {
            config.camera_triggers.push_back({
                stmt.columnText(0),
                stmt.columnInt(1) != 0,
                static_cast<std::int32_t>(stmt.columnInt(2)),
                stmt.columnDouble(3),
                checkedUint32(stmt.columnInt64(4), "pulse_width_us"),
                static_cast<std::int64_t>(stmt.columnInt64(5)),
            });
        }
    }

    {
        Statement stmt(
            db_,
            "SELECT serial_port, fused_pose_can_id, status_can_id, pose_publish_hz, "
            "baud_rate, reconnect_interval_ms, read_timeout_ms "
            "FROM teensy_config WHERE id = 1");
        if (stmt.stepRow()) {
            config.teensy.serial_port = stmt.columnText(0);
            config.teensy.fused_pose_can_id =
                checkedUint32(stmt.columnInt64(1), "fused_pose_can_id");
            config.teensy.status_can_id =
                checkedUint32(stmt.columnInt64(2), "status_can_id");
            config.teensy.pose_publish_hz = stmt.columnDouble(3);
            config.teensy.baud_rate = checkedUint32(stmt.columnInt64(4), "baud_rate");
            config.teensy.reconnect_interval_ms =
                checkedUint32(stmt.columnInt64(5), "reconnect_interval_ms");
            config.teensy.read_timeout_ms =
                checkedUint32(stmt.columnInt64(6), "read_timeout_ms");
        }
    }

    validateRuntimeConfig(config);
    return config;
}

void SqliteConfigStore::saveRuntimeConfig(const runtime::RuntimeConfig& config) {
    validateRuntimeConfig(config);

    std::lock_guard<std::mutex> g(mu_);
    Transaction tx(db_);

    exec(db_, "DELETE FROM camera_triggers");
    exec(db_, "DELETE FROM calibrations");
    exec(db_, "DELETE FROM camera_pipeline_bindings");
    exec(db_, "DELETE FROM camera_controls");
    exec(db_, "DELETE FROM pipelines");
    exec(db_, "DELETE FROM cameras");
    exec(db_, "DELETE FROM teensy_config");

    for (const auto& camera : config.cameras) {
        Statement insert(
            db_,
            "INSERT INTO cameras "
            "(id, backend_type, device, enabled, width, height, fps, pixel_format) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?)");
        insert.bindText(1, camera.id);
        insert.bindText(2, camera.type);
        insert.bindText(3, camera.device);
        insert.bindInt(4, camera.enabled ? 1 : 0);
        insert.bindInt(5, camera.format.width);
        insert.bindInt(6, camera.format.height);
        insert.bindDouble(7, camera.format.fps);
        insert.bindText(8, camera.format.pixel_format);
        insert.stepDone();
    }

    for (const auto& camera : config.cameras) {
        for (const auto& control : camera.controls) {
            Statement insert(
                db_,
                "INSERT INTO camera_controls (camera_id, name, value) VALUES (?, ?, ?)");
            insert.bindText(1, camera.id);
            insert.bindText(2, control.name);
            insert.bindInt(3, control.value);
            insert.stepDone();
        }
    }

    for (const auto& pipeline : config.pipelines) {
        Statement insert(
            db_,
            "INSERT INTO pipelines (id, type, enabled, parameters_json) "
            "VALUES (?, ?, ?, ?)");
        insert.bindText(1, pipeline.id);
        insert.bindText(2, pipeline.type);
        insert.bindInt(3, pipeline.enabled ? 1 : 0);
        insert.bindText(4, pipeline.parameters_json);
        insert.stepDone();
    }

    for (const auto& binding : config.bindings) {
        Statement insert(
            db_,
            "INSERT INTO camera_pipeline_bindings (camera_id, pipeline_id) "
            "VALUES (?, ?)");
        insert.bindText(1, binding.camera_id);
        insert.bindText(2, binding.pipeline_id);
        insert.stepDone();
    }

    for (const auto& calibration : config.calibrations) {
        Statement insert(
            db_,
            "INSERT INTO calibrations (camera_id, file_path, version, created_at) "
            "VALUES (?, ?, ?, ?)");
        insert.bindText(1, calibration.camera_id);
        insert.bindText(2, calibration.file_path);
        insert.bindText(3, calibration.version);
        insert.bindText(4, calibration.created_at);
        insert.stepDone();
    }

    for (const auto& trigger : config.camera_triggers) {
        Statement insert(
            db_,
            "INSERT INTO camera_triggers "
            "(camera_id, enabled, teensy_pin, rate_hz, pulse_width_us, phase_offset_us) "
            "VALUES (?, ?, ?, ?, ?, ?)");
        insert.bindText(1, trigger.camera_id);
        insert.bindInt(2, trigger.enabled ? 1 : 0);
        insert.bindInt(3, trigger.teensy_pin);
        insert.bindDouble(4, trigger.rate_hz);
        insert.bindInt64(5, static_cast<sqlite3_int64>(trigger.pulse_width_us));
        insert.bindInt64(6, static_cast<sqlite3_int64>(trigger.phase_offset_us));
        insert.stepDone();
    }

    {
        Statement insert(
            db_,
            "INSERT INTO teensy_config "
            "(id, serial_port, fused_pose_can_id, status_can_id, pose_publish_hz, "
            "baud_rate, reconnect_interval_ms, read_timeout_ms) "
            "VALUES (1, ?, ?, ?, ?, ?, ?, ?)");
        insert.bindText(1, config.teensy.serial_port);
        insert.bindInt64(2, static_cast<sqlite3_int64>(config.teensy.fused_pose_can_id));
        insert.bindInt64(3, static_cast<sqlite3_int64>(config.teensy.status_can_id));
        insert.bindDouble(4, config.teensy.pose_publish_hz);
        insert.bindInt64(5, static_cast<sqlite3_int64>(config.teensy.baud_rate));
        insert.bindInt64(6, static_cast<sqlite3_int64>(config.teensy.reconnect_interval_ms));
        insert.bindInt64(7, static_cast<sqlite3_int64>(config.teensy.read_timeout_ms));
        insert.stepDone();
    }

    tx.commit();
}

}  // namespace posest::config
