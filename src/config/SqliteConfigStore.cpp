#include "posest/config/SqliteConfigStore.h"

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
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

std::vector<double> parseDoubleArrayJson(const std::string& value, const char* field_name) {
    const auto json = nlohmann::json::parse(value);
    if (!json.is_array()) {
        throw std::runtime_error(std::string("SQLite field is not a JSON array: ") + field_name);
    }
    std::vector<double> out;
    out.reserve(json.size());
    for (const auto& entry : json) {
        if (!entry.is_number()) {
            throw std::runtime_error(std::string("SQLite JSON array contains non-number: ") +
                                     field_name);
        }
        out.push_back(entry.get<double>());
    }
    return out;
}

std::string doubleArrayToJson(const std::vector<double>& values) {
    return nlohmann::json(values).dump();
}

std::vector<std::string> parseStringArrayJson(const std::string& value, const char* field_name) {
    const auto json = nlohmann::json::parse(value);
    if (!json.is_array()) {
        throw std::runtime_error(std::string("SQLite field is not a JSON array: ") + field_name);
    }
    std::vector<std::string> out;
    out.reserve(json.size());
    for (const auto& entry : json) {
        if (!entry.is_string()) {
            throw std::runtime_error(std::string("SQLite JSON array contains non-string: ") +
                                     field_name);
        }
        out.push_back(entry.get<std::string>());
    }
    return out;
}

std::string stringArrayToJson(const std::vector<std::string>& values) {
    return nlohmann::json(values).dump();
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
            "SELECT camera_id, version, active, source_file_path, created_at, image_width, "
            "image_height, camera_model, distortion_model, fx, fy, cx, cy, "
            "distortion_coefficients_json FROM calibrations "
            "ORDER BY camera_id, version");
        while (stmt.stepRow()) {
            runtime::CameraCalibrationConfig calibration;
            calibration.camera_id = stmt.columnText(0);
            calibration.version = stmt.columnText(1);
            calibration.active = stmt.columnInt(2) != 0;
            calibration.source_file_path = stmt.columnText(3);
            calibration.created_at = stmt.columnText(4);
            calibration.image_width = stmt.columnInt(5);
            calibration.image_height = stmt.columnInt(6);
            calibration.camera_model = stmt.columnText(7);
            calibration.distortion_model = stmt.columnText(8);
            calibration.fx = stmt.columnDouble(9);
            calibration.fy = stmt.columnDouble(10);
            calibration.cx = stmt.columnDouble(11);
            calibration.cy = stmt.columnDouble(12);
            calibration.distortion_coefficients =
                parseDoubleArrayJson(stmt.columnText(13), "distortion_coefficients_json");
            config.calibrations.push_back(std::move(calibration));
        }
    }

    {
        Statement stmt(
            db_,
            "SELECT camera_id, version, tx_m, ty_m, tz_m, roll_rad, pitch_rad, yaw_rad "
            "FROM camera_extrinsics ORDER BY camera_id, version");
        while (stmt.stepRow()) {
            runtime::CameraExtrinsicsConfig extrinsics;
            extrinsics.camera_id = stmt.columnText(0);
            extrinsics.version = stmt.columnText(1);
            extrinsics.camera_to_robot.translation_m = {
                stmt.columnDouble(2),
                stmt.columnDouble(3),
                stmt.columnDouble(4),
            };
            extrinsics.camera_to_robot.rotation_rpy_rad = {
                stmt.columnDouble(5),
                stmt.columnDouble(6),
                stmt.columnDouble(7),
            };
            config.camera_extrinsics.push_back(extrinsics);
        }
    }

    {
        Statement stmt(
            db_,
            "SELECT camera_id, version, active, source_file_path, created_at, "
            "cam_imu_tx_m, cam_imu_ty_m, cam_imu_tz_m, "
            "cam_imu_roll_rad, cam_imu_pitch_rad, cam_imu_yaw_rad, "
            "imu_cam_tx_m, imu_cam_ty_m, imu_cam_tz_m, "
            "imu_cam_roll_rad, imu_cam_pitch_rad, imu_cam_yaw_rad, time_shift_s "
            "FROM camera_imu_calibrations ORDER BY camera_id, version");
        while (stmt.stepRow()) {
            runtime::CameraImuCalibrationConfig calibration;
            calibration.camera_id = stmt.columnText(0);
            calibration.version = stmt.columnText(1);
            calibration.active = stmt.columnInt(2) != 0;
            calibration.source_file_path = stmt.columnText(3);
            calibration.created_at = stmt.columnText(4);
            calibration.camera_to_imu.translation_m = {
                stmt.columnDouble(5),
                stmt.columnDouble(6),
                stmt.columnDouble(7),
            };
            calibration.camera_to_imu.rotation_rpy_rad = {
                stmt.columnDouble(8),
                stmt.columnDouble(9),
                stmt.columnDouble(10),
            };
            calibration.imu_to_camera.translation_m = {
                stmt.columnDouble(11),
                stmt.columnDouble(12),
                stmt.columnDouble(13),
            };
            calibration.imu_to_camera.rotation_rpy_rad = {
                stmt.columnDouble(14),
                stmt.columnDouble(15),
                stmt.columnDouble(16),
            };
            calibration.time_shift_s = stmt.columnDouble(17);
            config.camera_imu_calibrations.push_back(calibration);
        }
    }

    {
        Statement stmt(
            db_,
            "SELECT id, path, created_at, duration_s, camera_ids_json "
            "FROM kalibr_datasets ORDER BY id");
        while (stmt.stepRow()) {
            runtime::KalibrDatasetConfig dataset;
            dataset.id = stmt.columnText(0);
            dataset.path = stmt.columnText(1);
            dataset.created_at = stmt.columnText(2);
            dataset.duration_s = stmt.columnDouble(3);
            dataset.camera_ids = parseStringArrayJson(stmt.columnText(4), "camera_ids_json");
            config.kalibr_datasets.push_back(std::move(dataset));
        }
    }

    {
        Statement stmt(
            db_,
            "SELECT docker_image FROM calibration_tool_config WHERE id = 1");
        if (stmt.stepRow()) {
            config.calibration_tools.docker_image = stmt.columnText(0);
        }
    }

    std::unordered_map<std::string, std::size_t> field_layout_index;
    {
        Statement stmt(
            db_,
            "SELECT id, name, source_file_path, field_length_m, field_width_m, active "
            "FROM field_layouts ORDER BY id");
        while (stmt.stepRow()) {
            runtime::FieldLayoutConfig layout;
            layout.id = stmt.columnText(0);
            layout.name = stmt.columnText(1);
            layout.source_file_path = stmt.columnText(2);
            layout.field_length_m = stmt.columnDouble(3);
            layout.field_width_m = stmt.columnDouble(4);
            if (stmt.columnInt(5) != 0) {
                config.active_field_layout_id = layout.id;
            }
            field_layout_index.emplace(layout.id, config.field_layouts.size());
            config.field_layouts.push_back(std::move(layout));
        }
    }

    {
        Statement stmt(
            db_,
            "SELECT layout_id, tag_id, tx_m, ty_m, tz_m, roll_rad, pitch_rad, yaw_rad "
            "FROM field_tags ORDER BY layout_id, tag_id");
        while (stmt.stepRow()) {
            const std::string layout_id = stmt.columnText(0);
            const auto it = field_layout_index.find(layout_id);
            if (it == field_layout_index.end()) {
                throw std::runtime_error("SQLite field_tags row references unknown layout");
            }
            runtime::FieldTagConfig tag;
            tag.tag_id = stmt.columnInt(1);
            tag.field_to_tag.translation_m = {
                stmt.columnDouble(2),
                stmt.columnDouble(3),
                stmt.columnDouble(4),
            };
            tag.field_to_tag.rotation_rpy_rad = {
                stmt.columnDouble(5),
                stmt.columnDouble(6),
                stmt.columnDouble(7),
            };
            config.field_layouts[it->second].tags.push_back(tag);
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
    exec(db_, "DELETE FROM field_tags");
    exec(db_, "DELETE FROM field_layouts");
    exec(db_, "DELETE FROM camera_imu_calibrations");
    exec(db_, "DELETE FROM kalibr_datasets");
    exec(db_, "DELETE FROM calibration_tool_config");
    exec(db_, "DELETE FROM camera_extrinsics");
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
            "INSERT INTO calibrations "
            "(camera_id, version, active, source_file_path, created_at, image_width, "
            "image_height, camera_model, distortion_model, fx, fy, cx, cy, "
            "distortion_coefficients_json) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)");
        insert.bindText(1, calibration.camera_id);
        insert.bindText(2, calibration.version);
        insert.bindInt(3, calibration.active ? 1 : 0);
        insert.bindText(4, calibration.source_file_path);
        insert.bindText(5, calibration.created_at);
        insert.bindInt(6, calibration.image_width);
        insert.bindInt(7, calibration.image_height);
        insert.bindText(8, calibration.camera_model);
        insert.bindText(9, calibration.distortion_model);
        insert.bindDouble(10, calibration.fx);
        insert.bindDouble(11, calibration.fy);
        insert.bindDouble(12, calibration.cx);
        insert.bindDouble(13, calibration.cy);
        insert.bindText(14, doubleArrayToJson(calibration.distortion_coefficients));
        insert.stepDone();
    }

    for (const auto& extrinsics : config.camera_extrinsics) {
        Statement insert(
            db_,
            "INSERT INTO camera_extrinsics "
            "(camera_id, version, tx_m, ty_m, tz_m, roll_rad, pitch_rad, yaw_rad) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?)");
        insert.bindText(1, extrinsics.camera_id);
        insert.bindText(2, extrinsics.version);
        insert.bindDouble(3, extrinsics.camera_to_robot.translation_m.x);
        insert.bindDouble(4, extrinsics.camera_to_robot.translation_m.y);
        insert.bindDouble(5, extrinsics.camera_to_robot.translation_m.z);
        insert.bindDouble(6, extrinsics.camera_to_robot.rotation_rpy_rad.x);
        insert.bindDouble(7, extrinsics.camera_to_robot.rotation_rpy_rad.y);
        insert.bindDouble(8, extrinsics.camera_to_robot.rotation_rpy_rad.z);
        insert.stepDone();
    }

    for (const auto& calibration : config.camera_imu_calibrations) {
        Statement insert(
            db_,
            "INSERT INTO camera_imu_calibrations "
            "(camera_id, version, active, source_file_path, created_at, "
            "cam_imu_tx_m, cam_imu_ty_m, cam_imu_tz_m, "
            "cam_imu_roll_rad, cam_imu_pitch_rad, cam_imu_yaw_rad, "
            "imu_cam_tx_m, imu_cam_ty_m, imu_cam_tz_m, "
            "imu_cam_roll_rad, imu_cam_pitch_rad, imu_cam_yaw_rad, time_shift_s) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)");
        insert.bindText(1, calibration.camera_id);
        insert.bindText(2, calibration.version);
        insert.bindInt(3, calibration.active ? 1 : 0);
        insert.bindText(4, calibration.source_file_path);
        insert.bindText(5, calibration.created_at);
        insert.bindDouble(6, calibration.camera_to_imu.translation_m.x);
        insert.bindDouble(7, calibration.camera_to_imu.translation_m.y);
        insert.bindDouble(8, calibration.camera_to_imu.translation_m.z);
        insert.bindDouble(9, calibration.camera_to_imu.rotation_rpy_rad.x);
        insert.bindDouble(10, calibration.camera_to_imu.rotation_rpy_rad.y);
        insert.bindDouble(11, calibration.camera_to_imu.rotation_rpy_rad.z);
        insert.bindDouble(12, calibration.imu_to_camera.translation_m.x);
        insert.bindDouble(13, calibration.imu_to_camera.translation_m.y);
        insert.bindDouble(14, calibration.imu_to_camera.translation_m.z);
        insert.bindDouble(15, calibration.imu_to_camera.rotation_rpy_rad.x);
        insert.bindDouble(16, calibration.imu_to_camera.rotation_rpy_rad.y);
        insert.bindDouble(17, calibration.imu_to_camera.rotation_rpy_rad.z);
        insert.bindDouble(18, calibration.time_shift_s);
        insert.stepDone();
    }

    for (const auto& dataset : config.kalibr_datasets) {
        Statement insert(
            db_,
            "INSERT INTO kalibr_datasets (id, path, created_at, duration_s, camera_ids_json) "
            "VALUES (?, ?, ?, ?, ?)");
        insert.bindText(1, dataset.id);
        insert.bindText(2, dataset.path);
        insert.bindText(3, dataset.created_at);
        insert.bindDouble(4, dataset.duration_s);
        insert.bindText(5, stringArrayToJson(dataset.camera_ids));
        insert.stepDone();
    }

    {
        Statement insert(
            db_,
            "INSERT INTO calibration_tool_config (id, docker_image) VALUES (1, ?)");
        insert.bindText(1, config.calibration_tools.docker_image);
        insert.stepDone();
    }

    for (const auto& layout : config.field_layouts) {
        Statement insert(
            db_,
            "INSERT INTO field_layouts "
            "(id, name, source_file_path, field_length_m, field_width_m, active) "
            "VALUES (?, ?, ?, ?, ?, ?)");
        insert.bindText(1, layout.id);
        insert.bindText(2, layout.name);
        insert.bindText(3, layout.source_file_path);
        insert.bindDouble(4, layout.field_length_m);
        insert.bindDouble(5, layout.field_width_m);
        insert.bindInt(6, layout.id == config.active_field_layout_id ? 1 : 0);
        insert.stepDone();

        for (const auto& tag : layout.tags) {
            Statement tag_insert(
                db_,
                "INSERT INTO field_tags "
                "(layout_id, tag_id, tx_m, ty_m, tz_m, roll_rad, pitch_rad, yaw_rad) "
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?)");
            tag_insert.bindText(1, layout.id);
            tag_insert.bindInt(2, tag.tag_id);
            tag_insert.bindDouble(3, tag.field_to_tag.translation_m.x);
            tag_insert.bindDouble(4, tag.field_to_tag.translation_m.y);
            tag_insert.bindDouble(5, tag.field_to_tag.translation_m.z);
            tag_insert.bindDouble(6, tag.field_to_tag.rotation_rpy_rad.x);
            tag_insert.bindDouble(7, tag.field_to_tag.rotation_rpy_rad.y);
            tag_insert.bindDouble(8, tag.field_to_tag.rotation_rpy_rad.z);
            tag_insert.stepDone();
        }
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
