#include "posest/config/SqliteConfigStore.h"

#include <cmath>
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

    void bindNull(int index) {
        if (sqlite3_bind_null(stmt_, index) != SQLITE_OK) {
            throw std::runtime_error("SQLite null bind failed: " + sqliteError(db_));
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
            "SELECT id, backend_type, device, enabled, width, height, fps, pixel_format, "
            "trigger_mode, reconnect_interval_ms, reconnect_max_attempts "
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
            const std::string trigger_text = stmt.columnText(8);
            const auto parsed_trigger = triggerModeFromString(trigger_text);
            if (!parsed_trigger) {
                throw std::runtime_error(
                    "SQLite cameras row has invalid trigger_mode: " + trigger_text);
            }
            camera.trigger_mode = *parsed_trigger;
            camera.reconnect.interval_ms =
                checkedUint32(stmt.columnInt64(9), "reconnect_interval_ms");
            camera.reconnect.max_attempts =
                checkedUint32(stmt.columnInt64(10), "reconnect_max_attempts");
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
            "baud_rate, reconnect_interval_ms, read_timeout_ms, time_sync_interval_ms "
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
            config.teensy.time_sync_interval_ms =
                checkedUint32(stmt.columnInt64(7), "time_sync_interval_ms");
        }
    }

    {
        Statement stmt(
            db_,
            "SELECT accel_range_g, accel_odr_hz, accel_bandwidth_code, "
            "gyro_range_dps, gyro_bandwidth_code, data_sync_rate_hz, "
            "run_selftest_on_boot FROM imu_config WHERE id = 1");
        if (stmt.stepRow()) {
            config.teensy.imu.accel_range_g =
                checkedUint32(stmt.columnInt64(0), "accel_range_g");
            config.teensy.imu.accel_odr_hz =
                checkedUint32(stmt.columnInt64(1), "accel_odr_hz");
            config.teensy.imu.accel_bandwidth_code =
                checkedUint32(stmt.columnInt64(2), "accel_bandwidth_code");
            config.teensy.imu.gyro_range_dps =
                checkedUint32(stmt.columnInt64(3), "gyro_range_dps");
            config.teensy.imu.gyro_bandwidth_code =
                checkedUint32(stmt.columnInt64(4), "gyro_bandwidth_code");
            config.teensy.imu.data_sync_rate_hz =
                checkedUint32(stmt.columnInt64(5), "data_sync_rate_hz");
            config.teensy.imu.run_selftest_on_boot = stmt.columnInt64(6) != 0;
        }
    }

    {
        Statement stmt(
            db_,
            "SELECT enabled, nominal_bitrate_bps, data_bitrate_bps, pose_publish_hz, "
            "rio_offset_stale_ms, rio_pose_can_id, rio_time_sync_can_id, "
            "teensy_pose_can_id FROM can_config WHERE id = 1");
        if (stmt.stepRow()) {
            config.teensy.can.enabled = stmt.columnInt64(0) != 0;
            config.teensy.can.nominal_bitrate_bps =
                checkedUint32(stmt.columnInt64(1), "nominal_bitrate_bps");
            config.teensy.can.data_bitrate_bps =
                checkedUint32(stmt.columnInt64(2), "data_bitrate_bps");
            config.teensy.can.pose_publish_hz =
                checkedUint32(stmt.columnInt64(3), "pose_publish_hz");
            config.teensy.can.rio_offset_stale_ms =
                checkedUint32(stmt.columnInt64(4), "rio_offset_stale_ms");
            config.teensy.can.rio_pose_can_id =
                checkedUint32(stmt.columnInt64(5), "rio_pose_can_id");
            config.teensy.can.rio_time_sync_can_id =
                checkedUint32(stmt.columnInt64(6), "rio_time_sync_can_id");
            config.teensy.can.teensy_pose_can_id =
                checkedUint32(stmt.columnInt64(7), "teensy_pose_can_id");
        }
    }

    {
        Statement stmt(
            db_,
            "SELECT enabled, vio_camera_id, vio_slot_index, ir_led_enabled, "
            "ir_led_pulse_width_us, tof_enabled, tof_i2c_address, "
            "tof_timing_budget_ms, tof_intermeasurement_period_ms, "
            "tof_offset_after_flash_us, tof_divisor, tof_mounting_offset_mm, "
            "tof_expected_min_mm, tof_expected_max_mm "
            "FROM vio_config WHERE id = 1");
        if (stmt.stepRow()) {
            config.vio.enabled = stmt.columnInt64(0) != 0;
            config.vio.vio_camera_id = stmt.columnText(1);
            config.vio.vio_slot_index =
                static_cast<std::int32_t>(stmt.columnInt64(2));
            config.vio.ir_led_enabled = stmt.columnInt64(3) != 0;
            config.vio.ir_led_pulse_width_us =
                checkedUint32(stmt.columnInt64(4), "ir_led_pulse_width_us");
            config.vio.tof_enabled = stmt.columnInt64(5) != 0;
            config.vio.tof_i2c_address =
                checkedUint32(stmt.columnInt64(6), "tof_i2c_address");
            config.vio.tof_timing_budget_ms =
                checkedUint32(stmt.columnInt64(7), "tof_timing_budget_ms");
            config.vio.tof_intermeasurement_period_ms =
                checkedUint32(stmt.columnInt64(8), "tof_intermeasurement_period_ms");
            config.vio.tof_offset_after_flash_us =
                checkedUint32(stmt.columnInt64(9), "tof_offset_after_flash_us");
            config.vio.tof_divisor =
                checkedUint32(stmt.columnInt64(10), "tof_divisor");
            // Stored as integer mm; materialize in meters for the SI-domain
            // measurement struct.
            config.vio.tof_mounting_offset_m =
                static_cast<double>(stmt.columnInt64(11)) * 1e-3;
            config.vio.tof_expected_min_m =
                static_cast<double>(stmt.columnInt64(12)) * 1e-3;
            config.vio.tof_expected_max_m =
                static_cast<double>(stmt.columnInt64(13)) * 1e-3;
        }
    }

    {
        Statement stmt(
            db_,
            "SELECT chassis_sigma_rx, chassis_sigma_ry, chassis_sigma_rz, "
            "chassis_sigma_tx, chassis_sigma_ty, chassis_sigma_tz, "
            "origin_prior_sigma_rx, origin_prior_sigma_ry, origin_prior_sigma_rz, "
            "origin_prior_sigma_tx, origin_prior_sigma_ty, origin_prior_sigma_tz, "
            "shock_threshold_mps2, freefall_threshold_mps2, shock_inflation_factor, "
            "imu_window_seconds, max_chassis_dt_seconds, "
            "gravity_local_x, gravity_local_y, gravity_local_z, "
            "enable_vio, vio_sigma_rx, vio_sigma_ry, vio_sigma_rz, "
            "vio_sigma_tx, vio_sigma_ty, vio_sigma_tz, "
            "huber_k, enable_imu_preintegration, "
            "imu_extrinsic_tx_m, imu_extrinsic_ty_m, imu_extrinsic_tz_m, "
            "imu_extrinsic_roll_rad, imu_extrinsic_pitch_rad, imu_extrinsic_yaw_rad, "
            "accel_noise_sigma, gyro_noise_sigma, "
            "accel_bias_rw_sigma, gyro_bias_rw_sigma, integration_cov_sigma, "
            "persisted_bias_ax, persisted_bias_ay, persisted_bias_az, "
            "persisted_bias_gx, persisted_bias_gy, persisted_bias_gz, "
            "bias_calibration_seconds, bias_calibration_chassis_threshold, "
            "max_keyframe_dt_seconds, max_imu_gap_seconds, "
            "marginalize_keyframe_window, slip_disagreement_mps, "
            "enable_floor_constraint, "
            "floor_constraint_sigma_z, floor_constraint_sigma_roll, "
            "floor_constraint_sigma_pitch, max_chassis_speed_mps "
            "FROM fusion_config WHERE id = 1");
        if (stmt.stepRow()) {
            for (int i = 0; i < 6; ++i) {
                config.fusion.chassis_sigmas[static_cast<std::size_t>(i)] =
                    stmt.columnDouble(i);
            }
            for (int i = 0; i < 6; ++i) {
                config.fusion.origin_prior_sigmas[static_cast<std::size_t>(i)] =
                    stmt.columnDouble(6 + i);
            }
            config.fusion.shock_threshold_mps2 = stmt.columnDouble(12);
            config.fusion.freefall_threshold_mps2 = stmt.columnDouble(13);
            config.fusion.shock_inflation_factor = stmt.columnDouble(14);
            config.fusion.imu_window_seconds = stmt.columnDouble(15);
            config.fusion.max_chassis_dt_seconds = stmt.columnDouble(16);
            config.fusion.gravity_local_mps2 = {
                stmt.columnDouble(17),
                stmt.columnDouble(18),
                stmt.columnDouble(19),
            };
            config.fusion.enable_vio = stmt.columnInt64(20) != 0;
            for (int i = 0; i < 6; ++i) {
                config.fusion.vio_default_sigmas[static_cast<std::size_t>(i)] =
                    stmt.columnDouble(21 + i);
            }
            config.fusion.huber_k = stmt.columnDouble(27);
            config.fusion.enable_imu_preintegration = stmt.columnInt64(28) != 0;
            config.fusion.imu_extrinsic_body_to_imu.translation_m = {
                stmt.columnDouble(29),
                stmt.columnDouble(30),
                stmt.columnDouble(31),
            };
            config.fusion.imu_extrinsic_body_to_imu.rotation_rpy_rad = {
                stmt.columnDouble(32),
                stmt.columnDouble(33),
                stmt.columnDouble(34),
            };
            config.fusion.accel_noise_sigma = stmt.columnDouble(35);
            config.fusion.gyro_noise_sigma = stmt.columnDouble(36);
            config.fusion.accel_bias_rw_sigma = stmt.columnDouble(37);
            config.fusion.gyro_bias_rw_sigma = stmt.columnDouble(38);
            config.fusion.integration_cov_sigma = stmt.columnDouble(39);
            for (int i = 0; i < 6; ++i) {
                config.fusion.persisted_bias[static_cast<std::size_t>(i)] =
                    stmt.columnDouble(40 + i);
            }
            config.fusion.bias_calibration_seconds = stmt.columnDouble(46);
            config.fusion.bias_calibration_chassis_threshold = stmt.columnDouble(47);
            config.fusion.max_keyframe_dt_seconds = stmt.columnDouble(48);
            config.fusion.max_imu_gap_seconds = stmt.columnDouble(49);
            config.fusion.marginalize_keyframe_window =
                checkedUint32(stmt.columnInt64(50), "marginalize_keyframe_window");
            config.fusion.slip_disagreement_mps = stmt.columnDouble(51);
            config.fusion.enable_floor_constraint = stmt.columnInt64(52) != 0;
            config.fusion.floor_constraint_sigmas = {
                stmt.columnDouble(53),
                stmt.columnDouble(54),
                stmt.columnDouble(55),
            };
            config.fusion.max_chassis_speed_mps = stmt.columnDouble(56);
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
    exec(db_, "DELETE FROM imu_config");
    exec(db_, "DELETE FROM can_config");
    exec(db_, "DELETE FROM vio_config");
    exec(db_, "DELETE FROM fusion_config");

    for (const auto& camera : config.cameras) {
        Statement insert(
            db_,
            "INSERT INTO cameras "
            "(id, backend_type, device, enabled, width, height, fps, pixel_format, "
            " trigger_mode, reconnect_interval_ms, reconnect_max_attempts) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)");
        insert.bindText(1, camera.id);
        insert.bindText(2, camera.type);
        insert.bindText(3, camera.device);
        insert.bindInt(4, camera.enabled ? 1 : 0);
        insert.bindInt(5, camera.format.width);
        insert.bindInt(6, camera.format.height);
        insert.bindDouble(7, camera.format.fps);
        insert.bindText(8, camera.format.pixel_format);
        insert.bindText(9, triggerModeToString(camera.trigger_mode));
        insert.bindInt64(10, static_cast<sqlite3_int64>(camera.reconnect.interval_ms));
        insert.bindInt64(11, static_cast<sqlite3_int64>(camera.reconnect.max_attempts));
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
            "baud_rate, reconnect_interval_ms, read_timeout_ms, time_sync_interval_ms) "
            "VALUES (1, ?, ?, ?, ?, ?, ?, ?, ?)");
        insert.bindText(1, config.teensy.serial_port);
        insert.bindInt64(2, static_cast<sqlite3_int64>(config.teensy.fused_pose_can_id));
        insert.bindInt64(3, static_cast<sqlite3_int64>(config.teensy.status_can_id));
        insert.bindDouble(4, config.teensy.pose_publish_hz);
        insert.bindInt64(5, static_cast<sqlite3_int64>(config.teensy.baud_rate));
        insert.bindInt64(6, static_cast<sqlite3_int64>(config.teensy.reconnect_interval_ms));
        insert.bindInt64(7, static_cast<sqlite3_int64>(config.teensy.read_timeout_ms));
        insert.bindInt64(8, static_cast<sqlite3_int64>(config.teensy.time_sync_interval_ms));
        insert.stepDone();
    }

    {
        Statement insert(
            db_,
            "INSERT INTO imu_config "
            "(id, accel_range_g, accel_odr_hz, accel_bandwidth_code, "
            "gyro_range_dps, gyro_bandwidth_code, data_sync_rate_hz, "
            "run_selftest_on_boot) "
            "VALUES (1, ?, ?, ?, ?, ?, ?, ?)");
        insert.bindInt64(1, static_cast<sqlite3_int64>(config.teensy.imu.accel_range_g));
        insert.bindInt64(2, static_cast<sqlite3_int64>(config.teensy.imu.accel_odr_hz));
        insert.bindInt64(3,
            static_cast<sqlite3_int64>(config.teensy.imu.accel_bandwidth_code));
        insert.bindInt64(4, static_cast<sqlite3_int64>(config.teensy.imu.gyro_range_dps));
        insert.bindInt64(5,
            static_cast<sqlite3_int64>(config.teensy.imu.gyro_bandwidth_code));
        insert.bindInt64(6,
            static_cast<sqlite3_int64>(config.teensy.imu.data_sync_rate_hz));
        insert.bindInt(7, config.teensy.imu.run_selftest_on_boot ? 1 : 0);
        insert.stepDone();
    }

    {
        Statement insert(
            db_,
            "INSERT INTO can_config "
            "(id, enabled, nominal_bitrate_bps, data_bitrate_bps, pose_publish_hz, "
            "rio_offset_stale_ms, rio_pose_can_id, rio_time_sync_can_id, "
            "teensy_pose_can_id) "
            "VALUES (1, ?, ?, ?, ?, ?, ?, ?, ?)");
        insert.bindInt(1, config.teensy.can.enabled ? 1 : 0);
        insert.bindInt64(2,
            static_cast<sqlite3_int64>(config.teensy.can.nominal_bitrate_bps));
        insert.bindInt64(3,
            static_cast<sqlite3_int64>(config.teensy.can.data_bitrate_bps));
        insert.bindInt64(4,
            static_cast<sqlite3_int64>(config.teensy.can.pose_publish_hz));
        insert.bindInt64(5,
            static_cast<sqlite3_int64>(config.teensy.can.rio_offset_stale_ms));
        insert.bindInt64(6,
            static_cast<sqlite3_int64>(config.teensy.can.rio_pose_can_id));
        insert.bindInt64(7,
            static_cast<sqlite3_int64>(config.teensy.can.rio_time_sync_can_id));
        insert.bindInt64(8,
            static_cast<sqlite3_int64>(config.teensy.can.teensy_pose_can_id));
        insert.stepDone();
    }

    {
        Statement insert(
            db_,
            "INSERT INTO vio_config "
            "(id, enabled, vio_camera_id, vio_slot_index, ir_led_enabled, "
            "ir_led_pulse_width_us, tof_enabled, tof_i2c_address, "
            "tof_timing_budget_ms, tof_intermeasurement_period_ms, "
            "tof_offset_after_flash_us, tof_divisor, tof_mounting_offset_mm, "
            "tof_expected_min_mm, tof_expected_max_mm) "
            "VALUES (1, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)");
        insert.bindInt(1, config.vio.enabled ? 1 : 0);
        if (config.vio.vio_camera_id.empty()) {
            insert.bindNull(2);
        } else {
            insert.bindText(2, config.vio.vio_camera_id);
        }
        insert.bindInt64(3, static_cast<sqlite3_int64>(config.vio.vio_slot_index));
        insert.bindInt(4, config.vio.ir_led_enabled ? 1 : 0);
        insert.bindInt64(5,
            static_cast<sqlite3_int64>(config.vio.ir_led_pulse_width_us));
        insert.bindInt(6, config.vio.tof_enabled ? 1 : 0);
        insert.bindInt64(7,
            static_cast<sqlite3_int64>(config.vio.tof_i2c_address));
        insert.bindInt64(8,
            static_cast<sqlite3_int64>(config.vio.tof_timing_budget_ms));
        insert.bindInt64(9,
            static_cast<sqlite3_int64>(config.vio.tof_intermeasurement_period_ms));
        insert.bindInt64(10,
            static_cast<sqlite3_int64>(config.vio.tof_offset_after_flash_us));
        insert.bindInt64(11,
            static_cast<sqlite3_int64>(config.vio.tof_divisor));
        // Round-trip in mm to keep the SQL column an integer.
        insert.bindInt64(12,
            static_cast<sqlite3_int64>(std::llround(config.vio.tof_mounting_offset_m * 1000.0)));
        insert.bindInt64(13,
            static_cast<sqlite3_int64>(std::llround(config.vio.tof_expected_min_m * 1000.0)));
        insert.bindInt64(14,
            static_cast<sqlite3_int64>(std::llround(config.vio.tof_expected_max_m * 1000.0)));
        insert.stepDone();
    }

    {
        // 57 columns total (excluding id). The VALUES placeholder block is
        // grouped six-per-line below to keep the count visually verifiable.
        Statement insert(
            db_,
            "INSERT INTO fusion_config "
            "(id, "
            "chassis_sigma_rx, chassis_sigma_ry, chassis_sigma_rz, "
            "chassis_sigma_tx, chassis_sigma_ty, chassis_sigma_tz, "
            "origin_prior_sigma_rx, origin_prior_sigma_ry, origin_prior_sigma_rz, "
            "origin_prior_sigma_tx, origin_prior_sigma_ty, origin_prior_sigma_tz, "
            "shock_threshold_mps2, freefall_threshold_mps2, shock_inflation_factor, "
            "imu_window_seconds, max_chassis_dt_seconds, "
            "gravity_local_x, gravity_local_y, gravity_local_z, "
            "enable_vio, vio_sigma_rx, vio_sigma_ry, vio_sigma_rz, "
            "vio_sigma_tx, vio_sigma_ty, vio_sigma_tz, "
            "huber_k, enable_imu_preintegration, "
            "imu_extrinsic_tx_m, imu_extrinsic_ty_m, imu_extrinsic_tz_m, "
            "imu_extrinsic_roll_rad, imu_extrinsic_pitch_rad, imu_extrinsic_yaw_rad, "
            "accel_noise_sigma, gyro_noise_sigma, "
            "accel_bias_rw_sigma, gyro_bias_rw_sigma, integration_cov_sigma, "
            "persisted_bias_ax, persisted_bias_ay, persisted_bias_az, "
            "persisted_bias_gx, persisted_bias_gy, persisted_bias_gz, "
            "bias_calibration_seconds, bias_calibration_chassis_threshold, "
            "max_keyframe_dt_seconds, max_imu_gap_seconds, "
            "marginalize_keyframe_window, slip_disagreement_mps, "
            "enable_floor_constraint, "
            "floor_constraint_sigma_z, floor_constraint_sigma_roll, "
            "floor_constraint_sigma_pitch, max_chassis_speed_mps) "
            "VALUES (1, "
            "?, ?, ?, ?, ?, ?, "  // chassis_sigmas
            "?, ?, ?, ?, ?, ?, "  // origin_prior_sigmas
            "?, ?, ?, "           // shock/freefall/inflation
            "?, ?, "              // imu_window/max_chassis_dt
            "?, ?, ?, "           // gravity_local
            "?, ?, ?, ?, ?, ?, ?, "  // enable_vio + 6 vio sigmas
            "?, ?, "              // huber_k + enable_imu_preintegration
            "?, ?, ?, ?, ?, ?, "  // imu_extrinsic
            "?, ?, ?, ?, ?, "     // accel/gyro/bias_rw/integration_cov
            "?, ?, ?, ?, ?, ?, "  // persisted_bias
            "?, ?, "              // bias_calibration_seconds + threshold
            "?, ?, "              // max_keyframe_dt + max_imu_gap
            "?, ?, "              // marginalize_window + slip_disagreement
            "?, "                 // enable_floor_constraint
            "?, ?, ?, "           // floor_constraint_sigmas
            "?)");                // max_chassis_speed_mps
        int idx = 1;
        for (int i = 0; i < 6; ++i) {
            insert.bindDouble(idx++,
                config.fusion.chassis_sigmas[static_cast<std::size_t>(i)]);
        }
        for (int i = 0; i < 6; ++i) {
            insert.bindDouble(idx++,
                config.fusion.origin_prior_sigmas[static_cast<std::size_t>(i)]);
        }
        insert.bindDouble(idx++, config.fusion.shock_threshold_mps2);
        insert.bindDouble(idx++, config.fusion.freefall_threshold_mps2);
        insert.bindDouble(idx++, config.fusion.shock_inflation_factor);
        insert.bindDouble(idx++, config.fusion.imu_window_seconds);
        insert.bindDouble(idx++, config.fusion.max_chassis_dt_seconds);
        insert.bindDouble(idx++, config.fusion.gravity_local_mps2.x);
        insert.bindDouble(idx++, config.fusion.gravity_local_mps2.y);
        insert.bindDouble(idx++, config.fusion.gravity_local_mps2.z);
        insert.bindInt(idx++, config.fusion.enable_vio ? 1 : 0);
        for (int i = 0; i < 6; ++i) {
            insert.bindDouble(idx++,
                config.fusion.vio_default_sigmas[static_cast<std::size_t>(i)]);
        }
        insert.bindDouble(idx++, config.fusion.huber_k);
        insert.bindInt(idx++, config.fusion.enable_imu_preintegration ? 1 : 0);
        insert.bindDouble(idx++, config.fusion.imu_extrinsic_body_to_imu.translation_m.x);
        insert.bindDouble(idx++, config.fusion.imu_extrinsic_body_to_imu.translation_m.y);
        insert.bindDouble(idx++, config.fusion.imu_extrinsic_body_to_imu.translation_m.z);
        insert.bindDouble(idx++, config.fusion.imu_extrinsic_body_to_imu.rotation_rpy_rad.x);
        insert.bindDouble(idx++, config.fusion.imu_extrinsic_body_to_imu.rotation_rpy_rad.y);
        insert.bindDouble(idx++, config.fusion.imu_extrinsic_body_to_imu.rotation_rpy_rad.z);
        insert.bindDouble(idx++, config.fusion.accel_noise_sigma);
        insert.bindDouble(idx++, config.fusion.gyro_noise_sigma);
        insert.bindDouble(idx++, config.fusion.accel_bias_rw_sigma);
        insert.bindDouble(idx++, config.fusion.gyro_bias_rw_sigma);
        insert.bindDouble(idx++, config.fusion.integration_cov_sigma);
        for (int i = 0; i < 6; ++i) {
            insert.bindDouble(idx++,
                config.fusion.persisted_bias[static_cast<std::size_t>(i)]);
        }
        insert.bindDouble(idx++, config.fusion.bias_calibration_seconds);
        insert.bindDouble(idx++, config.fusion.bias_calibration_chassis_threshold);
        insert.bindDouble(idx++, config.fusion.max_keyframe_dt_seconds);
        insert.bindDouble(idx++, config.fusion.max_imu_gap_seconds);
        insert.bindInt64(idx++,
            static_cast<sqlite3_int64>(config.fusion.marginalize_keyframe_window));
        insert.bindDouble(idx++, config.fusion.slip_disagreement_mps);
        insert.bindInt(idx++, config.fusion.enable_floor_constraint ? 1 : 0);
        for (std::size_t i = 0; i < 3; ++i) {
            insert.bindDouble(idx++, config.fusion.floor_constraint_sigmas[i]);
        }
        insert.bindDouble(idx++, config.fusion.max_chassis_speed_mps);
        insert.stepDone();
    }

    tx.commit();
}

}  // namespace posest::config
