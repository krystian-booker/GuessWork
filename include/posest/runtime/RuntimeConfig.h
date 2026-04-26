#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "posest/CameraConfig.h"
#include "posest/MeasurementTypes.h"
#include "posest/runtime/PipelineConfig.h"

namespace posest::runtime {

struct CameraCalibrationConfig {
    std::string camera_id;
    std::string version;
    bool active{true};
    std::string source_file_path;
    std::string created_at;
    int image_width{0};
    int image_height{0};
    std::string camera_model;
    std::string distortion_model;
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    std::vector<double> distortion_coefficients;
};

struct CameraExtrinsicsConfig {
    std::string camera_id;
    std::string version;
    Pose3d camera_to_robot;
};

struct CameraImuCalibrationConfig {
    std::string camera_id;
    std::string version;
    bool active{true};
    std::string source_file_path;
    std::string created_at;
    Pose3d camera_to_imu;
    Pose3d imu_to_camera;
    double time_shift_s{0.0};
};

struct KalibrDatasetConfig {
    std::string id;
    std::string path;
    std::string created_at;
    double duration_s{0.0};
    std::vector<std::string> camera_ids;
};

struct CalibrationToolConfig {
    std::string docker_image{"kalibr:latest"};
};

struct FieldTagConfig {
    int tag_id{0};
    Pose3d field_to_tag;
};

struct FieldLayoutConfig {
    std::string id;
    std::string name;
    std::string source_file_path;
    double field_length_m{0.0};
    double field_width_m{0.0};
    std::vector<FieldTagConfig> tags;
};

struct ImuConfig {
    std::uint32_t accel_range_g{24};
    std::uint32_t accel_odr_hz{1000};
    std::uint32_t accel_bandwidth_code{2};
    std::uint32_t gyro_range_dps{2000};
    std::uint32_t gyro_bandwidth_code{2};
    std::uint32_t data_sync_rate_hz{1000};
    bool run_selftest_on_boot{true};
};

struct CanBusConfig {
    bool enabled{false};
    std::uint32_t nominal_bitrate_bps{1'000'000};
    std::uint32_t data_bitrate_bps{2'000'000};
    std::uint32_t pose_publish_hz{100};
    std::uint32_t rio_offset_stale_ms{1000};
    // Placeholder arbitration IDs documented in firmware/teensy41/README.md.
    std::uint32_t rio_pose_can_id{0x100};
    std::uint32_t rio_time_sync_can_id{0x101};
    std::uint32_t teensy_pose_can_id{0x180};
};

struct TeensyConfig {
    std::string serial_port;
    std::uint32_t baud_rate{921600};
    std::uint32_t reconnect_interval_ms{1000};
    std::uint32_t read_timeout_ms{20};
    std::uint32_t fused_pose_can_id{0};
    std::uint32_t status_can_id{0};
    double pose_publish_hz{50.0};
    std::uint32_t time_sync_interval_ms{1000};
    ImuConfig imu;
    CanBusConfig can;
};

struct CameraTriggerConfig {
    std::string camera_id;
    bool enabled{true};
    std::int32_t teensy_pin{-1};
    double rate_hz{0.0};
    std::uint32_t pulse_width_us{1000};
    std::int64_t phase_offset_us{0};
};

// Single-camera VIO workflow configuration. Bundles IR LED illumination
// control and VL53L4CD ToF settings; the firmware enforces temporal
// multiplexing (no LED-on / ToF-ranging overlap) via the validator constraint
// tof_offset_after_flash_us >= ir_led_pulse_width_us. The IR LED MOSFET gate
// pin is a firmware constant — not host-configurable.
struct VioConfig {
    bool enabled{false};
    std::string vio_camera_id;
    std::int32_t vio_slot_index{0};
    bool ir_led_enabled{true};
    std::uint32_t ir_led_pulse_width_us{400};
    bool tof_enabled{true};
    std::uint32_t tof_i2c_address{0x29};
    std::uint32_t tof_timing_budget_ms{10};
    std::uint32_t tof_intermeasurement_period_ms{20};
    std::uint32_t tof_offset_after_flash_us{500};
    std::uint32_t tof_divisor{1};
    // Host-side mounting offset added to the raw ToF distance before any
    // consumer sees it. Stored on the wire as integer mm but materialized
    // here in meters to match the rest of the SI-domain measurement types.
    double tof_mounting_offset_m{0.0};
    double tof_expected_min_m{0.05};
    double tof_expected_max_m{4.0};
};

struct RuntimeConfig {
    std::vector<CameraConfig> cameras;
    std::vector<PipelineConfig> pipelines;
    std::vector<CameraPipelineBinding> bindings;
    std::vector<CameraCalibrationConfig> calibrations;
    std::vector<CameraExtrinsicsConfig> camera_extrinsics;
    std::vector<CameraImuCalibrationConfig> camera_imu_calibrations;
    std::vector<KalibrDatasetConfig> kalibr_datasets;
    CalibrationToolConfig calibration_tools;
    std::vector<FieldLayoutConfig> field_layouts;
    std::string active_field_layout_id;
    std::vector<CameraTriggerConfig> camera_triggers;
    TeensyConfig teensy;
    VioConfig vio;
};

}  // namespace posest::runtime
