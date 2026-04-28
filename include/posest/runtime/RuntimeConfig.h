#pragma once

#include <array>
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

// Calibration target description (e.g. AprilGrid, checkerboard, circle grid).
// Persisted in SQLite so the operator selects a target by id when starting a
// run; the daemon then materializes a Kalibr-shaped target.yaml on demand.
struct CalibrationTargetConfig {
    std::string id;
    // Allowed: "aprilgrid", "checkerboard", "circlegrid".
    std::string type;
    int rows{0};
    int cols{0};
    // AprilGrid: edge length of one tag, in meters.
    double tag_size_m{0.0};
    // AprilGrid: spacing-to-size ratio (Kalibr's tagSpacing).
    double tag_spacing_ratio{0.0};
    // Checkerboard / circlegrid: square or circle spacing, in meters.
    double square_size_m{0.0};
    // AprilGrid family. Defaults to tag36h11 to match Kalibr's bundled board.
    std::string tag_family{"tag36h11"};
    std::string notes;
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

// GTSAM fusion-graph tunables. Persisted as a singleton row; all fields have
// compile-time defaults so an unmigrated DB still produces a runnable config.
// Ordering of the six-element sigma arrays matches gtsam::Pose3 tangent
// [rx, ry, rz, tx, ty, tz]. See docs/features/fusion-service.md §3 and §6.
struct FusionConfig {
    // BetweenFactor process noise per Δt of integrated chassis speeds.
    std::array<double, 6> chassis_sigmas{0.05, 0.05, 0.05, 0.02, 0.02, 0.02};
    // Bootstrap PriorFactor sigmas at x_0 (very loose by design).
    std::array<double, 6> origin_prior_sigmas{10.0, 10.0, 3.14, 10.0, 10.0, 10.0};

    // Shock-detection thresholds (IMU-driven sigma inflation, not state).
    double shock_threshold_mps2{50.0};
    double freefall_threshold_mps2{3.0};
    double shock_inflation_factor{100.0};
    // Sliding window over which IMU samples are scanned for shock/free-fall.
    double imu_window_seconds{0.05};
    // ChassisSpeeds samples spaced wider than this skip the BetweenFactor.
    double max_chassis_dt_seconds{0.5};
    // Body-frame gravity vector. Subtracted from accel before shock test;
    // override for tilted IMU mounts. Distinct from the nav-frame gravity used
    // by IMU preintegration (Phase C builds that from g = 9.80665 directly).
    Vec3 gravity_local_mps2{0.0, 0.0, 9.80665};

    // VioMeasurement → BetweenFactor<Pose3> ingestion. Mirrors the fields on
    // posest::fusion::FusionConfig; persisting them lets the website turn VIO
    // on/off without a recompile when the real frontend lands.
    bool enable_vio{false};
    std::array<double, 6> vio_default_sigmas{0.05, 0.05, 0.05, 0.02, 0.02, 0.02};

    // Phase B: Huber kernel constant on the chassis BetweenFactor (in
    // standardized residual units). Lower values downweight legitimate hard
    // accelerations; higher values let outright slip through.
    double huber_k{1.5};

    // Phase C: IMU preintegration. Feature-flagged off until Phase C ships and
    // is validated end-to-end. All fields below are inert when the flag is
    // false and the existing pose-only graph keeps running.
    bool enable_imu_preintegration{false};
    // Static body-to-IMU extrinsic. Identity is correct for an IMU mounted
    // axis-aligned with the robot frame.
    Pose3d imu_extrinsic_body_to_imu;
    // Continuous-time noise densities (per √Hz) and bias random-walk sigmas
    // for PreintegrationParams. Defaults are typical for a BMI270/ICM-42688.
    double accel_noise_sigma{0.05};
    double gyro_noise_sigma{0.001};
    double accel_bias_rw_sigma{1e-4};
    double gyro_bias_rw_sigma{1e-5};
    double integration_cov_sigma{1e-8};
    // Persisted IMU bias seed [ax, ay, az, gx, gy, gz] across power cycles.
    // Updated by the boot stationary-calibration loop when one runs.
    std::array<double, 6> persisted_bias{};
    double bias_calibration_seconds{1.5};
    // Stationary detector: chassis must read |v| and |ω| below this for the
    // entire calibration window for the bias mean to be accepted.
    double bias_calibration_chassis_threshold{0.02};
    // Keyframe interval: a new pose+velocity key is committed when either a
    // vision arrives or this much time has elapsed since the last keyframe.
    double max_keyframe_dt_seconds{0.020};
    // IMU gap that triggers a preintegrator reset. Beyond this, integration
    // accumulates too much error to commit a useful ImuFactor.
    double max_imu_gap_seconds{0.100};
    // Sliding-window length for ISAM2 marginalization (in keyframe count).
    std::uint32_t marginalize_keyframe_window{500};
    // Threshold for the chassis-vs-IMU velocity disagreement slip detector.
    double slip_disagreement_mps{1.0};

    // F-2: soft floor-constraint prior pinning z / roll / pitch toward zero
    // on every new pose key. Encodes the "robot is always grounded" platform
    // assumption directly in the graph so vision priors and (Phase C) IMU
    // factors cannot drift the unobserved-by-chassis DOFs. Order is
    // [σ_z (m), σ_roll (rad), σ_pitch (rad)].
    bool enable_floor_constraint{true};
    std::array<double, 3> floor_constraint_sigmas{0.01, 0.0087, 0.0087};

    // F-3: hard upper bound on chassis speed before the sample is dropped
    // and kFusionStatusDegradedInput is OR'd into the next estimate. Default
    // is the platform's 5.2 m/s spec with a 25 % margin.
    double max_chassis_speed_mps{6.5};
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
    std::vector<CalibrationTargetConfig> calibration_targets;
    std::vector<FieldLayoutConfig> field_layouts;
    std::string active_field_layout_id;
    std::vector<CameraTriggerConfig> camera_triggers;
    TeensyConfig teensy;
    VioConfig vio;
    FusionConfig fusion;
};

}  // namespace posest::runtime
