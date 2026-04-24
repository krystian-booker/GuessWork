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

struct TeensyConfig {
    std::string serial_port;
    std::uint32_t baud_rate{115200};
    std::uint32_t reconnect_interval_ms{1000};
    std::uint32_t read_timeout_ms{20};
    std::uint32_t fused_pose_can_id{0};
    std::uint32_t status_can_id{0};
    double pose_publish_hz{50.0};
};

struct CameraTriggerConfig {
    std::string camera_id;
    bool enabled{true};
    std::int32_t teensy_pin{-1};
    double rate_hz{0.0};
    std::uint32_t pulse_width_us{1000};
    std::int64_t phase_offset_us{0};
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
};

}  // namespace posest::runtime
