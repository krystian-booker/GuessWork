#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "posest/CameraConfig.h"
#include "posest/runtime/PipelineConfig.h"

namespace posest::runtime {

struct CalibrationConfig {
    std::string camera_id;
    std::string file_path;
    std::string version;
    std::string created_at;
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
    std::vector<CalibrationConfig> calibrations;
    std::vector<CameraTriggerConfig> camera_triggers;
    TeensyConfig teensy;
};

}  // namespace posest::runtime
