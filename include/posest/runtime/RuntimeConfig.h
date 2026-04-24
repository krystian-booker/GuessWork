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
    std::uint32_t fused_pose_can_id{0};
    std::uint32_t status_can_id{0};
    double pose_publish_hz{50.0};
};

struct RuntimeConfig {
    std::vector<CameraConfig> cameras;
    std::vector<PipelineConfig> pipelines;
    std::vector<CameraPipelineBinding> bindings;
    std::vector<CalibrationConfig> calibrations;
    TeensyConfig teensy;
};

}  // namespace posest::runtime
