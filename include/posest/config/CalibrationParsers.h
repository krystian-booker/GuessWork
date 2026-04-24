#pragma once

#include <filesystem>
#include <optional>
#include <string>

#include "posest/MeasurementTypes.h"
#include "posest/runtime/RuntimeConfig.h"

namespace posest::config {

runtime::CameraCalibrationConfig parseKalibrCameraCalibration(
    const std::filesystem::path& path,
    const std::string& camera_id,
    const std::string& version,
    bool active,
    const std::string& created_at,
    const std::optional<std::string>& topic = std::nullopt);

runtime::FieldLayoutConfig parseWpilibFieldLayout(
    const std::filesystem::path& path,
    std::string id,
    std::string name);

Pose3d parsePoseCsv(const std::string& value);

}  // namespace posest::config
