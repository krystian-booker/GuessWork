#pragma once

#include <filesystem>

#include "posest/runtime/RuntimeConfig.h"

namespace posest::calibration {

// Materialize a Kalibr-shaped target.yaml from a CalibrationTargetConfig.
// Output format follows Kalibr's calibration target conventions for the
// supported types: aprilgrid, checkerboard, circlegrid. Throws
// std::invalid_argument if the config is malformed (e.g. unsupported type).
void writeKalibrTargetYaml(
    const runtime::CalibrationTargetConfig& target,
    const std::filesystem::path& path);

}  // namespace posest::calibration
