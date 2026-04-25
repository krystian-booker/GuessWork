#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "posest/CameraCapabilities.h"
#include "posest/MeasurementTypes.h"

namespace posest::runtime {

struct TelemetrySnapshot {
    std::uint64_t measurements_dropped{0};
    std::uint64_t measurements_processed{0};
    std::uint64_t stale_measurements{0};
    std::optional<FusedPoseEstimate> latest_pose;
    std::vector<CameraLiveStats> cameras;
};

}  // namespace posest::runtime
