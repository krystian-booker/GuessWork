#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "apriltag/pose_math.hpp"
#include "core/measurement_bus.hpp"

namespace gw::apriltag {

// One field-frame robot-pose measurement derived from AprilTag detections
// in a single camera frame. Published on the TagPoseBus and consumed by the
// fusion layer (Phase 6).
struct TagPoseMeasurement {
    // Frame timestamp. kTeensy = camera_ts_ns (the hardware-sync pulse clock
    // shared with the IMU); kHost = mach_absolute_time fallback when no
    // pulse matched — fusion should reject these when mixing with
    // Teensy-clock sources.
    enum class Clock : uint8_t { kTeensy, kHost };
    int64_t t_ns         = 0;
    Clock   clock_source = Clock::kHost;

    int64_t camera_id = 0;

    Mat4 T_field_robot = mat4_identity();

    // Row-major 6×6 covariance in the tangent order [ωx ωy ωz, tx ty tz]
    // with RIGHT (body-frame) perturbation: T_true = T_est · Exp(ξ).
    // This is exactly GTSAM Pose3's retract convention.
    std::array<double, 36> cov{};

    uint32_t             n_tags             = 0;
    double               mean_reproj_err_px = 0.0;
    std::vector<int32_t> tag_ids;
};

using TagPoseBus = gw::MeasurementBus<TagPoseMeasurement>;

}  // namespace gw::apriltag
