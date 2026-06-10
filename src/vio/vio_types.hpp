#pragma once

#include <array>
#include <cstdint>

#include "apriltag/pose_math.hpp"
#include "core/measurement_bus.hpp"

namespace gw::vio {

// One VIO update published on the VioBus.
//
// Fusion contract (Phase 6): the odom frame is OpenVINS's gravity-aligned
// global frame for the CURRENT epoch — its origin resets on every VIO
// (re)initialization, and `epoch` increments each time. Consumers must fuse
// only relative deltas ΔT = T[k−1]⁻¹ · T[k] between consecutive samples
// with EQUAL epoch; never difference across epochs, and never fuse an
// absolute VIO pose (VIO drifts).
struct VioOdometry {
    int64_t  t_ns  = 0;        // Teensy clock — the stereo pair's pulse stamp
    uint64_t epoch = 0;        // odom-frame session; ++ on every (re)init
    bool     initialized = false;

    // IMU pose in the epoch's odom frame.
    gw::apriltag::Mat4 T_odom_imu = gw::apriltag::mat4_identity();

    // Row-major 6×6 covariance in the body tangent [ωx ωy ωz, tx ty tz]
    // with RIGHT perturbation T_true = T_est · Exp(ξ) — the same convention
    // as TagPoseMeasurement::cov, so Phase 6 ingests both uniformly.
    std::array<double, 36> cov{};

    uint32_t tracked_features = 0;  // good MSCKF features in this update
};

using VioBus = gw::MeasurementBus<VioOdometry>;

}  // namespace gw::vio
