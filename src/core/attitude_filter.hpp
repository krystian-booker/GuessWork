#pragma once

#include <array>
#include <cstdint>

#include "core/imu_types.hpp"

namespace gw {

// Complementary attitude filter for the IMU 3D preview: integrates gyro
// rates and bleeds roll/pitch toward the accelerometer's gravity vector.
// Pure — no clocks, no threads; the owner serializes feed()/snapshot().
//
// This is a VISUALIZATION aid (does the physical IMU move the way the
// pipeline thinks it does?), not an estimation input — OpenVINS and the
// fusion engine never see it. Yaw is unobservable without a magnetometer
// and drifts at the gyro bias rate; zero_yaw() re-references it.
//
// Conventions: world frame Z-up; q is the body→world rotation (Hamilton,
// w-first). A static, level IMU reads accel ≈ (0, 0, +9.81) body-frame
// (specific force opposes gravity), so the filter pulls body +Z toward
// world +Z.
class AttitudeFilter {
public:
    struct Snapshot {
        bool                  initialized = false;
        std::array<double, 4> q{1, 0, 0, 0};  // w, x, y, z (body→world)
        double                roll_deg  = 0;  // ZYX euler of q
        double                pitch_deg = 0;
        double                yaw_deg   = 0;
        uint64_t              last_t_ns = 0;  // sync controller clock of last sample
        uint64_t              samples   = 0;
    };

    void feed(const ImuSample& s);

    // Rotate about world Z so the current yaw reads 0 (roll/pitch keep
    // their gravity-referenced values).
    void zero_yaw();

    void reset();

    Snapshot snapshot() const;

private:
    // Accel correction time constant ~1 s (gain · dt per sample); gated to
    // samples whose magnitude is near 1 g so dynamics don't tilt the
    // estimate.
    static constexpr double kAccelGain    = 1.0;   // 1/s
    static constexpr double kGravity      = 9.80665;
    static constexpr double kAccelGateLo  = 0.5 * kGravity;
    static constexpr double kAccelGateHi  = 1.5 * kGravity;
    static constexpr double kMaxStepSec   = 0.05;  // clamp dt across gaps

    std::array<double, 4> q_{1, 0, 0, 0};
    bool                  initialized_ = false;
    uint64_t              last_t_ns_   = 0;
    uint64_t              samples_     = 0;
};

}  // namespace gw
