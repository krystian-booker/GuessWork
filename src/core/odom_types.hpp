#pragma once

#include <cstdint>

#include "core/measurement_bus.hpp"

namespace gw {

// One chassis-speeds sample from the robot controller, decoded from the
// RobotLink's UDP telemetry (docs/ethernet-protocol.md). Robot-frame
// velocities come from drivetrain forward kinematics; GuessWork never sees
// wheel/module math.
//
// t_ns is the best-estimate SAMPLE time on the sync controller clock: when the RIO
// timestamp is known and RioClockSync is healthy it is the mapped
// rio_time_us, otherwise it falls back to the mapped UDP arrival time.
// Same time domain as ImuSample::t_ns / Frame::camera_ts_ns.
struct ChassisSpeeds {
    uint64_t t_ns         = 0;
    uint64_t t_arrival_ns = 0;  // sync-controller clock at UDP receive
    uint64_t rio_time_us  = 0;  // controller FPGA µs at sampling; 0 = unknown
    float    vx_mps       = 0.0f;  // robot +X (forward)
    float    vy_mps       = 0.0f;  // robot +Y (left); nonzero only on holonomic drives
    float    omega_radps  = 0.0f;  // yaw rate, CCW positive
    uint16_t status_flags = 0;     // bit0 stale encoder data, bit1 wheel slip
    uint8_t  counter      = 0;     // rolling, +1 per controller sample
};

// Fused field pose for the robot-controller UDP downlink.
struct FusedPose {
    uint64_t t_ns      = 0;  // sync controller clock, pose validity time
    float    x_m       = 0.0f;  // WPILib field frame
    float    y_m       = 0.0f;
    float    theta_rad = 0.0f;
    uint8_t  quality   = 0;  // 0–255; semantics defined in Phase 6
};

using OdomBus = MeasurementBus<ChassisSpeeds>;

}  // namespace gw
