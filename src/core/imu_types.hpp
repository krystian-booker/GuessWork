#pragma once

#include <cstdint>

namespace gw {

// One BMI088 sample as decoded from the sync controller's binary telemetry stream.
//
// t_ns is on the sync controller clock (wrap-extended micros × 1000) — the same time
// domain as hardware-synced Frame::camera_ts_ns, so IMU samples and camera
// frames are directly comparable with no host-side clock translation.
struct ImuSample {
    uint64_t t_ns  = 0;
    float    accel[3] = {0, 0, 0};  // m/s²
    float    gyro[3]  = {0, 0, 0};  // rad/s
};

}  // namespace gw
