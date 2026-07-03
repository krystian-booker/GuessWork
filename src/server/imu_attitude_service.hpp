#pragma once

#include <cstdint>
#include <memory>

#include "core/attitude_filter.hpp"

namespace gw::server {

class TeensyManager;

// Drains the live ImuBus into an AttitudeFilter for the web UI's 3D IMU
// preview (`GET /api/imu/attitude`). Visualization-only — nothing in the
// estimation pipeline consumes this.
class ImuAttitudeService {
public:
    struct Status {
        gw::AttitudeFilter::Snapshot attitude;
        double                       rate_hz = 0.0;  // measured over ~1 s
        // Latest raw sample, for the UI's numeric readout.
        float accel[3] = {0, 0, 0};
        float gyro[3]  = {0, 0, 0};
        std::int64_t last_age_ms = -1;  // -1 = never seen
    };

    explicit ImuAttitudeService(TeensyManager& teensy);
    ~ImuAttitudeService();

    ImuAttitudeService(const ImuAttitudeService&)            = delete;
    ImuAttitudeService& operator=(const ImuAttitudeService&) = delete;

    Status status() const;
    void   zero_yaw();
    void   reset();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace gw::server
