#pragma once

#include <crow.h>

namespace gw::server {

class ImuConfigRepository;
class TeensyManager;

// /api/imu/config  GET, PUT  — single-row IMU noise/transform configuration.
// /api/imu/status  GET       — live telemetry health (rate, drops, CRC).
void register_imu_routes(crow::SimpleApp&     app,
                         ImuConfigRepository& imu_config,
                         TeensyManager&       teensy);

}  // namespace gw::server
