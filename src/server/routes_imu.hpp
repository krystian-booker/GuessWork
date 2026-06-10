#pragma once

#include <crow.h>

namespace gw::server {

class ApriltagSupervisor;
class ImuConfigRepository;
class TeensyManager;

// /api/imu/config  GET, PUT  — single-row IMU noise/transform configuration.
// /api/imu/status  GET       — live telemetry health (rate, drops, CRC).
//
// PUT validates t_imu_robot through gw::calib::parse_t_robot_imu and pushes
// a fresh shared config into the AprilTag pipeline on success.
void register_imu_routes(crow::SimpleApp&     app,
                         ImuConfigRepository& imu_config,
                         TeensyManager&       teensy,
                         ApriltagSupervisor&  apriltag);

}  // namespace gw::server
