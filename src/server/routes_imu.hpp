#pragma once

#include <crow.h>

namespace gw::server {

class ApriltagSupervisor;
class FusionSupervisor;
class ImuConfigRepository;
class TeensyManager;

// /api/imu/config  GET, PUT  — single-row IMU noise/transform configuration.
// /api/imu/status  GET       — live telemetry health (rate, drops, CRC).
//
// PUT validates t_imu_robot through gw::calib::parse_t_robot_imu, pushes a
// fresh shared config into the AprilTag pipeline, and reloads the fusion
// supervisor (T_robot_imu gates its VIO ingestion) on success.
void register_imu_routes(crow::SimpleApp&     app,
                         ImuConfigRepository& imu_config,
                         TeensyManager&       teensy,
                         ApriltagSupervisor&  apriltag,
                         FusionSupervisor&    fusion);

}  // namespace gw::server
