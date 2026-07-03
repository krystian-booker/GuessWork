#pragma once

#include <crow.h>

namespace gw::server {

class ApriltagSupervisor;
class FusionSupervisor;
class ImuAllanService;
class ImuAttitudeService;
class ImuConfigRepository;
class TeensyManager;

// /api/imu/config  GET, PUT  — single-row IMU noise/transform configuration.
// /api/imu/status  GET       — live telemetry health (rate, drops, CRC).
//
// PUT validates t_imu_robot through gw::calib::parse_t_robot_imu, pushes a
// fresh shared config into the AprilTag pipeline, and reloads the fusion
// supervisor (T_robot_imu gates its VIO ingestion) on success.
//
// Allan-variance refinement (record a long static IMU log → analyze →
// apply suggested noise values):
//   POST   /api/imu/allan/recording {"duration_s":N}  (409 while running)
//   DELETE /api/imu/allan/recording                   (409 when idle)
//   GET    /api/imu/allan/status
//   POST   /api/imu/allan/analyze   {"file"?: basename}
//   POST   /api/imu/allan/apply                       (409 without analysis)
//
// 3D attitude preview (visualization-only complementary filter):
//   GET  /api/imu/attitude          — quaternion + euler + raw sample
//   POST /api/imu/attitude/zero-yaw — re-reference the drifting yaw
void register_imu_routes(crow::SimpleApp&     app,
                         ImuConfigRepository& imu_config,
                         TeensyManager&       teensy,
                         ApriltagSupervisor&  apriltag,
                         FusionSupervisor&    fusion,
                         ImuAllanService&     allan,
                         ImuAttitudeService&  attitude);

}  // namespace gw::server
