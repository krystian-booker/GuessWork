#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace gw::server {

class Database;

// The single imu_config row. Noise terms are Kalibr continuous-time units:
//   accel_noise_density  m/s²/√Hz     accel_random_walk  m/s³/√Hz
//   gyro_noise_density   rad/s/√Hz    gyro_random_walk   rad/s²/√Hz
// t_imu_robot_json is an opaque JSON document (CAD-derived IMU→robot
// transform); the calibration layer interprets it.
struct ImuConfig {
    double                     rate_hz             = 400.0;
    double                     accel_noise_density = 0.0;
    double                     accel_random_walk   = 0.0;
    double                     gyro_noise_density  = 0.0;
    double                     gyro_random_walk    = 0.0;
    std::optional<std::string> t_imu_robot_json;
    int64_t                    updated_at          = 0;  // unix seconds
};

// Partial update. nullopt fields stay unchanged. t_imu_robot_json uses a
// double-optional so a present-but-null outer value clears the column.
struct ImuConfigUpdate {
    std::optional<double>                     rate_hz;
    std::optional<double>                     accel_noise_density;
    std::optional<double>                     accel_random_walk;
    std::optional<double>                     gyro_noise_density;
    std::optional<double>                     gyro_random_walk;
    std::optional<std::optional<std::string>> t_imu_robot_json;

    bool empty() const {
        return !rate_hz && !accel_noise_density && !accel_random_walk &&
               !gyro_noise_density && !gyro_random_walk && !t_imu_robot_json;
    }
};

// Accessor for the single-row imu_config table (seeded at schema creation,
// so get() always succeeds on a healthy database).
class ImuConfigRepository {
public:
    explicit ImuConfigRepository(Database& db) : db_(db) {}

    ImuConfig get();
    ImuConfig update(const ImuConfigUpdate& patch);

private:
    Database& db_;
};

}  // namespace gw::server
