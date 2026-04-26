#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "posest/Timestamp.h"

namespace posest {

struct Vec2 {
    double x{0.0};
    double y{0.0};
};

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct Pose2d {
    double x_m{0.0};
    double y_m{0.0};
    double theta_rad{0.0};
};

struct Pose3d {
    Vec3 translation_m;
    Vec3 rotation_rpy_rad;
};

struct AprilTagDetection {
    int tag_id{0};
    std::array<Vec2, 4> image_corners_px{};
    std::optional<Pose3d> camera_to_tag;
    double ambiguity{0.0};
    double reprojection_error_px{0.0};
};

struct AprilTagObservation {
    std::string camera_id;
    std::uint64_t frame_sequence{0};
    Timestamp capture_time{};
    // Teensy-stamped shutter time and trigger pulse sequence when a matching
    // CameraTriggerEvent was paired with the source Frame; nullopt otherwise.
    std::optional<std::uint64_t> teensy_time_us;
    std::optional<std::uint32_t> trigger_sequence;
    std::vector<AprilTagDetection> detections;

    // Aggregated PnP solution. nullopt when no field-known tags were
    // detected, the solver failed, or single-tag ambiguity exceeded the
    // configured threshold. When set, the covariance below is meaningful and
    // uses gtsam Pose3 tangent order: [rx, ry, rz, tx, ty, tz], 6x6 row-major.
    std::optional<Pose3d> field_to_robot;
    std::array<double, 36> covariance{};
    double reprojection_rms_px{0.0};
    int solved_tag_count{0};
};

struct VioMeasurement {
    std::string camera_id;
    Timestamp timestamp{};
    Pose3d relative_motion;
    std::array<double, 36> covariance{};
    bool tracking_ok{false};
    std::string backend_status;
};

struct ImuSample {
    Timestamp timestamp{};
    Vec3 accel_mps2;
    Vec3 gyro_radps;
    std::optional<double> temperature_c;
    std::uint32_t status_flags{0};
};

// Bit positions in ChassisSpeedsSample::status_flags. The RIO owns this byte;
// these constants are the host-side mirror of the agreed bit layout. A new
// flag must be added on both sides in lockstep.
inline constexpr std::uint32_t kChassisStatusSlip = 1u << 0u;

struct ChassisSpeedsSample {
    Timestamp timestamp{};
    double vx_mps{0.0};
    double vy_mps{0.0};
    double omega_radps{0.0};
    std::uint64_t rio_time_us{0};
    // Bitfield owned by the RIO; see kChassisStatus* constants above. Any
    // non-zero value flags the sample as kFusionStatusDegradedInput; specific
    // bits (e.g. kChassisStatusSlip) trigger targeted noise inflation.
    std::uint32_t status_flags{0};
};

struct CameraTriggerEvent {
    Timestamp timestamp{};
    std::uint64_t teensy_time_us{0};
    std::int32_t pin{-1};
    std::uint32_t trigger_sequence{0};
    std::uint32_t status_flags{0};
};

// VL53L4CD ranging result, time-aligned to the camera trigger that drove the
// IR LED flash that immediately preceded this ranging. distance_m has the
// host-side mounting offset applied; raw_distance_m is the unaltered chip
// reading converted from mm.
struct ToFSample {
    Timestamp timestamp{};
    std::uint64_t teensy_time_us{0};
    std::uint32_t trigger_sequence{0};
    double raw_distance_m{0.0};
    double distance_m{0.0};
    double signal_rate_kcps{0.0};
    double ambient_rate_kcps{0.0};
    std::uint32_t ranging_duration_us{0};
    std::uint32_t range_status{0};
    std::uint32_t status_flags{0};
};

struct FusedPoseEstimate {
    Timestamp timestamp{};
    Pose2d field_to_robot;
    std::optional<Vec3> velocity;
    std::array<double, 36> covariance{};
    std::uint32_t status_flags{0};
};

}  // namespace posest
