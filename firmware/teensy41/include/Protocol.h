#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

namespace posest::firmware {

constexpr std::uint16_t kFrameMagic = 0x4757;
// v4: FusedPose payload prefixed with host_send_time_us (376 B);
// TeensyHealthPayload extended with firmware-measured fused-pose latency
// (76 B). Mirrors include/posest/teensy/Protocol.h.
constexpr std::uint8_t kProtocolVersion = 4;

// Per-sample status flags shared with include/posest/teensy/Protocol.h. See
// scripts/check_protocol_constants.py for the host-vs-firmware drift check.
constexpr std::uint32_t kStatusUnsynchronizedTime = 1u << 0u;
constexpr std::uint32_t kStatusUnsynchronizedRioTime = 1u << 1u;
constexpr std::uint32_t kStatusRobotSlipping = 1u << 2u;

// Bits emitted on TeensyHealthPayload::rio_status_flags. Must match the
// host's kHealthRio* constants.
constexpr std::uint32_t kHealthRioUnsynchronized = 1u << 0u;
constexpr std::uint32_t kHealthRioPingRejected = 1u << 1u;

// Bits set on ToFSamplePayload::firmware_status_flags. Bits 0–7 share
// semantics with the per-sample kStatus* word so consumers can apply the
// same time-sync gating; bits 8+ are ToF-specific observability. The
// host clamp bit (1<<16) is host-only — firmware never sets it.
constexpr std::uint32_t kToFStatusRangingOverrun = 1u << 8u;
constexpr std::uint32_t kToFStatusI2cFailure = 1u << 9u;
constexpr std::uint32_t kToFStatusInvalidRangeStatus = 1u << 10u;
constexpr std::size_t kHeaderSize = 10;
constexpr std::size_t kCrcSize = 4;
constexpr std::size_t kMaxPayloadSize = 4096;
constexpr std::size_t kMaxFrameSize = kHeaderSize + kMaxPayloadSize + kCrcSize;

enum class MessageType : std::uint8_t {
    ImuSample = 1,
    CanRx = 3,
    TeensyHealth = 4,
    TimeSyncResponse = 5,
    ToFSample = 6,
    CameraTriggerEvent = 7,
    ConfigAck = 8,
    ChassisSpeeds = 9,
    FusedPose = 64,
    CanTx = 65,
    TimeSyncRequest = 66,
    ConfigCommand = 67,
};

enum class ConfigCommandKind : std::uint32_t {
    CameraTriggers = 1,
    ImuConfig = 2,
    VioCompanion = 3,
};

constexpr std::uint32_t kConfigAckUnsupportedCount = 1u << 0u;
constexpr std::uint32_t kConfigAckInvalidPin = 1u << 1u;
constexpr std::uint32_t kConfigAckDuplicatePin = 1u << 2u;
constexpr std::uint32_t kConfigAckInvalidRate = 1u << 3u;
constexpr std::uint32_t kConfigAckPulseTooWide = 1u << 4u;
constexpr std::uint32_t kConfigAckInvalidImuConfig = 1u << 5u;
constexpr std::uint32_t kConfigAckImuSelfTestFailure = 1u << 6u;
constexpr std::uint32_t kConfigAckUnknownKind = 1u << 7u;
constexpr std::uint32_t kConfigAckInvalidVioConfig = 1u << 8u;
constexpr std::uint32_t kConfigAckVioMultiplexViolation = 1u << 9u;
constexpr std::uint32_t kConfigAckVioInitFailure = 1u << 10u;

struct Frame {
    MessageType type{MessageType::TeensyHealth};
    std::uint32_t sequence{0};
    std::uint16_t payload_size{0};
    std::uint8_t payload[kMaxPayloadSize]{};
};

struct DecoderStats {
    std::uint64_t crc_failures{0};
    std::uint64_t sequence_gaps{0};
    bool has_last_sequence{false};
    std::uint32_t last_sequence{0};
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

struct ImuPayload {
    std::uint64_t teensy_time_us{0};
    Vec3 accel_mps2;
    Vec3 gyro_radps;
    double temperature_c{std::numeric_limits<double>::quiet_NaN()};
    std::uint32_t status_flags{0};
};

struct ChassisSpeedsPayload {
    std::uint64_t teensy_time_us{0};
    std::uint64_t rio_time_us{0};
    double vx_mps{0.0};
    double vy_mps{0.0};
    double omega_radps{0.0};
    std::uint32_t status_flags{0};
};

struct TeensyHealthPayload {
    std::uint64_t uptime_us{0};
    std::uint32_t error_flags{0};
    std::uint32_t trigger_status_flags{0};
    std::uint32_t rx_queue_depth{0};
    std::uint32_t tx_queue_depth{0};
    std::int64_t rio_offset_us{0};
    std::uint32_t rio_status_flags{0};
    std::uint32_t accel_saturations{0};
    std::uint32_t gyro_saturations{0};
    std::uint32_t tof_samples_emitted{0};
    std::uint32_t tof_overruns{0};
    std::uint32_t tof_i2c_failures{0};
    std::uint32_t tof_status_flags{0};
    // F-6 (v4): rolling fused-pose latency window measured from USB-decode to
    // CAN-FD-TX. Reset every health emit. samples == 0 ↔ no FusedPose has
    // crossed the firmware in the most recent window.
    std::uint32_t fused_pose_decode_to_tx_min_us{0};
    std::uint32_t fused_pose_decode_to_tx_avg_us{0};
    std::uint32_t fused_pose_decode_to_tx_max_us{0};
    std::uint32_t fused_pose_latency_samples{0};
};

struct CameraTriggerEventPayload {
    std::uint64_t teensy_time_us{0};
    std::int32_t pin{-1};
    std::uint32_t trigger_sequence{0};
    std::uint32_t status_flags{0};
};

// VL53L4CD ranging result. Mirrors include/posest/teensy/Protocol.h.
struct ToFSamplePayload {
    std::uint64_t teensy_time_us{0};
    std::uint32_t trigger_sequence{0};
    std::uint32_t distance_mm{0};
    std::uint32_t ranging_duration_us{0};
    std::uint32_t firmware_status_flags{0};
    std::uint32_t signal_rate_kcps{0};
    std::uint32_t ambient_rate_kcps{0};
    std::uint32_t range_status{0};
};

struct TimeSyncRequestPayload {
    std::uint32_t request_sequence{0};
    std::uint64_t host_send_time_us{0};
};

struct TimeSyncResponsePayload {
    std::uint32_t request_sequence{0};
    std::uint64_t teensy_receive_time_us{0};
    std::uint64_t teensy_transmit_time_us{0};
};

// v3 FusedPose USB payload, mirroring posest::FusedPoseEstimate. The wire
// layout (368 bytes, see decodeFusedPosePayload) carries:
//   - 6 doubles for Pose3 (x, y, z, roll, pitch, yaw)
//   - 3 doubles for body-frame velocity (vx, vy, vz)
//   - has_velocity flag (uint32, 0 or 1) — false drops the velocity block
//     into "unavailable" state without forcing a NaN sentinel
//   - status_flags (uint32)
//   - covariance: 36 doubles, gtsam::Pose3 tangent order [rx, ry, rz, tx, ty, tz]
// CAN-side packing (CanBridge::writeFusedPoseFrame) compresses this into a
// 64-byte FD frame; see firmware/teensy41/include/CanSchema.h.
struct FusedPose3Payload {
    // F-6 (v4): host steady_clock micros at the moment the host encoded this
    // frame. The firmware compares it (via the host↔Teensy time-sync offset)
    // against micros() at CAN-FD-TX to surface decode-to-TX latency.
    std::uint64_t host_send_time_us{0};
    double x_m{0.0};
    double y_m{0.0};
    double z_m{0.0};
    double roll_rad{0.0};
    double pitch_rad{0.0};
    double yaw_rad{0.0};
    double vx_mps{0.0};
    double vy_mps{0.0};
    double vz_mps{0.0};
    bool has_velocity{false};
    std::uint32_t status_flags{0};
    double covariance[36]{};
};

// Backwards-compatible alias retained for any incidental callers; new code
// should use FusedPose3Payload.
using FusedPosePayload = FusedPose3Payload;

struct CameraTriggerCommand {
    bool enabled{false};
    std::int32_t pin{-1};
    double rate_hz{0.0};
    std::uint32_t pulse_width_us{0};
    std::int64_t phase_offset_us{0};
};

struct TriggerAckEntry {
    std::int32_t pin{-1};
    std::uint32_t period_us{0};
    std::uint32_t pulse_us{0};
};

struct ImuConfigAckEntry {
    std::uint32_t accel_range_g{0};
    std::uint32_t accel_odr_hz{0};
    std::uint32_t accel_bandwidth_code{0};
    std::uint32_t gyro_range_dps{0};
    std::uint32_t gyro_bandwidth_code{0};
    std::uint32_t data_sync_rate_hz{0};
    std::uint32_t reserved{0};
};

struct VioConfigAckEntry {
    std::uint32_t vio_slot_index{0};
    std::uint32_t led_enabled{0};
    std::uint32_t led_pulse_width_us{0};
    std::uint32_t tof_enabled{0};
    std::uint32_t tof_i2c_address{0};
    std::uint32_t tof_timing_budget_ms{0};
    std::uint32_t tof_intermeasurement_period_ms{0};
    std::uint32_t tof_offset_after_flash_us{0};
    std::uint32_t tof_divisor{0};
};

// Decoded VioCompanion command. valid=true after a successful decode; the
// remaining fields are the literal wire values.
struct VioCompanionCommand {
    bool valid{false};
    std::uint32_t vio_slot_index{0};
    std::uint32_t led_enabled{0};
    std::uint32_t led_pulse_width_us{400};
    std::uint32_t tof_enabled{0};
    std::uint32_t tof_i2c_address{0x29};
    std::uint32_t tof_timing_budget_ms{10};
    std::uint32_t tof_intermeasurement_period_ms{20};
    std::uint32_t tof_offset_after_flash_us{500};
    std::uint32_t tof_divisor{1};
};

struct ConfigAckHeader {
    std::uint32_t kind{0};
    std::uint32_t status_flags{0};
    std::uint32_t effective_count{0};
};

struct ImuConfigCommand {
    bool valid{false};
    std::uint32_t accel_range_g{24};
    std::uint32_t accel_odr_hz{1000};
    std::uint32_t accel_bandwidth_code{2};
    std::uint32_t gyro_range_dps{2000};
    std::uint32_t gyro_bandwidth_code{2};
    std::uint32_t data_sync_rate_hz{1000};
    bool run_selftest_on_boot{true};
};

std::uint32_t crc32(const std::uint8_t* data, std::size_t size);

bool encodeFrame(
    MessageType type,
    std::uint32_t sequence,
    const std::uint8_t* payload,
    std::uint16_t payload_size,
    std::uint8_t* out,
    std::size_t capacity,
    std::size_t& out_size);

bool encodeImuPayload(const ImuPayload& payload, std::uint8_t* out, std::size_t capacity,
                      std::uint16_t& out_size);
bool encodeChassisSpeedsPayload(const ChassisSpeedsPayload& payload, std::uint8_t* out,
                                std::size_t capacity, std::uint16_t& out_size);
bool encodeTeensyHealthPayload(const TeensyHealthPayload& payload, std::uint8_t* out,
                               std::size_t capacity, std::uint16_t& out_size);
bool encodeCameraTriggerEventPayload(
    const CameraTriggerEventPayload& payload,
    std::uint8_t* out,
    std::size_t capacity,
    std::uint16_t& out_size);
bool encodeTimeSyncResponsePayload(const TimeSyncResponsePayload& payload, std::uint8_t* out,
                                   std::size_t capacity, std::uint16_t& out_size);
bool encodeToFSamplePayload(const ToFSamplePayload& payload, std::uint8_t* out,
                            std::size_t capacity, std::uint16_t& out_size);

bool decodeTimeSyncRequestPayload(const Frame& frame, TimeSyncRequestPayload& payload);
bool decodeFusedPosePayload(const Frame& frame, FusedPosePayload& payload);
bool decodeCameraTriggerConfigPayload(
    const Frame& frame,
    CameraTriggerCommand* commands,
    std::size_t command_capacity,
    std::size_t& command_count);
bool decodeImuConfigPayload(const Frame& frame, ImuConfigCommand& command);
bool decodeVioCompanionConfigPayload(const Frame& frame, VioCompanionCommand& command);

bool encodeConfigAckPayload(
    const ConfigAckHeader& header,
    const TriggerAckEntry* trigger_entries,
    std::size_t trigger_entry_count,
    const ImuConfigAckEntry* imu_entry,
    const VioConfigAckEntry* vio_entry,
    std::uint8_t* out,
    std::size_t capacity,
    std::uint16_t& out_size);

class StreamDecoder final {
public:
    void push(std::uint8_t byte);
    bool nextFrame(Frame& frame);
    void clear();
    const DecoderStats& stats() const { return stats_; }
    std::size_t bufferedBytes() const { return size_; }

private:
    void discard(std::size_t count);

    std::uint8_t buffer_[kMaxFrameSize]{};
    std::size_t size_{0};
    DecoderStats stats_;
};

}  // namespace posest::firmware
