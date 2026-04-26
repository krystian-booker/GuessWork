#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "posest/MeasurementTypes.h"

namespace posest::teensy {

constexpr std::uint16_t kFrameMagic = 0x4757;  // "GW"
// v3: FusedPose payload extended from 28 B (Pose2d + status) to a full Pose3
// + velocity + 6×6 covariance + status (368 B). Single-vehicle deployment so
// the version bump is breaking, not negotiated.
constexpr std::uint8_t kProtocolVersion = 3;
constexpr std::uint32_t kStatusUnsynchronizedTime = 1u << 0u;
constexpr std::uint32_t kStatusUnsynchronizedRioTime = 1u << 1u;
constexpr std::uint32_t kStatusRobotSlipping = 1u << 2u;

// ToF-specific bits set on ToFSamplePayload::firmware_status_flags. Bits 0–7
// share semantics with the per-sample kStatus* word so consumers can apply the
// same time-sync gating; bits 8+ are ToF-specific observability.
constexpr std::uint32_t kToFStatusRangingOverrun = 1u << 8u;
constexpr std::uint32_t kToFStatusI2cFailure = 1u << 9u;
constexpr std::uint32_t kToFStatusInvalidRangeStatus = 1u << 10u;
constexpr std::uint32_t kToFStatusClampedHostSide = 1u << 16u;

// Bits set on TeensyHealthPayload::rio_status_flags. These mirror the
// firmware's CanBridge status surface and are independent of the per-sample
// status flags above.
constexpr std::uint32_t kHealthRioUnsynchronized = 1u << 0u;
constexpr std::uint32_t kHealthRioPingRejected = 1u << 1u;

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

// Bits set on ConfigAckPayload::status_flags. 0 == accepted.
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
    std::vector<std::uint8_t> payload;
};

struct ImuPayload {
    std::uint64_t teensy_time_us{0};
    Vec3 accel_mps2;
    Vec3 gyro_radps;
    std::optional<double> temperature_c;
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
};

struct CameraTriggerEventPayload {
    std::uint64_t teensy_time_us{0};
    std::int32_t pin{-1};
    std::uint32_t trigger_sequence{0};
    std::uint32_t status_flags{0};
};

// VL53L4CD ranging result, emitted once per single-shot ranging cycle. The
// firmware tags each ranging with the trigger_sequence of the camera pulse
// that immediately preceded it, so the host can join ToF samples to camera
// frames by exact sequence (no fuzzy time-window match needed).
struct ToFSamplePayload {
    std::uint64_t teensy_time_us{0};
    std::uint32_t trigger_sequence{0};
    std::uint32_t distance_mm{0};       // raw chip output, pre-mounting-offset
    std::uint32_t ranging_duration_us{0};
    std::uint32_t firmware_status_flags{0};
    std::uint32_t signal_rate_kcps{0};
    std::uint32_t ambient_rate_kcps{0};
    std::uint32_t range_status{0};      // VL53L4CD-defined: 0 = valid, others encode failure modes
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

// Echoes back the effective VioCompanion configuration the firmware accepted.
// Layout matches VioCompanionConfigPayload field-for-field so a host can diff
// requested vs. effective without mapping logic.
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

struct ConfigAckPayload {
    std::uint32_t kind{0};
    std::uint32_t status_flags{0};
    std::uint32_t effective_count{0};
    std::vector<TriggerAckEntry> trigger_entries;
    std::optional<ImuConfigAckEntry> imu_entry;
    std::optional<VioConfigAckEntry> vio_entry;
};

struct ImuConfigPayload {
    std::uint32_t accel_range_g{24};
    std::uint32_t accel_odr_hz{1000};
    std::uint32_t accel_bandwidth_code{2};
    std::uint32_t gyro_range_dps{2000};
    std::uint32_t gyro_bandwidth_code{2};
    std::uint32_t data_sync_rate_hz{1000};
    std::uint32_t run_selftest_on_boot{1};
};

// VioCompanion config command body (sits after the 4-byte kind discriminator
// in the ConfigCommand frame). Bundles IR LED control and VL53L4CD ToF
// settings; the firmware enforces the temporal-multiplex invariant by the
// constraint tof_offset_after_flash_us >= led_pulse_width_us.
//
// The IR LED MOSFET gate pin is a firmware constant (see kIrLedMosfetPin),
// not host-configurable: physical pin changes require a board respin and a
// reflash anyway.
struct VioCompanionConfigPayload {
    std::uint32_t vio_slot_index{0};
    std::uint32_t led_enabled{1};
    std::uint32_t led_pulse_width_us{400};
    std::uint32_t tof_enabled{1};
    std::uint32_t tof_i2c_address{0x29};
    std::uint32_t tof_timing_budget_ms{10};
    std::uint32_t tof_intermeasurement_period_ms{20};
    std::uint32_t tof_offset_after_flash_us{500};
    std::uint32_t tof_divisor{1};
};

struct DecodeResult {
    Frame frame;
    std::size_t bytes_consumed{0};
};

struct StreamStats {
    std::uint64_t crc_failures{0};
    std::uint64_t sequence_gaps{0};
    std::optional<std::uint32_t> last_sequence;
};

std::uint32_t crc32(const std::uint8_t* data, std::size_t size);
std::vector<std::uint8_t> encodeFrame(const Frame& frame);
std::optional<DecodeResult> decodeFrame(const std::vector<std::uint8_t>& bytes);

std::vector<std::uint8_t> encodeImuPayload(const ImuPayload& payload);
std::optional<ImuPayload> decodeImuPayload(const std::vector<std::uint8_t>& bytes);

std::vector<std::uint8_t> encodeChassisSpeedsPayload(
    const ChassisSpeedsPayload& payload);
std::optional<ChassisSpeedsPayload> decodeChassisSpeedsPayload(
    const std::vector<std::uint8_t>& bytes);

std::vector<std::uint8_t> encodeTeensyHealthPayload(const TeensyHealthPayload& payload);
std::optional<TeensyHealthPayload> decodeTeensyHealthPayload(
    const std::vector<std::uint8_t>& bytes);

std::vector<std::uint8_t> encodeCameraTriggerEventPayload(
    const CameraTriggerEventPayload& payload);
std::optional<CameraTriggerEventPayload> decodeCameraTriggerEventPayload(
    const std::vector<std::uint8_t>& bytes);

std::vector<std::uint8_t> encodeToFSamplePayload(const ToFSamplePayload& payload);
std::optional<ToFSamplePayload> decodeToFSamplePayload(
    const std::vector<std::uint8_t>& bytes);

std::vector<std::uint8_t> encodeVioCompanionConfigPayload(
    const VioCompanionConfigPayload& payload);
std::optional<VioCompanionConfigPayload> decodeVioCompanionConfigPayload(
    const std::vector<std::uint8_t>& bytes);

std::vector<std::uint8_t> encodeTimeSyncRequestPayload(
    const TimeSyncRequestPayload& payload);
std::optional<TimeSyncRequestPayload> decodeTimeSyncRequestPayload(
    const std::vector<std::uint8_t>& bytes);

std::vector<std::uint8_t> encodeTimeSyncResponsePayload(
    const TimeSyncResponsePayload& payload);
std::optional<TimeSyncResponsePayload> decodeTimeSyncResponsePayload(
    const std::vector<std::uint8_t>& bytes);

std::vector<std::uint8_t> encodeConfigAckPayload(const ConfigAckPayload& payload);
std::optional<ConfigAckPayload> decodeConfigAckPayload(
    const std::vector<std::uint8_t>& bytes);

std::vector<std::uint8_t> encodeImuConfigPayload(const ImuConfigPayload& payload);
std::optional<ImuConfigPayload> decodeImuConfigPayload(
    const std::vector<std::uint8_t>& bytes);

class StreamDecoder final {
public:
    std::vector<Frame> push(const std::uint8_t* data, std::size_t size);
    const StreamStats& stats() const { return stats_; }
    void clear();

private:
    std::vector<std::uint8_t> buffer_;
    StreamStats stats_;
};

}  // namespace posest::teensy
