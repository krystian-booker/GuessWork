#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <array>
#include <vector>

#include "posest/MeasurementTypes.h"

namespace posest::teensy {

constexpr std::uint16_t kFrameMagic = 0x4757;  // "GW"
constexpr std::uint8_t kProtocolVersion = 1;
constexpr std::uint32_t kStatusUnsynchronizedTime = 1u << 0u;
constexpr std::uint32_t kStatusUnsynchronizedRioTime = 1u << 1u;
constexpr std::uint32_t kStatusRobotSlipping = 1u << 2u;

enum class MessageType : std::uint8_t {
    ImuSample = 1,
    WheelOdometry = 2,
    CanRx = 3,
    TeensyHealth = 4,
    TimeSyncResponse = 5,
    RobotOdometry = 6,
    CameraTriggerEvent = 7,
    FusedPose = 64,
    CanTx = 65,
    TimeSyncRequest = 66,
    ConfigCommand = 67,
};

enum class ConfigCommandKind : std::uint32_t {
    CameraTriggers = 1,
};

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

struct WheelOdometryPayload {
    std::uint64_t teensy_time_us{0};
    Pose2d chassis_delta;
    std::array<double, 4> wheel_delta_m{};
    std::uint32_t status_flags{0};
};

struct RobotOdometryPayload {
    std::uint64_t teensy_time_us{0};
    std::uint64_t rio_time_us{0};
    Pose2d field_to_robot;
    std::uint32_t status_flags{0};
};

struct TeensyHealthPayload {
    std::uint64_t uptime_us{0};
    std::uint32_t error_flags{0};
    std::uint32_t trigger_status_flags{0};
    std::uint32_t rx_queue_depth{0};
    std::uint32_t tx_queue_depth{0};
};

struct CameraTriggerEventPayload {
    std::uint64_t teensy_time_us{0};
    std::int32_t pin{-1};
    std::uint32_t trigger_sequence{0};
    std::uint32_t status_flags{0};
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

std::vector<std::uint8_t> encodeWheelOdometryPayload(const WheelOdometryPayload& payload);
std::optional<WheelOdometryPayload> decodeWheelOdometryPayload(
    const std::vector<std::uint8_t>& bytes);

std::vector<std::uint8_t> encodeRobotOdometryPayload(
    const RobotOdometryPayload& payload);
std::optional<RobotOdometryPayload> decodeRobotOdometryPayload(
    const std::vector<std::uint8_t>& bytes);

std::vector<std::uint8_t> encodeTeensyHealthPayload(const TeensyHealthPayload& payload);
std::optional<TeensyHealthPayload> decodeTeensyHealthPayload(
    const std::vector<std::uint8_t>& bytes);

std::vector<std::uint8_t> encodeCameraTriggerEventPayload(
    const CameraTriggerEventPayload& payload);
std::optional<CameraTriggerEventPayload> decodeCameraTriggerEventPayload(
    const std::vector<std::uint8_t>& bytes);

std::vector<std::uint8_t> encodeTimeSyncRequestPayload(
    const TimeSyncRequestPayload& payload);
std::optional<TimeSyncRequestPayload> decodeTimeSyncRequestPayload(
    const std::vector<std::uint8_t>& bytes);

std::vector<std::uint8_t> encodeTimeSyncResponsePayload(
    const TimeSyncResponsePayload& payload);
std::optional<TimeSyncResponsePayload> decodeTimeSyncResponsePayload(
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
