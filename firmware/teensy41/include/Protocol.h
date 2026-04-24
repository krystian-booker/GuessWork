#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

namespace posest::firmware {

constexpr std::uint16_t kFrameMagic = 0x4757;
constexpr std::uint8_t kProtocolVersion = 1;
constexpr std::size_t kHeaderSize = 10;
constexpr std::size_t kCrcSize = 4;
constexpr std::size_t kMaxPayloadSize = 4096;
constexpr std::size_t kMaxFrameSize = kHeaderSize + kMaxPayloadSize + kCrcSize;

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

struct WheelOdometryPayload {
    std::uint64_t teensy_time_us{0};
    Pose2d chassis_delta;
    double wheel_delta_m[4]{};
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

struct FusedPosePayload {
    Pose2d field_to_robot;
    std::uint32_t status_flags{0};
};

struct CameraTriggerCommand {
    bool enabled{false};
    std::int32_t pin{-1};
    double rate_hz{0.0};
    std::uint32_t pulse_width_us{0};
    std::int64_t phase_offset_us{0};
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
bool encodeWheelOdometryPayload(const WheelOdometryPayload& payload, std::uint8_t* out,
                                std::size_t capacity, std::uint16_t& out_size);
bool encodeRobotOdometryPayload(const RobotOdometryPayload& payload, std::uint8_t* out,
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

bool decodeTimeSyncRequestPayload(const Frame& frame, TimeSyncRequestPayload& payload);
bool decodeFusedPosePayload(const Frame& frame, FusedPosePayload& payload);
bool decodeCameraTriggerConfigPayload(
    const Frame& frame,
    CameraTriggerCommand* commands,
    std::size_t command_capacity,
    std::size_t& command_count);

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
