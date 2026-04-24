#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace posest::teensy {

constexpr std::uint16_t kFrameMagic = 0x4757;  // "GW"
constexpr std::uint8_t kProtocolVersion = 1;

enum class MessageType : std::uint8_t {
    ImuSample = 1,
    WheelOdometry = 2,
    CanRx = 3,
    TeensyHealth = 4,
    TimeSyncResponse = 5,
    FusedPose = 64,
    CanTx = 65,
    TimeSyncRequest = 66,
    ConfigCommand = 67,
};

struct Frame {
    MessageType type{MessageType::TeensyHealth};
    std::uint32_t sequence{0};
    std::vector<std::uint8_t> payload;
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
