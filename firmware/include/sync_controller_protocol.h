#pragma once

// GuessWork sync-controller USB wire contract.
//
// The MicoAir F405 V2 exposes one USB CDC port, so commands, acknowledgements,
// trigger events, IMU samples, and health telemetry share one CRC-framed byte
// stream. This header is deliberately freestanding: it is compiled by the
// STM32 firmware, the macOS host, and the host-side golden-byte tests.

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace gw_sync {

constexpr uint8_t  kMagic0          = 0xA5;
constexpr uint8_t  kMagic1          = 0x5A;
constexpr uint8_t  kProtocolVersion = 1;
constexpr uint16_t kMaxPayloadBytes = 256;
constexpr size_t   kHeaderBytes     = 8;
constexpr size_t   kTrailerBytes    = 2;
constexpr size_t   kMaxFrameBytes   = kHeaderBytes + kMaxPayloadBytes + kTrailerBytes;

constexpr uint32_t kBoardIdMicoAirF405V2 = 0x3241344Du;  // "MA42" little-endian
constexpr uint16_t kFirmwareVersion       = 1;
constexpr uint8_t  kOutputCount           = 6;
constexpr uint8_t  kMaxGroups             = 4;
constexpr uint32_t kMaxRateMilliHz        = 1'000'000;  // 1000 Hz

enum class MessageType : uint8_t {
    Hello       = 0x01,
    DeviceInfo  = 0x02,
    SetConfig   = 0x03,
    Arm         = 0x04,
    Stop        = 0x05,
    TestOutput  = 0x06,
    Ack         = 0x07,

    Trigger     = 0x20,
    ImuBatch    = 0x21,
    Heartbeat   = 0x22,
};

enum class AckStatus : uint8_t {
    Ok             = 0,
    BadMessage     = 1,
    BadConfig      = 2,
    Busy           = 3,
    NoConfig       = 4,
    Unsupported    = 5,
    InternalError  = 6,
};

enum Capability : uint32_t {
    kCapabilityTriggers       = 1u << 0,
    kCapabilityImu            = 1u << 1,
    kCapabilityAtomicConfig   = 1u << 2,
    kCapabilityTestOutput     = 1u << 3,
    kCapabilityBoardFrameImu  = 1u << 4,
};

enum HeartbeatFlag : uint32_t {
    kHeartbeatImuOk = 1u << 0,
    kHeartbeatArmed = 1u << 1,
};

struct FrameView {
    MessageType    type       = MessageType::Hello;
    uint16_t       request_id = 0;
    const uint8_t* payload    = nullptr;
    uint16_t       payload_len = 0;
};

struct DeviceInfo {
    uint32_t board_id        = 0;
    uint16_t firmware_version = 0;
    uint8_t  output_count    = 0;
    uint8_t  max_groups      = 0;
    uint32_t capabilities    = 0;
    uint32_t reset_reason    = 0;
    uint32_t uid[3]          = {};
};

struct GroupConfig {
    uint8_t  slot           = 0;
    uint8_t  pin_mask       = 0;
    uint16_t reserved       = 0;
    uint32_t rate_millihz   = 0;
};

struct TriggerEvent {
    uint8_t  slot     = 0;
    uint32_t index    = 0;
    uint64_t t_us     = 0;
};

struct ImuRecord {
    uint64_t t_us = 0;
    float    accel[3] = {};
    float    gyro[3]  = {};
};

struct Heartbeat {
    uint64_t t_us          = 0;
    uint32_t flags         = 0;
    uint32_t imu_samples   = 0;
    uint32_t imu_drops     = 0;
    uint32_t trigger_drops = 0;
    uint32_t usb_errors    = 0;
};

inline size_t put_u8(uint8_t* p, size_t off, uint8_t v) {
    p[off] = v;
    return off + 1;
}
inline size_t put_u16(uint8_t* p, size_t off, uint16_t v) {
    p[off] = static_cast<uint8_t>(v);
    p[off + 1] = static_cast<uint8_t>(v >> 8);
    return off + 2;
}
inline size_t put_u32(uint8_t* p, size_t off, uint32_t v) {
    off = put_u16(p, off, static_cast<uint16_t>(v));
    return put_u16(p, off, static_cast<uint16_t>(v >> 16));
}
inline size_t put_u64(uint8_t* p, size_t off, uint64_t v) {
    off = put_u32(p, off, static_cast<uint32_t>(v));
    return put_u32(p, off, static_cast<uint32_t>(v >> 32));
}
inline size_t put_f32(uint8_t* p, size_t off, float v) {
    uint32_t bits = 0;
    memcpy(&bits, &v, sizeof(bits));
    return put_u32(p, off, bits);
}

inline uint16_t get_u16(const uint8_t* p, size_t off) {
    return static_cast<uint16_t>(p[off]) |
           static_cast<uint16_t>(static_cast<uint16_t>(p[off + 1]) << 8);
}
inline uint32_t get_u32(const uint8_t* p, size_t off) {
    return static_cast<uint32_t>(get_u16(p, off)) |
           (static_cast<uint32_t>(get_u16(p, off + 2)) << 16);
}
inline uint64_t get_u64(const uint8_t* p, size_t off) {
    return static_cast<uint64_t>(get_u32(p, off)) |
           (static_cast<uint64_t>(get_u32(p, off + 4)) << 32);
}
inline float get_f32(const uint8_t* p, size_t off) {
    const uint32_t bits = get_u32(p, off);
    float v = 0.0f;
    memcpy(&v, &bits, sizeof(v));
    return v;
}

inline uint16_t crc16_ccitt(const uint8_t* data, size_t n,
                            uint16_t crc = 0xFFFF) {
    for (size_t i = 0; i < n; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000)
                      ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                      : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

// Frame layout:
//   magic[2], protocol u8, type u8, request_id u16, payload_len u16,
//   payload, CRC16-CCITT u16 LE over protocol..payload.
inline size_t build_frame(MessageType type, uint16_t request_id,
                          const uint8_t* payload, uint16_t payload_len,
                          uint8_t* out) {
    if (!out || payload_len > kMaxPayloadBytes || (payload_len && !payload)) return 0;
    size_t off = 0;
    off = put_u8(out, off, kMagic0);
    off = put_u8(out, off, kMagic1);
    off = put_u8(out, off, kProtocolVersion);
    off = put_u8(out, off, static_cast<uint8_t>(type));
    off = put_u16(out, off, request_id);
    off = put_u16(out, off, payload_len);
    if (payload_len) memcpy(out + off, payload, payload_len);
    off += payload_len;
    const uint16_t crc = crc16_ccitt(out + 2, off - 2);
    return put_u16(out, off, crc);
}

inline size_t encode_device_info(const DeviceInfo& info, uint8_t* out) {
    size_t off = 0;
    off = put_u32(out, off, info.board_id);
    off = put_u16(out, off, info.firmware_version);
    off = put_u8(out, off, info.output_count);
    off = put_u8(out, off, info.max_groups);
    off = put_u32(out, off, info.capabilities);
    off = put_u32(out, off, info.reset_reason);
    for (uint32_t word : info.uid) off = put_u32(out, off, word);
    return off;
}

inline bool decode_device_info(const uint8_t* p, size_t n, DeviceInfo& out) {
    if (!p || n != 28) return false;
    out.board_id         = get_u32(p, 0);
    out.firmware_version = get_u16(p, 4);
    out.output_count     = p[6];
    out.max_groups       = p[7];
    out.capabilities     = get_u32(p, 8);
    out.reset_reason     = get_u32(p, 12);
    for (int i = 0; i < 3; ++i) out.uid[i] = get_u32(p, 16 + 4 * i);
    return true;
}

inline size_t encode_group_config(const GroupConfig& group, uint8_t* out) {
    size_t off = 0;
    off = put_u8(out, off, group.slot);
    off = put_u8(out, off, group.pin_mask);
    off = put_u16(out, off, 0);
    return put_u32(out, off, group.rate_millihz);
}

inline bool decode_group_config(const uint8_t* p, size_t n, GroupConfig& out) {
    if (!p || n < 8) return false;
    out.slot         = p[0];
    out.pin_mask     = p[1];
    out.reserved     = get_u16(p, 2);
    out.rate_millihz = get_u32(p, 4);
    return true;
}

}  // namespace gw_sync
