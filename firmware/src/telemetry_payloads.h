#pragma once

// Teensy USB telemetry framing contract — the wire format of the binary
// stream on the second USB-CDC interface (`SerialUSB1`): little-endian
// put/get helpers, the CCITT CRC16, the frame builder, and the frame-type
// constants for the IMU batch and heartbeat packets. binary_proto.h is the
// firmware-side twin (it pulls in Arduino.h); robot communication is UDP —
// docs/ethernet-protocol.md is the normative contract for that path.
//
// Deliberately freestanding (no Arduino.h) so the host test suite compiles
// it directly: tests/test_telemetry_payloads.cpp golden-checks the framing.
// All multi-byte fields little-endian.

#include <stdint.h>
#include <string.h>

namespace gw_fw {
namespace telem {

// ---------------------------------------------------------------------------
// Little-endian helpers (binary_proto.h has put_* too, but it pulls in
// Arduino.h — this header must stay freestanding).
inline size_t le_put_u8(uint8_t* p, size_t off, uint8_t v) {
    p[off] = v;
    return off + 1;
}
inline size_t le_put_u16(uint8_t* p, size_t off, uint16_t v) {
    p[off]     = static_cast<uint8_t>(v);
    p[off + 1] = static_cast<uint8_t>(v >> 8);
    return off + 2;
}
inline size_t le_put_u32(uint8_t* p, size_t off, uint32_t v) {
    off = le_put_u16(p, off, static_cast<uint16_t>(v));
    return le_put_u16(p, off, static_cast<uint16_t>(v >> 16));
}
inline size_t le_put_u64(uint8_t* p, size_t off, uint64_t v) {
    off = le_put_u32(p, off, static_cast<uint32_t>(v));
    return le_put_u32(p, off, static_cast<uint32_t>(v >> 32));
}
inline size_t le_put_i16(uint8_t* p, size_t off, int16_t v) {
    return le_put_u16(p, off, static_cast<uint16_t>(v));
}
inline size_t le_put_f32(uint8_t* p, size_t off, float v) {
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    return le_put_u32(p, off, bits);
}

inline uint16_t le_get_u16(const uint8_t* p, size_t off) {
    return static_cast<uint16_t>(p[off] | (p[off + 1] << 8));
}
inline uint32_t le_get_u32(const uint8_t* p, size_t off) {
    return static_cast<uint32_t>(le_get_u16(p, off)) |
           (static_cast<uint32_t>(le_get_u16(p, off + 2)) << 16);
}
inline uint64_t le_get_u64(const uint8_t* p, size_t off) {
    return static_cast<uint64_t>(le_get_u32(p, off)) |
           (static_cast<uint64_t>(le_get_u32(p, off + 4)) << 32);
}
inline int16_t le_get_i16(const uint8_t* p, size_t off) {
    return static_cast<int16_t>(le_get_u16(p, off));
}
inline float le_get_f32(const uint8_t* p, size_t off) {
    const uint32_t bits = le_get_u32(p, off);
    float v;
    memcpy(&v, &bits, sizeof(v));
    return v;
}

// ---------------------------------------------------------------------------
// Telemetry framing (freestanding mirror of binary_proto.h's build_frame):
//   [0xA5][0x5A][type u8][len u8][payload: len bytes][crc16 u16 LE]
// crc16 is CCITT (poly 0x1021, init 0xFFFF) over type + len + payload.

constexpr uint8_t kTelemetryMagic0  = 0xA5;
constexpr uint8_t kTelemetryMagic1  = 0x5A;
constexpr uint8_t kBinTypeImuBatch  = 0x01;
constexpr uint8_t kBinTypeHeartbeat = 0x02;

inline uint16_t telemetry_crc16(const uint8_t* data, size_t n,
                                uint16_t crc = 0xFFFF) {
    for (size_t i = 0; i < n; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

// Serializes one frame into `out` (must hold 6 + payload_len bytes) and
// returns the total frame length.
inline size_t build_telemetry_frame(uint8_t type, const uint8_t* payload,
                                    uint8_t payload_len, uint8_t* out) {
    size_t off = 0;
    out[off++] = kTelemetryMagic0;
    out[off++] = kTelemetryMagic1;
    out[off++] = type;
    out[off++] = payload_len;
    memcpy(out + off, payload, payload_len);
    off += payload_len;
    const uint16_t crc =
        telemetry_crc16(out + 2, static_cast<size_t>(payload_len) + 2);
    out[off++] = static_cast<uint8_t>(crc);
    out[off++] = static_cast<uint8_t>(crc >> 8);
    return off;
}

}  // namespace telem
}  // namespace gw_fw
