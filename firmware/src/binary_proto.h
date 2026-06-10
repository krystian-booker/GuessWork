#pragma once

#include <Arduino.h>

// One-way binary telemetry stream, Teensy → host, carried on the second
// USB-CDC interface (`SerialUSB1`, enabled by -DUSB_DUAL_SERIAL). The ASCII
// command protocol on `Serial` is untouched — see serial_proto.h.
//
// Frame layout (all multi-byte fields little-endian):
//   [0xA5][0x5A][type u8][len u8][payload: len bytes][crc16 u16]
// crc16 is CCITT (poly 0x1021, init 0xFFFF) over type + len + payload.
//
// Packet types:
//   0x01 IMU_BATCH  payload = count u8, then count × ImuRecord:
//                     t_us u64   wrap-extended micros at gyro DRDY edge
//                     ax ay az   f32, m/s²
//                     gx gy gz   f32, rad/s
//                   (32 bytes per record, count ≤ kImuBatchMax)
//   0x02 HEARTBEAT  payload = t_us u64, flags u8 (bit0: imu_ok),
//                   imu_samples u32, imu_drops u32
//   0x03 ODOM       reserved (Phase 5: CAN-forwarded chassis speeds)
//   0x10 POSE       reserved (Phase 5: host → Teensy fused pose; opposite
//                   direction, not emitted here)

namespace gw_fw {

constexpr uint8_t kBinMagic0 = 0xA5;
constexpr uint8_t kBinMagic1 = 0x5A;

enum class BinType : uint8_t {
    ImuBatch  = 0x01,
    Heartbeat = 0x02,
    Odom      = 0x03,
    Pose      = 0x10,
};

constexpr int kImuBatchMax    = 4;
constexpr int kImuRecordBytes = 8 + 6 * 4;  // t_us + 6 floats

uint16_t crc16_ccitt(const uint8_t* data, size_t n, uint16_t crc = 0xFFFF);

// Serializes one frame into `out` (must hold 6 + payload_len bytes) and
// returns the total frame length.
size_t build_frame(BinType type, const uint8_t* payload, uint8_t payload_len,
                   uint8_t* out);

// Append helpers for building payloads in place (little-endian).
inline size_t put_u8(uint8_t* p, size_t off, uint8_t v) {
    p[off] = v;
    return off + 1;
}
inline size_t put_u32(uint8_t* p, size_t off, uint32_t v) {
    p[off]     = static_cast<uint8_t>(v);
    p[off + 1] = static_cast<uint8_t>(v >> 8);
    p[off + 2] = static_cast<uint8_t>(v >> 16);
    p[off + 3] = static_cast<uint8_t>(v >> 24);
    return off + 4;
}
inline size_t put_u64(uint8_t* p, size_t off, uint64_t v) {
    off = put_u32(p, off, static_cast<uint32_t>(v));
    return put_u32(p, off, static_cast<uint32_t>(v >> 32));
}
inline size_t put_f32(uint8_t* p, size_t off, float v) {
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    return put_u32(p, off, bits);
}

}  // namespace gw_fw
