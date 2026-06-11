#pragma once

#include <Arduino.h>

// Binary telemetry framing on the second USB-CDC interface (`SerialUSB1`,
// enabled by -DUSB_DUAL_SERIAL). Mostly Teensy → host; the single host →
// Teensy packet (POSE) rides the same CDC in the reverse direction and is
// decoded by BinRxParser below. The ASCII command protocol on `Serial` is
// untouched — see serial_proto.h.
//
// Frame layout (all multi-byte fields little-endian):
//   [0xA5][0x5A][type u8][len u8][payload: len bytes][crc16 u16]
// crc16 is CCITT (poly 0x1021, init 0xFFFF) over type + len + payload.
//
// Packet types (CAN-related payload layouts live in can_payloads.h and
// docs/can-protocol.md — normative there):
//   0x01 IMU_BATCH  payload = count u8, then count × ImuRecord:
//                     t_us u64   wrap-extended micros at gyro DRDY edge
//                     ax ay az   f32, m/s²
//                     gx gy gz   f32, rad/s
//                   (32 bytes per record, count ≤ kImuBatchMax)
//   0x02 HEARTBEAT  payload (fw=3, 34 bytes) = t_us u64,
//                   flags u8 (bit0: imu_ok, bit1: can_ok),
//                   imu_samples u32, imu_drops u32, can_rx u32,
//                   can_rx_drops u32, odom_tx_drops u32, pose_tx u32,
//                   can_mode u8 (0 off, 1 classic, 2 fd)
//   0x03 ODOM       Teensy → host, 32 bytes: t_arrival_us u64,
//                   rio_time_us u64 (0 = unknown), vx f32, vy f32,
//                   omega f32, status_flags u16, counter u8, mode u8
//   0x10 POSE       host → Teensy, 22 bytes: rio_time_us u64, x f32,
//                   y f32, theta f32, quality u8, counter u8

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

inline size_t put_u16(uint8_t* p, size_t off, uint16_t v) {
    p[off]     = static_cast<uint8_t>(v);
    p[off + 1] = static_cast<uint8_t>(v >> 8);
    return off + 2;
}

// Incremental frame parser for the host → Teensy direction on SerialUSB1.
// Allocation-free byte-at-a-time state machine; on a CRC-valid frame the
// payload is latched and `take()` hands it to the caller (main loop) exactly
// once. Bad CRCs / oversized payloads resync on the next 0xA5 and bump
// crc_errors().
class BinRxParser {
public:
    static constexpr uint8_t kMaxPayload = 64;

    void feed(uint8_t b);

    // True iff a complete frame is pending; copies it out and clears the
    // latch. `payload` must hold kMaxPayload bytes.
    bool take(uint8_t& type, uint8_t* payload, uint8_t& len);

    uint32_t crc_errors() const { return crc_errors_; }

private:
    enum class State : uint8_t { Magic0, Magic1, Type, Len, Payload, CrcLo, CrcHi };

    State    state_ = State::Magic0;
    uint8_t  type_  = 0;
    uint8_t  len_   = 0;
    uint8_t  got_   = 0;
    uint8_t  buf_[kMaxPayload];
    uint16_t crc_lo_     = 0;
    bool     pending_    = false;
    uint8_t  pend_type_  = 0;
    uint8_t  pend_len_   = 0;
    uint8_t  pend_buf_[kMaxPayload];
    uint32_t crc_errors_ = 0;
};

}  // namespace gw_fw
