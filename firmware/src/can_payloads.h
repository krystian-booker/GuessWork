#pragma once

// Pure encode/decode for every CAN frame and CAN-related telemetry payload —
// the wire contract shared by the firmware (can_bridge.cpp) and, normatively,
// the controller-side code. docs/can-protocol.md is the prose copy of this
// header; change them together.
//
// Deliberately freestanding (no Arduino.h) so the host test suite compiles it
// directly: tests/test_can_payloads.cpp golden-checks every layout. All
// multi-byte fields little-endian.

#include <stdint.h>
#include <string.h>

namespace gw_fw {
namespace canp {

// ---------------------------------------------------------------------------
// FRC 29-bit extended arbitration IDs (WPILib team-use convention):
//   deviceType=10 (Miscellaneous) <<24 | manufacturer=8 (TeamUse) <<16
//   | apiId<<6 | deviceNumber
constexpr uint8_t kCanDeviceNumber = 33;

constexpr uint32_t make_frc_id(uint16_t api_id, uint8_t device_number) {
    return (10u << 24) | (8u << 16) |
           (static_cast<uint32_t>(api_id & 0x3FF) << 6) |
           (device_number & 0x3F);
}

constexpr uint16_t kApiChassisSpeeds = 0x110;  // controller -> Teensy, both modes
constexpr uint16_t kApiChassisStamp  = 0x111;  // controller -> Teensy, classic only
constexpr uint16_t kApiPose          = 0x120;  // Teensy -> controller, FD only
constexpr uint16_t kApiPoseXy        = 0x121;  // Teensy -> controller, classic
constexpr uint16_t kApiPoseTheta     = 0x122;  // Teensy -> controller, classic

constexpr uint32_t kIdChassisSpeeds = make_frc_id(kApiChassisSpeeds, kCanDeviceNumber);
constexpr uint32_t kIdChassisStamp  = make_frc_id(kApiChassisStamp, kCanDeviceNumber);
constexpr uint32_t kIdPose          = make_frc_id(kApiPose, kCanDeviceNumber);
constexpr uint32_t kIdPoseXy        = make_frc_id(kApiPoseXy, kCanDeviceNumber);
constexpr uint32_t kIdPoseTheta     = make_frc_id(kApiPoseTheta, kCanDeviceNumber);

// CAN mode discriminator carried in ODOM / HEARTBEAT telemetry.
constexpr uint8_t kModeOff     = 0;
constexpr uint8_t kModeClassic = 1;
constexpr uint8_t kModeFd      = 2;

// Frame / payload sizes.
constexpr uint8_t kFdChassisSpeedsLen      = 24;
constexpr uint8_t kClassicFrameLen         = 8;
constexpr uint8_t kFdPoseLen               = 24;
constexpr uint8_t kOdomTelemetryPayloadLen = 32;
constexpr uint8_t kPoseTelemetryPayloadLen = 22;

// ---------------------------------------------------------------------------
// Local little-endian helpers (binary_proto.h has put_* too, but it pulls in
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

// Clamp a float to the int16 milli-unit encoding used by classic frames
// (mm/s, mrad/s). ±32.767 in SI units.
inline int16_t to_milli_i16(float v) {
    const float milli = v * 1000.0f;
    if (milli >= 32767.0f) return 32767;
    if (milli <= -32767.0f) return -32767;
    return static_cast<int16_t>(milli + (milli >= 0.0f ? 0.5f : -0.5f));
}

// Wrap-extend a u32 microsecond counter (the RIO FPGA clock wraps every
// ~71.6 min). Caller owns `hi`/`last_lo`, both zero-initialised. Assumes
// successive `lo` values are in-order (true at 50-100 Hz frame rates).
inline uint64_t extend_u32(uint32_t lo, uint32_t& hi, uint32_t& last_lo) {
    if (lo < last_lo) ++hi;
    last_lo = lo;
    return (static_cast<uint64_t>(hi) << 32) | lo;
}

// ---------------------------------------------------------------------------
// Wire structs (decoded form; field meanings in docs/can-protocol.md).

struct ChassisSpeedsWire {
    float    vx           = 0.0f;  // m/s, robot +X (forward)
    float    vy           = 0.0f;  // m/s, robot +Y (left)
    float    omega        = 0.0f;  // rad/s, CCW positive
    uint64_t rio_time_us  = 0;     // FPGA us at sampling; 0 = unknown
    uint16_t status_flags = 0;     // bit0 stale encoders, bit1 wheel slip
    uint8_t  counter      = 0;     // rolling, +1 per sample
};

struct PoseWire {
    uint64_t rio_time_us = 0;  // pose sample time in the FPGA clock; 0 = unmapped
    float    x           = 0.0f;
    float    y           = 0.0f;
    float    theta       = 0.0f;
    uint8_t  quality     = 0;
    uint8_t  counter     = 0;  // rolling; controller staleness detection
};

// ---------------------------------------------------------------------------
// FD CHASSIS_SPEEDS (24 bytes):
//   vx f32 | vy f32 | omega f32 | rio_time_us u64 | status u16 | counter u8 | rsvd u8

inline void encode_fd_chassis_speeds(const ChassisSpeedsWire& s, uint8_t out[24]) {
    size_t off = 0;
    off = le_put_f32(out, off, s.vx);
    off = le_put_f32(out, off, s.vy);
    off = le_put_f32(out, off, s.omega);
    off = le_put_u64(out, off, s.rio_time_us);
    off = le_put_u16(out, off, s.status_flags);
    off = le_put_u8(out, off, s.counter);
    le_put_u8(out, off, 0);
}

inline bool decode_fd_chassis_speeds(const uint8_t* buf, uint8_t len,
                                     ChassisSpeedsWire& out) {
    if (len != kFdChassisSpeedsLen) return false;
    out.vx           = le_get_f32(buf, 0);
    out.vy           = le_get_f32(buf, 4);
    out.omega        = le_get_f32(buf, 8);
    out.rio_time_us  = le_get_u64(buf, 12);
    out.status_flags = le_get_u16(buf, 20);
    out.counter      = buf[22];
    return true;
}

// ---------------------------------------------------------------------------
// Classic CHASSIS_STAMP (8 bytes, sent FIRST):
//   rio_time_lo u32 | counter u8 | rsvd u8[3]

inline void encode_classic_stamp(uint32_t rio_time_lo, uint8_t counter,
                                 uint8_t out[8]) {
    size_t off = 0;
    off = le_put_u32(out, off, rio_time_lo);
    off = le_put_u8(out, off, counter);
    out[5] = out[6] = out[7] = 0;
}

inline bool decode_classic_stamp(const uint8_t* buf, uint8_t len,
                                 uint32_t& rio_time_lo, uint8_t& counter) {
    if (len != kClassicFrameLen) return false;
    rio_time_lo = le_get_u32(buf, 0);
    counter     = buf[4];
    return true;
}

// ---------------------------------------------------------------------------
// Classic CHASSIS_SPEEDS (8 bytes):
//   vx i16 mm/s | vy i16 mm/s | omega i16 mrad/s | status u8 | counter u8
// rio_time_us is NOT carried here — the bridge attaches it from the latched
// STAMP frame when the counters match.

inline void encode_classic_chassis_speeds(const ChassisSpeedsWire& s,
                                          uint8_t out[8]) {
    size_t off = 0;
    off = le_put_i16(out, off, to_milli_i16(s.vx));
    off = le_put_i16(out, off, to_milli_i16(s.vy));
    off = le_put_i16(out, off, to_milli_i16(s.omega));
    off = le_put_u8(out, off, static_cast<uint8_t>(s.status_flags & 0xFF));
    le_put_u8(out, off, s.counter);
}

inline bool decode_classic_chassis_speeds(const uint8_t* buf, uint8_t len,
                                          ChassisSpeedsWire& out) {
    if (len != kClassicFrameLen) return false;
    out.vx           = static_cast<float>(le_get_i16(buf, 0)) * 0.001f;
    out.vy           = static_cast<float>(le_get_i16(buf, 2)) * 0.001f;
    out.omega        = static_cast<float>(le_get_i16(buf, 4)) * 0.001f;
    out.status_flags = buf[6];
    out.counter      = buf[7];
    out.rio_time_us  = 0;  // attached by the caller from the STAMP latch
    return true;
}

// ---------------------------------------------------------------------------
// FD POSE (24 bytes):
//   rio_time_us u64 | x f32 | y f32 | theta f32 | quality u8 | counter u8 | rsvd u16

inline void encode_fd_pose(const PoseWire& p, uint8_t out[24]) {
    size_t off = 0;
    off = le_put_u64(out, off, p.rio_time_us);
    off = le_put_f32(out, off, p.x);
    off = le_put_f32(out, off, p.y);
    off = le_put_f32(out, off, p.theta);
    off = le_put_u8(out, off, p.quality);
    off = le_put_u8(out, off, p.counter);
    le_put_u16(out, off, 0);
}

inline bool decode_fd_pose(const uint8_t* buf, uint8_t len, PoseWire& out) {
    if (len != kFdPoseLen) return false;
    out.rio_time_us = le_get_u64(buf, 0);
    out.x           = le_get_f32(buf, 8);
    out.y           = le_get_f32(buf, 12);
    out.theta       = le_get_f32(buf, 16);
    out.quality     = buf[20];
    out.counter     = buf[21];
    return true;
}

// ---------------------------------------------------------------------------
// Classic POSE split (8 bytes each, XY sent first):
//   POSE_XY:    x f32 | y f32
//   POSE_THETA: theta f32 | quality u8 | counter u8 | rsvd u8[2]
// No timestamp (no room) — staleness is counter-advancement only.

inline void encode_classic_pose_xy(const PoseWire& p, uint8_t out[8]) {
    size_t off = 0;
    off = le_put_f32(out, off, p.x);
    le_put_f32(out, off, p.y);
}

inline void encode_classic_pose_theta(const PoseWire& p, uint8_t out[8]) {
    size_t off = 0;
    off = le_put_f32(out, off, p.theta);
    off = le_put_u8(out, off, p.quality);
    off = le_put_u8(out, off, p.counter);
    out[6] = out[7] = 0;
}

inline bool decode_classic_pose_xy(const uint8_t* buf, uint8_t len, PoseWire& out) {
    if (len != kClassicFrameLen) return false;
    out.x = le_get_f32(buf, 0);
    out.y = le_get_f32(buf, 4);
    return true;
}

inline bool decode_classic_pose_theta(const uint8_t* buf, uint8_t len,
                                      PoseWire& out) {
    if (len != kClassicFrameLen) return false;
    out.theta   = le_get_f32(buf, 0);
    out.quality = buf[4];
    out.counter = buf[5];
    return true;
}

// ---------------------------------------------------------------------------
// ODOM telemetry payload (BinType 0x03, Teensy -> host, 32 bytes), identical
// regardless of CAN mode:
//   t_arrival_us u64 | rio_time_us u64 (0 = unknown) | vx f32 | vy f32
//   | omega f32 | status u16 | counter u8 | mode u8

inline void encode_odom_telemetry(uint64_t t_arrival_us,
                                  const ChassisSpeedsWire& s, uint8_t mode,
                                  uint8_t out[32]) {
    size_t off = 0;
    off = le_put_u64(out, off, t_arrival_us);
    off = le_put_u64(out, off, s.rio_time_us);
    off = le_put_f32(out, off, s.vx);
    off = le_put_f32(out, off, s.vy);
    off = le_put_f32(out, off, s.omega);
    off = le_put_u16(out, off, s.status_flags);
    off = le_put_u8(out, off, s.counter);
    le_put_u8(out, off, mode);
}

inline bool decode_odom_telemetry(const uint8_t* buf, uint8_t len,
                                  uint64_t& t_arrival_us, ChassisSpeedsWire& s,
                                  uint8_t& mode) {
    if (len != kOdomTelemetryPayloadLen) return false;
    t_arrival_us   = le_get_u64(buf, 0);
    s.rio_time_us  = le_get_u64(buf, 8);
    s.vx           = le_get_f32(buf, 16);
    s.vy           = le_get_f32(buf, 20);
    s.omega        = le_get_f32(buf, 24);
    s.status_flags = le_get_u16(buf, 28);
    s.counter      = buf[30];
    mode           = buf[31];
    return true;
}

// ---------------------------------------------------------------------------
// Telemetry framing (freestanding mirror of binary_proto.h's build_frame, for
// the host side — binary_proto.h pulls in Arduino.h).

constexpr uint8_t kTelemetryMagic0  = 0xA5;
constexpr uint8_t kTelemetryMagic1  = 0x5A;
constexpr uint8_t kBinTypeOdom      = 0x03;
constexpr uint8_t kBinTypePose      = 0x10;

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

// ---------------------------------------------------------------------------
// POSE telemetry payload (BinType 0x10, host -> Teensy, 22 bytes):
//   rio_time_us u64 | x f32 | y f32 | theta f32 | quality u8 | counter u8

inline void encode_pose_telemetry(const PoseWire& p, uint8_t out[22]) {
    size_t off = 0;
    off = le_put_u64(out, off, p.rio_time_us);
    off = le_put_f32(out, off, p.x);
    off = le_put_f32(out, off, p.y);
    off = le_put_f32(out, off, p.theta);
    off = le_put_u8(out, off, p.quality);
    le_put_u8(out, off, p.counter);
}

inline bool decode_pose_telemetry(const uint8_t* buf, uint8_t len, PoseWire& out) {
    if (len != kPoseTelemetryPayloadLen) return false;
    out.rio_time_us = le_get_u64(buf, 0);
    out.x           = le_get_f32(buf, 8);
    out.y           = le_get_f32(buf, 12);
    out.theta       = le_get_f32(buf, 16);
    out.quality     = buf[20];
    out.counter     = buf[21];
    return true;
}

}  // namespace canp
}  // namespace gw_fw
