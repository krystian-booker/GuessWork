#pragma once

// Pure encode/decode for the GuessWork ↔ robot-controller UDP protocol — the
// wire contract shared by the host (robot_link.cpp) and, normatively, the
// controller-side code (RoboRIO Java today, SystemCore next season).
// docs/ethernet-protocol.md is the prose copy of this header; change them
// together.
//
// Deliberately freestanding (no gw headers) so robot-side ports and the host
// test suite can treat it as the single source of truth:
// tests/test_udp_payloads.cpp golden-checks every layout. All multi-byte
// fields little-endian.
//
// Transport notes:
//   - One datagram = one message; no fragmentation (both packets ≤ 64 B).
//   - UDP's kernel checksum covers corruption on the wired robot LAN; there
//     is deliberately no application CRC. Misdirected/foreign datagrams are
//     rejected by magic + version + exact-length checks.
//   - Both ports sit in the FRC field-legal team-use range (5800–5810).
//   - The host learns the controller's address from the source of the last
//     valid CHASSIS_SPEEDS packet, so the robot side only needs the host's
//     static IP.

#include <stdint.h>
#include <string.h>

namespace gw {
namespace udpp {

constexpr uint16_t kMagic   = 0x5747;  // "GW" little-endian on the wire
constexpr uint8_t  kVersion = 1;

// Message types.
constexpr uint8_t kTypeChassisSpeeds = 1;  // controller -> host
constexpr uint8_t kTypePose          = 2;  // host -> controller

// Default ports (FRC team-use range).
constexpr uint16_t kDefaultHostPort  = 5809;  // host binds; speeds arrive here
constexpr uint16_t kDefaultRobotPort = 5810;  // controller binds; poses arrive here

constexpr size_t kChassisSpeedsLen = 32;
constexpr size_t kPoseLen          = 64;

// ---------------------------------------------------------------------------
// Little-endian helpers (mirrors firmware/src/telemetry_payloads.h; kept
// local so this header stays dependency-free).
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
inline float le_get_f32(const uint8_t* p, size_t off) {
    const uint32_t bits = le_get_u32(p, off);
    float v;
    memcpy(&v, &bits, sizeof(v));
    return v;
}

// ---------------------------------------------------------------------------
// CHASSIS_SPEEDS — controller -> host, 50–100 Hz, 32 bytes.
//
//   off  size  field
//     0     2  magic          0x5747
//     2     1  version        1
//     3     1  type           1
//     4     4  counter        rolling, +1 per sample (drop/reorder detection)
//     8     8  rio_time_us    controller FPGA sample time, FULL 64-bit µs
//                             (RobotController.getFPGATime() — no wrap
//                             handling anywhere, by construction)
//    16     4  vx_mps    f32  robot +X (forward)
//    20     4  vy_mps    f32  robot +Y (left); nonzero only on holonomic
//    24     4  omega_radps f32 yaw rate, CCW positive
//    28     4  status_flags   bit0 stale encoder data, bit1 wheel slip;
//                             bits 16+ free for team use
struct ChassisSpeedsPacket {
    uint32_t counter      = 0;
    uint64_t rio_time_us  = 0;
    float    vx_mps       = 0.0f;
    float    vy_mps       = 0.0f;
    float    omega_radps  = 0.0f;
    uint32_t status_flags = 0;
};

inline size_t encode_chassis_speeds(const ChassisSpeedsPacket& in,
                                    uint8_t (&out)[kChassisSpeedsLen]) {
    size_t off = 0;
    off = le_put_u16(out, off, kMagic);
    out[off++] = kVersion;
    out[off++] = kTypeChassisSpeeds;
    off = le_put_u32(out, off, in.counter);
    off = le_put_u64(out, off, in.rio_time_us);
    off = le_put_f32(out, off, in.vx_mps);
    off = le_put_f32(out, off, in.vy_mps);
    off = le_put_f32(out, off, in.omega_radps);
    off = le_put_u32(out, off, in.status_flags);
    return off;  // == kChassisSpeedsLen
}

inline bool decode_chassis_speeds(const uint8_t* p, size_t len,
                                  ChassisSpeedsPacket& out) {
    if (len != kChassisSpeedsLen) return false;
    if (le_get_u16(p, 0) != kMagic) return false;
    if (p[2] != kVersion) return false;
    if (p[3] != kTypeChassisSpeeds) return false;
    out.counter      = le_get_u32(p, 4);
    out.rio_time_us  = le_get_u64(p, 8);
    out.vx_mps       = le_get_f32(p, 16);
    out.vy_mps       = le_get_f32(p, 20);
    out.omega_radps  = le_get_f32(p, 24);
    out.status_flags = le_get_u32(p, 28);
    return true;
}

// ---------------------------------------------------------------------------
// POSE — host -> controller, at the fusion output rate, 64 bytes.
//
//   off  size  field
//     0     2  magic          0x5747
//     2     1  version        1
//     3     1  type           2
//     4     4  counter        rolling, +1 per send. Staleness rule for the
//                             controller: counter frozen > 200 ms => stop
//                             trusting the pose.
//     8     8  rio_time_us    pose validity time mapped onto the controller
//                             clock; 0 when clock sync is not healthy (flags
//                             bit0 clear) — consumers must check.
//    16     4  x_m       f32  WPILib field frame
//    20     4  y_m       f32
//    24     4  theta_rad f32  CCW positive
//    28     1  quality        0–255 fusion confidence
//    29     1  mode           FusionMode enum below
//    30     2  flags          bit0 clock_sync_healthy (rio_time_us valid),
//                             bit1 extrapolation clamped (pose older than
//                             max_extrapolation_ms)
//    32    24  cov[6]    f32  planar covariance, row-major upper triangle of
//                             the (x, y, theta) marginal: xx yy tt xy xt yt
//                             (m², rad², m·rad)
//    56     8  reserved       zero; room for a future field without a
//                             version bump (receivers must ignore)
constexpr uint16_t kPoseFlagClockSynced  = 1u << 0;
constexpr uint16_t kPoseFlagExtrapClamped = 1u << 1;

// Degraded-mode enum carried in POSE.mode — mirrors derive_fusion_mode's
// strings (docs/pose_pipeline.md §6).
constexpr uint8_t kModeUninitialized = 0;
constexpr uint8_t kModeNominal       = 1;
constexpr uint8_t kModeNoVio         = 2;
constexpr uint8_t kModeNoOdom        = 3;
constexpr uint8_t kModeTagsOnly      = 4;
constexpr uint8_t kModeDeadReckoning = 5;
constexpr uint8_t kModeCollision     = 6;

struct PosePacket {
    uint32_t counter     = 0;
    uint64_t rio_time_us = 0;
    float    x_m         = 0.0f;
    float    y_m         = 0.0f;
    float    theta_rad   = 0.0f;
    uint8_t  quality     = 0;
    uint8_t  mode        = kModeUninitialized;
    uint16_t flags       = 0;
    float    cov[6]      = {0, 0, 0, 0, 0, 0};  // xx yy tt xy xt yt
};

inline size_t encode_pose(const PosePacket& in, uint8_t (&out)[kPoseLen]) {
    size_t off = 0;
    off = le_put_u16(out, off, kMagic);
    out[off++] = kVersion;
    out[off++] = kTypePose;
    off = le_put_u32(out, off, in.counter);
    off = le_put_u64(out, off, in.rio_time_us);
    off = le_put_f32(out, off, in.x_m);
    off = le_put_f32(out, off, in.y_m);
    off = le_put_f32(out, off, in.theta_rad);
    out[off++] = in.quality;
    out[off++] = in.mode;
    off = le_put_u16(out, off, in.flags);
    for (int i = 0; i < 6; ++i) off = le_put_f32(out, off, in.cov[i]);
    off = le_put_u64(out, off, 0);  // reserved
    return off;  // == kPoseLen
}

inline bool decode_pose(const uint8_t* p, size_t len, PosePacket& out) {
    if (len != kPoseLen) return false;
    if (le_get_u16(p, 0) != kMagic) return false;
    if (p[2] != kVersion) return false;
    if (p[3] != kTypePose) return false;
    out.counter     = le_get_u32(p, 4);
    out.rio_time_us = le_get_u64(p, 8);
    out.x_m         = le_get_f32(p, 16);
    out.y_m         = le_get_f32(p, 20);
    out.theta_rad   = le_get_f32(p, 24);
    out.quality     = p[28];
    out.mode        = p[29];
    out.flags       = le_get_u16(p, 30);
    for (int i = 0; i < 6; ++i) out.cov[i] = le_get_f32(p, 32 + 4 * i);
    return true;
}

}  // namespace udpp
}  // namespace gw
