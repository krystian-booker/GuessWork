#pragma once

#include <cstddef>
#include <cstdint>

// CAN-FD message catalogue for the Teensy <-> RoboRIO link. All payloads are
// little-endian.
//
// Frame sizes are chosen so they fit a single CAN-FD DLC code (8, 12, 16, 20,
// 24, 32, 48, 64). The RIO -> Teensy chassis-speeds frame uses 48 bytes with
// trailing zero-padding so a future field can be added without touching the
// wire format. The Teensy -> RIO fused-pose frame is unchanged so the FRC
// robot code on the RIO does not need to be re-flashed for this contract bump.
//
// ---------------------------------------------------------------------------
// RoboRIO clock-domain contract (read this before writing the RIO firmware).
// ---------------------------------------------------------------------------
//
//  rio_time_us is the monotonic FPGA microsecond counter:
//      rio_time_us = (uint64_t) HAL_GetFPGATime();
//  or, equivalently in WPILib Java/C++:
//      rio_time_us = (uint64_t)(Timer.getFPGATimestamp() * 1.0e6);
//
//  - Epoch: zero at RIO power-on / FPGA reset. Free-running 64-bit counter,
//    so wraparound is not a real concern (~584,000 years).
//  - Monotonicity: must be strictly non-decreasing across RIO firmware
//    iterations. Do NOT swap epochs (e.g. to system wall-clock) mid-run.
//  - Reboot: rio_time_us drops back to ~0 after RIO power cycle. The Teensy
//    side detects this via 32 consecutive offset-gate rejections on 0x101
//    pings and re-anchors automatically; expect ~0.3-30 s of dropped chassis
//    samples depending on the configured ping rate.
//
// 0x101 ping rate (RoboRIO -> Teensy):
//
//  - Recommended: 100 Hz (1 ms period).
//  - Teensy filter: alpha = 1/8 EMA, single-step rejection at +/- 5 ms,
//    bootstrap window of 3 unconditional samples.
//  - At 100 Hz the filter converges in ~10 samples (~100 ms) after RIO
//    power-on or after a reboot-triggered re-bootstrap.
//
// 0x100 chassis speeds (RoboRIO -> Teensy):
//
//  - rio_time_us must be the FPGA microsecond at the instant kinematics is
//    computed (the moment the wheel speeds are sampled), NOT the time the
//    CAN frame is queued for transmit. Otherwise the host's RIO->host time
//    conversion will reflect TX-queue latency rather than measurement time.
//  - status_flags: bit 0 = stale wheel encoder data, bit 1 = wheel slip
//    detected. The Teensy passes status_flags through verbatim and OR's in
//    its own kStatusUnsynchronizedRioTime if its local offset is invalid.
//
// Endianness:
//
//  - All multi-byte fields are little-endian (matches both Teensy and RIO
//    native byte order; no byte-swap on either side).
//  - double = IEEE 754 binary64.
//  - Trailing pad bytes MUST be zero on transmit; receivers ignore them.

namespace posest::firmware::can_schema {

constexpr std::uint32_t kRioChassisSpeedsId = 0x100;
constexpr std::uint32_t kRioTimeSyncId = 0x101;
constexpr std::uint32_t kTeensyPoseId = 0x180;

constexpr std::size_t kRioChassisSpeedsPayloadBytes = 48;
constexpr std::size_t kRioTimeSyncPayloadBytes = 16;
constexpr std::size_t kTeensyPosePayloadBytes = 48;

// Layout of `kRioChassisSpeedsId` (RIO -> Teensy):
//   int64  rio_time_us    (offset 0)
//   double vx_mps         (offset 8)
//   double vy_mps         (offset 16)
//   double omega_radps    (offset 24)
//   uint32 status_flags   (offset 32)
//   uint8  pad[12]        (offset 36)
constexpr std::size_t kRioChassisSpeedsRioTimeOffset = 0;
constexpr std::size_t kRioChassisSpeedsVxOffset = 8;
constexpr std::size_t kRioChassisSpeedsVyOffset = 16;
constexpr std::size_t kRioChassisSpeedsOmegaOffset = 24;
constexpr std::size_t kRioChassisSpeedsStatusOffset = 32;

// Layout of `kRioTimeSyncId` (RIO -> Teensy):
//   uint64 rio_time_us    (offset 0)
//   uint32 ping_seq       (offset 8)
//   uint32 reserved       (offset 12)
constexpr std::size_t kRioTimeSyncRioTimeOffset = 0;
constexpr std::size_t kRioTimeSyncSeqOffset = 8;

// Layout of `kTeensyPoseId` (Teensy -> RIO):
//   int64  teensy_time_us (offset 0)
//   double x_m            (offset 8)
//   double y_m            (offset 16)
//   double theta_rad      (offset 24)
//   uint32 status_flags   (offset 32)
//   uint8  pad[12]        (offset 36)
constexpr std::size_t kTeensyPoseTeensyTimeOffset = 0;
constexpr std::size_t kTeensyPoseXOffset = 8;
constexpr std::size_t kTeensyPoseYOffset = 16;
constexpr std::size_t kTeensyPoseThetaOffset = 24;
constexpr std::size_t kTeensyPoseStatusOffset = 32;

}  // namespace posest::firmware::can_schema
