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
