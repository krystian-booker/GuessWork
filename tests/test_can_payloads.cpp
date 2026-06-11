#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>

// The freestanding firmware wire contract — compiled directly on the host.
#include "firmware/src/can_payloads.h"

namespace canp = gw_fw::canp;

namespace {

float get_f32_le(const uint8_t* p) {
    const uint32_t bits = static_cast<uint32_t>(p[0]) | (p[1] << 8) |
                          (p[2] << 16) | (static_cast<uint32_t>(p[3]) << 24);
    float v;
    std::memcpy(&v, &bits, sizeof(v));
    return v;
}

}  // namespace

TEST(CanPayloadsTest, FrcArbitrationIds) {
    // deviceType=10 <<24 | manufacturer=8 <<16 | apiId<<6 | deviceNumber.
    EXPECT_EQ(canp::make_frc_id(0x110, 33), 0x0A084421u);
    EXPECT_EQ(canp::kIdChassisSpeeds, 0x0A084421u);
    EXPECT_EQ(canp::kIdChassisStamp, 0x0A084461u);
    EXPECT_EQ(canp::kIdPose, 0x0A084821u);
    EXPECT_EQ(canp::kIdPoseXy, 0x0A084861u);
    EXPECT_EQ(canp::kIdPoseTheta, 0x0A0848A1u);
}

TEST(CanPayloadsTest, FdChassisSpeedsRoundTripAndLayout) {
    canp::ChassisSpeedsWire s;
    s.vx           = 1.5f;
    s.vy           = -0.25f;
    s.omega        = 3.0f;
    s.rio_time_us  = 0x0123456789ABCDEFull;
    s.status_flags = 0x0003;
    s.counter      = 42;

    uint8_t buf[24] = {};
    canp::encode_fd_chassis_speeds(s, buf);

    // Golden offsets per docs/can-protocol.md.
    EXPECT_FLOAT_EQ(get_f32_le(buf + 0), 1.5f);
    EXPECT_FLOAT_EQ(get_f32_le(buf + 4), -0.25f);
    EXPECT_FLOAT_EQ(get_f32_le(buf + 8), 3.0f);
    EXPECT_EQ(canp::le_get_u64(buf, 12), 0x0123456789ABCDEFull);
    EXPECT_EQ(canp::le_get_u16(buf, 20), 0x0003);
    EXPECT_EQ(buf[22], 42);
    EXPECT_EQ(buf[23], 0);

    canp::ChassisSpeedsWire out;
    ASSERT_TRUE(canp::decode_fd_chassis_speeds(buf, 24, out));
    EXPECT_FLOAT_EQ(out.vx, s.vx);
    EXPECT_FLOAT_EQ(out.vy, s.vy);
    EXPECT_FLOAT_EQ(out.omega, s.omega);
    EXPECT_EQ(out.rio_time_us, s.rio_time_us);
    EXPECT_EQ(out.status_flags, s.status_flags);
    EXPECT_EQ(out.counter, s.counter);

    EXPECT_FALSE(canp::decode_fd_chassis_speeds(buf, 8, out));  // wrong DLC
}

TEST(CanPayloadsTest, ClassicChassisSpeedsRoundTrip) {
    canp::ChassisSpeedsWire s;
    s.vx           = 1.234f;   // 1234 mm/s
    s.vy           = -0.5f;    // -500 mm/s
    s.omega        = 2.0f;     // 2000 mrad/s
    s.status_flags = 0x0002;
    s.counter      = 7;

    uint8_t buf[8] = {};
    canp::encode_classic_chassis_speeds(s, buf);
    EXPECT_EQ(canp::le_get_i16(buf, 0), 1234);
    EXPECT_EQ(canp::le_get_i16(buf, 2), -500);
    EXPECT_EQ(canp::le_get_i16(buf, 4), 2000);
    EXPECT_EQ(buf[6], 0x02);
    EXPECT_EQ(buf[7], 7);

    canp::ChassisSpeedsWire out;
    ASSERT_TRUE(canp::decode_classic_chassis_speeds(buf, 8, out));
    EXPECT_FLOAT_EQ(out.vx, 1.234f);
    EXPECT_FLOAT_EQ(out.vy, -0.5f);
    EXPECT_FLOAT_EQ(out.omega, 2.0f);
    EXPECT_EQ(out.status_flags, 0x0002);
    EXPECT_EQ(out.counter, 7);
    EXPECT_EQ(out.rio_time_us, 0u);  // caller attaches from the STAMP latch
}

TEST(CanPayloadsTest, MilliClampAndRounding) {
    EXPECT_EQ(canp::to_milli_i16(0.0f), 0);
    EXPECT_EQ(canp::to_milli_i16(0.0014f), 1);     // rounds, not truncates
    EXPECT_EQ(canp::to_milli_i16(-0.0014f), -1);
    EXPECT_EQ(canp::to_milli_i16(100.0f), 32767);  // clamp +
    EXPECT_EQ(canp::to_milli_i16(-100.0f), -32767);
    EXPECT_EQ(canp::to_milli_i16(32.766f), 32766);
}

TEST(CanPayloadsTest, ClassicStampRoundTrip) {
    uint8_t buf[8] = {};
    canp::encode_classic_stamp(0xDEADBEEFu, 99, buf);
    uint32_t rio_lo  = 0;
    uint8_t  counter = 0;
    ASSERT_TRUE(canp::decode_classic_stamp(buf, 8, rio_lo, counter));
    EXPECT_EQ(rio_lo, 0xDEADBEEFu);
    EXPECT_EQ(counter, 99);
}

TEST(CanPayloadsTest, ExtendU32AcrossWrap) {
    uint32_t hi = 0, last_lo = 0;
    EXPECT_EQ(canp::extend_u32(100, hi, last_lo), 100ull);
    EXPECT_EQ(canp::extend_u32(0xFFFFFFF0u, hi, last_lo), 0xFFFFFFF0ull);
    // lo wrapped: 0xFFFFFFF0 → 0x10 means +0x20 µs of real time.
    EXPECT_EQ(canp::extend_u32(0x10u, hi, last_lo), 0x100000010ull);
    EXPECT_EQ(canp::extend_u32(0x20u, hi, last_lo), 0x100000020ull);
}

TEST(CanPayloadsTest, FdPoseRoundTrip) {
    canp::PoseWire p;
    p.rio_time_us = 123456789ull;
    p.x           = 8.27f;
    p.y           = 4.03f;
    p.theta       = -1.57f;
    p.quality     = 200;
    p.counter     = 17;

    uint8_t buf[24] = {};
    canp::encode_fd_pose(p, buf);
    EXPECT_EQ(canp::le_get_u64(buf, 0), 123456789ull);
    EXPECT_FLOAT_EQ(get_f32_le(buf + 8), 8.27f);
    EXPECT_EQ(buf[20], 200);
    EXPECT_EQ(buf[21], 17);

    canp::PoseWire out;
    ASSERT_TRUE(canp::decode_fd_pose(buf, 24, out));
    EXPECT_EQ(out.rio_time_us, p.rio_time_us);
    EXPECT_FLOAT_EQ(out.x, p.x);
    EXPECT_FLOAT_EQ(out.y, p.y);
    EXPECT_FLOAT_EQ(out.theta, p.theta);
    EXPECT_EQ(out.quality, p.quality);
    EXPECT_EQ(out.counter, p.counter);
}

TEST(CanPayloadsTest, ClassicPoseSplitRoundTrip) {
    canp::PoseWire p;
    p.x       = 1.0f;
    p.y       = 2.0f;
    p.theta   = 0.5f;
    p.quality = 128;
    p.counter = 3;

    uint8_t xy[8] = {}, th[8] = {};
    canp::encode_classic_pose_xy(p, xy);
    canp::encode_classic_pose_theta(p, th);

    canp::PoseWire out;
    ASSERT_TRUE(canp::decode_classic_pose_xy(xy, 8, out));
    ASSERT_TRUE(canp::decode_classic_pose_theta(th, 8, out));
    EXPECT_FLOAT_EQ(out.x, 1.0f);
    EXPECT_FLOAT_EQ(out.y, 2.0f);
    EXPECT_FLOAT_EQ(out.theta, 0.5f);
    EXPECT_EQ(out.quality, 128);
    EXPECT_EQ(out.counter, 3);
}

TEST(CanPayloadsTest, OdomTelemetryRoundTrip) {
    canp::ChassisSpeedsWire s;
    s.vx           = -2.5f;
    s.vy           = 0.75f;
    s.omega        = -0.1f;
    s.rio_time_us  = 55'123'456ull;
    s.status_flags = 0x0001;
    s.counter      = 250;

    uint8_t payload[32] = {};
    canp::encode_odom_telemetry(0xAABBCCDDEEFF0011ull, s, canp::kModeClassic,
                                payload);

    uint64_t                t_arrival = 0;
    canp::ChassisSpeedsWire out;
    uint8_t                 mode = 0;
    ASSERT_TRUE(canp::decode_odom_telemetry(payload, 32, t_arrival, out, mode));
    EXPECT_EQ(t_arrival, 0xAABBCCDDEEFF0011ull);
    EXPECT_EQ(out.rio_time_us, 55'123'456ull);
    EXPECT_FLOAT_EQ(out.vx, -2.5f);
    EXPECT_FLOAT_EQ(out.vy, 0.75f);
    EXPECT_FLOAT_EQ(out.omega, -0.1f);
    EXPECT_EQ(out.status_flags, 0x0001);
    EXPECT_EQ(out.counter, 250);
    EXPECT_EQ(mode, canp::kModeClassic);

    EXPECT_FALSE(canp::decode_odom_telemetry(payload, 31, t_arrival, out, mode));
}

TEST(CanPayloadsTest, PoseTelemetryRoundTrip) {
    canp::PoseWire p;
    p.rio_time_us = 99ull;
    p.x           = -3.25f;
    p.y           = 0.0f;
    p.theta       = 3.14f;
    p.quality     = 1;
    p.counter     = 255;

    uint8_t payload[22] = {};
    canp::encode_pose_telemetry(p, payload);

    canp::PoseWire out;
    ASSERT_TRUE(canp::decode_pose_telemetry(payload, 22, out));
    EXPECT_EQ(out.rio_time_us, 99ull);
    EXPECT_FLOAT_EQ(out.x, -3.25f);
    EXPECT_FLOAT_EQ(out.theta, 3.14f);
    EXPECT_EQ(out.quality, 1);
    EXPECT_EQ(out.counter, 255);
}

TEST(CanPayloadsTest, TelemetryFrameMatchesDecoderFraming) {
    // build_telemetry_frame is the host-side mirror of the firmware's
    // build_frame; golden-check the framing bytes.
    const uint8_t payload[3] = {0x01, 0x02, 0x03};
    uint8_t       frame[6 + 3];
    const size_t  n = canp::build_telemetry_frame(0x10, payload, 3, frame);
    ASSERT_EQ(n, 9u);
    EXPECT_EQ(frame[0], 0xA5);
    EXPECT_EQ(frame[1], 0x5A);
    EXPECT_EQ(frame[2], 0x10);
    EXPECT_EQ(frame[3], 3);
    const uint16_t crc = canp::telemetry_crc16(frame + 2, 5);
    EXPECT_EQ(frame[7], static_cast<uint8_t>(crc));
    EXPECT_EQ(frame[8], static_cast<uint8_t>(crc >> 8));
}
