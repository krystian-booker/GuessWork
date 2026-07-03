#include <gtest/gtest.h>

#include <cstring>

#include "net/udp_payloads.h"

// Wire-contract tests for the GuessWork <-> robot-controller UDP protocol.
// Byte-offset goldens pin the layout documented in udp_payloads.h /
// docs/ethernet-protocol.md — controller-side ports depend on them.

namespace gw::udpp {

TEST(UdpPayloadsTest, ChassisSpeedsRoundTrip) {
    ChassisSpeedsPacket in;
    in.counter      = 0x11223344;
    in.rio_time_us  = 0x0102030405060708ull;
    in.vx_mps       = 1.25f;
    in.vy_mps       = -0.5f;
    in.omega_radps  = 3.5f;
    in.status_flags = 0x80000003;

    uint8_t buf[kChassisSpeedsLen];
    ASSERT_EQ(encode_chassis_speeds(in, buf), kChassisSpeedsLen);

    ChassisSpeedsPacket out;
    ASSERT_TRUE(decode_chassis_speeds(buf, sizeof(buf), out));
    EXPECT_EQ(out.counter, in.counter);
    EXPECT_EQ(out.rio_time_us, in.rio_time_us);
    EXPECT_EQ(out.vx_mps, in.vx_mps);
    EXPECT_EQ(out.vy_mps, in.vy_mps);
    EXPECT_EQ(out.omega_radps, in.omega_radps);
    EXPECT_EQ(out.status_flags, in.status_flags);
}

TEST(UdpPayloadsTest, ChassisSpeedsGoldenBytes) {
    ChassisSpeedsPacket in;
    in.counter      = 7;
    in.rio_time_us  = 0x0102030405060708ull;
    in.vx_mps       = 1.0f;  // 0x3F800000
    in.status_flags = 0x00000001;

    uint8_t buf[kChassisSpeedsLen];
    encode_chassis_speeds(in, buf);

    EXPECT_EQ(buf[0], 0x47);  // 'G'
    EXPECT_EQ(buf[1], 0x57);  // 'W'
    EXPECT_EQ(buf[2], kVersion);
    EXPECT_EQ(buf[3], kTypeChassisSpeeds);
    EXPECT_EQ(buf[4], 7);                       // counter LE
    EXPECT_EQ(buf[8], 0x08);                    // rio_time_us LE low byte
    EXPECT_EQ(buf[15], 0x01);                   // rio_time_us LE high byte
    EXPECT_EQ(buf[16], 0x00);                   // vx f32 LE: 00 00 80 3F
    EXPECT_EQ(buf[18], 0x80);
    EXPECT_EQ(buf[19], 0x3F);
    EXPECT_EQ(buf[28], 0x01);                   // status_flags LE
}

TEST(UdpPayloadsTest, ChassisSpeedsRejectsWrongLenMagicVersionType) {
    ChassisSpeedsPacket in, out;
    uint8_t buf[kChassisSpeedsLen];
    encode_chassis_speeds(in, buf);

    EXPECT_FALSE(decode_chassis_speeds(buf, sizeof(buf) - 1, out));
    EXPECT_FALSE(decode_chassis_speeds(buf, sizeof(buf) + 1, out));

    uint8_t bad[kChassisSpeedsLen];
    memcpy(bad, buf, sizeof(buf));
    bad[0] = 0x00;  // magic
    EXPECT_FALSE(decode_chassis_speeds(bad, sizeof(bad), out));

    memcpy(bad, buf, sizeof(buf));
    bad[2] = kVersion + 1;
    EXPECT_FALSE(decode_chassis_speeds(bad, sizeof(bad), out));

    memcpy(bad, buf, sizeof(buf));
    bad[3] = kTypePose;  // wrong type for this decoder
    EXPECT_FALSE(decode_chassis_speeds(bad, sizeof(bad), out));
}

TEST(UdpPayloadsTest, PoseRoundTrip) {
    PosePacket in;
    in.counter     = 99;
    in.rio_time_us = 123456789ull;
    in.x_m         = 8.25f;
    in.y_m         = 4.0f;
    in.theta_rad   = -1.5f;
    in.quality     = 200;
    in.mode        = kModeNoVio;
    in.flags       = kPoseFlagClockSynced | kPoseFlagExtrapClamped;
    for (int i = 0; i < 6; ++i) in.cov[i] = 0.01f * static_cast<float>(i + 1);

    uint8_t buf[kPoseLen];
    ASSERT_EQ(encode_pose(in, buf), kPoseLen);

    PosePacket out;
    ASSERT_TRUE(decode_pose(buf, sizeof(buf), out));
    EXPECT_EQ(out.counter, in.counter);
    EXPECT_EQ(out.rio_time_us, in.rio_time_us);
    EXPECT_EQ(out.x_m, in.x_m);
    EXPECT_EQ(out.y_m, in.y_m);
    EXPECT_EQ(out.theta_rad, in.theta_rad);
    EXPECT_EQ(out.quality, in.quality);
    EXPECT_EQ(out.mode, in.mode);
    EXPECT_EQ(out.flags, in.flags);
    for (int i = 0; i < 6; ++i) EXPECT_EQ(out.cov[i], in.cov[i]);
}

TEST(UdpPayloadsTest, PoseGoldenBytes) {
    PosePacket in;
    in.counter   = 1;
    in.x_m       = 1.0f;  // 0x3F800000
    in.quality   = 0xAB;
    in.mode      = kModeTagsOnly;
    in.flags     = kPoseFlagClockSynced;
    in.cov[0]    = 1.0f;

    uint8_t buf[kPoseLen];
    encode_pose(in, buf);

    EXPECT_EQ(buf[0], 0x47);
    EXPECT_EQ(buf[1], 0x57);
    EXPECT_EQ(buf[2], kVersion);
    EXPECT_EQ(buf[3], kTypePose);
    EXPECT_EQ(buf[4], 1);       // counter
    EXPECT_EQ(buf[19], 0x3F);   // x_m f32 LE high byte
    EXPECT_EQ(buf[28], 0xAB);   // quality
    EXPECT_EQ(buf[29], kModeTagsOnly);
    EXPECT_EQ(buf[30], 0x01);   // flags LE
    EXPECT_EQ(buf[35], 0x3F);   // cov[0] f32 LE high byte
    for (int i = 56; i < 64; ++i) EXPECT_EQ(buf[i], 0x00);  // reserved
}

TEST(UdpPayloadsTest, PoseRejectsBadEnvelope) {
    PosePacket in, out;
    uint8_t buf[kPoseLen];
    encode_pose(in, buf);

    EXPECT_FALSE(decode_pose(buf, sizeof(buf) - 8, out));
    uint8_t bad[kPoseLen];
    memcpy(bad, buf, sizeof(buf));
    bad[3] = kTypeChassisSpeeds;
    EXPECT_FALSE(decode_pose(bad, sizeof(bad), out));
}

}  // namespace gw::udpp
