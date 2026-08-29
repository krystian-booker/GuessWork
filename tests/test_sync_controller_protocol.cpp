#include <gtest/gtest.h>

#include <cstring>

#include "firmware/include/sync_controller_protocol.h"

TEST(SyncControllerProtocolTest, ConstantsAndCrcGolden) {
    EXPECT_EQ(gw_sync::kMagic0, 0xA5);
    EXPECT_EQ(gw_sync::kMagic1, 0x5A);
    EXPECT_EQ(gw_sync::kProtocolVersion, 1);
    EXPECT_EQ(gw_sync::kOutputCount, 6);
    const uint8_t check[] = {'1','2','3','4','5','6','7','8','9'};
    EXPECT_EQ(gw_sync::crc16_ccitt(check, sizeof(check)), 0x29B1);
}

TEST(SyncControllerProtocolTest, FrameAndEndianGoldenBytes) {
    const uint8_t payload[] = {0x10, 0x20, 0x30};
    uint8_t frame[gw_sync::kMaxFrameBytes]{};
    const size_t n = gw_sync::build_frame(gw_sync::MessageType::SetConfig,
                                           0x1234, payload, sizeof(payload), frame);
    ASSERT_EQ(n, 13u);
    const uint8_t prefix[] = {
        0xA5, 0x5A, 0x01, 0x03, 0x34, 0x12, 0x03, 0x00,
        0x10, 0x20, 0x30};
    EXPECT_EQ(std::memcmp(frame, prefix, sizeof(prefix)), 0);
    EXPECT_EQ(gw_sync::get_u16(frame, 11),
              gw_sync::crc16_ccitt(frame + 2, 9));
}

TEST(SyncControllerProtocolTest, DeviceInfoRoundTrip) {
    gw_sync::DeviceInfo in;
    in.board_id = gw_sync::kBoardIdMicoAirF405V2;
    in.firmware_version = 7;
    in.output_count = 6;
    in.max_groups = 4;
    in.capabilities = 0x1F;
    in.reset_reason = 0xAABBCCDD;
    in.uid[0] = 1; in.uid[1] = 2; in.uid[2] = 3;
    uint8_t bytes[28];
    EXPECT_EQ(gw_sync::encode_device_info(in, bytes), sizeof(bytes));
    gw_sync::DeviceInfo out;
    ASSERT_TRUE(gw_sync::decode_device_info(bytes, sizeof(bytes), out));
    EXPECT_EQ(out.board_id, in.board_id);
    EXPECT_EQ(out.firmware_version, 7);
    EXPECT_EQ(out.reset_reason, 0xAABBCCDDu);
    EXPECT_EQ(out.uid[2], 3u);
}

TEST(SyncControllerProtocolTest, GroupConfigRoundTrip) {
    gw_sync::GroupConfig in;
    in.slot = 2;
    in.pin_mask = 0x24;
    in.rate_millihz = 29'970;
    uint8_t bytes[8];
    EXPECT_EQ(gw_sync::encode_group_config(in, bytes), sizeof(bytes));
    gw_sync::GroupConfig out;
    ASSERT_TRUE(gw_sync::decode_group_config(bytes, sizeof(bytes), out));
    EXPECT_EQ(out.slot, 2);
    EXPECT_EQ(out.pin_mask, 0x24);
    EXPECT_EQ(out.rate_millihz, 29'970u);
}

