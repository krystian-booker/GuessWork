#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>

// The freestanding firmware wire contract — compiled directly on the host.
#include "firmware/src/telemetry_payloads.h"

namespace telem = gw_fw::telem;

TEST(TelemetryPayloadsTest, FrameTypeConstants) {
    EXPECT_EQ(telem::kTelemetryMagic0, 0xA5);
    EXPECT_EQ(telem::kTelemetryMagic1, 0x5A);
    EXPECT_EQ(telem::kBinTypeImuBatch, 0x01);
    EXPECT_EQ(telem::kBinTypeHeartbeat, 0x02);
}

TEST(TelemetryPayloadsTest, Crc16CcittFalseGolden) {
    // Standard CRC-16/CCITT-FALSE check value: crc("123456789") = 0x29B1.
    const uint8_t check[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    EXPECT_EQ(telem::telemetry_crc16(check, sizeof(check)), 0x29B1);
    // Incremental feeding via the `crc` seed parameter matches one-shot.
    uint16_t crc = telem::telemetry_crc16(check, 4);
    crc          = telem::telemetry_crc16(check + 4, 5, crc);
    EXPECT_EQ(crc, 0x29B1);
}

TEST(TelemetryPayloadsTest, TelemetryFrameGoldenBytes) {
    // build_telemetry_frame is the host-side mirror of the firmware's
    // build_frame; golden-check every framing byte, CRC included.
    const uint8_t payload[3] = {0x01, 0x02, 0x03};
    uint8_t       frame[6 + 3];
    const size_t  n = telem::build_telemetry_frame(telem::kBinTypeHeartbeat,
                                                   payload, 3, frame);
    ASSERT_EQ(n, 9u);
    EXPECT_EQ(frame[0], 0xA5);
    EXPECT_EQ(frame[1], 0x5A);
    EXPECT_EQ(frame[2], 0x02);
    EXPECT_EQ(frame[3], 3);
    EXPECT_EQ(frame[4], 0x01);
    EXPECT_EQ(frame[5], 0x02);
    EXPECT_EQ(frame[6], 0x03);
    // CRC over type + len + payload = 0xAF62, little-endian on the wire.
    EXPECT_EQ(frame[7], 0x62);
    EXPECT_EQ(frame[8], 0xAF);
    EXPECT_EQ(telem::telemetry_crc16(frame + 2, 5), 0xAF62);
}

TEST(TelemetryPayloadsTest, TelemetryFrameZeroLengthPayload) {
    uint8_t      frame[6];
    const size_t n =
        telem::build_telemetry_frame(telem::kBinTypeImuBatch, nullptr, 0, frame);
    ASSERT_EQ(n, 6u);
    EXPECT_EQ(frame[0], 0xA5);
    EXPECT_EQ(frame[1], 0x5A);
    EXPECT_EQ(frame[2], 0x01);
    EXPECT_EQ(frame[3], 0);
    // CRC over {0x01, 0x00} = 0x2E3E, little-endian.
    EXPECT_EQ(frame[4], 0x3E);
    EXPECT_EQ(frame[5], 0x2E);
}

TEST(TelemetryPayloadsTest, LittleEndianPutGolden) {
    uint8_t buf[8 + 4 + 2 + 2 + 1 + 4] = {};
    size_t  off = 0;
    off = telem::le_put_u64(buf, off, 0x0123456789ABCDEFull);
    off = telem::le_put_u32(buf, off, 0xDEADBEEFu);
    off = telem::le_put_u16(buf, off, 0x1234);
    off = telem::le_put_i16(buf, off, -2);
    off = telem::le_put_u8(buf, off, 0x7F);
    off = telem::le_put_f32(buf, off, 1.0f);
    ASSERT_EQ(off, sizeof(buf));

    const uint8_t golden[] = {
        0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01,  // u64 LE
        0xEF, 0xBE, 0xAD, 0xDE,                          // u32 LE
        0x34, 0x12,                                      // u16 LE
        0xFE, 0xFF,                                      // i16 -2 LE
        0x7F,                                            // u8
        0x00, 0x00, 0x80, 0x3F,                          // f32 1.0 LE
    };
    static_assert(sizeof(golden) == sizeof(buf));
    EXPECT_EQ(std::memcmp(buf, golden, sizeof(golden)), 0);
}

TEST(TelemetryPayloadsTest, LittleEndianGetRoundTrip) {
    uint8_t buf[8 + 4 + 2 + 2 + 4] = {};
    size_t  off = 0;
    off = telem::le_put_u64(buf, off, 0xFEDCBA9876543210ull);
    off = telem::le_put_u32(buf, off, 0x89ABCDEFu);
    off = telem::le_put_u16(buf, off, 0xBEEF);
    off = telem::le_put_i16(buf, off, -32768);
    off = telem::le_put_f32(buf, off, -0.25f);
    ASSERT_EQ(off, sizeof(buf));

    EXPECT_EQ(telem::le_get_u64(buf, 0), 0xFEDCBA9876543210ull);
    EXPECT_EQ(telem::le_get_u32(buf, 8), 0x89ABCDEFu);
    EXPECT_EQ(telem::le_get_u16(buf, 12), 0xBEEF);
    EXPECT_EQ(telem::le_get_i16(buf, 14), -32768);
    EXPECT_FLOAT_EQ(telem::le_get_f32(buf, 16), -0.25f);
}
