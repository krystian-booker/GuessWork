#include <array>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "posest/teensy/Protocol.h"

// Lock in the on-the-wire *size* of every payload that crosses USB. This is a
// drift detector: the firmware's Protocol.h is a hand-mirrored copy of this
// header, and a contributor adding/removing/reordering a field on one side but
// not the other is the most likely break. The companion script
// scripts/check_protocol_constants.py asserts the enum / constant values agree.

namespace {

template <std::size_t Expected, typename Payload, typename Encoder>
void expectEncodedSize(const Payload& payload, Encoder&& encoder) {
    const auto bytes = encoder(payload);
    EXPECT_EQ(bytes.size(), Expected);
}

}  // namespace

TEST(TeensyProtocolGolden, FrameHeaderConstants) {
    EXPECT_EQ(posest::teensy::kFrameMagic, 0x4757u);
    EXPECT_EQ(posest::teensy::kProtocolVersion, 2u);
}

TEST(TeensyProtocolGolden, MessageTypeNumericValues) {
    EXPECT_EQ(static_cast<std::uint8_t>(posest::teensy::MessageType::ImuSample), 1u);
    EXPECT_EQ(static_cast<std::uint8_t>(posest::teensy::MessageType::CanRx), 3u);
    EXPECT_EQ(static_cast<std::uint8_t>(posest::teensy::MessageType::TeensyHealth), 4u);
    EXPECT_EQ(static_cast<std::uint8_t>(posest::teensy::MessageType::TimeSyncResponse), 5u);
    EXPECT_EQ(static_cast<std::uint8_t>(posest::teensy::MessageType::CameraTriggerEvent), 7u);
    EXPECT_EQ(static_cast<std::uint8_t>(posest::teensy::MessageType::ConfigAck), 8u);
    EXPECT_EQ(static_cast<std::uint8_t>(posest::teensy::MessageType::ChassisSpeeds), 9u);
    EXPECT_EQ(static_cast<std::uint8_t>(posest::teensy::MessageType::FusedPose), 64u);
    EXPECT_EQ(static_cast<std::uint8_t>(posest::teensy::MessageType::CanTx), 65u);
    EXPECT_EQ(static_cast<std::uint8_t>(posest::teensy::MessageType::TimeSyncRequest), 66u);
    EXPECT_EQ(static_cast<std::uint8_t>(posest::teensy::MessageType::ConfigCommand), 67u);
}

TEST(TeensyProtocolGolden, ConfigCommandKindValues) {
    EXPECT_EQ(static_cast<std::uint32_t>(posest::teensy::ConfigCommandKind::CameraTriggers), 1u);
    EXPECT_EQ(static_cast<std::uint32_t>(posest::teensy::ConfigCommandKind::ImuConfig), 2u);
}

TEST(TeensyProtocolGolden, StatusFlagBits) {
    EXPECT_EQ(posest::teensy::kStatusUnsynchronizedTime, 1u << 0u);
    EXPECT_EQ(posest::teensy::kStatusUnsynchronizedRioTime, 1u << 1u);
    EXPECT_EQ(posest::teensy::kStatusRobotSlipping, 1u << 2u);
    EXPECT_EQ(posest::teensy::kHealthRioUnsynchronized, 1u << 0u);
    EXPECT_EQ(posest::teensy::kHealthRioPingRejected, 1u << 1u);
    EXPECT_EQ(posest::teensy::kConfigAckUnsupportedCount, 1u << 0u);
    EXPECT_EQ(posest::teensy::kConfigAckInvalidPin, 1u << 1u);
    EXPECT_EQ(posest::teensy::kConfigAckDuplicatePin, 1u << 2u);
    EXPECT_EQ(posest::teensy::kConfigAckInvalidRate, 1u << 3u);
    EXPECT_EQ(posest::teensy::kConfigAckPulseTooWide, 1u << 4u);
    EXPECT_EQ(posest::teensy::kConfigAckInvalidImuConfig, 1u << 5u);
    EXPECT_EQ(posest::teensy::kConfigAckImuSelfTestFailure, 1u << 6u);
    EXPECT_EQ(posest::teensy::kConfigAckUnknownKind, 1u << 7u);
}

TEST(TeensyProtocolGolden, PayloadEncodedSizes) {
    expectEncodedSize<68>(posest::teensy::ImuPayload{},
                          posest::teensy::encodeImuPayload);
    expectEncodedSize<44>(posest::teensy::ChassisSpeedsPayload{},
                          posest::teensy::encodeChassisSpeedsPayload);
    expectEncodedSize<44>(posest::teensy::TeensyHealthPayload{},
                          posest::teensy::encodeTeensyHealthPayload);
    expectEncodedSize<20>(posest::teensy::CameraTriggerEventPayload{},
                          posest::teensy::encodeCameraTriggerEventPayload);
    expectEncodedSize<12>(posest::teensy::TimeSyncRequestPayload{},
                          posest::teensy::encodeTimeSyncRequestPayload);
    expectEncodedSize<20>(posest::teensy::TimeSyncResponsePayload{},
                          posest::teensy::encodeTimeSyncResponsePayload);
    expectEncodedSize<28>(posest::teensy::ImuConfigPayload{},
                          posest::teensy::encodeImuConfigPayload);
}

TEST(TeensyProtocolGolden, ImuPayloadEncodedBytesAreStable) {
    posest::teensy::ImuPayload payload;
    payload.teensy_time_us = 0x1122334455667788ULL;
    payload.accel_mps2 = {1.0, 2.0, 3.0};
    payload.gyro_radps = {4.0, 5.0, 6.0};
    payload.temperature_c = 25.5;
    payload.status_flags = 0xDEADBEEFu;

    const auto bytes = posest::teensy::encodeImuPayload(payload);
    ASSERT_EQ(bytes.size(), 68u);
    // teensy_time_us little-endian.
    EXPECT_EQ(bytes[0], 0x88u);
    EXPECT_EQ(bytes[7], 0x11u);
    // status_flags little-endian at offset 64.
    EXPECT_EQ(bytes[64], 0xEFu);
    EXPECT_EQ(bytes[65], 0xBEu);
    EXPECT_EQ(bytes[66], 0xADu);
    EXPECT_EQ(bytes[67], 0xDEu);

    const auto round_tripped = posest::teensy::decodeImuPayload(bytes);
    ASSERT_TRUE(round_tripped.has_value());
    EXPECT_EQ(round_tripped->teensy_time_us, payload.teensy_time_us);
    EXPECT_DOUBLE_EQ(*round_tripped->temperature_c, 25.5);
    EXPECT_EQ(round_tripped->status_flags, payload.status_flags);
}

TEST(TeensyProtocolGolden, ConfigAckLayoutMatchesFirmwareContract) {
    posest::teensy::ConfigAckPayload ack;
    ack.kind = static_cast<std::uint32_t>(posest::teensy::ConfigCommandKind::CameraTriggers);
    ack.status_flags = 0;
    ack.effective_count = 1;
    ack.trigger_entries.push_back({3, 33333u, 1000u});
    const auto bytes = posest::teensy::encodeConfigAckPayload(ack);
    // 12 byte header + 1 entry × 12 bytes.
    ASSERT_EQ(bytes.size(), 24u);
    // kind little-endian at offset 0.
    EXPECT_EQ(bytes[0], 1u);
    EXPECT_EQ(bytes[1], 0u);
    // effective_count at offset 8.
    EXPECT_EQ(bytes[8], 1u);
    // pin (signed 32-bit) at offset 12.
    EXPECT_EQ(bytes[12], 3u);
    EXPECT_EQ(bytes[13], 0u);
}
