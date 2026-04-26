#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "posest/teensy/Protocol.h"

TEST(TeensyProtocol, EncodesAndDecodesFrameRoundTrip) {
    posest::teensy::Frame frame;
    frame.type = posest::teensy::MessageType::ImuSample;
    frame.sequence = 42;
    frame.payload = {1, 2, 3, 4};

    const auto bytes = posest::teensy::encodeFrame(frame);
    const auto decoded = posest::teensy::decodeFrame(bytes);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->frame.type, frame.type);
    EXPECT_EQ(decoded->frame.sequence, frame.sequence);
    EXPECT_EQ(decoded->frame.payload, frame.payload);
    EXPECT_EQ(decoded->bytes_consumed, bytes.size());
}

TEST(TeensyProtocol, RejectsCorruptCrc) {
    posest::teensy::Frame frame;
    frame.type = posest::teensy::MessageType::CanRx;
    frame.payload = {9, 8, 7};

    auto bytes = posest::teensy::encodeFrame(frame);
    bytes.back() ^= 0xFFu;

    EXPECT_FALSE(posest::teensy::decodeFrame(bytes).has_value());
}

TEST(TeensyProtocol, StreamDecoderHandlesChunksAndSequenceGaps) {
    posest::teensy::Frame a;
    a.sequence = 1;
    a.payload = {1};
    posest::teensy::Frame b;
    b.sequence = 3;
    b.payload = {2};

    auto bytes_a = posest::teensy::encodeFrame(a);
    auto bytes_b = posest::teensy::encodeFrame(b);
    std::vector<std::uint8_t> combined;
    combined.insert(combined.end(), bytes_a.begin(), bytes_a.end());
    combined.insert(combined.end(), bytes_b.begin(), bytes_b.end());

    posest::teensy::StreamDecoder decoder;
    const auto frames = decoder.push(combined.data(), combined.size());

    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[0].sequence, 1u);
    EXPECT_EQ(frames[1].sequence, 3u);
    EXPECT_EQ(decoder.stats().sequence_gaps, 1u);
}

TEST(TeensyProtocol, ImuPayloadRoundTrips) {
    posest::teensy::ImuPayload payload;
    payload.teensy_time_us = 123456;
    payload.accel_mps2 = {1.0, 2.0, 3.0};
    payload.gyro_radps = {4.0, 5.0, 6.0};
    payload.temperature_c = 42.5;
    payload.status_flags = 0xA5;

    const auto decoded = posest::teensy::decodeImuPayload(
        posest::teensy::encodeImuPayload(payload));

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->teensy_time_us, payload.teensy_time_us);
    EXPECT_DOUBLE_EQ(decoded->accel_mps2.x, 1.0);
    EXPECT_DOUBLE_EQ(decoded->gyro_radps.z, 6.0);
    ASSERT_TRUE(decoded->temperature_c.has_value());
    EXPECT_DOUBLE_EQ(*decoded->temperature_c, 42.5);
    EXPECT_EQ(decoded->status_flags, 0xA5u);
}

TEST(TeensyProtocol, ChassisSpeedsPayloadRoundTrips) {
    posest::teensy::ChassisSpeedsPayload payload;
    payload.teensy_time_us = 1234;
    payload.rio_time_us = 5678;
    payload.vx_mps = 1.5;
    payload.vy_mps = -0.25;
    payload.omega_radps = 0.75;
    payload.status_flags = posest::teensy::kStatusUnsynchronizedRioTime;

    const auto bytes = posest::teensy::encodeChassisSpeedsPayload(payload);
    EXPECT_EQ(bytes.size(), 44u);
    const auto decoded = posest::teensy::decodeChassisSpeedsPayload(bytes);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->teensy_time_us, 1234u);
    EXPECT_EQ(decoded->rio_time_us, 5678u);
    EXPECT_DOUBLE_EQ(decoded->vx_mps, 1.5);
    EXPECT_DOUBLE_EQ(decoded->vy_mps, -0.25);
    EXPECT_DOUBLE_EQ(decoded->omega_radps, 0.75);
    EXPECT_NE(decoded->status_flags & posest::teensy::kStatusUnsynchronizedRioTime, 0u);
}

TEST(TeensyProtocol, ChassisSpeedsPayloadRejectsWrongSize) {
    EXPECT_FALSE(posest::teensy::decodeChassisSpeedsPayload(
        std::vector<std::uint8_t>(43, 0)).has_value());
    EXPECT_FALSE(posest::teensy::decodeChassisSpeedsPayload(
        std::vector<std::uint8_t>(45, 0)).has_value());
}

TEST(TeensyProtocol, CameraTriggerEventPayloadRoundTrips) {
    posest::teensy::CameraTriggerEventPayload payload;
    payload.teensy_time_us = 123456;
    payload.pin = 6;
    payload.trigger_sequence = 42;
    payload.status_flags = 7;

    const auto decoded = posest::teensy::decodeCameraTriggerEventPayload(
        posest::teensy::encodeCameraTriggerEventPayload(payload));

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->teensy_time_us, 123456u);
    EXPECT_EQ(decoded->pin, 6);
    EXPECT_EQ(decoded->trigger_sequence, 42u);
    EXPECT_EQ(decoded->status_flags, 7u);
}

TEST(TeensyProtocol, TeensyHealthPayloadRoundTripsExtended) {
    posest::teensy::TeensyHealthPayload payload;
    payload.uptime_us = 1234;
    payload.error_flags = 0xA5;
    payload.trigger_status_flags = 0x10;
    payload.rx_queue_depth = 7;
    payload.tx_queue_depth = 3;
    payload.rio_offset_us = -987654;
    payload.rio_status_flags = posest::teensy::kHealthRioPingRejected;
    payload.accel_saturations = 11;
    payload.gyro_saturations = 22;

    const auto bytes = posest::teensy::encodeTeensyHealthPayload(payload);
    EXPECT_EQ(bytes.size(), 44u);
    const auto decoded = posest::teensy::decodeTeensyHealthPayload(bytes);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->uptime_us, 1234u);
    EXPECT_EQ(decoded->rio_offset_us, -987654);
    EXPECT_EQ(decoded->rio_status_flags, posest::teensy::kHealthRioPingRejected);
    EXPECT_EQ(decoded->accel_saturations, 11u);
    EXPECT_EQ(decoded->gyro_saturations, 22u);
}

TEST(TeensyProtocol, TeensyHealthPayloadAcceptsLegacy24ByteForm) {
    // Legacy firmware stamped just the original 24 bytes; the host must keep
    // accepting that form (rio fields default to zero) so older firmware can
    // still report basic health while it's being reflashed.
    posest::teensy::TeensyHealthPayload payload;
    payload.uptime_us = 99;
    payload.error_flags = 1;
    payload.trigger_status_flags = 2;
    payload.rx_queue_depth = 3;
    payload.tx_queue_depth = 4;
    payload.rio_offset_us = 0;
    payload.rio_status_flags = 0;
    payload.accel_saturations = 0;
    payload.gyro_saturations = 0;

    auto bytes = posest::teensy::encodeTeensyHealthPayload(payload);
    bytes.resize(24);
    const auto decoded = posest::teensy::decodeTeensyHealthPayload(bytes);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->uptime_us, 99u);
    EXPECT_EQ(decoded->rio_offset_us, 0);
    EXPECT_EQ(decoded->rio_status_flags, 0u);
}

TEST(TeensyProtocol, RejectsBadPayloadSizes) {
    EXPECT_FALSE(posest::teensy::decodeImuPayload({1, 2, 3}).has_value());
    EXPECT_FALSE(posest::teensy::decodeChassisSpeedsPayload({1, 2, 3}).has_value());
    EXPECT_FALSE(posest::teensy::decodeCameraTriggerEventPayload({1, 2, 3}).has_value());
    EXPECT_FALSE(posest::teensy::decodeTimeSyncResponsePayload({1, 2, 3}).has_value());
}

TEST(TeensyProtocol, ConfigAckCameraTriggersRoundTrips) {
    posest::teensy::ConfigAckPayload payload;
    payload.kind = static_cast<std::uint32_t>(
        posest::teensy::ConfigCommandKind::CameraTriggers);
    payload.status_flags = posest::teensy::kConfigAckInvalidRate;
    payload.effective_count = 2;
    payload.trigger_entries.push_back({2, 33333u, 1000u});
    payload.trigger_entries.push_back({4, 8333u, 200u});

    const auto bytes = posest::teensy::encodeConfigAckPayload(payload);
    const auto decoded = posest::teensy::decodeConfigAckPayload(bytes);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->kind, payload.kind);
    EXPECT_EQ(decoded->status_flags, posest::teensy::kConfigAckInvalidRate);
    EXPECT_EQ(decoded->effective_count, 2u);
    ASSERT_EQ(decoded->trigger_entries.size(), 2u);
    EXPECT_EQ(decoded->trigger_entries[0].pin, 2);
    EXPECT_EQ(decoded->trigger_entries[1].period_us, 8333u);
    EXPECT_EQ(decoded->trigger_entries[1].pulse_us, 200u);
    EXPECT_FALSE(decoded->imu_entry.has_value());
}

TEST(TeensyProtocol, ConfigAckImuConfigRoundTrips) {
    posest::teensy::ConfigAckPayload payload;
    payload.kind = static_cast<std::uint32_t>(
        posest::teensy::ConfigCommandKind::ImuConfig);
    payload.status_flags = 0;
    payload.effective_count = 1;
    posest::teensy::ImuConfigAckEntry entry;
    entry.accel_range_g = 6;
    entry.accel_odr_hz = 800;
    entry.accel_bandwidth_code = 2;
    entry.gyro_range_dps = 1000;
    entry.gyro_bandwidth_code = 1;
    entry.data_sync_rate_hz = 1000;
    entry.reserved = 0;
    payload.imu_entry = entry;

    const auto bytes = posest::teensy::encodeConfigAckPayload(payload);
    const auto decoded = posest::teensy::decodeConfigAckPayload(bytes);

    ASSERT_TRUE(decoded.has_value());
    ASSERT_TRUE(decoded->imu_entry.has_value());
    EXPECT_EQ(decoded->imu_entry->accel_range_g, 6u);
    EXPECT_EQ(decoded->imu_entry->gyro_range_dps, 1000u);
    EXPECT_TRUE(decoded->trigger_entries.empty());
}

TEST(TeensyProtocol, ConfigAckRejectsTruncatedPayload) {
    EXPECT_FALSE(posest::teensy::decodeConfigAckPayload({1, 2, 3}).has_value());
}

TEST(TeensyProtocol, ImuConfigPayloadRoundTrips) {
    posest::teensy::ImuConfigPayload payload;
    payload.accel_range_g = 24;
    payload.accel_odr_hz = 1600;
    payload.accel_bandwidth_code = 1;
    payload.gyro_range_dps = 500;
    payload.gyro_bandwidth_code = 3;
    payload.data_sync_rate_hz = 2000;
    payload.run_selftest_on_boot = 0;

    const auto bytes = posest::teensy::encodeImuConfigPayload(payload);
    EXPECT_EQ(bytes.size(), 28u);
    const auto decoded = posest::teensy::decodeImuConfigPayload(bytes);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->accel_range_g, 24u);
    EXPECT_EQ(decoded->data_sync_rate_hz, 2000u);
    EXPECT_EQ(decoded->run_selftest_on_boot, 0u);
}

TEST(TeensyProtocol, TimeSyncPayloadRoundTrips) {
    posest::teensy::TimeSyncResponsePayload payload;
    payload.request_sequence = 12;
    payload.teensy_receive_time_us = 1000;
    payload.teensy_transmit_time_us = 1100;

    const auto decoded = posest::teensy::decodeTimeSyncResponsePayload(
        posest::teensy::encodeTimeSyncResponsePayload(payload));

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->request_sequence, 12u);
    EXPECT_EQ(decoded->teensy_receive_time_us, 1000u);
    EXPECT_EQ(decoded->teensy_transmit_time_us, 1100u);
}
