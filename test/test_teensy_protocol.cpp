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

TEST(TeensyProtocol, WheelOdometryPayloadRoundTrips) {
    posest::teensy::WheelOdometryPayload payload;
    payload.teensy_time_us = 99;
    payload.chassis_delta = {1.0, 2.0, 3.0};
    payload.wheel_delta_m = {0.1, 0.2, 0.3, 0.4};
    payload.status_flags = 7;

    const auto decoded = posest::teensy::decodeWheelOdometryPayload(
        posest::teensy::encodeWheelOdometryPayload(payload));

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->teensy_time_us, 99u);
    EXPECT_DOUBLE_EQ(decoded->chassis_delta.theta_rad, 3.0);
    EXPECT_DOUBLE_EQ(decoded->wheel_delta_m[3], 0.4);
    EXPECT_EQ(decoded->status_flags, 7u);
}

TEST(TeensyProtocol, RobotOdometryPayloadRoundTrips) {
    posest::teensy::RobotOdometryPayload payload;
    payload.teensy_time_us = 1234;
    payload.rio_time_us = 5678;
    payload.field_to_robot = {1.0, 2.0, 3.0};
    payload.status_flags = posest::teensy::kStatusRobotSlipping |
                           posest::teensy::kStatusUnsynchronizedRioTime;

    const auto decoded = posest::teensy::decodeRobotOdometryPayload(
        posest::teensy::encodeRobotOdometryPayload(payload));

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->teensy_time_us, 1234u);
    EXPECT_EQ(decoded->rio_time_us, 5678u);
    EXPECT_DOUBLE_EQ(decoded->field_to_robot.x_m, 1.0);
    EXPECT_DOUBLE_EQ(decoded->field_to_robot.y_m, 2.0);
    EXPECT_DOUBLE_EQ(decoded->field_to_robot.theta_rad, 3.0);
    EXPECT_NE(decoded->status_flags & posest::teensy::kStatusRobotSlipping, 0u);
    EXPECT_NE(decoded->status_flags & posest::teensy::kStatusUnsynchronizedRioTime, 0u);
}

TEST(TeensyProtocol, RejectsBadPayloadSizes) {
    EXPECT_FALSE(posest::teensy::decodeImuPayload({1, 2, 3}).has_value());
    EXPECT_FALSE(posest::teensy::decodeWheelOdometryPayload({1, 2, 3}).has_value());
    EXPECT_FALSE(posest::teensy::decodeRobotOdometryPayload({1, 2, 3}).has_value());
    EXPECT_FALSE(posest::teensy::decodeTimeSyncResponsePayload({1, 2, 3}).has_value());
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
