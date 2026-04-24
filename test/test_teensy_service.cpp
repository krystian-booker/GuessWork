#include <chrono>
#include <functional>
#include <stdexcept>
#include <thread>

#include <gtest/gtest.h>

#include "posest/MeasurementBus.h"
#include "posest/teensy/FakeSerialTransport.h"
#include "posest/teensy/TeensyService.h"

using namespace std::chrono_literals;

namespace {

posest::runtime::TeensyConfig fastConfig() {
    posest::runtime::TeensyConfig config;
    config.serial_port = "/dev/fake-teensy";
    config.reconnect_interval_ms = 5;
    config.read_timeout_ms = 2;
    return config;
}

bool waitFor(
    const std::function<bool()>& predicate,
    std::chrono::milliseconds timeout = 500ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(2ms);
    }
    return predicate();
}

posest::teensy::Frame decodeWrittenFrame(
    const std::vector<std::vector<std::uint8_t>>& writes,
    posest::teensy::MessageType type) {
    for (const auto& bytes : writes) {
        auto decoded = posest::teensy::decodeFrame(bytes);
        if (decoded && decoded->frame.type == type) {
            return decoded->frame;
        }
    }
    throw std::runtime_error("expected written frame was not found");
}

}  // namespace

TEST(TeensyService, DisabledEmptySerialPortStartsAndStopsWithoutOpening) {
    posest::MeasurementBus bus(4);
    posest::teensy::FakeSerialTransport fake;
    auto state = fake.sharedState();

    posest::runtime::TeensyConfig config;
    posest::teensy::TeensyService service(
        config,
        {},
        bus,
        [state] { return std::make_unique<posest::teensy::FakeSerialTransport>(state); });

    service.start();
    service.stop();

    EXPECT_FALSE(service.stats().enabled);
    EXPECT_EQ(fake.openCount(), 0u);
}

TEST(TeensyService, PublishesInboundImuAndWheelOdometrySamples) {
    posest::MeasurementBus bus(8);
    posest::teensy::FakeSerialTransport fake;
    auto state = fake.sharedState();
    posest::teensy::TeensyService service(
        fastConfig(),
        {},
        bus,
        [state] { return std::make_unique<posest::teensy::FakeSerialTransport>(state); });

    service.start();
    ASSERT_TRUE(fake.waitForOpenCount(1, 500ms));

    posest::teensy::ImuPayload imu_payload;
    imu_payload.teensy_time_us = 10;
    imu_payload.accel_mps2 = {1.0, 2.0, 3.0};
    imu_payload.gyro_radps = {4.0, 5.0, 6.0};

    posest::teensy::Frame imu_frame;
    imu_frame.type = posest::teensy::MessageType::ImuSample;
    imu_frame.sequence = 0;
    imu_frame.payload = posest::teensy::encodeImuPayload(imu_payload);
    fake.pushReadBytes(posest::teensy::encodeFrame(imu_frame));

    posest::teensy::WheelOdometryPayload odom_payload;
    odom_payload.teensy_time_us = 20;
    odom_payload.chassis_delta = {0.1, 0.2, 0.3};
    odom_payload.wheel_delta_m = {1.0, 2.0, 3.0, 4.0};

    posest::teensy::Frame odom_frame;
    odom_frame.type = posest::teensy::MessageType::WheelOdometry;
    odom_frame.sequence = 1;
    odom_frame.payload = posest::teensy::encodeWheelOdometryPayload(odom_payload);
    fake.pushReadBytes(posest::teensy::encodeFrame(odom_frame));

    ASSERT_TRUE(waitFor([&] {
        const auto stats = service.stats();
        return stats.inbound_imu_samples == 1u &&
               stats.inbound_wheel_odometry_samples == 1u;
    }));

    auto first = bus.take();
    auto second = bus.take();
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());

    const auto& imu = std::get<posest::ImuSample>(*first);
    EXPECT_DOUBLE_EQ(imu.accel_mps2.y, 2.0);
    EXPECT_NE(imu.status_flags & posest::teensy::kStatusUnsynchronizedTime, 0u);

    const auto& odom = std::get<posest::WheelOdometrySample>(*second);
    EXPECT_DOUBLE_EQ(odom.chassis_delta.theta_rad, 0.3);
    ASSERT_EQ(odom.wheel_delta_m.size(), 4u);
    EXPECT_DOUBLE_EQ(odom.wheel_delta_m[3], 4.0);

    service.stop();
}

TEST(TeensyService, TracksCrcFailuresSequenceGapsAndInvalidPayloads) {
    posest::MeasurementBus bus(8);
    posest::teensy::FakeSerialTransport fake;
    auto state = fake.sharedState();
    posest::teensy::TeensyService service(
        fastConfig(),
        {},
        bus,
        [state] { return std::make_unique<posest::teensy::FakeSerialTransport>(state); });

    service.start();
    ASSERT_TRUE(fake.waitForOpenCount(1, 500ms));

    posest::teensy::Frame invalid_payload;
    invalid_payload.type = posest::teensy::MessageType::ImuSample;
    invalid_payload.sequence = 1;
    invalid_payload.payload = {1, 2, 3};
    fake.pushReadBytes(posest::teensy::encodeFrame(invalid_payload));

    posest::teensy::Frame gap;
    gap.type = posest::teensy::MessageType::TeensyHealth;
    gap.sequence = 3;
    gap.payload = posest::teensy::encodeTeensyHealthPayload({});
    auto gap_bytes = posest::teensy::encodeFrame(gap);
    gap_bytes.back() ^= 0xFFu;
    fake.pushReadBytes(gap_bytes);

    gap_bytes = posest::teensy::encodeFrame(gap);
    fake.pushReadBytes(gap_bytes);

    ASSERT_TRUE(waitFor([&] {
        const auto stats = service.stats();
        return stats.invalid_payloads >= 1u &&
               stats.crc_failures >= 1u &&
               stats.sequence_gaps >= 1u;
    }));

    service.stop();
}

TEST(TeensyService, SendsFusedPoseFramesAndDropsWhenQueueIsFull) {
    posest::MeasurementBus bus(4);
    posest::teensy::FakeSerialTransport fake;
    auto state = fake.sharedState();
    posest::teensy::TeensyService service(
        fastConfig(),
        {},
        bus,
        [state] { return std::make_unique<posest::teensy::FakeSerialTransport>(state); });

    service.start();
    ASSERT_TRUE(fake.waitForOpenCount(1, 500ms));

    posest::FusedPoseEstimate estimate;
    estimate.field_to_robot = {1.0, 2.0, 3.0};
    service.publish(estimate);

    ASSERT_TRUE(waitFor([&] {
        return service.stats().outbound_frames_sent >= 3u;
    }));
    const auto fused = decodeWrittenFrame(
        fake.writes(),
        posest::teensy::MessageType::FusedPose);
    EXPECT_EQ(fused.type, posest::teensy::MessageType::FusedPose);
    service.stop();

    posest::MeasurementBus unused_bus(4);
    posest::teensy::TeensyService stopped_service(
        fastConfig(),
        {},
        unused_bus,
        [state] { return std::make_unique<posest::teensy::FakeSerialTransport>(state); });
    for (int i = 0; i < 65; ++i) {
        stopped_service.publish(estimate);
    }
    EXPECT_EQ(stopped_service.stats().outbound_frames_queued, 64u);
    EXPECT_EQ(stopped_service.stats().outbound_frames_dropped, 1u);
}

TEST(TeensyService, ReconnectsAndResendsCameraTriggerConfig) {
    posest::MeasurementBus bus(4);
    posest::teensy::FakeSerialTransport fake;
    auto state = fake.sharedState();

    std::vector<posest::runtime::CameraTriggerConfig> triggers;
    triggers.push_back({"cam0", true, 4, 25.0, 1000, 0});

    posest::teensy::TeensyService service(
        fastConfig(),
        triggers,
        bus,
        [state] { return std::make_unique<posest::teensy::FakeSerialTransport>(state); });

    service.start();
    ASSERT_TRUE(fake.waitForOpenCount(1, 500ms));
    fake.failNextReads(1);
    ASSERT_TRUE(fake.waitForOpenCount(2, 500ms));

    std::size_t trigger_config_writes = 0;
    for (const auto& bytes : fake.writes()) {
        auto decoded = posest::teensy::decodeFrame(bytes);
        if (decoded && decoded->frame.type == posest::teensy::MessageType::ConfigCommand) {
            ++trigger_config_writes;
        }
    }
    EXPECT_GE(trigger_config_writes, 2u);
    EXPECT_GE(service.stats().disconnects, 1u);

    service.stop();
}

TEST(TeensyService, TimeSyncResponseEstablishesClockOffset) {
    posest::MeasurementBus bus(4);
    posest::teensy::FakeSerialTransport fake;
    auto state = fake.sharedState();
    posest::teensy::TeensyService service(
        fastConfig(),
        {},
        bus,
        [state] { return std::make_unique<posest::teensy::FakeSerialTransport>(state); });

    service.start();
    ASSERT_TRUE(fake.waitForOpenCount(1, 500ms));
    ASSERT_TRUE(fake.waitForWriteCount(2, 500ms));

    const auto request_frame = decodeWrittenFrame(
        fake.writes(),
        posest::teensy::MessageType::TimeSyncRequest);
    const auto request = posest::teensy::decodeTimeSyncRequestPayload(request_frame.payload);
    ASSERT_TRUE(request.has_value());

    posest::teensy::TimeSyncResponsePayload response;
    response.request_sequence = request->request_sequence;
    response.teensy_receive_time_us = request->host_send_time_us + 100;
    response.teensy_transmit_time_us = request->host_send_time_us + 200;

    posest::teensy::Frame frame;
    frame.type = posest::teensy::MessageType::TimeSyncResponse;
    frame.sequence = 0;
    frame.payload = posest::teensy::encodeTimeSyncResponsePayload(response);
    fake.pushReadBytes(posest::teensy::encodeFrame(frame));

    ASSERT_TRUE(waitFor([&] { return service.stats().time_sync_established; }));

    service.stop();
}
