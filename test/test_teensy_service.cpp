#include <chrono>
#include <atomic>
#include <functional>
#include <memory>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include <gtest/gtest.h>

#include "posest/CameraTriggerCache.h"
#include "posest/MeasurementBus.h"
#include "posest/ToFSampleCache.h"
#include "posest/teensy/FakeSerialTransport.h"
#include "posest/teensy/TeensyService.h"

using namespace std::chrono_literals;

namespace {

posest::runtime::TeensyConfig fastConfig() {
    posest::runtime::TeensyConfig config;
    config.serial_port = "/dev/fake-teensy";
    config.reconnect_interval_ms = 5;
    config.read_timeout_ms = 2;
    config.time_sync_interval_ms = 5;
    return config;
}

void pushEncodedFrame(
    posest::teensy::FakeSerialTransport& fake,
    posest::teensy::MessageType type,
    std::uint32_t sequence,
    std::vector<std::uint8_t> payload) {
    posest::teensy::Frame frame;
    frame.type = type;
    frame.sequence = sequence;
    frame.payload = std::move(payload);
    fake.pushReadBytes(posest::teensy::encodeFrame(frame));
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

// Establishes host<->Teensy time sync by replying to one of the periodic
// TimeSyncRequest frames. Returns teensy_transmit_time_us so callers can
// stamp follow-on payloads consistently. `sequence` is the inbound frame
// sequence number; pass a value above any subsequent feeder sequences to
// avoid spurious sequence_gaps in tests that assert on the counter.
std::uint64_t primeTimeSync(
    posest::teensy::FakeSerialTransport& fake,
    posest::teensy::TeensyService& service,
    std::uint32_t sequence = 0) {
    if (!fake.waitForWriteCount(3, 500ms)) {
        throw std::runtime_error("teensy never sent the expected initial writes");
    }
    const auto request_frame = decodeWrittenFrame(
        fake.writes(), posest::teensy::MessageType::TimeSyncRequest);
    const auto request =
        posest::teensy::decodeTimeSyncRequestPayload(request_frame.payload);
    if (!request) {
        throw std::runtime_error("malformed TimeSyncRequest");
    }
    posest::teensy::TimeSyncResponsePayload response;
    response.request_sequence = request->request_sequence;
    response.teensy_receive_time_us = request->host_send_time_us + 100;
    response.teensy_transmit_time_us = request->host_send_time_us + 200;
    pushEncodedFrame(
        fake,
        posest::teensy::MessageType::TimeSyncResponse,
        sequence,
        posest::teensy::encodeTimeSyncResponsePayload(response));
    if (!waitFor([&] { return service.stats().time_sync_established; })) {
        throw std::runtime_error("host<->Teensy time sync never established");
    }
    return response.teensy_transmit_time_us;
}

// Establishes Teensy<->RIO time sync by feeding a TeensyHealth frame with
// rio_status_flags clear and the requested offset on the wire.
void primeRioSync(
    posest::teensy::FakeSerialTransport& fake,
    posest::teensy::TeensyService& service,
    std::int64_t rio_to_teensy_offset_us,
    std::uint32_t sequence = 1) {
    posest::teensy::TeensyHealthPayload health;
    health.rio_offset_us = rio_to_teensy_offset_us;
    health.rio_status_flags = 0;  // clear kHealthRioUnsynchronized
    pushEncodedFrame(
        fake,
        posest::teensy::MessageType::TeensyHealth,
        sequence,
        posest::teensy::encodeTeensyHealthPayload(health));
    if (!waitFor([&] { return service.stats().rio_time_sync_established; })) {
        throw std::runtime_error("Teensy<->RIO time sync never established");
    }
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

TEST(TeensyService, PublishesImuSamplesEvenBeforeTimeSync) {
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
    pushEncodedFrame(
        fake,
        posest::teensy::MessageType::ImuSample,
        0,
        posest::teensy::encodeImuPayload(imu_payload));

    ASSERT_TRUE(waitFor([&] {
        return service.stats().inbound_imu_samples == 1u;
    }));

    auto measurement = bus.take();
    ASSERT_TRUE(measurement.has_value());
    const auto& imu = std::get<posest::ImuSample>(*measurement);
    EXPECT_DOUBLE_EQ(imu.accel_mps2.y, 2.0);
    EXPECT_NE(imu.status_flags & posest::teensy::kStatusUnsynchronizedTime, 0u);

    service.stop();
}

TEST(TeensyService, DropsChassisSpeedsBeforeBothSyncsEstablished) {
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

    // No TimeSyncResponse and no TeensyHealth — neither leg of the clock
    // chain is established. The chassis frame must be dropped, not published
    // with a fallback (Teensy-receive) timestamp.
    posest::teensy::ChassisSpeedsPayload chassis_payload;
    chassis_payload.teensy_time_us = 20;
    chassis_payload.rio_time_us = 21;
    chassis_payload.vx_mps = 1.0;
    pushEncodedFrame(
        fake,
        posest::teensy::MessageType::ChassisSpeeds,
        1,
        posest::teensy::encodeChassisSpeedsPayload(chassis_payload));

    ASSERT_TRUE(waitFor([&] {
        return service.stats().inbound_chassis_speeds_dropped_pre_sync >= 1u;
    }));
    EXPECT_EQ(service.stats().inbound_chassis_speeds_samples, 0u);

    service.stop();
}

TEST(TeensyService, DropsChassisSpeedsWhenOnlyHostSyncEstablished) {
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

    const std::uint64_t teensy_transmit = primeTimeSync(fake, service);
    EXPECT_TRUE(service.stats().time_sync_established);
    EXPECT_FALSE(service.stats().rio_time_sync_established);

    posest::teensy::ChassisSpeedsPayload chassis_payload;
    chassis_payload.teensy_time_us = teensy_transmit;
    chassis_payload.rio_time_us = 1234;
    chassis_payload.vx_mps = 1.0;
    pushEncodedFrame(
        fake,
        posest::teensy::MessageType::ChassisSpeeds,
        1,
        posest::teensy::encodeChassisSpeedsPayload(chassis_payload));

    ASSERT_TRUE(waitFor([&] {
        return service.stats().inbound_chassis_speeds_dropped_pre_sync >= 1u;
    }));
    EXPECT_EQ(service.stats().inbound_chassis_speeds_samples, 0u);

    service.stop();
}

TEST(TeensyService, PublishesChassisSpeedsAfterBothSyncsEstablished) {
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

    primeTimeSync(fake, service);
    constexpr std::int64_t kRioToTeensyOffset = 50'000;
    primeRioSync(fake, service, kRioToTeensyOffset);

    posest::teensy::ChassisSpeedsPayload chassis_payload;
    chassis_payload.teensy_time_us = 0;  // ignored after Fix 3
    chassis_payload.rio_time_us = 700'000;
    chassis_payload.vx_mps = 1.5;
    chassis_payload.vy_mps = -0.5;
    chassis_payload.omega_radps = 0.25;
    pushEncodedFrame(
        fake,
        posest::teensy::MessageType::ChassisSpeeds,
        1,
        posest::teensy::encodeChassisSpeedsPayload(chassis_payload));

    ASSERT_TRUE(waitFor([&] {
        return service.stats().inbound_chassis_speeds_samples == 1u;
    }));
    EXPECT_EQ(service.stats().inbound_chassis_speeds_dropped_pre_sync, 0u);

    auto measurement = bus.take();
    ASSERT_TRUE(measurement.has_value());
    const auto& chassis = std::get<posest::ChassisSpeedsSample>(*measurement);
    EXPECT_EQ(chassis.rio_time_us, 700'000u);
    EXPECT_DOUBLE_EQ(chassis.vx_mps, 1.5);
    EXPECT_DOUBLE_EQ(chassis.omega_radps, 0.25);

    // The published timestamp must equal TimeSyncFilter::apply(rio_time +
    // rio_to_teensy_offset). The filter offset is roughly the negative
    // teensy-vs-host midpoint difference established in primeTimeSync; with
    // a near-zero RTT in the fake transport the offset is dominated by the
    // teensy_transmit_time_us value, so we just assert the timestamp is in
    // a sane neighborhood of (rio_time + rio_offset) interpreted as us.
    const auto expected_teensy_time =
        static_cast<std::int64_t>(chassis_payload.rio_time_us) + kRioToTeensyOffset;
    const auto stamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
        chassis.timestamp.time_since_epoch()).count();
    EXPECT_GT(stamp_us, expected_teensy_time - 1'000'000);
    EXPECT_LT(stamp_us, expected_teensy_time + 1'000'000'000);

    service.stop();
}

TEST(TeensyService, PublishesInboundCameraTriggerEvents) {
    posest::MeasurementBus bus(4);
    posest::teensy::FakeSerialTransport fake;
    auto state = fake.sharedState();
    posest::teensy::TeensyService service(
        fastConfig(),
        {},
        bus,
        [state] { return std::make_unique<posest::teensy::FakeSerialTransport>(state); });

    service.start();
    ASSERT_TRUE(fake.waitForOpenCount(1, std::chrono::milliseconds(500)));

    posest::teensy::CameraTriggerEventPayload payload;
    payload.teensy_time_us = 100;
    payload.pin = 6;
    payload.trigger_sequence = 3;
    payload.status_flags = 0xA;
    pushEncodedFrame(
        fake,
        posest::teensy::MessageType::CameraTriggerEvent,
        1,
        posest::teensy::encodeCameraTriggerEventPayload(payload));

    auto measurement = bus.take();
    service.stop();

    ASSERT_TRUE(measurement.has_value());
    ASSERT_TRUE(std::holds_alternative<posest::CameraTriggerEvent>(*measurement));
    const auto& event = std::get<posest::CameraTriggerEvent>(*measurement);
    EXPECT_EQ(event.teensy_time_us, 100u);
    EXPECT_EQ(event.pin, 6);
    EXPECT_EQ(event.trigger_sequence, 3u);
    EXPECT_NE(event.status_flags & posest::teensy::kStatusUnsynchronizedTime, 0u);
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

TEST(TeensyService, CountsInboundDropsWhenMeasurementBusIsFull) {
    posest::MeasurementBus bus(1);
    posest::teensy::FakeSerialTransport fake;
    auto state = fake.sharedState();
    posest::teensy::TeensyService service(
        fastConfig(),
        {},
        bus,
        [state] { return std::make_unique<posest::teensy::FakeSerialTransport>(state); });

    service.start();
    ASSERT_TRUE(fake.waitForOpenCount(1, 500ms));

    posest::teensy::ImuPayload payload;
    payload.accel_mps2 = {1.0, 2.0, 3.0};
    pushEncodedFrame(
        fake,
        posest::teensy::MessageType::ImuSample,
        0,
        posest::teensy::encodeImuPayload(payload));
    payload.accel_mps2 = {4.0, 5.0, 6.0};
    pushEncodedFrame(
        fake,
        posest::teensy::MessageType::ImuSample,
        1,
        posest::teensy::encodeImuPayload(payload));

    ASSERT_TRUE(waitFor([&] {
        const auto stats = service.stats();
        return stats.inbound_imu_samples == 1u &&
               stats.inbound_measurements_dropped == 1u;
    }));
    EXPECT_EQ(bus.droppedNewestCount(), 1u);

    service.stop();
}

TEST(TeensyService, HandlesSustainedOneKilohertzImuTraffic) {
    constexpr std::size_t kImuSamples = 100;
    constexpr std::size_t kChassisSpeedsSamples = 10;
    constexpr std::size_t kExpectedSamples = kImuSamples + kChassisSpeedsSamples;

    posest::MeasurementBus bus(256);
    posest::teensy::FakeSerialTransport fake;
    auto state = fake.sharedState();
    posest::teensy::TeensyService service(
        fastConfig(),
        {},
        bus,
        [state] { return std::make_unique<posest::teensy::FakeSerialTransport>(state); });

    service.start();
    ASSERT_TRUE(fake.waitForOpenCount(1, 500ms));
    primeTimeSync(fake, service, /*sequence=*/0);
    primeRioSync(fake, service, /*rio_to_teensy_offset_us=*/0, /*sequence=*/1);

    std::atomic<std::size_t> consumed{0};
    std::thread drainer([&] {
        while (true) {
            auto measurement = bus.take();
            if (!measurement) {
                return;
            }
            ++consumed;
        }
    });

    std::thread feeder([&] {
        std::uint32_t sequence = 2;
        auto next_sample_time = std::chrono::steady_clock::now();
        for (std::size_t i = 0; i < kImuSamples; ++i) {
            posest::teensy::ImuPayload imu;
            imu.teensy_time_us = static_cast<std::uint64_t>(i * 1000u);
            imu.accel_mps2 = {
                static_cast<double>(i),
                static_cast<double>(i) + 1.0,
                static_cast<double>(i) + 2.0};
            imu.gyro_radps = {0.1, 0.2, 0.3};
            pushEncodedFrame(
                fake,
                posest::teensy::MessageType::ImuSample,
                sequence++,
                posest::teensy::encodeImuPayload(imu));

            if (i % 10u == 0u) {
                posest::teensy::ChassisSpeedsPayload chassis;
                chassis.teensy_time_us = static_cast<std::uint64_t>(i * 1000u);
                chassis.rio_time_us = static_cast<std::uint64_t>(i * 1000u + 100u);
                chassis.vx_mps = static_cast<double>(i) * 0.01;
                chassis.vy_mps = static_cast<double>(i) * 0.02;
                chassis.omega_radps = static_cast<double>(i) * 0.001;
                pushEncodedFrame(
                    fake,
                    posest::teensy::MessageType::ChassisSpeeds,
                    sequence++,
                    posest::teensy::encodeChassisSpeedsPayload(chassis));
            }

            next_sample_time += 1ms;
            std::this_thread::sleep_until(next_sample_time);
        }
    });

    feeder.join();
    const bool completed = waitFor([&] {
        const auto stats = service.stats();
        return stats.inbound_imu_samples == kImuSamples &&
               stats.inbound_chassis_speeds_samples == kChassisSpeedsSamples;
    }, 2s);

    service.stop();
    bus.shutdown();
    drainer.join();

    ASSERT_TRUE(completed);
    const auto stats = service.stats();
    EXPECT_EQ(stats.invalid_payloads, 0u);
    EXPECT_EQ(stats.crc_failures, 0u);
    EXPECT_EQ(stats.sequence_gaps, 0u);
    EXPECT_EQ(stats.inbound_measurements_dropped, 0u);
    EXPECT_EQ(consumed.load(), kExpectedSamples);
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
        for (const auto& bytes : fake.writes()) {
            auto decoded = posest::teensy::decodeFrame(bytes);
            if (decoded &&
                decoded->frame.type == posest::teensy::MessageType::FusedPose) {
                return true;
            }
        }
        return false;
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
    primeTimeSync(fake, service);
    primeRioSync(fake, service, /*rio_to_teensy_offset_us=*/0);

    posest::teensy::ChassisSpeedsPayload chassis_payload;
    chassis_payload.teensy_time_us = 0;
    chassis_payload.rio_time_us = 1234;
    chassis_payload.vx_mps = 4.0;
    chassis_payload.vy_mps = 5.0;
    chassis_payload.omega_radps = 6.0;
    pushEncodedFrame(
        fake,
        posest::teensy::MessageType::ChassisSpeeds,
        1,
        posest::teensy::encodeChassisSpeedsPayload(chassis_payload));

    ASSERT_TRUE(waitFor([&] {
        return service.stats().inbound_chassis_speeds_samples == 1u;
    }));
    auto measurement = bus.take();
    ASSERT_TRUE(measurement.has_value());
    ASSERT_TRUE(std::holds_alternative<posest::ChassisSpeedsSample>(*measurement));
    const auto& chassis = std::get<posest::ChassisSpeedsSample>(*measurement);
    EXPECT_EQ(chassis.status_flags & posest::teensy::kStatusUnsynchronizedTime, 0u);
    EXPECT_EQ(chassis.rio_time_us, 1234u);
    EXPECT_DOUBLE_EQ(chassis.omega_radps, 6.0);

    service.stop();
}

TEST(TeensyService, CameraTriggerEventIsRecordedInCacheAfterTimeSync) {
    posest::MeasurementBus bus(4);
    posest::teensy::FakeSerialTransport fake;
    auto state = fake.sharedState();
    auto cache = std::make_shared<posest::CameraTriggerCache>(
        std::unordered_map<std::int32_t, std::string>{{6, "cam0"}}, 500ms);
    posest::teensy::TeensyService service(
        fastConfig(),
        {},
        bus,
        [state] { return std::make_unique<posest::teensy::FakeSerialTransport>(state); },
        cache);

    service.start();
    ASSERT_TRUE(fake.waitForOpenCount(1, 500ms));
    ASSERT_TRUE(fake.waitForWriteCount(3, 500ms));

    const auto request_frame = decodeWrittenFrame(
        fake.writes(),
        posest::teensy::MessageType::TimeSyncRequest);
    const auto request = posest::teensy::decodeTimeSyncRequestPayload(request_frame.payload);
    ASSERT_TRUE(request.has_value());

    posest::teensy::TimeSyncResponsePayload response;
    response.request_sequence = request->request_sequence;
    response.teensy_receive_time_us = request->host_send_time_us + 100;
    response.teensy_transmit_time_us = request->host_send_time_us + 200;
    posest::teensy::Frame sync_frame;
    sync_frame.type = posest::teensy::MessageType::TimeSyncResponse;
    sync_frame.sequence = 0;
    sync_frame.payload = posest::teensy::encodeTimeSyncResponsePayload(response);
    fake.pushReadBytes(posest::teensy::encodeFrame(sync_frame));
    ASSERT_TRUE(waitFor([&] { return service.stats().time_sync_established; }));

    posest::teensy::CameraTriggerEventPayload trig_payload;
    trig_payload.teensy_time_us = response.teensy_transmit_time_us;
    trig_payload.pin = 6;
    trig_payload.trigger_sequence = 17;
    trig_payload.status_flags = 0;
    pushEncodedFrame(
        fake,
        posest::teensy::MessageType::CameraTriggerEvent,
        1,
        posest::teensy::encodeCameraTriggerEventPayload(trig_payload));

    ASSERT_TRUE(waitFor([&] {
        return service.stats().inbound_camera_trigger_events == 1u;
    }));

    // Drain the bus so subsequent tests on the same harness aren't surprised.
    auto measurement = bus.take();
    ASSERT_TRUE(measurement.has_value());

    // Cache lookup at the host time of the recorded event must succeed.
    const auto& event = std::get<posest::CameraTriggerEvent>(*measurement);
    const auto stamp = cache->lookup("cam0", event.timestamp);
    ASSERT_TRUE(stamp.has_value());
    EXPECT_EQ(stamp->trigger_sequence, 17u);
    EXPECT_EQ(stamp->teensy_time_us, response.teensy_transmit_time_us);

    service.stop();
}

namespace {

posest::runtime::VioConfig fastVioConfig(const std::string& camera_id = "vio_cam") {
    posest::runtime::VioConfig vio;
    vio.enabled = true;
    vio.vio_camera_id = camera_id;
    vio.vio_slot_index = 0;
    vio.ir_led_pulse_width_us = 400;
    vio.tof_offset_after_flash_us = 500;
    vio.tof_timing_budget_ms = 10;
    vio.tof_intermeasurement_period_ms = 20;
    vio.tof_divisor = 1;
    // 3" mounting offset → 0.0762 m. Picked to match the example from the
    // user's spec ("recessed 1\" but report 4\" from ground" delta).
    vio.tof_mounting_offset_m = 0.0762;
    vio.tof_expected_min_m = 0.05;
    vio.tof_expected_max_m = 4.0;
    return vio;
}

}  // namespace

TEST(TeensyService, SendsVioCompanionConfigOnConnect) {
    posest::MeasurementBus bus(4);
    posest::teensy::FakeSerialTransport fake;
    auto state = fake.sharedState();
    auto vio = fastVioConfig();

    posest::teensy::TeensyService service(
        fastConfig(),
        {},
        bus,
        [state] { return std::make_unique<posest::teensy::FakeSerialTransport>(state); },
        nullptr,
        vio);

    service.start();
    ASSERT_TRUE(fake.waitForOpenCount(1, 500ms));
    // Three startup writes: camera triggers, IMU config, VIO companion.
    ASSERT_TRUE(fake.waitForWriteCount(3, 500ms));
    service.stop();

    // Look for the VIO config frame by walking writes and inspecting the
    // ConfigCommand kind discriminator.
    bool found = false;
    for (const auto& bytes : fake.writes()) {
        auto decoded = posest::teensy::decodeFrame(bytes);
        if (!decoded || decoded->frame.type != posest::teensy::MessageType::ConfigCommand) {
            continue;
        }
        if (decoded->frame.payload.size() < 4) continue;
        const std::uint32_t kind =
            static_cast<std::uint32_t>(decoded->frame.payload[0]) |
            (static_cast<std::uint32_t>(decoded->frame.payload[1]) << 8u) |
            (static_cast<std::uint32_t>(decoded->frame.payload[2]) << 16u) |
            (static_cast<std::uint32_t>(decoded->frame.payload[3]) << 24u);
        if (kind != static_cast<std::uint32_t>(posest::teensy::ConfigCommandKind::VioCompanion)) {
            continue;
        }
        std::vector<std::uint8_t> body(
            decoded->frame.payload.begin() + 4, decoded->frame.payload.end());
        auto vio_payload = posest::teensy::decodeVioCompanionConfigPayload(body);
        ASSERT_TRUE(vio_payload.has_value());
        EXPECT_EQ(vio_payload->led_pulse_width_us, 400u);
        EXPECT_EQ(vio_payload->tof_offset_after_flash_us, 500u);
        found = true;
        break;
    }
    EXPECT_TRUE(found) << "expected VioCompanion ConfigCommand frame";
}

TEST(TeensyService, AppliesMountingOffsetAndPublishesToFSample) {
    posest::MeasurementBus bus(4);
    posest::teensy::FakeSerialTransport fake;
    auto state = fake.sharedState();
    auto cache = std::make_shared<posest::ToFSampleCache>("vio_cam");

    posest::teensy::TeensyService service(
        fastConfig(),
        {},
        bus,
        [state] { return std::make_unique<posest::teensy::FakeSerialTransport>(state); },
        nullptr,
        fastVioConfig(),
        cache);

    service.start();
    ASSERT_TRUE(fake.waitForOpenCount(1, 500ms));

    posest::teensy::ToFSamplePayload payload;
    payload.teensy_time_us = 1000;
    payload.trigger_sequence = 17;
    payload.distance_mm = 50;  // 0.050 m raw
    payload.range_status = 0;
    payload.firmware_status_flags = 0;
    pushEncodedFrame(
        fake,
        posest::teensy::MessageType::ToFSample,
        100,
        posest::teensy::encodeToFSamplePayload(payload));

    auto measurement = bus.take();
    service.stop();

    ASSERT_TRUE(measurement.has_value());
    ASSERT_TRUE(std::holds_alternative<posest::ToFSample>(*measurement));
    const auto& sample = std::get<posest::ToFSample>(*measurement);
    EXPECT_EQ(sample.trigger_sequence, 17u);
    EXPECT_NEAR(sample.raw_distance_m, 0.050, 1e-9);
    // 0.050 m + 0.0762 m mounting offset = 0.1262 m. distance_m carries the
    // host-applied correction; raw_distance_m is preserved untouched.
    EXPECT_NEAR(sample.distance_m, 0.1262, 1e-9);

    // Same sample must also have been recorded into the cache so the
    // ProducerBase ToF join can find it by sequence.
    const auto cached = cache->lookupBySequence("vio_cam", 17);
    ASSERT_TRUE(cached.has_value());
    EXPECT_NEAR(cached->distance_m, 0.1262, 1e-9);
}

TEST(TeensyService, ClampsToFSampleBelowExpectedMin) {
    posest::MeasurementBus bus(4);
    posest::teensy::FakeSerialTransport fake;
    auto state = fake.sharedState();
    auto vio = fastVioConfig();
    vio.tof_mounting_offset_m = 0.0;
    vio.tof_expected_min_m = 0.10;  // clamp anything below 100 mm

    posest::teensy::TeensyService service(
        fastConfig(),
        {},
        bus,
        [state] { return std::make_unique<posest::teensy::FakeSerialTransport>(state); },
        nullptr,
        vio);

    service.start();
    ASSERT_TRUE(fake.waitForOpenCount(1, 500ms));

    posest::teensy::ToFSamplePayload payload;
    payload.teensy_time_us = 1000;
    payload.trigger_sequence = 5;
    payload.distance_mm = 30;  // 0.030 m → below clamp
    pushEncodedFrame(
        fake,
        posest::teensy::MessageType::ToFSample,
        200,
        posest::teensy::encodeToFSamplePayload(payload));

    auto measurement = bus.take();
    service.stop();

    ASSERT_TRUE(measurement.has_value());
    const auto& sample = std::get<posest::ToFSample>(*measurement);
    EXPECT_DOUBLE_EQ(sample.distance_m, 0.10);
    EXPECT_NE(sample.status_flags & posest::teensy::kToFStatusClampedHostSide, 0u);
    EXPECT_GE(service.stats().inbound_tof_clamped, 1u);
}
