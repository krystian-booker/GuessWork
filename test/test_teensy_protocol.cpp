#include <cstdint>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

#include "posest/MeasurementTypes.h"
#include "posest/teensy/Protocol.h"
#include "posest/teensy/TeensyService.h"

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

    payload.tof_samples_emitted = 33;
    payload.tof_overruns = 4;
    payload.tof_i2c_failures = 1;
    payload.tof_status_flags = 0xCAFE;

    const auto bytes = posest::teensy::encodeTeensyHealthPayload(payload);
    EXPECT_EQ(bytes.size(), 60u);
    const auto decoded = posest::teensy::decodeTeensyHealthPayload(bytes);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->uptime_us, 1234u);
    EXPECT_EQ(decoded->rio_offset_us, -987654);
    EXPECT_EQ(decoded->rio_status_flags, posest::teensy::kHealthRioPingRejected);
    EXPECT_EQ(decoded->accel_saturations, 11u);
    EXPECT_EQ(decoded->gyro_saturations, 22u);
    EXPECT_EQ(decoded->tof_samples_emitted, 33u);
    EXPECT_EQ(decoded->tof_overruns, 4u);
    EXPECT_EQ(decoded->tof_i2c_failures, 1u);
    EXPECT_EQ(decoded->tof_status_flags, 0xCAFEu);
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


TEST(TeensyProtocol, ToFSamplePayloadRoundTrips) {
    posest::teensy::ToFSamplePayload payload;
    payload.teensy_time_us = 0xAABBCCDDEEFF1122ULL;
    payload.trigger_sequence = 0x12345678u;
    payload.distance_mm = 1234u;
    payload.ranging_duration_us = 9876u;
    payload.firmware_status_flags =
        posest::teensy::kStatusUnsynchronizedTime |
        posest::teensy::kToFStatusRangingOverrun;
    payload.signal_rate_kcps = 4321u;
    payload.ambient_rate_kcps = 567u;
    payload.range_status = 4u;

    const auto bytes = posest::teensy::encodeToFSamplePayload(payload);
    EXPECT_EQ(bytes.size(), 36u);
    const auto decoded = posest::teensy::decodeToFSamplePayload(bytes);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->teensy_time_us, payload.teensy_time_us);
    EXPECT_EQ(decoded->trigger_sequence, payload.trigger_sequence);
    EXPECT_EQ(decoded->distance_mm, payload.distance_mm);
    EXPECT_EQ(decoded->ranging_duration_us, payload.ranging_duration_us);
    EXPECT_EQ(decoded->firmware_status_flags, payload.firmware_status_flags);
    EXPECT_EQ(decoded->signal_rate_kcps, payload.signal_rate_kcps);
    EXPECT_EQ(decoded->ambient_rate_kcps, payload.ambient_rate_kcps);
    EXPECT_EQ(decoded->range_status, payload.range_status);
}

TEST(TeensyProtocol, ToFSamplePayloadRejectsWrongSize) {
    EXPECT_FALSE(posest::teensy::decodeToFSamplePayload(
        std::vector<std::uint8_t>(35, 0)).has_value());
    EXPECT_FALSE(posest::teensy::decodeToFSamplePayload(
        std::vector<std::uint8_t>(37, 0)).has_value());
}

TEST(TeensyProtocol, VioCompanionConfigPayloadRoundTrips) {
    posest::teensy::VioCompanionConfigPayload payload;
    payload.vio_slot_index = 0;
    payload.led_enabled = 1;
    payload.led_pulse_width_us = 400;
    payload.tof_enabled = 1;
    payload.tof_i2c_address = 0x29;
    payload.tof_timing_budget_ms = 10;
    payload.tof_intermeasurement_period_ms = 20;
    payload.tof_offset_after_flash_us = 500;
    payload.tof_divisor = 1;

    const auto bytes = posest::teensy::encodeVioCompanionConfigPayload(payload);
    EXPECT_EQ(bytes.size(), 36u);
    const auto decoded = posest::teensy::decodeVioCompanionConfigPayload(bytes);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->vio_slot_index, 0u);
    EXPECT_EQ(decoded->led_pulse_width_us, 400u);
    EXPECT_EQ(decoded->tof_offset_after_flash_us, 500u);
}

namespace {

double readDoubleLE(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
    std::uint64_t bits = 0;
    for (int shift = 0; shift <= 56; shift += 8) {
        bits |= static_cast<std::uint64_t>(
                    bytes[offset + static_cast<std::size_t>(shift / 8)])
                << static_cast<unsigned>(shift);
    }
    double value = 0.0;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

std::uint32_t readU32LE(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
    return static_cast<std::uint32_t>(bytes[offset]) |
           (static_cast<std::uint32_t>(bytes[offset + 1]) << 8u) |
           (static_cast<std::uint32_t>(bytes[offset + 2]) << 16u) |
           (static_cast<std::uint32_t>(bytes[offset + 3]) << 24u);
}

}  // namespace

TEST(TeensyProtocol, FusedPosePayloadV3LayoutAndSize) {
    // v3 layout:
    //   0    8   x_m (double)
    //   8    8   y_m (double)
    //  16    8   z_m (double, 0.0 until Phase C widens FusedPoseEstimate)
    //  24    8   roll_rad (double)
    //  32    8   pitch_rad (double)
    //  40    8   yaw_rad (double)
    //  48    8   vx_mps (double)
    //  56    8   vy_mps (double)
    //  64    8   vz_mps (double)
    //  72    4   has_velocity (uint32)
    //  76    4   status_flags (uint32)
    //  80  288   covariance (36 doubles, gtsam::Pose3 tangent row-major)
    posest::FusedPoseEstimate estimate;
    estimate.field_to_robot = {1.25, -2.75, 0.5};
    estimate.velocity = posest::Vec3{3.0, -1.0, 0.1};
    estimate.status_flags = 0xABCDu;
    for (std::size_t i = 0; i < estimate.covariance.size(); ++i) {
        estimate.covariance[i] = 0.001 * static_cast<double>(i + 1);
    }

    const auto bytes = posest::teensy::TeensyService::encodeFusedPosePayload(estimate);
    ASSERT_EQ(bytes.size(), 368u);

    EXPECT_DOUBLE_EQ(readDoubleLE(bytes, 0), 1.25);
    EXPECT_DOUBLE_EQ(readDoubleLE(bytes, 8), -2.75);
    EXPECT_DOUBLE_EQ(readDoubleLE(bytes, 16), 0.0);   // z (Pose2d-derived)
    EXPECT_DOUBLE_EQ(readDoubleLE(bytes, 24), 0.0);   // roll
    EXPECT_DOUBLE_EQ(readDoubleLE(bytes, 32), 0.0);   // pitch
    EXPECT_DOUBLE_EQ(readDoubleLE(bytes, 40), 0.5);   // yaw
    EXPECT_DOUBLE_EQ(readDoubleLE(bytes, 48), 3.0);
    EXPECT_DOUBLE_EQ(readDoubleLE(bytes, 56), -1.0);
    EXPECT_DOUBLE_EQ(readDoubleLE(bytes, 64), 0.1);
    EXPECT_EQ(readU32LE(bytes, 72), 1u);              // has_velocity
    EXPECT_EQ(readU32LE(bytes, 76), 0xABCDu);         // status_flags
    for (std::size_t i = 0; i < 36; ++i) {
        EXPECT_DOUBLE_EQ(readDoubleLE(bytes, 80 + i * 8),
                         0.001 * static_cast<double>(i + 1));
    }
}

TEST(TeensyProtocol, FusedPosePayloadV3OmitsVelocityWhenAbsent) {
    posest::FusedPoseEstimate estimate;
    estimate.field_to_robot = {0.0, 0.0, 0.0};
    estimate.velocity.reset();  // explicit "no velocity"
    estimate.status_flags = 0;

    const auto bytes = posest::teensy::TeensyService::encodeFusedPosePayload(estimate);
    ASSERT_EQ(bytes.size(), 368u);
    EXPECT_EQ(readU32LE(bytes, 72), 0u);  // has_velocity = false
    // Velocity fields are zero-padded when absent (firmware never reads them
    // in this case — the RIO ignores them based on has_velocity).
    EXPECT_DOUBLE_EQ(readDoubleLE(bytes, 48), 0.0);
    EXPECT_DOUBLE_EQ(readDoubleLE(bytes, 56), 0.0);
    EXPECT_DOUBLE_EQ(readDoubleLE(bytes, 64), 0.0);
}

TEST(TeensyProtocol, ConfigAckEncodesVioEntry) {
    posest::teensy::ConfigAckPayload ack;
    ack.kind = static_cast<std::uint32_t>(posest::teensy::ConfigCommandKind::VioCompanion);
    ack.status_flags = 0;
    ack.effective_count = 1;
    posest::teensy::VioConfigAckEntry entry;
    entry.vio_slot_index = 0;
    entry.led_enabled = 1;
    entry.led_pulse_width_us = 400;
    entry.tof_enabled = 1;
    entry.tof_i2c_address = 0x29;
    entry.tof_timing_budget_ms = 10;
    entry.tof_intermeasurement_period_ms = 20;
    entry.tof_offset_after_flash_us = 500;
    entry.tof_divisor = 1;
    ack.vio_entry = entry;

    const auto bytes = posest::teensy::encodeConfigAckPayload(ack);
    // 12-byte header + 36-byte VIO body.
    ASSERT_EQ(bytes.size(), 48u);
    const auto decoded = posest::teensy::decodeConfigAckPayload(bytes);
    ASSERT_TRUE(decoded.has_value());
    ASSERT_TRUE(decoded->vio_entry.has_value());
    EXPECT_EQ(decoded->vio_entry->led_pulse_width_us, 400u);
    EXPECT_EQ(decoded->vio_entry->tof_offset_after_flash_us, 500u);
}
