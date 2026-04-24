#include <unity.h>

#include <cmath>

#include "Bmi088Imu.h"
#include "Protocol.h"

using namespace posest::firmware;

void test_raw_sample_conversion_uses_configured_ranges() {
    TEST_ASSERT_TRUE(std::fabs(Bmi088Imu::accelRawToMps2(16384) - (9.80665 * 6.0)) < 0.001);
    TEST_ASSERT_TRUE(
        std::fabs(Bmi088Imu::gyroRawToRadps(16384) -
                  (1000.0 * 0.017453292519943295)) < 0.001);
}

void test_drdy_queue_caps_and_counts_overruns() {
    Bmi088Imu imu;
    for (std::uint32_t i = 0; i < 10u; ++i) {
        imu.onDataReadyIsr(1000u + i);
    }

    TEST_ASSERT_EQUAL_UINT8(8, imu.pendingDrdyCount());
    TEST_ASSERT_EQUAL_UINT32(2, imu.pendingDrdyOverruns());
    TEST_ASSERT_EQUAL_UINT32(8, imu.stats().drdy_events);
}

void test_imu_payload_wire_size_matches_host_contract() {
    ImuPayload payload;
    payload.teensy_time_us = 1234;
    payload.accel_mps2 = {1.0, 2.0, 3.0};
    payload.gyro_radps = {4.0, 5.0, 6.0};
    payload.temperature_c = 25.25;
    payload.status_flags = kImuStatusAccelSaturated;

    std::uint8_t bytes[kMaxPayloadSize]{};
    std::uint16_t size = 0;
    TEST_ASSERT_TRUE(encodeImuPayload(payload, bytes, sizeof(bytes), size));
    TEST_ASSERT_EQUAL_UINT16(68, size);
}

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;
    UNITY_BEGIN();
    RUN_TEST(test_raw_sample_conversion_uses_configured_ranges);
    RUN_TEST(test_drdy_queue_caps_and_counts_overruns);
    RUN_TEST(test_imu_payload_wire_size_matches_host_contract);
    return UNITY_END();
}
