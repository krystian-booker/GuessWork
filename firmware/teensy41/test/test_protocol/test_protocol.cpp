#include <unity.h>

#include "Protocol.h"

using namespace posest::firmware;

void test_time_sync_response_payload_size() {
    std::uint8_t payload[kMaxPayloadSize]{};
    std::uint16_t size = 0;
    TimeSyncResponsePayload response;
    response.request_sequence = 12;
    response.teensy_receive_time_us = 1000;
    response.teensy_transmit_time_us = 1100;

    TEST_ASSERT_TRUE(encodeTimeSyncResponsePayload(response, payload, sizeof(payload), size));
    TEST_ASSERT_EQUAL_UINT16(20, size);
}

void test_chassis_speeds_payload_round_trips() {
    std::uint8_t payload[kMaxPayloadSize]{};
    std::uint16_t size = 0;
    ChassisSpeedsPayload sample;
    sample.teensy_time_us = 1234;
    sample.rio_time_us = 5678;
    sample.vx_mps = 1.5;
    sample.vy_mps = -0.25;
    sample.omega_radps = 0.75;
    sample.status_flags = kStatusUnsynchronizedRioTime;

    TEST_ASSERT_TRUE(
        encodeChassisSpeedsPayload(sample, payload, sizeof(payload), size));
    TEST_ASSERT_EQUAL_UINT16(44, size);
}

void test_frame_round_trip_through_decoder() {
    std::uint8_t payload[3]{1, 2, 3};
    std::uint8_t bytes[kMaxFrameSize]{};
    std::size_t size = 0;

    TEST_ASSERT_TRUE(encodeFrame(
        MessageType::TeensyHealth,
        42,
        payload,
        sizeof(payload),
        bytes,
        sizeof(bytes),
        size));

    StreamDecoder decoder;
    for (std::size_t i = 0; i < size; ++i) {
        decoder.push(bytes[i]);
    }

    Frame frame;
    TEST_ASSERT_TRUE(decoder.nextFrame(frame));
    TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(MessageType::TeensyHealth),
                            static_cast<std::uint8_t>(frame.type));
    TEST_ASSERT_EQUAL_UINT32(42, frame.sequence);
    TEST_ASSERT_EQUAL_UINT16(3, frame.payload_size);
    TEST_ASSERT_EQUAL_UINT8(2, frame.payload[1]);
}

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;
    UNITY_BEGIN();
    RUN_TEST(test_time_sync_response_payload_size);
    RUN_TEST(test_chassis_speeds_payload_round_trips);
    RUN_TEST(test_frame_round_trip_through_decoder);
    return UNITY_END();
}
