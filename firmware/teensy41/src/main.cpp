#include <Arduino.h>

#include <cstddef>
#include <cstdint>

#include "CameraTriggerScheduler.h"
#include "CanBridge.h"
#include "FirmwareConfig.h"
#include "Protocol.h"

namespace {

using namespace posest::firmware;

StreamDecoder g_decoder;
CameraTriggerScheduler g_triggers;
CanBridge g_can;

std::uint32_t g_tx_sequence = 0;
std::uint32_t g_error_flags = 0;
std::uint64_t g_last_crc_failures = 0;
std::uint64_t g_next_health_us = 0;
std::uint32_t g_next_led_toggle_us = 0;
bool g_led_on = false;

std::uint8_t g_payload_buffer[kMaxPayloadSize]{};
std::uint8_t g_frame_buffer[kMaxFrameSize]{};

std::uint64_t micros64() {
    static std::uint32_t previous = 0;
    static std::uint64_t high = 0;

    const std::uint32_t now = micros();
    if (now < previous) {
        high += (1ULL << 32u);
    }
    previous = now;
    return high + now;
}

std::uint32_t readU32(const std::uint8_t* bytes, std::size_t offset) {
    return static_cast<std::uint32_t>(bytes[offset]) |
           (static_cast<std::uint32_t>(bytes[offset + 1]) << 8u) |
           (static_cast<std::uint32_t>(bytes[offset + 2]) << 16u) |
           (static_cast<std::uint32_t>(bytes[offset + 3]) << 24u);
}

void writeFrame(MessageType type, const std::uint8_t* payload, std::uint16_t payload_size) {
    std::size_t frame_size = 0;
    if (!encodeFrame(
            type,
            g_tx_sequence++,
            payload,
            payload_size,
            g_frame_buffer,
            sizeof(g_frame_buffer),
            frame_size)) {
        g_error_flags |= kErrorInvalidPayload;
        return;
    }
    Serial.write(g_frame_buffer, frame_size);
}

void sendTimeSyncResponse(const TimeSyncRequestPayload& request, std::uint64_t receive_time_us) {
    TimeSyncResponsePayload response;
    response.request_sequence = request.request_sequence;
    response.teensy_receive_time_us = receive_time_us;
    response.teensy_transmit_time_us = micros64();

    std::uint16_t payload_size = 0;
    if (!encodeTimeSyncResponsePayload(
            response,
            g_payload_buffer,
            sizeof(g_payload_buffer),
            payload_size)) {
        g_error_flags |= kErrorInvalidPayload;
        return;
    }
    writeFrame(MessageType::TimeSyncResponse, g_payload_buffer, payload_size);
}

void sendHealth(std::uint64_t now_us) {
    TeensyHealthPayload health;
    health.uptime_us = now_us;
    health.error_flags = g_error_flags | g_can.errorFlags();
    health.trigger_status_flags = g_triggers.statusFlags();
    health.rx_queue_depth = static_cast<std::uint32_t>(g_decoder.bufferedBytes());
    health.tx_queue_depth = g_can.txQueueDepth();

    std::uint16_t payload_size = 0;
    if (!encodeTeensyHealthPayload(
            health,
            g_payload_buffer,
            sizeof(g_payload_buffer),
            payload_size)) {
        g_error_flags |= kErrorInvalidPayload;
        return;
    }
    writeFrame(MessageType::TeensyHealth, g_payload_buffer, payload_size);
}

void applyCameraTriggerConfig(const Frame& frame) {
    CameraTriggerCommand commands[kMaxCameraSyncOutputs]{};
    std::size_t count = 0;
    if (!decodeCameraTriggerConfigPayload(frame, commands, kMaxCameraSyncOutputs, count)) {
        g_error_flags |= kErrorInvalidPayload;
        if (frame.payload_size >= 8u && readU32(frame.payload, 4) > kMaxCameraSyncOutputs) {
            g_triggers.apply(nullptr, kMaxCameraSyncOutputs + 1u, micros());
        }
        return;
    }
    g_triggers.apply(commands, count, micros());
}

void handleFrame(const Frame& frame, std::uint64_t receive_time_us) {
    switch (frame.type) {
        case MessageType::ConfigCommand:
            applyCameraTriggerConfig(frame);
            break;
        case MessageType::TimeSyncRequest: {
            TimeSyncRequestPayload request;
            if (!decodeTimeSyncRequestPayload(frame, request)) {
                g_error_flags |= kErrorInvalidPayload;
                return;
            }
            sendTimeSyncResponse(request, receive_time_us);
            break;
        }
        case MessageType::FusedPose: {
            FusedPosePayload pose;
            if (!decodeFusedPosePayload(frame, pose)) {
                g_error_flags |= kErrorInvalidPayload;
                return;
            }
            g_can.setLatestFusedPose(pose);
            break;
        }
        case MessageType::CanTx:
            g_can.handleUnsupportedCanTx();
            g_error_flags |= kErrorUnsupportedCommand;
            break;
        default:
            g_error_flags |= kErrorUnsupportedCommand;
            break;
    }
}

void pollSerial() {
    while (Serial.available() > 0) {
        const int value = Serial.read();
        if (value < 0) {
            break;
        }
        g_decoder.push(static_cast<std::uint8_t>(value));
    }

    const auto& stats = g_decoder.stats();
    if (stats.crc_failures != g_last_crc_failures) {
        g_error_flags |= kErrorCrcFailure;
        g_last_crc_failures = stats.crc_failures;
    }

    Frame frame;
    while (g_decoder.nextFrame(frame)) {
        handleFrame(frame, micros64());
    }
}

void updateLed(std::uint32_t now_us) {
    const std::uint32_t interval_us = g_error_flags == 0u ? 500000u : 125000u;
    if (static_cast<std::int32_t>(now_us - g_next_led_toggle_us) < 0) {
        return;
    }
    g_next_led_toggle_us = now_us + interval_us;
    g_led_on = !g_led_on;
    digitalWrite(kStatusLedPin, g_led_on ? HIGH : LOW);
}

}  // namespace

void setup() {
    pinMode(kStatusLedPin, OUTPUT);
    digitalWrite(kStatusLedPin, LOW);
    g_triggers.begin();
    g_can.begin();
    Serial.begin(kDefaultSerialBaud);
    g_next_health_us = micros64() + kHealthPublishIntervalUs;
    g_next_led_toggle_us = micros() + 250000u;
}

void loop() {
    pollSerial();

    const std::uint32_t now32 = micros();
    const std::uint64_t now64 = micros64();
    g_triggers.update(now32);
    updateLed(now32);

    if (now64 >= g_next_health_us) {
        g_next_health_us = now64 + kHealthPublishIntervalUs;
        sendHealth(now64);
    }
}
