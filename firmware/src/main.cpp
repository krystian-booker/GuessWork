// MicoAir F405 V2 hardware-sync controller.
// One CRC-framed USB CDC stream multiplexes commands, trigger timestamps,
// BMI088 samples, and health.  The board always boots stopped.

#include <Arduino.h>
#include <string.h>

#include "bmi088.hpp"
#include "sync_controller_protocol.h"
#include "timebase.hpp"
#include "trigger_engine.hpp"

namespace {

gw_fw::Bmi088 g_imu;
uint32_t g_usb_errors = 0;
uint32_t g_last_heartbeat_us = 0;
bool g_was_usb_connected = false;
uint8_t g_rx[gw_sync::kMaxFrameBytes];
size_t g_rx_size = 0;

bool usb_send(gw_sync::MessageType type, uint16_t request_id,
              const uint8_t* payload, uint16_t payload_len) {
    uint8_t frame[gw_sync::kMaxFrameBytes];
    const size_t n = gw_sync::build_frame(type, request_id, payload,
                                           payload_len, frame);
    if (!Serial) return false;
    if (!n || Serial.availableForWrite() < static_cast<int>(n)) {
        ++g_usb_errors;
        return false;
    }
    if (Serial.write(frame, n) != n) {
        ++g_usb_errors;
        return false;
    }
    return true;
}

void send_ack(uint16_t request_id, gw_sync::MessageType command,
              gw_sync::AckStatus status) {
    const uint8_t payload[4] = {
        static_cast<uint8_t>(command), static_cast<uint8_t>(status), 0, 0};
    usb_send(gw_sync::MessageType::Ack, request_id, payload, sizeof(payload));
}

void send_device_info(uint16_t request_id) {
    gw_sync::DeviceInfo info;
    info.board_id = gw_sync::kBoardIdMicoAirF405V2;
    info.firmware_version = gw_sync::kFirmwareVersion;
    info.output_count = gw_sync::kOutputCount;
    info.max_groups = gw_sync::kMaxGroups;
    info.capabilities = gw_sync::kCapabilityTriggers |
                        gw_sync::kCapabilityImu |
                        gw_sync::kCapabilityAtomicConfig |
                        gw_sync::kCapabilityTestOutput |
                        gw_sync::kCapabilityBoardFrameImu;
    info.reset_reason = RCC->CSR;
    info.uid[0] = HAL_GetUIDw0();
    info.uid[1] = HAL_GetUIDw1();
    info.uid[2] = HAL_GetUIDw2();
    uint8_t payload[28];
    const size_t n = gw_sync::encode_device_info(info, payload);
    usb_send(gw_sync::MessageType::DeviceInfo, request_id, payload,
             static_cast<uint16_t>(n));
}

void handle_command(gw_sync::MessageType type, uint16_t request_id,
                    const uint8_t* payload, uint16_t len) {
    switch (type) {
        case gw_sync::MessageType::Hello:
            if (len == 0) send_device_info(request_id);
            else send_ack(request_id, type, gw_sync::AckStatus::BadMessage);
            return;
        case gw_sync::MessageType::SetConfig: {
            if (len < 4 || payload[0] > gw_sync::kMaxGroups ||
                len != static_cast<uint16_t>(4 + payload[0] * 8)) {
                send_ack(request_id, type, gw_sync::AckStatus::BadMessage);
                return;
            }
            gw_sync::GroupConfig groups[gw_sync::kMaxGroups];
            for (uint8_t i = 0; i < payload[0]; ++i) {
                if (!gw_sync::decode_group_config(payload + 4 + i * 8, 8,
                                                   groups[i])) {
                    send_ack(request_id, type, gw_sync::AckStatus::BadMessage);
                    return;
                }
            }
            send_ack(request_id, type,
                     gw_fw::trigger_set_config(groups, payload[0]));
            return;
        }
        case gw_sync::MessageType::Arm:
            send_ack(request_id, type, len == 0 ? gw_fw::trigger_arm()
                                                : gw_sync::AckStatus::BadMessage);
            return;
        case gw_sync::MessageType::Stop:
            if (len == 0) {
                gw_fw::trigger_stop();
                send_ack(request_id, type, gw_sync::AckStatus::Ok);
            } else {
                send_ack(request_id, type, gw_sync::AckStatus::BadMessage);
            }
            return;
        case gw_sync::MessageType::TestOutput:
            send_ack(request_id, type,
                     len == 1 ? gw_fw::trigger_test_output(payload[0])
                              : gw_sync::AckStatus::BadMessage);
            return;
        default:
            send_ack(request_id, type, gw_sync::AckStatus::Unsupported);
            return;
    }
}

void consume_rx() {
    size_t consumed = 0;
    while (g_rx_size - consumed >= 2) {
        uint8_t* p = g_rx + consumed;
        if (p[0] != gw_sync::kMagic0 || p[1] != gw_sync::kMagic1) {
            ++consumed;
            continue;
        }
        if (g_rx_size - consumed < gw_sync::kHeaderBytes) break;
        const uint16_t payload_len = gw_sync::get_u16(p, 6);
        if (p[2] != gw_sync::kProtocolVersion ||
            payload_len > gw_sync::kMaxPayloadBytes) {
            ++consumed;
            continue;
        }
        const size_t frame_len = gw_sync::kHeaderBytes + payload_len +
                                 gw_sync::kTrailerBytes;
        if (g_rx_size - consumed < frame_len) break;
        const uint16_t expected = gw_sync::get_u16(
            p, gw_sync::kHeaderBytes + payload_len);
        const uint16_t actual = gw_sync::crc16_ccitt(p + 2, 6 + payload_len);
        if (expected != actual) {
            ++consumed;
            continue;
        }
        handle_command(static_cast<gw_sync::MessageType>(p[3]),
                       gw_sync::get_u16(p, 4), p + gw_sync::kHeaderBytes,
                       payload_len);
        consumed += frame_len;
    }
    if (consumed) {
        memmove(g_rx, g_rx + consumed, g_rx_size - consumed);
        g_rx_size -= consumed;
    }
}

void poll_usb_rx() {
    while (Serial.available() && g_rx_size < sizeof(g_rx)) {
        const int byte = Serial.read();
        if (byte >= 0) g_rx[g_rx_size++] = static_cast<uint8_t>(byte);
    }
    if (g_rx_size == sizeof(g_rx) &&
        !(g_rx[0] == gw_sync::kMagic0 && g_rx[1] == gw_sync::kMagic1)) {
        g_rx_size = 0;
        ++g_usb_errors;
    }
    consume_rx();
}

void send_trigger_events() {
    static bool pending = false;
    static gw_fw::TriggerQueueEvent event;
    for (;;) {
        if (!pending) {
            if (!gw_fw::trigger_pop(event)) return;
            pending = true;
        }
        uint8_t payload[16]{};
        payload[0] = event.slot;
        gw_sync::put_u32(payload, 4, event.index);
        gw_sync::put_u64(payload, 8, event.t_us);
        if (!usb_send(gw_sync::MessageType::Trigger, 0, payload,
                      sizeof(payload))) return;
        pending = false;
    }
}

void poll_imu() {
    gw_sync::ImuRecord sample;
    if (!g_imu.poll(sample)) return;
    uint8_t payload[36]{};
    payload[0] = 1;
    size_t off = 4;
    off = gw_sync::put_u64(payload, off, sample.t_us);
    for (float v : sample.accel) off = gw_sync::put_f32(payload, off, v);
    for (float v : sample.gyro) off = gw_sync::put_f32(payload, off, v);
    usb_send(gw_sync::MessageType::ImuBatch, 0, payload,
             static_cast<uint16_t>(off));
}

void send_heartbeat() {
    const uint32_t now32 = gw_fw::timebase::now_us32();
    if (static_cast<uint32_t>(now32 - g_last_heartbeat_us) < 1'000'000u) return;
    g_last_heartbeat_us = now32;
    uint8_t payload[32]{};
    size_t off = 0;
    off = gw_sync::put_u64(payload, off, gw_fw::timebase::now_us());
    uint32_t flags = g_imu.ok()
        ? static_cast<uint32_t>(gw_sync::kHeartbeatImuOk) : 0u;
    if (gw_fw::trigger_is_armed()) flags |= gw_sync::kHeartbeatArmed;
    off = gw_sync::put_u32(payload, off, flags);
    off = gw_sync::put_u32(payload, off, g_imu.sample_count());
    off = gw_sync::put_u32(payload, off, g_imu.drop_count());
    off = gw_sync::put_u32(payload, off, gw_fw::trigger_drop_count());
    gw_sync::put_u32(payload, off, g_usb_errors);
    usb_send(gw_sync::MessageType::Heartbeat, 0, payload, sizeof(payload));
}

}  // namespace

void setup() {
    gw_fw::trigger_begin();
    Serial.begin(115200);
    g_imu.begin();
}

void loop() {
    const bool usb_connected = static_cast<bool>(Serial);
    if (g_was_usb_connected && !usb_connected) gw_fw::trigger_stop();
    g_was_usb_connected = usb_connected;
    poll_usb_rx();
    send_trigger_events();
    poll_imu();
    send_heartbeat();
}
