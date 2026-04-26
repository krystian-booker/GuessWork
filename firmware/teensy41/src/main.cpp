#include <Arduino.h>

#include <cstddef>
#include <cstdint>

#include "Bmi088Imu.h"
#include "CameraTriggerScheduler.h"
#include "CanBridge.h"
#include "FirmwareConfig.h"
#include "Protocol.h"
#include "Vl53l4cdToF.h"

namespace {

using namespace posest::firmware;

StreamDecoder g_decoder;
CameraTriggerScheduler g_triggers;
CanBridge g_can;
Bmi088Imu g_imu;
Vl53l4cdToF g_tof;

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
    health.error_flags = g_error_flags | g_can.errorFlags() | g_imu.errorFlags();
    health.trigger_status_flags = g_triggers.statusFlags();
    health.rx_queue_depth = static_cast<std::uint32_t>(g_decoder.bufferedBytes());
    health.tx_queue_depth = g_can.txQueueDepth();
    health.rio_offset_us = g_can.rioOffsetUs();
    health.rio_status_flags = g_can.rioStatusFlags();
    const auto imu_stats = g_imu.stats();
    health.accel_saturations = imu_stats.accel_saturations;
    health.gyro_saturations = imu_stats.gyro_saturations;
    health.tof_samples_emitted = g_tof.samplesEmitted();
    health.tof_overruns = g_tof.overruns();
    health.tof_i2c_failures = g_tof.i2cFailures();
    health.tof_status_flags = g_tof.errorFlags();

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

void sendChassisSpeeds(const ChassisSpeedsPayload& payload) {
    std::uint16_t payload_size = 0;
    if (!encodeChassisSpeedsPayload(
            payload,
            g_payload_buffer,
            sizeof(g_payload_buffer),
            payload_size)) {
        g_error_flags |= kErrorInvalidPayload;
        return;
    }
    writeFrame(MessageType::ChassisSpeeds, g_payload_buffer, payload_size);
}

void sendImuSample(const ImuPayload& sample) {
    std::uint16_t payload_size = 0;
    if (!encodeImuPayload(sample, g_payload_buffer, sizeof(g_payload_buffer), payload_size)) {
        g_error_flags |= kErrorInvalidPayload;
        return;
    }
    writeFrame(MessageType::ImuSample, g_payload_buffer, payload_size);
}

void sendCameraTriggerEvent(const CameraTriggerEventPayload& event) {
    std::uint16_t payload_size = 0;
    if (!encodeCameraTriggerEventPayload(
            event,
            g_payload_buffer,
            sizeof(g_payload_buffer),
            payload_size)) {
        g_error_flags |= kErrorInvalidPayload;
        return;
    }
    writeFrame(MessageType::CameraTriggerEvent, g_payload_buffer, payload_size);
}

void sendToFSample(const ToFSamplePayload& sample) {
    std::uint16_t payload_size = 0;
    if (!encodeToFSamplePayload(
            sample, g_payload_buffer, sizeof(g_payload_buffer), payload_size)) {
        g_error_flags |= kErrorInvalidPayload;
        return;
    }
    writeFrame(MessageType::ToFSample, g_payload_buffer, payload_size);
}

void sendConfigAck(
    const ConfigAckHeader& header,
    const TriggerAckEntry* trigger_entries,
    std::size_t trigger_entry_count,
    const ImuConfigAckEntry* imu_entry,
    const VioConfigAckEntry* vio_entry = nullptr) {
    std::uint16_t payload_size = 0;
    if (!encodeConfigAckPayload(
            header,
            trigger_entries,
            trigger_entry_count,
            imu_entry,
            vio_entry,
            g_payload_buffer,
            sizeof(g_payload_buffer),
            payload_size)) {
        g_error_flags |= kErrorInvalidPayload;
        return;
    }
    writeFrame(MessageType::ConfigAck, g_payload_buffer, payload_size);
}

std::uint32_t triggerStatusToAckFlags(std::uint32_t trigger_status) {
    std::uint32_t flags = 0;
    if (trigger_status & kTriggerUnsupportedCount) flags |= kConfigAckUnsupportedCount;
    if (trigger_status & kTriggerInvalidPin) flags |= kConfigAckInvalidPin;
    if (trigger_status & kTriggerDuplicatePin) flags |= kConfigAckDuplicatePin;
    if (trigger_status & kTriggerInvalidRate) flags |= kConfigAckInvalidRate;
    if (trigger_status & kTriggerPulseTooWide) flags |= kConfigAckPulseTooWide;
    return flags;
}

void applyCameraTriggerConfig(const Frame& frame) {
    CameraTriggerCommand commands[kMaxCameraSyncOutputs]{};
    std::size_t count = 0;
    if (!decodeCameraTriggerConfigPayload(frame, commands, kMaxCameraSyncOutputs, count)) {
        g_error_flags |= kErrorInvalidPayload;
        ConfigAckHeader header;
        header.kind = static_cast<std::uint32_t>(ConfigCommandKind::CameraTriggers);
        header.status_flags = kConfigAckUnsupportedCount;
        header.effective_count = 0;
        if (frame.payload_size >= 8u && readU32(frame.payload, 4) > kMaxCameraSyncOutputs) {
            g_triggers.apply(nullptr, kMaxCameraSyncOutputs + 1u, micros());
        }
        sendConfigAck(header, nullptr, 0, nullptr);
        return;
    }
    g_triggers.apply(commands, count, micros());

    TriggerAckEntry entries[kMaxCameraSyncOutputs]{};
    const std::size_t effective = g_triggers.effectiveEntries(entries, kMaxCameraSyncOutputs);
    ConfigAckHeader header;
    header.kind = static_cast<std::uint32_t>(ConfigCommandKind::CameraTriggers);
    header.status_flags = triggerStatusToAckFlags(g_triggers.statusFlags());
    header.effective_count = static_cast<std::uint32_t>(effective);
    sendConfigAck(header, entries, effective, nullptr);
}

void applyVioCompanionConfig(const Frame& frame) {
    ConfigAckHeader header;
    header.kind = static_cast<std::uint32_t>(ConfigCommandKind::VioCompanion);

    VioCompanionCommand command;
    if (!decodeVioCompanionConfigPayload(frame, command)) {
        g_error_flags |= kErrorInvalidPayload;
        header.status_flags = kConfigAckInvalidVioConfig;
        header.effective_count = 0;
        sendConfigAck(header, nullptr, 0, nullptr, nullptr);
        return;
    }

    VioCompanionConfig cfg;
    cfg.led_enabled = command.led_enabled != 0u;
    cfg.vio_slot_index = static_cast<std::uint8_t>(command.vio_slot_index);
    cfg.led_pulse_width_us = command.led_pulse_width_us;
    cfg.tof_enabled = command.tof_enabled != 0u;
    cfg.tof_offset_after_flash_us = command.tof_offset_after_flash_us;
    cfg.tof_divisor = command.tof_divisor;

    const std::uint32_t scheduler_status = g_triggers.applyVioCompanion(cfg);
    header.status_flags = scheduler_status;

    if (scheduler_status == 0u) {
        Vl53l4cdToF::Config tof_cfg;
        tof_cfg.enabled = cfg.tof_enabled;
        tof_cfg.i2c_address = static_cast<std::uint8_t>(command.tof_i2c_address);
        tof_cfg.timing_budget_ms = static_cast<std::uint16_t>(command.tof_timing_budget_ms);
        tof_cfg.intermeasurement_period_ms =
            static_cast<std::uint16_t>(command.tof_intermeasurement_period_ms);
        if (!g_tof.reconfigure(tof_cfg, micros64())) {
            g_error_flags |= kErrorTofInitFailed;
            header.status_flags |= kConfigAckVioInitFailure;
        }
    }

    VioConfigAckEntry entry;
    entry.vio_slot_index = command.vio_slot_index;
    entry.led_enabled = command.led_enabled;
    entry.led_pulse_width_us = command.led_pulse_width_us;
    entry.tof_enabled = command.tof_enabled;
    entry.tof_i2c_address = command.tof_i2c_address;
    entry.tof_timing_budget_ms = command.tof_timing_budget_ms;
    entry.tof_intermeasurement_period_ms = command.tof_intermeasurement_period_ms;
    entry.tof_offset_after_flash_us = command.tof_offset_after_flash_us;
    entry.tof_divisor = command.tof_divisor;
    header.effective_count = 1u;
    sendConfigAck(header, nullptr, 0, nullptr, &entry);
}

void applyImuConfig(const Frame& frame) {
    ImuConfigCommand command;
    ConfigAckHeader header;
    header.kind = static_cast<std::uint32_t>(ConfigCommandKind::ImuConfig);
    if (!decodeImuConfigPayload(frame, command)) {
        g_error_flags |= kErrorInvalidPayload;
        header.status_flags = kConfigAckInvalidImuConfig;
        header.effective_count = 0;
        sendConfigAck(header, nullptr, 0, nullptr);
        return;
    }

    ImuConfigAckEntry effective{};
    const bool ok = g_imu.reconfigure(command, micros64(), effective);
    header.status_flags = ok ? 0u : kConfigAckInvalidImuConfig;
    if (g_imu.errorFlags() & kErrorImuSelfTestFailure) {
        header.status_flags |= kConfigAckImuSelfTestFailure;
    }
    header.effective_count = 1;
    sendConfigAck(header, nullptr, 0, &effective);
}

void dispatchConfigCommand(const Frame& frame) {
    if (frame.payload_size < 4u) {
        g_error_flags |= kErrorInvalidPayload;
        return;
    }
    const std::uint32_t kind = readU32(frame.payload, 0);
    if (kind == static_cast<std::uint32_t>(ConfigCommandKind::CameraTriggers)) {
        applyCameraTriggerConfig(frame);
    } else if (kind == static_cast<std::uint32_t>(ConfigCommandKind::ImuConfig)) {
        applyImuConfig(frame);
    } else if (kind == static_cast<std::uint32_t>(ConfigCommandKind::VioCompanion)) {
        applyVioCompanionConfig(frame);
    } else {
        ConfigAckHeader header;
        header.kind = kind;
        header.status_flags = kConfigAckUnknownKind;
        header.effective_count = 0;
        sendConfigAck(header, nullptr, 0, nullptr);
    }
}

void handleFrame(const Frame& frame, std::uint64_t receive_time_us) {
    switch (frame.type) {
        case MessageType::ConfigCommand:
            dispatchConfigCommand(frame);
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
    {
        CanBusConfig can_config;
        // Defaults match RuntimeConfig::CanBusConfig; the host can override via
        // a future ConfigCommandKind::CanConfig push (not yet wired).
        g_can.begin(can_config);
    }
    if (!g_imu.begin(micros64())) {
        g_error_flags |= kErrorImuInitFailure;
    }
    {
        // ToF starts disabled — the host enables it via VioCompanion config
        // when the workflow is wired up. begin() with enabled=false is a
        // bus-untouching no-op; this just primes internal state.
        Vl53l4cdToF::Config tof_cfg;
        g_tof.begin(tof_cfg, micros64());
    }
    Serial.begin(kDefaultSerialBaud);
    g_next_health_us = micros64() + kHealthPublishIntervalUs;
    g_next_led_toggle_us = micros() + 250000u;
}

void loop() {
    pollSerial();

    const std::uint32_t now32 = micros();
    const std::uint64_t now64 = micros64();
    g_triggers.update(now32);
    CameraTriggerEventPayload trigger_event;
    while (g_triggers.popEvent(trigger_event)) {
        sendCameraTriggerEvent(trigger_event);
    }
    updateLed(now32);
    g_imu.checkForMissedDataReady(now64);

    g_can.poll(now64);
    ChassisSpeedsPayload chassis;
    while (g_can.popPendingChassisSpeeds(chassis)) {
        sendChassisSpeeds(chassis);
    }

    for (std::uint8_t i = 0; i < 4u; ++i) {
        ImuPayload sample;
        if (!g_imu.poll(sample)) {
            break;
        }
        sendImuSample(sample);
    }

    // VL53L4CD ToF: check if the trigger ISR has scheduled a new
    // "start ranging" deadline that has elapsed, and drain any completed
    // ranging. I2C runs only here in the main loop — never from any ISR.
    {
        std::uint32_t pending_seq = 0;
        if (g_triggers.consumeToFStartDue(now32, pending_seq)) {
            if (g_tof.isRanging()) {
                g_tof.noteOverrun();
                g_error_flags |= kErrorTofRangingOverrun;
            } else if (!g_tof.startRanging(pending_seq, now64)) {
                g_error_flags |= kErrorTofI2cFailure;
            }
        }
        ToFSamplePayload tof_sample;
        if (g_tof.poll(tof_sample, now64)) {
            sendToFSample(tof_sample);
        }
    }

    if (now64 >= g_next_health_us) {
        g_next_health_us = now64 + kHealthPublishIntervalUs;
        sendHealth(now64);
    }
}
