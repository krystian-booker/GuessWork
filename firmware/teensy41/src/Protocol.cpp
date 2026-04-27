#include "Protocol.h"

#include <cstring>

#include "ByteCodec.h"

namespace posest::firmware {

namespace {

bool ensure(std::size_t capacity, std::size_t required) {
    return capacity >= required;
}

}  // namespace

std::uint32_t crc32(const std::uint8_t* data, std::size_t size) {
    std::uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            const std::uint32_t mask = 0u - (crc & 1u);
            crc = (crc >> 1u) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

bool encodeFrame(
    MessageType type,
    std::uint32_t sequence,
    const std::uint8_t* payload,
    std::uint16_t payload_size,
    std::uint8_t* out,
    std::size_t capacity,
    std::size_t& out_size) {
    const std::size_t total_size = kHeaderSize + payload_size + kCrcSize;
    if (!out || !ensure(capacity, total_size) ||
        (payload_size > 0u && payload == nullptr)) {
        return false;
    }

    std::size_t offset = 0;
    appendU16(out, offset, kFrameMagic);
    out[offset++] = kProtocolVersion;
    out[offset++] = static_cast<std::uint8_t>(type);
    appendU32(out, offset, sequence);
    appendU16(out, offset, payload_size);
    if (payload_size > 0u) {
        std::memcpy(out + offset, payload, payload_size);
        offset += payload_size;
    }
    appendU32(out, offset, crc32(out, offset));
    out_size = offset;
    return true;
}

bool encodeImuPayload(
    const ImuPayload& payload,
    std::uint8_t* out,
    std::size_t capacity,
    std::uint16_t& out_size) {
    if (!out || !ensure(capacity, 68u)) {
        return false;
    }
    std::size_t offset = 0;
    appendU64(out, offset, payload.teensy_time_us);
    appendDouble(out, offset, payload.accel_mps2.x);
    appendDouble(out, offset, payload.accel_mps2.y);
    appendDouble(out, offset, payload.accel_mps2.z);
    appendDouble(out, offset, payload.gyro_radps.x);
    appendDouble(out, offset, payload.gyro_radps.y);
    appendDouble(out, offset, payload.gyro_radps.z);
    appendDouble(out, offset, payload.temperature_c);
    appendU32(out, offset, payload.status_flags);
    out_size = static_cast<std::uint16_t>(offset);
    return true;
}

bool encodeChassisSpeedsPayload(
    const ChassisSpeedsPayload& payload,
    std::uint8_t* out,
    std::size_t capacity,
    std::uint16_t& out_size) {
    if (!out || !ensure(capacity, 44u)) {
        return false;
    }
    std::size_t offset = 0;
    appendU64(out, offset, payload.teensy_time_us);
    appendU64(out, offset, payload.rio_time_us);
    appendDouble(out, offset, payload.vx_mps);
    appendDouble(out, offset, payload.vy_mps);
    appendDouble(out, offset, payload.omega_radps);
    appendU32(out, offset, payload.status_flags);
    out_size = static_cast<std::uint16_t>(offset);
    return true;
}

bool encodeTeensyHealthPayload(
    const TeensyHealthPayload& payload,
    std::uint8_t* out,
    std::size_t capacity,
    std::uint16_t& out_size) {
    if (!out || !ensure(capacity, 76u)) {
        return false;
    }
    std::size_t offset = 0;
    appendU64(out, offset, payload.uptime_us);
    appendU32(out, offset, payload.error_flags);
    appendU32(out, offset, payload.trigger_status_flags);
    appendU32(out, offset, payload.rx_queue_depth);
    appendU32(out, offset, payload.tx_queue_depth);
    appendU64(out, offset, static_cast<std::uint64_t>(payload.rio_offset_us));
    appendU32(out, offset, payload.rio_status_flags);
    appendU32(out, offset, payload.accel_saturations);
    appendU32(out, offset, payload.gyro_saturations);
    appendU32(out, offset, payload.tof_samples_emitted);
    appendU32(out, offset, payload.tof_overruns);
    appendU32(out, offset, payload.tof_i2c_failures);
    appendU32(out, offset, payload.tof_status_flags);
    appendU32(out, offset, payload.fused_pose_decode_to_tx_min_us);
    appendU32(out, offset, payload.fused_pose_decode_to_tx_avg_us);
    appendU32(out, offset, payload.fused_pose_decode_to_tx_max_us);
    appendU32(out, offset, payload.fused_pose_latency_samples);
    out_size = static_cast<std::uint16_t>(offset);
    return true;
}

bool encodeToFSamplePayload(
    const ToFSamplePayload& payload,
    std::uint8_t* out,
    std::size_t capacity,
    std::uint16_t& out_size) {
    if (!out || !ensure(capacity, 36u)) {
        return false;
    }
    std::size_t offset = 0;
    appendU64(out, offset, payload.teensy_time_us);
    appendU32(out, offset, payload.trigger_sequence);
    appendU32(out, offset, payload.distance_mm);
    appendU32(out, offset, payload.ranging_duration_us);
    appendU32(out, offset, payload.firmware_status_flags);
    appendU32(out, offset, payload.signal_rate_kcps);
    appendU32(out, offset, payload.ambient_rate_kcps);
    appendU32(out, offset, payload.range_status);
    out_size = static_cast<std::uint16_t>(offset);
    return true;
}

bool encodeCameraTriggerEventPayload(
    const CameraTriggerEventPayload& payload,
    std::uint8_t* out,
    std::size_t capacity,
    std::uint16_t& out_size) {
    if (!out || !ensure(capacity, 20u)) {
        return false;
    }
    std::size_t offset = 0;
    appendU64(out, offset, payload.teensy_time_us);
    appendU32(out, offset, static_cast<std::uint32_t>(payload.pin));
    appendU32(out, offset, payload.trigger_sequence);
    appendU32(out, offset, payload.status_flags);
    out_size = static_cast<std::uint16_t>(offset);
    return true;
}

bool encodeTimeSyncResponsePayload(
    const TimeSyncResponsePayload& payload,
    std::uint8_t* out,
    std::size_t capacity,
    std::uint16_t& out_size) {
    if (!out || !ensure(capacity, 20u)) {
        return false;
    }
    std::size_t offset = 0;
    appendU32(out, offset, payload.request_sequence);
    appendU64(out, offset, payload.teensy_receive_time_us);
    appendU64(out, offset, payload.teensy_transmit_time_us);
    out_size = static_cast<std::uint16_t>(offset);
    return true;
}

bool decodeTimeSyncRequestPayload(const Frame& frame, TimeSyncRequestPayload& payload) {
    if (frame.payload_size != 12u) {
        return false;
    }
    payload.request_sequence = readU32(frame.payload, 0);
    payload.host_send_time_us = readU64(frame.payload, 4);
    return true;
}

bool decodeFusedPosePayload(const Frame& frame, FusedPosePayload& payload) {
    // v4 wire layout: 8 B host_send_time_us + 9 doubles (pose + velocity) +
    // 4 B has_velocity + 4 B status_flags + 36 doubles (covariance) = 376 B.
    constexpr std::uint16_t kExpectedSize = 376u;
    if (frame.payload_size != kExpectedSize) {
        return false;
    }
    std::size_t off = 0;
    payload.host_send_time_us = readU64(frame.payload, off); off += 8;
    payload.x_m = readDouble(frame.payload, off); off += 8;
    payload.y_m = readDouble(frame.payload, off); off += 8;
    payload.z_m = readDouble(frame.payload, off); off += 8;
    payload.roll_rad = readDouble(frame.payload, off); off += 8;
    payload.pitch_rad = readDouble(frame.payload, off); off += 8;
    payload.yaw_rad = readDouble(frame.payload, off); off += 8;
    payload.vx_mps = readDouble(frame.payload, off); off += 8;
    payload.vy_mps = readDouble(frame.payload, off); off += 8;
    payload.vz_mps = readDouble(frame.payload, off); off += 8;
    payload.has_velocity = readU32(frame.payload, off) != 0u; off += 4;
    payload.status_flags = readU32(frame.payload, off); off += 4;
    for (int i = 0; i < 36; ++i) {
        payload.covariance[i] = readDouble(frame.payload, off);
        off += 8;
    }
    return true;
}

bool decodeVioCompanionConfigPayload(const Frame& frame, VioCompanionCommand& command) {
    if (frame.payload_size < 4u) {
        return false;
    }
    if (readU32(frame.payload, 0) !=
        static_cast<std::uint32_t>(ConfigCommandKind::VioCompanion)) {
        return false;
    }
    if (frame.payload_size != 4u + 36u) {
        return false;
    }
    command.valid = true;
    command.vio_slot_index = readU32(frame.payload, 4);
    command.led_enabled = readU32(frame.payload, 8);
    command.led_pulse_width_us = readU32(frame.payload, 12);
    command.tof_enabled = readU32(frame.payload, 16);
    command.tof_i2c_address = readU32(frame.payload, 20);
    command.tof_timing_budget_ms = readU32(frame.payload, 24);
    command.tof_intermeasurement_period_ms = readU32(frame.payload, 28);
    command.tof_offset_after_flash_us = readU32(frame.payload, 32);
    command.tof_divisor = readU32(frame.payload, 36);
    return true;
}

bool decodeImuConfigPayload(const Frame& frame, ImuConfigCommand& command) {
    if (frame.payload_size < 12u) {
        return false;
    }
    if (readU32(frame.payload, 0) !=
        static_cast<std::uint32_t>(ConfigCommandKind::ImuConfig)) {
        return false;
    }
    if (frame.payload_size != 32u) {
        return false;
    }
    command.valid = true;
    command.accel_range_g = readU32(frame.payload, 4);
    command.accel_odr_hz = readU32(frame.payload, 8);
    command.accel_bandwidth_code = readU32(frame.payload, 12);
    command.gyro_range_dps = readU32(frame.payload, 16);
    command.gyro_bandwidth_code = readU32(frame.payload, 20);
    command.data_sync_rate_hz = readU32(frame.payload, 24);
    command.run_selftest_on_boot = readU32(frame.payload, 28) != 0u;
    return true;
}

bool encodeConfigAckPayload(
    const ConfigAckHeader& header,
    const TriggerAckEntry* trigger_entries,
    std::size_t trigger_entry_count,
    const ImuConfigAckEntry* imu_entry,
    const VioConfigAckEntry* vio_entry,
    std::uint8_t* out,
    std::size_t capacity,
    std::uint16_t& out_size) {
    if (!out) {
        return false;
    }
    std::size_t required = 12u;
    if (header.kind == static_cast<std::uint32_t>(ConfigCommandKind::CameraTriggers)) {
        required += trigger_entry_count * 12u;
    } else if (header.kind == static_cast<std::uint32_t>(ConfigCommandKind::ImuConfig)) {
        if (!imu_entry) {
            return false;
        }
        required += 28u;
    } else if (header.kind == static_cast<std::uint32_t>(ConfigCommandKind::VioCompanion)) {
        if (!vio_entry) {
            return false;
        }
        required += 36u;
    }
    if (!ensure(capacity, required)) {
        return false;
    }

    std::size_t offset = 0;
    appendU32(out, offset, header.kind);
    appendU32(out, offset, header.status_flags);
    appendU32(out, offset, header.effective_count);

    if (header.kind == static_cast<std::uint32_t>(ConfigCommandKind::CameraTriggers)) {
        for (std::size_t i = 0; i < trigger_entry_count; ++i) {
            const TriggerAckEntry& entry = trigger_entries[i];
            appendU32(out, offset, static_cast<std::uint32_t>(entry.pin));
            appendU32(out, offset, entry.period_us);
            appendU32(out, offset, entry.pulse_us);
        }
    } else if (header.kind == static_cast<std::uint32_t>(ConfigCommandKind::ImuConfig)) {
        appendU32(out, offset, imu_entry->accel_range_g);
        appendU32(out, offset, imu_entry->accel_odr_hz);
        appendU32(out, offset, imu_entry->accel_bandwidth_code);
        appendU32(out, offset, imu_entry->gyro_range_dps);
        appendU32(out, offset, imu_entry->gyro_bandwidth_code);
        appendU32(out, offset, imu_entry->data_sync_rate_hz);
        appendU32(out, offset, imu_entry->reserved);
    } else if (header.kind == static_cast<std::uint32_t>(ConfigCommandKind::VioCompanion)) {
        appendU32(out, offset, vio_entry->vio_slot_index);
        appendU32(out, offset, vio_entry->led_enabled);
        appendU32(out, offset, vio_entry->led_pulse_width_us);
        appendU32(out, offset, vio_entry->tof_enabled);
        appendU32(out, offset, vio_entry->tof_i2c_address);
        appendU32(out, offset, vio_entry->tof_timing_budget_ms);
        appendU32(out, offset, vio_entry->tof_intermeasurement_period_ms);
        appendU32(out, offset, vio_entry->tof_offset_after_flash_us);
        appendU32(out, offset, vio_entry->tof_divisor);
    }

    out_size = static_cast<std::uint16_t>(offset);
    return true;
}

bool decodeCameraTriggerConfigPayload(
    const Frame& frame,
    CameraTriggerCommand* commands,
    std::size_t command_capacity,
    std::size_t& command_count) {
    command_count = 0;
    if (!commands || frame.payload_size < 8u) {
        return false;
    }
    if (readU32(frame.payload, 0) !=
        static_cast<std::uint32_t>(ConfigCommandKind::CameraTriggers)) {
        return false;
    }
    const std::uint32_t wire_count = readU32(frame.payload, 4);
    const std::size_t expected_size = 8u + static_cast<std::size_t>(wire_count) * 28u;
    if (expected_size != frame.payload_size || wire_count > command_capacity) {
        return false;
    }

    std::size_t offset = 8;
    for (std::uint32_t i = 0; i < wire_count; ++i) {
        CameraTriggerCommand& command = commands[i];
        command.enabled = readU32(frame.payload, offset) != 0u;
        offset += 4;
        command.pin = static_cast<std::int32_t>(readU32(frame.payload, offset));
        offset += 4;
        command.rate_hz = readDouble(frame.payload, offset);
        offset += 8;
        command.pulse_width_us = readU32(frame.payload, offset);
        offset += 4;
        command.phase_offset_us = readI64(frame.payload, offset);
        offset += 8;
    }
    command_count = wire_count;
    return true;
}

void StreamDecoder::push(std::uint8_t byte) {
    if (size_ >= kMaxFrameSize) {
        clear();
    }
    buffer_[size_++] = byte;
}

bool StreamDecoder::nextFrame(Frame& frame) {
    while (size_ >= kHeaderSize + kCrcSize) {
        std::size_t magic_offset = 0;
        while (magic_offset + 1u < size_ &&
               (buffer_[magic_offset] != static_cast<std::uint8_t>(kFrameMagic & 0xFFu) ||
                buffer_[magic_offset + 1u] !=
                    static_cast<std::uint8_t>((kFrameMagic >> 8u) & 0xFFu))) {
            ++magic_offset;
        }
        if (magic_offset > 0u) {
            discard(magic_offset);
        }
        if (size_ < kHeaderSize + kCrcSize) {
            return false;
        }
        if (buffer_[2] != kProtocolVersion) {
            discard(1);
            continue;
        }

        const std::uint16_t payload_size = readU16(buffer_, 8);
        if (payload_size > kMaxPayloadSize) {
            discard(1);
            continue;
        }
        const std::size_t total_size = kHeaderSize + payload_size + kCrcSize;
        if (size_ < total_size) {
            return false;
        }

        const std::uint32_t expected_crc = readU32(buffer_, kHeaderSize + payload_size);
        const std::uint32_t actual_crc = crc32(buffer_, kHeaderSize + payload_size);
        if (expected_crc != actual_crc) {
            ++stats_.crc_failures;
            discard(1);
            continue;
        }

        frame.type = static_cast<MessageType>(buffer_[3]);
        frame.sequence = readU32(buffer_, 4);
        frame.payload_size = payload_size;
        if (payload_size > 0u) {
            std::memcpy(frame.payload, buffer_ + kHeaderSize, payload_size);
        }
        if (stats_.has_last_sequence &&
            frame.sequence != static_cast<std::uint32_t>(stats_.last_sequence + 1u)) {
            ++stats_.sequence_gaps;
        }
        stats_.has_last_sequence = true;
        stats_.last_sequence = frame.sequence;
        discard(total_size);
        return true;
    }
    return false;
}

void StreamDecoder::clear() {
    size_ = 0;
    stats_ = DecoderStats{};
}

void StreamDecoder::discard(std::size_t count) {
    if (count >= size_) {
        size_ = 0;
        return;
    }
    std::memmove(buffer_, buffer_ + count, size_ - count);
    size_ -= count;
}

}  // namespace posest::firmware
