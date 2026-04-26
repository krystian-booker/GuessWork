#include "posest/teensy/Protocol.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

namespace posest::teensy {

namespace {

constexpr std::size_t kHeaderSize = 10;
constexpr std::size_t kCrcSize = 4;
constexpr std::size_t kMaxPayloadSize = 4096;

void appendU16(std::vector<std::uint8_t>& out, std::uint16_t value) {
    out.push_back(static_cast<std::uint8_t>(value & 0xFFu));
    out.push_back(static_cast<std::uint8_t>((value >> 8u) & 0xFFu));
}

void appendU32(std::vector<std::uint8_t>& out, std::uint32_t value) {
    for (int shift = 0; shift <= 24; shift += 8) {
        out.push_back(static_cast<std::uint8_t>((value >> static_cast<unsigned>(shift)) & 0xFFu));
    }
}

void appendU64(std::vector<std::uint8_t>& out, std::uint64_t value) {
    for (int shift = 0; shift <= 56; shift += 8) {
        out.push_back(static_cast<std::uint8_t>((value >> static_cast<unsigned>(shift)) & 0xFFu));
    }
}

void appendDouble(std::vector<std::uint8_t>& out, double value) {
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(value));
    appendU64(out, bits);
}

std::uint16_t readU16(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
    return static_cast<std::uint16_t>(bytes[offset]) |
           static_cast<std::uint16_t>(static_cast<std::uint16_t>(bytes[offset + 1]) << 8u);
}

std::uint32_t readU32(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
    return static_cast<std::uint32_t>(bytes[offset]) |
           (static_cast<std::uint32_t>(bytes[offset + 1]) << 8u) |
           (static_cast<std::uint32_t>(bytes[offset + 2]) << 16u) |
           (static_cast<std::uint32_t>(bytes[offset + 3]) << 24u);
}

std::uint64_t readU64(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
    std::uint64_t value = 0;
    for (int shift = 0; shift <= 56; shift += 8) {
        value |= static_cast<std::uint64_t>(bytes[offset + static_cast<std::size_t>(shift / 8)])
                 << static_cast<unsigned>(shift);
    }
    return value;
}

double readDouble(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
    const std::uint64_t bits = readU64(bytes, offset);
    double value = 0.0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&value, &bits, sizeof(value));
    return value;
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

std::vector<std::uint8_t> encodeFrame(const Frame& frame) {
    std::vector<std::uint8_t> out;
    out.reserve(kHeaderSize + frame.payload.size() + kCrcSize);
    appendU16(out, kFrameMagic);
    out.push_back(kProtocolVersion);
    out.push_back(static_cast<std::uint8_t>(frame.type));
    appendU32(out, frame.sequence);
    appendU16(out, static_cast<std::uint16_t>(frame.payload.size()));
    out.insert(out.end(), frame.payload.begin(), frame.payload.end());
    appendU32(out, crc32(out.data(), out.size()));
    return out;
}

std::optional<DecodeResult> decodeFrame(const std::vector<std::uint8_t>& bytes) {
    if (bytes.size() < kHeaderSize + kCrcSize) {
        return std::nullopt;
    }
    if (readU16(bytes, 0) != kFrameMagic || bytes[2] != kProtocolVersion) {
        return std::nullopt;
    }

    const std::uint16_t payload_size = readU16(bytes, 8);
    if (payload_size > kMaxPayloadSize) {
        return std::nullopt;
    }

    const std::size_t total_size = kHeaderSize + payload_size + kCrcSize;
    if (bytes.size() < total_size) {
        return std::nullopt;
    }

    const std::uint32_t expected_crc = readU32(bytes, kHeaderSize + payload_size);
    const std::uint32_t actual_crc = crc32(bytes.data(), kHeaderSize + payload_size);
    if (expected_crc != actual_crc) {
        return std::nullopt;
    }

    Frame frame;
    frame.type = static_cast<MessageType>(bytes[3]);
    frame.sequence = readU32(bytes, 4);
    frame.payload.assign(bytes.begin() + static_cast<std::ptrdiff_t>(kHeaderSize),
                         bytes.begin() + static_cast<std::ptrdiff_t>(kHeaderSize + payload_size));
    return DecodeResult{std::move(frame), total_size};
}

std::vector<std::uint8_t> encodeImuPayload(const ImuPayload& payload) {
    std::vector<std::uint8_t> out;
    out.reserve(68);
    appendU64(out, payload.teensy_time_us);
    appendDouble(out, payload.accel_mps2.x);
    appendDouble(out, payload.accel_mps2.y);
    appendDouble(out, payload.accel_mps2.z);
    appendDouble(out, payload.gyro_radps.x);
    appendDouble(out, payload.gyro_radps.y);
    appendDouble(out, payload.gyro_radps.z);
    appendDouble(out, payload.temperature_c.value_or(std::numeric_limits<double>::quiet_NaN()));
    appendU32(out, payload.status_flags);
    return out;
}

std::optional<ImuPayload> decodeImuPayload(const std::vector<std::uint8_t>& bytes) {
    if (bytes.size() != 68u) {
        return std::nullopt;
    }

    ImuPayload payload;
    std::size_t offset = 0;
    payload.teensy_time_us = readU64(bytes, offset);
    offset += 8;
    payload.accel_mps2.x = readDouble(bytes, offset);
    offset += 8;
    payload.accel_mps2.y = readDouble(bytes, offset);
    offset += 8;
    payload.accel_mps2.z = readDouble(bytes, offset);
    offset += 8;
    payload.gyro_radps.x = readDouble(bytes, offset);
    offset += 8;
    payload.gyro_radps.y = readDouble(bytes, offset);
    offset += 8;
    payload.gyro_radps.z = readDouble(bytes, offset);
    offset += 8;
    const double temperature = readDouble(bytes, offset);
    offset += 8;
    if (!std::isnan(temperature)) {
        payload.temperature_c = temperature;
    }
    payload.status_flags = readU32(bytes, offset);
    return payload;
}

std::vector<std::uint8_t> encodeChassisSpeedsPayload(
    const ChassisSpeedsPayload& payload) {
    std::vector<std::uint8_t> out;
    out.reserve(44);
    appendU64(out, payload.teensy_time_us);
    appendU64(out, payload.rio_time_us);
    appendDouble(out, payload.vx_mps);
    appendDouble(out, payload.vy_mps);
    appendDouble(out, payload.omega_radps);
    appendU32(out, payload.status_flags);
    return out;
}

std::optional<ChassisSpeedsPayload> decodeChassisSpeedsPayload(
    const std::vector<std::uint8_t>& bytes) {
    if (bytes.size() != 44u) {
        return std::nullopt;
    }

    ChassisSpeedsPayload payload;
    std::size_t offset = 0;
    payload.teensy_time_us = readU64(bytes, offset);
    offset += 8;
    payload.rio_time_us = readU64(bytes, offset);
    offset += 8;
    payload.vx_mps = readDouble(bytes, offset);
    offset += 8;
    payload.vy_mps = readDouble(bytes, offset);
    offset += 8;
    payload.omega_radps = readDouble(bytes, offset);
    offset += 8;
    payload.status_flags = readU32(bytes, offset);
    return payload;
}

std::vector<std::uint8_t> encodeTeensyHealthPayload(const TeensyHealthPayload& payload) {
    std::vector<std::uint8_t> out;
    out.reserve(60);
    appendU64(out, payload.uptime_us);
    appendU32(out, payload.error_flags);
    appendU32(out, payload.trigger_status_flags);
    appendU32(out, payload.rx_queue_depth);
    appendU32(out, payload.tx_queue_depth);
    appendU64(out, static_cast<std::uint64_t>(payload.rio_offset_us));
    appendU32(out, payload.rio_status_flags);
    appendU32(out, payload.accel_saturations);
    appendU32(out, payload.gyro_saturations);
    appendU32(out, payload.tof_samples_emitted);
    appendU32(out, payload.tof_overruns);
    appendU32(out, payload.tof_i2c_failures);
    appendU32(out, payload.tof_status_flags);
    return out;
}

std::optional<TeensyHealthPayload> decodeTeensyHealthPayload(
    const std::vector<std::uint8_t>& bytes) {
    if (bytes.size() != 60u) {
        return std::nullopt;
    }
    TeensyHealthPayload payload;
    payload.uptime_us = readU64(bytes, 0);
    payload.error_flags = readU32(bytes, 8);
    payload.trigger_status_flags = readU32(bytes, 12);
    payload.rx_queue_depth = readU32(bytes, 16);
    payload.tx_queue_depth = readU32(bytes, 20);
    payload.rio_offset_us = static_cast<std::int64_t>(readU64(bytes, 24));
    payload.rio_status_flags = readU32(bytes, 32);
    payload.accel_saturations = readU32(bytes, 36);
    payload.gyro_saturations = readU32(bytes, 40);
    payload.tof_samples_emitted = readU32(bytes, 44);
    payload.tof_overruns = readU32(bytes, 48);
    payload.tof_i2c_failures = readU32(bytes, 52);
    payload.tof_status_flags = readU32(bytes, 56);
    return payload;
}

std::vector<std::uint8_t> encodeCameraTriggerEventPayload(
    const CameraTriggerEventPayload& payload) {
    std::vector<std::uint8_t> out;
    out.reserve(20);
    appendU64(out, payload.teensy_time_us);
    appendU32(out, static_cast<std::uint32_t>(payload.pin));
    appendU32(out, payload.trigger_sequence);
    appendU32(out, payload.status_flags);
    return out;
}

std::optional<CameraTriggerEventPayload> decodeCameraTriggerEventPayload(
    const std::vector<std::uint8_t>& bytes) {
    if (bytes.size() != 20u) {
        return std::nullopt;
    }
    CameraTriggerEventPayload payload;
    payload.teensy_time_us = readU64(bytes, 0);
    payload.pin = static_cast<std::int32_t>(readU32(bytes, 8));
    payload.trigger_sequence = readU32(bytes, 12);
    payload.status_flags = readU32(bytes, 16);
    return payload;
}

namespace {
constexpr std::size_t kToFSamplePayloadSize = 36;
constexpr std::size_t kVioCompanionConfigBodySize = 36;
}  // namespace

std::vector<std::uint8_t> encodeToFSamplePayload(const ToFSamplePayload& payload) {
    std::vector<std::uint8_t> out;
    out.reserve(kToFSamplePayloadSize);
    appendU64(out, payload.teensy_time_us);
    appendU32(out, payload.trigger_sequence);
    appendU32(out, payload.distance_mm);
    appendU32(out, payload.ranging_duration_us);
    appendU32(out, payload.firmware_status_flags);
    appendU32(out, payload.signal_rate_kcps);
    appendU32(out, payload.ambient_rate_kcps);
    appendU32(out, payload.range_status);
    return out;
}

std::optional<ToFSamplePayload> decodeToFSamplePayload(
    const std::vector<std::uint8_t>& bytes) {
    if (bytes.size() != kToFSamplePayloadSize) {
        return std::nullopt;
    }
    ToFSamplePayload payload;
    payload.teensy_time_us = readU64(bytes, 0);
    payload.trigger_sequence = readU32(bytes, 8);
    payload.distance_mm = readU32(bytes, 12);
    payload.ranging_duration_us = readU32(bytes, 16);
    payload.firmware_status_flags = readU32(bytes, 20);
    payload.signal_rate_kcps = readU32(bytes, 24);
    payload.ambient_rate_kcps = readU32(bytes, 28);
    payload.range_status = readU32(bytes, 32);
    return payload;
}

std::vector<std::uint8_t> encodeVioCompanionConfigPayload(
    const VioCompanionConfigPayload& payload) {
    std::vector<std::uint8_t> out;
    out.reserve(kVioCompanionConfigBodySize);
    appendU32(out, payload.vio_slot_index);
    appendU32(out, payload.led_enabled);
    appendU32(out, payload.led_pulse_width_us);
    appendU32(out, payload.tof_enabled);
    appendU32(out, payload.tof_i2c_address);
    appendU32(out, payload.tof_timing_budget_ms);
    appendU32(out, payload.tof_intermeasurement_period_ms);
    appendU32(out, payload.tof_offset_after_flash_us);
    appendU32(out, payload.tof_divisor);
    return out;
}

std::optional<VioCompanionConfigPayload> decodeVioCompanionConfigPayload(
    const std::vector<std::uint8_t>& bytes) {
    if (bytes.size() != kVioCompanionConfigBodySize) {
        return std::nullopt;
    }
    VioCompanionConfigPayload payload;
    payload.vio_slot_index = readU32(bytes, 0);
    payload.led_enabled = readU32(bytes, 4);
    payload.led_pulse_width_us = readU32(bytes, 8);
    payload.tof_enabled = readU32(bytes, 12);
    payload.tof_i2c_address = readU32(bytes, 16);
    payload.tof_timing_budget_ms = readU32(bytes, 20);
    payload.tof_intermeasurement_period_ms = readU32(bytes, 24);
    payload.tof_offset_after_flash_us = readU32(bytes, 28);
    payload.tof_divisor = readU32(bytes, 32);
    return payload;
}

std::vector<std::uint8_t> encodeTimeSyncRequestPayload(
    const TimeSyncRequestPayload& payload) {
    std::vector<std::uint8_t> out;
    out.reserve(12);
    appendU32(out, payload.request_sequence);
    appendU64(out, payload.host_send_time_us);
    return out;
}

std::optional<TimeSyncRequestPayload> decodeTimeSyncRequestPayload(
    const std::vector<std::uint8_t>& bytes) {
    if (bytes.size() != 12u) {
        return std::nullopt;
    }
    return TimeSyncRequestPayload{readU32(bytes, 0), readU64(bytes, 4)};
}

std::vector<std::uint8_t> encodeTimeSyncResponsePayload(
    const TimeSyncResponsePayload& payload) {
    std::vector<std::uint8_t> out;
    out.reserve(20);
    appendU32(out, payload.request_sequence);
    appendU64(out, payload.teensy_receive_time_us);
    appendU64(out, payload.teensy_transmit_time_us);
    return out;
}

std::optional<TimeSyncResponsePayload> decodeTimeSyncResponsePayload(
    const std::vector<std::uint8_t>& bytes) {
    if (bytes.size() != 20u) {
        return std::nullopt;
    }
    return TimeSyncResponsePayload{readU32(bytes, 0), readU64(bytes, 4), readU64(bytes, 12)};
}

namespace {

constexpr std::size_t kConfigAckHeaderSize = 12;
constexpr std::size_t kTriggerAckEntrySize = 12;
constexpr std::size_t kImuConfigAckBodySize = 28;
constexpr std::size_t kImuConfigPayloadSize = 28;
constexpr std::size_t kVioConfigAckBodySize = 36;

}  // namespace

std::vector<std::uint8_t> encodeConfigAckPayload(const ConfigAckPayload& payload) {
    std::vector<std::uint8_t> out;
    out.reserve(kConfigAckHeaderSize + payload.trigger_entries.size() * kTriggerAckEntrySize +
                (payload.imu_entry ? kImuConfigAckBodySize : 0u) +
                (payload.vio_entry ? kVioConfigAckBodySize : 0u));
    appendU32(out, payload.kind);
    appendU32(out, payload.status_flags);
    appendU32(out, payload.effective_count);

    if (payload.kind == static_cast<std::uint32_t>(ConfigCommandKind::CameraTriggers)) {
        for (const auto& entry : payload.trigger_entries) {
            appendU32(out, static_cast<std::uint32_t>(entry.pin));
            appendU32(out, entry.period_us);
            appendU32(out, entry.pulse_us);
        }
    } else if (payload.kind == static_cast<std::uint32_t>(ConfigCommandKind::ImuConfig) &&
               payload.imu_entry) {
        const auto& entry = *payload.imu_entry;
        appendU32(out, entry.accel_range_g);
        appendU32(out, entry.accel_odr_hz);
        appendU32(out, entry.accel_bandwidth_code);
        appendU32(out, entry.gyro_range_dps);
        appendU32(out, entry.gyro_bandwidth_code);
        appendU32(out, entry.data_sync_rate_hz);
        appendU32(out, entry.reserved);
    } else if (payload.kind == static_cast<std::uint32_t>(ConfigCommandKind::VioCompanion) &&
               payload.vio_entry) {
        const auto& entry = *payload.vio_entry;
        appendU32(out, entry.vio_slot_index);
        appendU32(out, entry.led_enabled);
        appendU32(out, entry.led_pulse_width_us);
        appendU32(out, entry.tof_enabled);
        appendU32(out, entry.tof_i2c_address);
        appendU32(out, entry.tof_timing_budget_ms);
        appendU32(out, entry.tof_intermeasurement_period_ms);
        appendU32(out, entry.tof_offset_after_flash_us);
        appendU32(out, entry.tof_divisor);
    }
    return out;
}

std::optional<ConfigAckPayload> decodeConfigAckPayload(
    const std::vector<std::uint8_t>& bytes) {
    if (bytes.size() < kConfigAckHeaderSize) {
        return std::nullopt;
    }
    ConfigAckPayload payload;
    payload.kind = readU32(bytes, 0);
    payload.status_flags = readU32(bytes, 4);
    payload.effective_count = readU32(bytes, 8);

    if (payload.kind == static_cast<std::uint32_t>(ConfigCommandKind::CameraTriggers)) {
        const std::size_t expected =
            kConfigAckHeaderSize +
            static_cast<std::size_t>(payload.effective_count) * kTriggerAckEntrySize;
        if (bytes.size() != expected) {
            return std::nullopt;
        }
        payload.trigger_entries.reserve(payload.effective_count);
        std::size_t offset = kConfigAckHeaderSize;
        for (std::uint32_t i = 0; i < payload.effective_count; ++i) {
            TriggerAckEntry entry;
            entry.pin = static_cast<std::int32_t>(readU32(bytes, offset));
            offset += 4;
            entry.period_us = readU32(bytes, offset);
            offset += 4;
            entry.pulse_us = readU32(bytes, offset);
            offset += 4;
            payload.trigger_entries.push_back(entry);
        }
        return payload;
    }

    if (payload.kind == static_cast<std::uint32_t>(ConfigCommandKind::ImuConfig)) {
        if (bytes.size() != kConfigAckHeaderSize + kImuConfigAckBodySize) {
            return std::nullopt;
        }
        ImuConfigAckEntry entry;
        std::size_t offset = kConfigAckHeaderSize;
        entry.accel_range_g = readU32(bytes, offset);
        offset += 4;
        entry.accel_odr_hz = readU32(bytes, offset);
        offset += 4;
        entry.accel_bandwidth_code = readU32(bytes, offset);
        offset += 4;
        entry.gyro_range_dps = readU32(bytes, offset);
        offset += 4;
        entry.gyro_bandwidth_code = readU32(bytes, offset);
        offset += 4;
        entry.data_sync_rate_hz = readU32(bytes, offset);
        offset += 4;
        entry.reserved = readU32(bytes, offset);
        payload.imu_entry = entry;
        return payload;
    }

    if (payload.kind == static_cast<std::uint32_t>(ConfigCommandKind::VioCompanion)) {
        // VIO ack always carries an entry — the firmware echoes whatever it
        // accepted (or zeros + the relevant kConfigAck* failure bit).
        if (bytes.size() != kConfigAckHeaderSize + kVioConfigAckBodySize) {
            return std::nullopt;
        }
        VioConfigAckEntry entry;
        std::size_t offset = kConfigAckHeaderSize;
        entry.vio_slot_index = readU32(bytes, offset);
        offset += 4;
        entry.led_enabled = readU32(bytes, offset);
        offset += 4;
        entry.led_pulse_width_us = readU32(bytes, offset);
        offset += 4;
        entry.tof_enabled = readU32(bytes, offset);
        offset += 4;
        entry.tof_i2c_address = readU32(bytes, offset);
        offset += 4;
        entry.tof_timing_budget_ms = readU32(bytes, offset);
        offset += 4;
        entry.tof_intermeasurement_period_ms = readU32(bytes, offset);
        offset += 4;
        entry.tof_offset_after_flash_us = readU32(bytes, offset);
        offset += 4;
        entry.tof_divisor = readU32(bytes, offset);
        payload.vio_entry = entry;
        return payload;
    }

    if (bytes.size() != kConfigAckHeaderSize) {
        return std::nullopt;
    }
    return payload;
}

std::vector<std::uint8_t> encodeImuConfigPayload(const ImuConfigPayload& payload) {
    std::vector<std::uint8_t> out;
    out.reserve(kImuConfigPayloadSize);
    appendU32(out, payload.accel_range_g);
    appendU32(out, payload.accel_odr_hz);
    appendU32(out, payload.accel_bandwidth_code);
    appendU32(out, payload.gyro_range_dps);
    appendU32(out, payload.gyro_bandwidth_code);
    appendU32(out, payload.data_sync_rate_hz);
    appendU32(out, payload.run_selftest_on_boot);
    return out;
}

std::optional<ImuConfigPayload> decodeImuConfigPayload(
    const std::vector<std::uint8_t>& bytes) {
    if (bytes.size() != kImuConfigPayloadSize) {
        return std::nullopt;
    }
    ImuConfigPayload payload;
    payload.accel_range_g = readU32(bytes, 0);
    payload.accel_odr_hz = readU32(bytes, 4);
    payload.accel_bandwidth_code = readU32(bytes, 8);
    payload.gyro_range_dps = readU32(bytes, 12);
    payload.gyro_bandwidth_code = readU32(bytes, 16);
    payload.data_sync_rate_hz = readU32(bytes, 20);
    payload.run_selftest_on_boot = readU32(bytes, 24);
    return payload;
}

std::vector<Frame> StreamDecoder::push(const std::uint8_t* data, std::size_t size) {
    buffer_.insert(buffer_.end(), data, data + size);

    std::vector<Frame> frames;
    while (buffer_.size() >= kHeaderSize + kCrcSize) {
        const std::array<std::uint8_t, 2> magic{
            static_cast<std::uint8_t>(kFrameMagic & 0xFFu),
            static_cast<std::uint8_t>((kFrameMagic >> 8u) & 0xFFu)};
        auto magic_it = std::search(
            buffer_.begin(), buffer_.end(), magic.begin(), magic.end());

        if (magic_it == buffer_.end()) {
            buffer_.clear();
            break;
        }
        if (magic_it != buffer_.begin()) {
            buffer_.erase(buffer_.begin(), magic_it);
        }

        if (buffer_.size() < kHeaderSize + kCrcSize) {
            break;
        }

        const std::uint16_t payload_size = readU16(buffer_, 8);
        if (payload_size > kMaxPayloadSize) {
            buffer_.erase(buffer_.begin());
            continue;
        }

        const std::size_t total_size = kHeaderSize + payload_size + kCrcSize;
        if (buffer_.size() < total_size) {
            break;
        }

        auto decoded = decodeFrame(buffer_);
        if (!decoded) {
            ++stats_.crc_failures;
            buffer_.erase(buffer_.begin());
            continue;
        }

        Frame frame = std::move(decoded->frame);
        if (stats_.last_sequence &&
            frame.sequence != static_cast<std::uint32_t>(*stats_.last_sequence + 1u)) {
            ++stats_.sequence_gaps;
        }
        stats_.last_sequence = frame.sequence;
        frames.push_back(std::move(frame));
        buffer_.erase(buffer_.begin(),
                      buffer_.begin() + static_cast<std::ptrdiff_t>(decoded->bytes_consumed));
    }

    return frames;
}

void StreamDecoder::clear() {
    buffer_.clear();
    stats_ = StreamStats{};
}

}  // namespace posest::teensy
