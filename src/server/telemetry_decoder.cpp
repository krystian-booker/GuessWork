#include "server/telemetry_decoder.hpp"

#include <cstring>

namespace gw::server {

namespace {

constexpr uint8_t kMagic0 = 0xA5;
constexpr uint8_t kMagic1 = 0x5A;

constexpr uint8_t kTypeImuBatch  = 0x01;
constexpr uint8_t kTypeHeartbeat = 0x02;

constexpr size_t kHeaderLen   = 4;            // magic0 magic1 type len
constexpr size_t kCrcLen      = 2;
constexpr size_t kImuRecLen   = 8 + 6 * 4;    // t_us u64 + 6 × f32

uint16_t crc16_ccitt(const uint8_t* data, size_t n) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

uint32_t get_u32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

uint64_t get_u64(const uint8_t* p) {
    return static_cast<uint64_t>(get_u32(p)) |
           (static_cast<uint64_t>(get_u32(p + 4)) << 32);
}

float get_f32(const uint8_t* p) {
    const uint32_t bits = get_u32(p);
    float v;
    std::memcpy(&v, &bits, sizeof(v));
    return v;
}

}  // namespace

void TelemetryDecoder::feed(const uint8_t* data, size_t n) {
    buf_.insert(buf_.end(), data, data + n);

    size_t consumed_total = 0;
    while (true) {
        // Hunt for the magic at the front of the unconsumed region.
        const size_t avail = buf_.size() - consumed_total;
        const uint8_t* p   = buf_.data() + consumed_total;
        if (avail < 2) break;
        if (p[0] != kMagic0 || p[1] != kMagic1) {
            ++consumed_total;
            ++stats_.bytes_skipped;
            continue;
        }
        if (avail < kHeaderLen) break;
        const uint8_t type        = p[2];
        const size_t  payload_len = p[3];
        const size_t  frame_len   = kHeaderLen + payload_len + kCrcLen;
        if (avail < frame_len) break;

        const uint16_t want = static_cast<uint16_t>(
            p[kHeaderLen + payload_len] |
            (p[kHeaderLen + payload_len + 1] << 8));
        const uint16_t got = crc16_ccitt(p + 2, payload_len + 2);
        if (want != got) {
            // Skip just the first magic byte — the real frame start may be
            // inside what we mistook for a frame.
            ++stats_.crc_errors;
            ++consumed_total;
            continue;
        }

        dispatch(type, p + kHeaderLen, payload_len);
        ++stats_.packets;
        consumed_total += frame_len;
    }

    buf_.erase(buf_.begin(), buf_.begin() + static_cast<ptrdiff_t>(consumed_total));
}

void TelemetryDecoder::dispatch(uint8_t type, const uint8_t* payload, size_t len) {
    switch (type) {
        case kTypeImuBatch: {
            if (len < 1) return;
            const size_t count = payload[0];
            if (len != 1 + count * kImuRecLen) return;  // malformed; drop
            for (size_t i = 0; i < count; ++i) {
                const uint8_t* rec = payload + 1 + i * kImuRecLen;
                gw::ImuSample s;
                s.t_ns = get_u64(rec) * 1000ull;  // µs → ns, Teensy clock
                for (int a = 0; a < 3; ++a) s.accel[a] = get_f32(rec + 8 + 4 * a);
                for (int g = 0; g < 3; ++g) s.gyro[g]  = get_f32(rec + 20 + 4 * g);
                ++stats_.imu_samples;
                if (on_imu) on_imu(s);
            }
            return;
        }
        case kTypeHeartbeat: {
            if (len != 8 + 1 + 4 + 4) return;
            Heartbeat hb;
            hb.t_us        = get_u64(payload);
            hb.imu_ok      = (payload[8] & 0x01) != 0;
            hb.imu_samples = get_u32(payload + 9);
            hb.imu_drops   = get_u32(payload + 13);
            if (on_heartbeat) on_heartbeat(hb);
            return;
        }
        default:
            // Unknown packet type (e.g. a newer firmware's ODOM before the
            // host learns it). The frame already passed CRC; skip silently
            // but count it.
            ++stats_.unknown_types;
            return;
    }
}

}  // namespace gw::server
