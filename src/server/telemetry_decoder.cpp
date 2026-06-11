#include "server/telemetry_decoder.hpp"

#include <cstring>

namespace gw::server {

namespace {

constexpr uint8_t kMagic0 = 0xA5;
constexpr uint8_t kMagic1 = 0x5A;

constexpr uint8_t kTypeImuBatch  = 0x01;
constexpr uint8_t kTypeHeartbeat = 0x02;
constexpr uint8_t kTypeOdom      = 0x03;

constexpr size_t kHeaderLen   = 4;            // magic0 magic1 type len
constexpr size_t kCrcLen      = 2;
constexpr size_t kImuRecLen   = 8 + 6 * 4;    // t_us u64 + 6 × f32
constexpr size_t kHeartbeatV2Len = 17;        // fw=2: t_us, flags, 2 × u32
constexpr size_t kHeartbeatV3Len = 34;        // fw=3: + 4 × u32 + can_mode u8
constexpr size_t kOdomLen        = 32;

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
            // fw=2 sends the 17-byte prefix only; fw=3 appends CAN counters.
            if (len != kHeartbeatV2Len && len != kHeartbeatV3Len) return;
            Heartbeat hb;
            hb.t_us        = get_u64(payload);
            hb.imu_ok      = (payload[8] & 0x01) != 0;
            hb.imu_samples = get_u32(payload + 9);
            hb.imu_drops   = get_u32(payload + 13);
            if (len >= kHeartbeatV3Len) {
                hb.can_present   = true;
                hb.can_ok        = (payload[8] & 0x02) != 0;
                hb.can_rx        = get_u32(payload + 17);
                hb.can_rx_drops  = get_u32(payload + 21);
                hb.odom_tx_drops = get_u32(payload + 25);
                hb.pose_tx       = get_u32(payload + 29);
                hb.can_mode      = payload[33];
            }
            if (on_heartbeat) on_heartbeat(hb);
            return;
        }
        case kTypeOdom: {
            if (len != kOdomLen) return;  // malformed; drop
            Odom o;
            o.t_arrival_us = get_u64(payload);
            o.rio_time_us  = get_u64(payload + 8);
            o.vx           = get_f32(payload + 16);
            o.vy           = get_f32(payload + 20);
            o.omega        = get_f32(payload + 24);
            o.status_flags = static_cast<uint16_t>(payload[28] |
                                                   (payload[29] << 8));
            o.counter      = payload[30];
            o.mode         = payload[31];
            ++stats_.odom_packets;
            if (on_odom) on_odom(o);
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
