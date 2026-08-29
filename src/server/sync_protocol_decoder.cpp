#include "server/sync_protocol_decoder.hpp"

#include <algorithm>

namespace gw::server {

void SyncProtocolDecoder::feed(const uint8_t* data, size_t n) {
    if (!data || n == 0) return;
    buf_.insert(buf_.end(), data, data + n);

    size_t consumed = 0;
    for (;;) {
        const size_t avail = buf_.size() - consumed;
        const uint8_t* p = buf_.data() + consumed;
        if (avail < 2) break;
        if (p[0] != gw_sync::kMagic0 || p[1] != gw_sync::kMagic1) {
            ++consumed;
            ++stats_.bytes_skipped;
            continue;
        }
        if (avail < gw_sync::kHeaderBytes) break;
        if (p[2] != gw_sync::kProtocolVersion) {
            ++consumed;
            ++stats_.version_errors;
            continue;
        }
        const uint16_t payload_len = gw_sync::get_u16(p, 6);
        if (payload_len > gw_sync::kMaxPayloadBytes) {
            ++consumed;
            ++stats_.malformed_frames;
            continue;
        }
        const size_t frame_len = gw_sync::kHeaderBytes + payload_len +
                                 gw_sync::kTrailerBytes;
        if (avail < frame_len) break;

        const uint16_t expected = gw_sync::get_u16(p, gw_sync::kHeaderBytes + payload_len);
        const uint16_t actual = gw_sync::crc16_ccitt(p + 2, 6 + payload_len);
        if (expected != actual) {
            ++consumed;
            ++stats_.crc_errors;
            continue;
        }

        dispatch(static_cast<gw_sync::MessageType>(p[3]),
                 gw_sync::get_u16(p, 4), p + gw_sync::kHeaderBytes,
                 payload_len);
        ++stats_.packets;
        consumed += frame_len;
    }

    if (consumed) {
        buf_.erase(buf_.begin(), buf_.begin() + static_cast<ptrdiff_t>(consumed));
    }
}

void SyncProtocolDecoder::reset() {
    buf_.clear();
    stats_ = {};
}

void SyncProtocolDecoder::dispatch(gw_sync::MessageType type,
                                   uint16_t request_id,
                                   const uint8_t* payload, size_t len) {
    switch (type) {
        case gw_sync::MessageType::DeviceInfo: {
            gw_sync::DeviceInfo info;
            if (!gw_sync::decode_device_info(payload, len, info)) {
                ++stats_.malformed_frames;
                return;
            }
            if (on_device_info) on_device_info(request_id, info);
            return;
        }
        case gw_sync::MessageType::Ack: {
            if (len != 4) {
                ++stats_.malformed_frames;
                return;
            }
            Ack ack;
            ack.request_id = request_id;
            ack.command = static_cast<gw_sync::MessageType>(payload[0]);
            ack.status = static_cast<gw_sync::AckStatus>(payload[1]);
            if (on_ack) on_ack(ack);
            return;
        }
        case gw_sync::MessageType::Trigger: {
            if (len != 16) {
                ++stats_.malformed_frames;
                return;
            }
            gw_sync::TriggerEvent event;
            event.slot  = payload[0];
            event.index = gw_sync::get_u32(payload, 4);
            event.t_us  = gw_sync::get_u64(payload, 8);
            if (on_trigger) on_trigger(event);
            return;
        }
        case gw_sync::MessageType::ImuBatch: {
            if (len < 4) {
                ++stats_.malformed_frames;
                return;
            }
            const size_t count = payload[0];
            if (count > 4 || len != 4 + count * 32) {
                ++stats_.malformed_frames;
                return;
            }
            for (size_t i = 0; i < count; ++i) {
                const uint8_t* rec = payload + 4 + i * 32;
                gw::ImuSample sample;
                sample.t_ns = gw_sync::get_u64(rec, 0) * 1000ull;
                for (int axis = 0; axis < 3; ++axis) {
                    sample.accel[axis] = gw_sync::get_f32(rec, 8 + 4 * axis);
                    sample.gyro[axis]  = gw_sync::get_f32(rec, 20 + 4 * axis);
                }
                ++stats_.imu_samples;
                if (on_imu) on_imu(sample);
            }
            return;
        }
        case gw_sync::MessageType::Heartbeat: {
            if (len != 32) {
                ++stats_.malformed_frames;
                return;
            }
            gw_sync::Heartbeat heartbeat;
            heartbeat.t_us          = gw_sync::get_u64(payload, 0);
            heartbeat.flags         = gw_sync::get_u32(payload, 8);
            heartbeat.imu_samples   = gw_sync::get_u32(payload, 12);
            heartbeat.imu_drops     = gw_sync::get_u32(payload, 16);
            heartbeat.trigger_drops = gw_sync::get_u32(payload, 20);
            heartbeat.usb_errors    = gw_sync::get_u32(payload, 24);
            // bytes 28..31 are reserved for forward-compatible counters.
            if (on_heartbeat) on_heartbeat(heartbeat);
            return;
        }
        default:
            ++stats_.unknown_types;
            return;
    }
}

}  // namespace gw::server
