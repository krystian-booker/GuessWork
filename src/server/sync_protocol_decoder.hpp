#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "core/imu_types.hpp"
#include "firmware/include/sync_controller_protocol.h"

namespace gw::server {

// Incremental decoder for the single bidirectional MicoAir USB CDC stream.
// It accepts arbitrary read() chunking, validates protocol version/length/CRC,
// and resynchronizes one byte at a time after malformed traffic.
class SyncProtocolDecoder {
public:
    struct Ack {
        uint16_t request_id = 0;
        gw_sync::MessageType command = gw_sync::MessageType::Hello;
        gw_sync::AckStatus status = gw_sync::AckStatus::BadMessage;
    };

    struct Stats {
        uint64_t packets          = 0;
        uint64_t imu_samples      = 0;
        uint64_t crc_errors       = 0;
        uint64_t version_errors   = 0;
        uint64_t malformed_frames = 0;
        uint64_t bytes_skipped    = 0;
        uint64_t unknown_types    = 0;
    };

    std::function<void(uint16_t, const gw_sync::DeviceInfo&)> on_device_info;
    std::function<void(const Ack&)>                            on_ack;
    std::function<void(const gw_sync::TriggerEvent&)>          on_trigger;
    std::function<void(const gw::ImuSample&)>                  on_imu;
    std::function<void(const gw_sync::Heartbeat&)>             on_heartbeat;

    void feed(const uint8_t* data, size_t n);
    void reset();

    const Stats& stats() const { return stats_; }

private:
    void dispatch(gw_sync::MessageType type, uint16_t request_id,
                  const uint8_t* payload, size_t len);

    std::vector<uint8_t> buf_;
    Stats stats_;
};

}  // namespace gw::server
