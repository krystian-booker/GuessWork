#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "core/imu_types.hpp"

namespace gw::server {

// Incremental decoder for the Teensy's binary telemetry stream (the mirror
// of firmware/src/binary_proto.h / firmware/src/telemetry_payloads.h):
//
//   [0xA5][0x5A][type u8][len u8][payload: len bytes][crc16 u16 LE]
//   crc16 = CCITT (poly 0x1021, init 0xFFFF) over type + len + payload.
//
// Packet types: 0x01 IMU_BATCH and 0x02 HEARTBEAT (17-byte payload). An
// un-reflashed fw=3 board appends CAN counters to the heartbeat (34 bytes);
// the decoder parses the 17-byte prefix and ignores the tail.
//
// feed() accepts arbitrary byte chunks (whatever read() returned), buffers
// partial frames, validates CRCs and dispatches complete packets through the
// callbacks. Garbage and CRC failures cause a one-byte resync scan — ASCII
// noise can never contain the 0xA5 0x5A magic, so recovery is prompt.
//
// Pure and single-threaded by design: no I/O, no locks. The owner
// (TeensyManager's telemetry read loop) provides the thread.
class TelemetryDecoder {
public:
    struct Heartbeat {
        uint64_t t_us        = 0;     // Teensy clock, microseconds
        bool     imu_ok      = false;
        uint32_t imu_samples = 0;     // firmware-side totals since boot
        uint32_t imu_drops   = 0;
    };

    struct Stats {
        uint64_t packets       = 0;
        uint64_t imu_samples   = 0;
        uint64_t crc_errors    = 0;
        uint64_t bytes_skipped = 0;   // discarded while hunting for magic
        uint64_t unknown_types = 0;
    };

    std::function<void(const gw::ImuSample&)> on_imu;
    std::function<void(const Heartbeat&)>     on_heartbeat;

    void feed(const uint8_t* data, size_t n);

    const Stats& stats() const { return stats_; }

private:
    // Attempts to decode one frame at the front of buf_. Returns the number
    // of bytes to consume (0 = need more data).
    size_t try_decode_front();
    void   dispatch(uint8_t type, const uint8_t* payload, size_t len);

    std::vector<uint8_t> buf_;
    Stats                stats_;
};

}  // namespace gw::server
