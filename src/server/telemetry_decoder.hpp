#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "core/imu_types.hpp"

namespace gw::server {

// Incremental decoder for the Teensy's binary telemetry stream (the mirror
// of firmware/src/binary_proto.h):
//
//   [0xA5][0x5A][type u8][len u8][payload: len bytes][crc16 u16 LE]
//   crc16 = CCITT (poly 0x1021, init 0xFFFF) over type + len + payload.
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
        // fw=3 extension (absent on fw=2 — can_present=false, rest zeroed).
        bool     can_present   = false;
        bool     can_ok        = false;
        uint32_t can_rx        = 0;   // accepted chassis-speeds frames
        uint32_t can_rx_drops  = 0;   // ring overflows + stamp mismatches
        uint32_t odom_tx_drops = 0;   // ODOM packets lost to USB backpressure
        uint32_t pose_tx       = 0;   // pose frames written to the CAN bus
        uint8_t  can_mode      = 0;   // 0 off, 1 classic, 2 fd
    };

    // One ODOM packet (chassis speeds forwarded off the CAN bus). Raw wire
    // values — no clock math here; TeensyManager applies RioClockSync.
    struct Odom {
        uint64_t t_arrival_us = 0;  // Teensy clock at CAN RX
        uint64_t rio_time_us  = 0;  // 0 = unknown
        float    vx           = 0.0f;
        float    vy           = 0.0f;
        float    omega        = 0.0f;
        uint16_t status_flags = 0;
        uint8_t  counter      = 0;
        uint8_t  mode         = 0;  // 1 classic, 2 fd
    };

    struct Stats {
        uint64_t packets       = 0;
        uint64_t imu_samples   = 0;
        uint64_t odom_packets  = 0;
        uint64_t crc_errors    = 0;
        uint64_t bytes_skipped = 0;   // discarded while hunting for magic
        uint64_t unknown_types = 0;
    };

    std::function<void(const gw::ImuSample&)> on_imu;
    std::function<void(const Heartbeat&)>     on_heartbeat;
    std::function<void(const Odom&)>          on_odom;

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
