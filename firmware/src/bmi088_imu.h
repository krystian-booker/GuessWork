#pragma once

#include <Arduino.h>

// BMI088 accel+gyro over SPI0, streamed to the host as IMU_BATCH binary
// packets on SerialUSB1 (see binary_proto.h).
//
// Timestamping: the gyro's INT3 data-ready output drives a GPIO interrupt;
// the ISR latches the wrap-extended 64-bit Teensy clock (time64.h) — the
// same clock that stamps camera trigger pulses, so IMU samples and frames
// share a time domain by construction. The main loop then reads both dies
// over SPI and appends the sample to the current batch. The accel is read
// at the gyro's cadence (one combined stream, the layout Kalibr and
// OpenVINS expect).
//
// If the main loop hasn't consumed the previous DRDY event when the next
// one fires, the older event is overwritten (latest-wins) and counted in
// drops — the host sees the count in HEARTBEAT packets.
//
// Wiring (SPI0: MOSI=11, MISO=12, SCK=13):
//   accel CS  -> pin 10
//   gyro  CS  -> pin 9
//   gyro INT3 -> pin 8 (push-pull, active-high)
// Trigger outputs own pins 2..7, so this block is contiguous and free.

namespace gw_fw {

constexpr uint8_t kImuAccelCsPin  = 10;
constexpr uint8_t kImuGyroCsPin   = 9;
constexpr uint8_t kImuGyroDrdyPin = 8;

class Bmi088Streamer {
public:
    // Initialise both dies, configure 400 Hz ODR, ±24 g / ±2000 dps ranges,
    // and attach the DRDY interrupt. Returns false if either die fails to
    // respond (IMU absent / miswired) — the streamer then stays inert and
    // ok() reports false.
    bool begin();

    // Main-loop hook: consume a pending DRDY event (SPI read + batch append),
    // flush full or stale batches to SerialUSB1.
    void poll();

    bool     ok() const          { return ok_; }
    uint32_t sample_count() const { return sample_count_; }
    // Samples lost to a slow main loop (ISR overruns) plus batches dropped
    // on a full USB buffer. Two single-writer counters — the ISR and the
    // main loop never RMW the same word.
    uint32_t drop_count() const   { return isr_drops_ + tx_drops_; }

private:
    void isr_drdy();
    static void isr_trampoline();
    static Bmi088Streamer* s_self_;

    void flush_batch();

    bool ok_ = false;

    // DRDY latch — written by the ISR, consumed by poll(). Latest-wins.
    volatile bool     drdy_pending_ = false;
    volatile uint64_t drdy_t_us_    = 0;

    // Batch under construction (raw ImuRecord bytes, see binary_proto.h).
    uint8_t  batch_buf_[1 + 4 * (8 + 6 * 4)];  // count + kImuBatchMax records
    int      batch_count_      = 0;
    uint64_t batch_oldest_us_  = 0;

    uint32_t          sample_count_ = 0;
    volatile uint32_t isr_drops_    = 0;   // ISR-only writer
    uint32_t          tx_drops_     = 0;   // main-loop-only writer
};

}  // namespace gw_fw
