#pragma once

#include <cstdint>

#include "Protocol.h"

namespace posest::firmware {

// Minimal driver for the STMicro VL53L4CD ToF sensor.
//
// Single-shot ranging only: each measurement is started by an explicit call
// to startRanging() (driven from the main loop after the camera-trigger ISR
// schedules a deadline at flash_end + offset). Continuous mode is rejected
// by design — it would self-clock at the chip's intermeasurement period and
// could overlap an IR LED flash, breaking the temporal-multiplex guarantee.
//
// All I2C transactions live in the main loop. The Wire library uses
// interrupts internally, so calling Wire from any other ISR risks priority
// inversion / deadlock.
//
// State machine:
//   kIdle         no ranging in flight; ready to accept startRanging
//   kRanging      single-shot in progress; poll() returns true once result is ready
//   kInitFailed   begin() never succeeded — startRanging/poll are no-ops
class Vl53l4cdToF final {
public:
    struct Config {
        bool enabled{false};
        std::uint8_t i2c_address{0x29};
        std::uint16_t timing_budget_ms{10};
        std::uint16_t intermeasurement_period_ms{20};
    };

    // Initialize the chip. Returns true if the device responds with the
    // expected model id; on false the driver enters kInitFailed and the
    // host receives kErrorTofInitFailed in TeensyHealth.
    bool begin(const Config& cfg, std::uint64_t now_us);

    // Reconfigure timing without re-initializing. Returns true on success.
    bool reconfigure(const Config& cfg, std::uint64_t now_us);

    // Kick off a single-shot ranging. trigger_sequence is stashed and copied
    // into the eventual ToFSamplePayload so the host can match the result
    // against the camera trigger that drove it. Returns false if a previous
    // ranging is still in flight (caller should set kErrorTofRangingOverrun).
    bool startRanging(std::uint32_t trigger_sequence, std::uint64_t now_us);

    // Drains a completed ranging if available. Returns true and writes
    // out_payload if data is ready; false otherwise (still ranging, or
    // idle). The driver clears the chip's interrupt before returning.
    bool poll(ToFSamplePayload& out_payload, std::uint64_t now_us);

    bool isRanging() const { return state_ == State::kRanging; }
    std::uint32_t errorFlags() const { return error_flags_; }
    std::uint32_t samplesEmitted() const { return samples_emitted_; }
    std::uint32_t i2cFailures() const { return i2c_failures_; }
    std::uint32_t overruns() const { return overruns_; }

    // Called from main when the trigger ISR observed an overrun condition
    // (a new ToF deadline arrived while the previous ranging hadn't
    // finished). Bumps the overrun counter without touching the I2C bus.
    void noteOverrun() {
        ++overruns_;
        error_flags_ |= 1u;  // mirrored into kToFStatusRangingOverrun by main
    }

private:
    enum class State : std::uint8_t {
        kIdle = 0,
        kRanging = 1,
        kInitFailed = 2,
    };

    // Minimal I2C primitives. Returns true on success. On any failure, sets
    // kErrorTofI2cFailure on the next sample's status_flags via error_flags_.
    bool writeReg16(std::uint16_t reg, std::uint8_t value);
    bool writeReg16U16(std::uint16_t reg, std::uint16_t value);
    bool readReg16(std::uint16_t reg, std::uint8_t& out);
    bool readReg16U16(std::uint16_t reg, std::uint16_t& out);

    Config cfg_{};
    State state_{State::kIdle};
    std::uint32_t error_flags_{0};
    std::uint32_t samples_emitted_{0};
    std::uint32_t i2c_failures_{0};
    std::uint32_t overruns_{0};
    std::uint64_t ranging_start_us_{0};
    std::uint32_t pending_trigger_sequence_{0};
};

}  // namespace posest::firmware
