#pragma once

#include <cstdint>

#include "Protocol.h"

namespace posest::firmware {

// Register-level driver for the STMicro VL53L4CD ToF sensor, implemented
// against the UM2931 application note (init blob, data-ready bit, and result
// registers). Single-shot ranging only: each measurement is started by an
// explicit call to startRanging() (driven from the main loop after the
// camera-trigger ISR schedules a deadline at flash_end + offset). Continuous
// mode is rejected by design — it would self-clock at the chip's
// intermeasurement period and could overlap an IR LED flash, breaking the
// temporal-multiplex guarantee.
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

    // VL53L4CD result frame — populated by poll() from registers 0x0089-0x009F
    // when a single-shot ranging completes.
    struct Result {
        std::uint8_t range_status{0};
        std::uint16_t distance_mm{0};
        std::uint16_t signal_rate_kcps{0};   // raw 9.7 fixed-point MCPS from chip
        std::uint16_t ambient_rate_kcps{0};  // raw 9.7 fixed-point MCPS from chip
        std::uint16_t sigma_mm{0};
        std::uint16_t spad_count{0};
    };

    bool begin(const Config& cfg, std::uint64_t now_us);
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

    // Test-only entry points. Production code never calls these. They flip a
    // runtime `test_mode_` flag that makes startRanging/poll bypass the I2C
    // bus, so the unit tests can drive the state machine without a chip
    // attached. Kept as unconditional methods (no #ifdef) so the same test
    // binary runs whether built on-device or natively.
    void testHookForceIdle(const Config& cfg) {
        cfg_ = cfg;
        state_ = State::kIdle;
        error_flags_ = 0;
        samples_emitted_ = 0;
        i2c_failures_ = 0;
        overruns_ = 0;
        ranging_start_us_ = 0;
        pending_trigger_sequence_ = 0;
        test_mode_ = true;
        test_pending_result_ = false;
    }
    void testHookInjectResult(std::uint16_t distance_mm,
                              std::uint8_t range_status,
                              std::uint16_t signal_rate_raw,
                              std::uint16_t ambient_rate_raw) {
        test_pending_result_ = true;
        test_result_.distance_mm = distance_mm;
        test_result_.range_status = range_status;
        test_result_.signal_rate_kcps = signal_rate_raw;
        test_result_.ambient_rate_kcps = ambient_rate_raw;
    }

private:
    enum class State : std::uint8_t {
        kIdle = 0,
        kRanging = 1,
        kInitFailed = 2,
    };

    // Low-level register I/O. 16-bit register pointer is sent big-endian on
    // the wire; payload bytes follow. Returns true on success.
    bool writeBytes(std::uint16_t reg, const std::uint8_t* data, std::size_t len);
    bool readBytes(std::uint16_t reg, std::uint8_t* out, std::size_t len);
    bool writeU8(std::uint16_t reg, std::uint8_t value);
    bool readU8(std::uint16_t reg, std::uint8_t& out);
    bool readU16(std::uint16_t reg, std::uint16_t& out);

    // High-level chip operations per UM2931.
    bool sensorInit();
    bool waitForBoot();
    bool startSingleShot();
    bool checkDataReady(bool& ready);
    bool readResult(Result& out);
    bool clearInterrupt();

    Config cfg_{};
    State state_{State::kIdle};
    std::uint32_t error_flags_{0};
    std::uint32_t samples_emitted_{0};
    std::uint32_t i2c_failures_{0};
    std::uint32_t overruns_{0};
    std::uint64_t ranging_start_us_{0};
    std::uint32_t pending_trigger_sequence_{0};

    bool test_mode_{false};
    bool test_pending_result_{false};
    Result test_result_{};
};

}  // namespace posest::firmware
