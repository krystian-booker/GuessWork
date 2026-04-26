#pragma once

#include <cstddef>
#include <cstdint>

#include <IntervalTimer.h>

#include "FirmwareConfig.h"
#include "Protocol.h"

namespace posest::firmware {

constexpr std::size_t kCameraTriggerEventQueueCapacity = 32;

// Single-VIO-camera companion state (IR LED gate + ToF deadline).
struct VioCompanionConfig {
    bool led_enabled{false};
    std::uint8_t vio_slot_index{0};
    std::uint32_t led_pulse_width_us{400};
    bool tof_enabled{false};
    std::uint32_t tof_offset_after_flash_us{500};
    std::uint32_t tof_divisor{1};
};

class CameraTriggerScheduler final {
public:
    void begin();
    void apply(
        const CameraTriggerCommand* commands,
        std::size_t count,
        std::uint32_t now_us);
    // Software-polled fallback. With the IntervalTimer running this is a no-op;
    // it remains so main.cpp can keep its existing call site without harm.
    void update(std::uint32_t now_us);
    void disableAll();

    std::uint32_t statusFlags() const { return status_flags_; }
    bool popEvent(CameraTriggerEventPayload& event);
    std::size_t effectiveEntries(TriggerAckEntry* out, std::size_t capacity) const;
    std::uint32_t masterPeriodUs() const { return master_period_us_; }

    // Configure (or disable) the IR LED + ToF deadline companion. Validates
    // vio_slot_index and led_pulse_width_us > 0 when led_enabled. Returns
    // status flags (kConfigAck* values) — 0 == accepted.
    std::uint32_t applyVioCompanion(const VioCompanionConfig& cfg);
    VioCompanionConfig effectiveVioCompanion() const { return vio_companion_; }

    // Main-loop accessor: if the trigger ISR scheduled a ToF "start ranging"
    // deadline that has now passed, returns true and writes the trigger
    // sequence captured at the originating rise into *out_seq. Clears the
    // pending flag. The caller (main loop) is responsible for actually
    // initiating I2C; I2C never runs from any ISR here.
    bool consumeToFStartDue(std::uint32_t now_us, std::uint32_t& out_seq);

    static CameraTriggerScheduler* activeInstance() { return active_instance_; }

    // Test-only entry points. Production code never calls these. They expose
    // private ISR methods and internal state so unit tests can drive the
    // VIO companion path without an IntervalTimer running. Mirrors
    // CanBridge::testHookFeedRioTimeSync.
    void testHookSimulateVioRise(std::uint32_t event_time_us) {
        // Mirror the bookkeeping that tickIsr does on a real rise: bump the
        // VIO slot's trigger_sequence and stash it for the ToF join, then
        // run the LED+ToF deadline scheduler.
        const std::size_t i = static_cast<std::size_t>(vio_companion_.vio_slot_index);
        ++channels_[i].trigger_sequence;
        tof_pending_trigger_sequence_ = channels_[i].trigger_sequence;
        onVioRiseFromIsr(event_time_us);
    }
    void testHookLedFallIsr() { ledFallIsr(); }
    bool testHookLedFallPending() const { return led_fall_pending_; }
    bool testHookTofStartPending() const { return tof_start_pending_; }
    std::uint32_t testHookTofStartDeadline() const { return tof_start_deadline_us_; }
    std::uint32_t testHookTofPendingTriggerSequence() const {
        return tof_pending_trigger_sequence_;
    }
    static std::uint32_t testHookGcdU32(std::uint32_t a, std::uint32_t b) {
        return gcdU32(a, b);
    }
    static std::uint32_t testHookPeriodFromRate(double rate_hz) {
        return periodFromRate(rate_hz);
    }

private:
    struct Channel {
        bool enabled{false};
        bool high{false};
        std::uint8_t pin{0};
        std::uint32_t period_us{0};
        std::uint32_t pulse_width_us{0};
        std::uint32_t period_ticks{0};
        std::uint32_t pulse_ticks{0};
        std::uint32_t next_rise_tick{0};
        std::uint32_t next_fall_tick{0};
        volatile std::uint32_t trigger_sequence{0};
    };

    struct EventQueueEntry {
        CameraTriggerEventPayload event;
        std::uint32_t raw_time_us{0};
    };

    void tickIsr();
    static void tickThunk();
    void enqueueEventFromIsr(
        const Channel& channel,
        std::uint32_t event_time_us);
    // ISR helpers for the VIO companion. Both run inside tickIsr when the
    // VIO-slot channel has just risen.
    void onVioRiseFromIsr(std::uint32_t event_time_us);
    static void ledFallThunk();
    void ledFallIsr();

    // Expand the 32-bit ISR-captured timestamp to 64 bits. Mirrors the IMU
    // pattern in Bmi088Imu::expandTimestamp — runs only from the main-loop
    // drain (popEvent), never from an ISR. Required so trigger timestamps
    // do not silently wrap every ~71 minutes.
    std::uint64_t expandTimestamp(std::uint32_t timestamp32);

    static bool isAllowedPin(std::int32_t pin);
    static bool dueTick(std::uint32_t now_tick, std::uint32_t target_tick);
    static std::uint32_t periodFromRate(double rate_hz);
    static std::uint32_t gcdU32(std::uint32_t a, std::uint32_t b);

    Channel channels_[kMaxCameraSyncOutputs]{};
    EventQueueEntry events_[kCameraTriggerEventQueueCapacity]{};
    volatile std::size_t event_head_{0};
    volatile std::size_t event_count_{0};
    std::uint32_t status_flags_{0};
    std::uint32_t master_period_us_{0};
    volatile std::uint32_t master_tick_count_{0};
    std::uint32_t time64_high_{0};
    std::uint32_t time64_low_prev_{0};
    bool timer_running_{false};
    IntervalTimer interval_timer_{};

    VioCompanionConfig vio_companion_{};
    IntervalTimer led_fall_timer_{};
    volatile bool led_fall_pending_{false};
    // VIO pulse counter — used with vio_companion_.tof_divisor to subsample
    // ToF deadlines (e.g. divisor=2 → ranging on every other camera frame).
    volatile std::uint32_t vio_pulse_count_{0};
    volatile bool tof_start_pending_{false};
    volatile std::uint32_t tof_start_deadline_us_{0};
    volatile std::uint32_t tof_pending_trigger_sequence_{0};

    static CameraTriggerScheduler* active_instance_;
};

}  // namespace posest::firmware
