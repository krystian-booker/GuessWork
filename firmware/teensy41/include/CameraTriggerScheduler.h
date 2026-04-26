#pragma once

#include <cstddef>
#include <cstdint>

#include <IntervalTimer.h>

#include "FirmwareConfig.h"
#include "Protocol.h"

namespace posest::firmware {

constexpr std::size_t kCameraTriggerEventQueueCapacity = 32;

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

    static CameraTriggerScheduler* activeInstance() { return active_instance_; }

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

    static CameraTriggerScheduler* active_instance_;
};

}  // namespace posest::firmware
