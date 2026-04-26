#include "CameraTriggerScheduler.h"

#include <Arduino.h>

#include <algorithm>

namespace posest::firmware {

CameraTriggerScheduler* CameraTriggerScheduler::active_instance_ = nullptr;

void CameraTriggerScheduler::begin() {
    for (std::uint8_t pin : kCameraSyncPins) {
        pinMode(pin, OUTPUT);
        digitalWriteFast(pin, LOW);
    }
    time64_low_prev_ = micros();
    time64_high_ = 0;
    active_instance_ = this;
}

void CameraTriggerScheduler::apply(
    const CameraTriggerCommand* commands,
    std::size_t count,
    std::uint32_t now_us) {
    (void)now_us;
    disableAll();
    status_flags_ = 0;

    if (!commands) {
        status_flags_ |= kTriggerUnsupportedCount;
        return;
    }
    if (count > kMaxCameraSyncOutputs) {
        status_flags_ |= kTriggerUnsupportedCount;
        return;
    }

    bool used_pins[kMaxCameraSyncOutputs]{};
    std::uint32_t requested_periods[kMaxCameraSyncOutputs]{};
    std::uint32_t requested_pulses[kMaxCameraSyncOutputs]{};
    std::int64_t requested_phases[kMaxCameraSyncOutputs]{};
    std::uint8_t requested_pins[kMaxCameraSyncOutputs]{};
    std::size_t valid_count = 0;

    for (std::size_t i = 0; i < count; ++i) {
        const CameraTriggerCommand& command = commands[i];
        if (!command.enabled) {
            continue;
        }
        if (!isAllowedPin(command.pin)) {
            status_flags_ |= kTriggerInvalidPin;
            continue;
        }
        const std::size_t pin_index = static_cast<std::size_t>(
            command.pin - static_cast<std::int32_t>(kCameraSyncPins[0]));
        if (pin_index >= kMaxCameraSyncOutputs || used_pins[pin_index]) {
            status_flags_ |= kTriggerDuplicatePin;
            continue;
        }
        const std::uint32_t period_us = periodFromRate(command.rate_hz);
        if (period_us == 0u) {
            status_flags_ |= kTriggerInvalidRate;
            continue;
        }
        if (command.pulse_width_us == 0u || command.pulse_width_us >= period_us) {
            status_flags_ |= kTriggerPulseTooWide;
            continue;
        }

        used_pins[pin_index] = true;
        requested_pins[valid_count] = static_cast<std::uint8_t>(command.pin);
        requested_periods[valid_count] = period_us;
        requested_pulses[valid_count] = command.pulse_width_us;
        requested_phases[valid_count] = command.phase_offset_us;
        ++valid_count;
    }

    if (valid_count == 0u) {
        return;
    }

    std::uint32_t master_period_us = requested_periods[0];
    for (std::size_t i = 1; i < valid_count; ++i) {
        master_period_us = gcdU32(master_period_us, requested_periods[i]);
    }
    if (master_period_us == 0u) {
        status_flags_ |= kTriggerInvalidRate;
        return;
    }

    master_period_us_ = master_period_us;
    master_tick_count_ = 0;

    for (std::size_t i = 0; i < valid_count; ++i) {
        const std::uint32_t period_ticks = requested_periods[i] / master_period_us;
        if (period_ticks == 0u) {
            status_flags_ |= kTriggerInvalidRate;
            continue;
        }
        std::uint32_t pulse_ticks = requested_pulses[i] / master_period_us;
        if (pulse_ticks == 0u) {
            pulse_ticks = 1u;
        }
        if (pulse_ticks >= period_ticks) {
            status_flags_ |= kTriggerPulseTooWide;
            continue;
        }

        const std::int64_t phase_us = requested_phases[i];
        std::int64_t phase_ticks_signed =
            phase_us / static_cast<std::int64_t>(master_period_us);
        const std::int64_t period_ticks_signed = static_cast<std::int64_t>(period_ticks);
        phase_ticks_signed %= period_ticks_signed;
        if (phase_ticks_signed < 0) {
            phase_ticks_signed += period_ticks_signed;
        }
        const std::uint32_t phase_ticks = static_cast<std::uint32_t>(phase_ticks_signed);

        Channel& channel = channels_[i];
        channel.enabled = true;
        channel.high = false;
        channel.pin = requested_pins[i];
        channel.period_us = period_ticks * master_period_us;
        channel.pulse_width_us = pulse_ticks * master_period_us;
        channel.period_ticks = period_ticks;
        channel.pulse_ticks = pulse_ticks;
        channel.next_rise_tick = phase_ticks;
        channel.next_fall_tick = 0;
        pinMode(channel.pin, OUTPUT);
        digitalWriteFast(channel.pin, LOW);
    }

    if (interval_timer_.begin(tickThunk, master_period_us_)) {
        timer_running_ = true;
    } else {
        status_flags_ |= kTriggerOverrun;
        timer_running_ = false;
    }
}

void CameraTriggerScheduler::update(std::uint32_t now_us) {
    (void)now_us;
}

bool CameraTriggerScheduler::popEvent(CameraTriggerEventPayload& event) {
    std::uint32_t raw_time_us = 0;
    noInterrupts();
    if (event_count_ == 0u) {
        interrupts();
        return false;
    }
    event = events_[event_head_].event;
    raw_time_us = events_[event_head_].raw_time_us;
    event_head_ = (event_head_ + 1u) % kCameraTriggerEventQueueCapacity;
    --event_count_;
    interrupts();
    event.teensy_time_us = expandTimestamp(raw_time_us);
    return true;
}

std::size_t CameraTriggerScheduler::effectiveEntries(
    TriggerAckEntry* out, std::size_t capacity) const {
    if (!out) {
        return 0;
    }
    std::size_t count = 0;
    for (const Channel& channel : channels_) {
        if (!channel.enabled) {
            continue;
        }
        if (count >= capacity) {
            break;
        }
        TriggerAckEntry& entry = out[count++];
        entry.pin = static_cast<std::int32_t>(channel.pin);
        entry.period_us = channel.period_us;
        entry.pulse_us = channel.pulse_width_us;
    }
    return count;
}

void CameraTriggerScheduler::disableAll() {
    if (timer_running_) {
        interval_timer_.end();
        timer_running_ = false;
    }
    for (Channel& channel : channels_) {
        if (channel.enabled || channel.high) {
            digitalWriteFast(channel.pin, LOW);
        }
        const std::uint32_t saved_sequence = channel.trigger_sequence;
        channel = Channel{};
        channel.trigger_sequence = saved_sequence;
    }
    for (std::uint8_t pin : kCameraSyncPins) {
        digitalWriteFast(pin, LOW);
    }
    master_period_us_ = 0;
    master_tick_count_ = 0;
    event_head_ = 0;
    event_count_ = 0;
}

void CameraTriggerScheduler::tickIsr() {
    const std::uint32_t tick = ++master_tick_count_;
    const std::uint32_t event_us = micros();
    for (Channel& channel : channels_) {
        if (!channel.enabled) {
            continue;
        }
        if (channel.high && dueTick(tick, channel.next_fall_tick)) {
            digitalWriteFast(channel.pin, LOW);
            channel.high = false;
        }
        if (!channel.high && dueTick(tick, channel.next_rise_tick)) {
            digitalWriteFast(channel.pin, HIGH);
            channel.high = true;
            channel.next_fall_tick = tick + channel.pulse_ticks;
            ++channel.trigger_sequence;
            enqueueEventFromIsr(channel, event_us);
            channel.next_rise_tick = tick + channel.period_ticks;
            while (dueTick(tick, channel.next_rise_tick)) {
                status_flags_ |= kTriggerOverrun;
                channel.next_rise_tick += channel.period_ticks;
            }
        }
    }
}

void CameraTriggerScheduler::tickThunk() {
    if (active_instance_) {
        active_instance_->tickIsr();
    }
}

void CameraTriggerScheduler::enqueueEventFromIsr(
    const Channel& channel,
    std::uint32_t event_time_us) {
    if (event_count_ >= kCameraTriggerEventQueueCapacity) {
        status_flags_ |= kTriggerOverrun;
        event_head_ = (event_head_ + 1u) % kCameraTriggerEventQueueCapacity;
        --event_count_;
    }
    const std::size_t tail =
        (event_head_ + event_count_) % kCameraTriggerEventQueueCapacity;
    events_[tail].raw_time_us = event_time_us;
    events_[tail].event.teensy_time_us = 0;
    events_[tail].event.pin = static_cast<std::int32_t>(channel.pin);
    events_[tail].event.trigger_sequence = channel.trigger_sequence;
    events_[tail].event.status_flags = status_flags_;
    ++event_count_;
}

std::uint64_t CameraTriggerScheduler::expandTimestamp(std::uint32_t timestamp32) {
    // Mirrors Bmi088Imu::expandTimestamp. Sample the 32-bit clock fresh so
    // the high-half anchor never reflects a stale reference; bump high every
    // time the low half wraps. Runs only from the main-loop drain.
    const std::uint32_t now_low = micros();
    if (now_low < time64_low_prev_) {
        ++time64_high_;
    }
    time64_low_prev_ = now_low;

    std::uint32_t high = time64_high_;
    if (timestamp32 > now_low) {
        if (high == 0u) {
            return static_cast<std::uint64_t>(timestamp32);
        }
        --high;
    }
    return (static_cast<std::uint64_t>(high) << 32u) | timestamp32;
}

bool CameraTriggerScheduler::isAllowedPin(std::int32_t pin) {
    for (std::uint8_t allowed : kCameraSyncPins) {
        if (pin == static_cast<std::int32_t>(allowed)) {
            return true;
        }
    }
    return false;
}

bool CameraTriggerScheduler::dueTick(std::uint32_t now_tick, std::uint32_t target_tick) {
    return static_cast<std::int32_t>(now_tick - target_tick) >= 0;
}

std::uint32_t CameraTriggerScheduler::periodFromRate(double rate_hz) {
    if (!(rate_hz > 0.0)) {
        return 0;
    }
    const double period = 1000000.0 / rate_hz;
    if (period < 2.0 || period > 4294967295.0) {
        return 0;
    }
    return static_cast<std::uint32_t>(period + 0.5);
}

std::uint32_t CameraTriggerScheduler::gcdU32(std::uint32_t a, std::uint32_t b) {
    while (b != 0u) {
        const std::uint32_t t = b;
        b = a % b;
        a = t;
    }
    return a;
}

}  // namespace posest::firmware
