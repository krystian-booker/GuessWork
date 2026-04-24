#include "CameraTriggerScheduler.h"

#include <Arduino.h>

namespace posest::firmware {

void CameraTriggerScheduler::begin() {
    for (std::uint8_t pin : kCameraSyncPins) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
}

void CameraTriggerScheduler::apply(
    const CameraTriggerCommand* commands,
    std::size_t count,
    std::uint32_t now_us) {
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
        used_pins[pin_index] = true;

        const std::uint32_t period_us = periodFromRate(command.rate_hz);
        if (period_us == 0u) {
            status_flags_ |= kTriggerInvalidRate;
            continue;
        }
        if (command.pulse_width_us == 0u || command.pulse_width_us >= period_us) {
            status_flags_ |= kTriggerPulseTooWide;
            continue;
        }

        Channel& channel = channels_[i];
        channel.enabled = true;
        channel.high = false;
        channel.pin = static_cast<std::uint8_t>(command.pin);
        channel.period_us = period_us;
        channel.pulse_width_us = command.pulse_width_us;
        channel.next_rise_us = now_us + normalizePhase(command.phase_offset_us, period_us);
        channel.fall_us = channel.next_rise_us + command.pulse_width_us;
        pinMode(channel.pin, OUTPUT);
        digitalWrite(channel.pin, LOW);
    }
}

void CameraTriggerScheduler::update(std::uint32_t now_us) {
    for (Channel& channel : channels_) {
        if (!channel.enabled) {
            continue;
        }

        if (channel.high && due(now_us, channel.fall_us)) {
            digitalWrite(channel.pin, LOW);
            channel.high = false;
            channel.next_rise_us += channel.period_us;
            while (due(now_us, channel.next_rise_us)) {
                status_flags_ |= kTriggerOverrun;
                channel.next_rise_us += channel.period_us;
            }
        }

        if (!channel.high && due(now_us, channel.next_rise_us)) {
            digitalWrite(channel.pin, HIGH);
            channel.high = true;
            channel.fall_us = channel.next_rise_us + channel.pulse_width_us;
            if (due(now_us, channel.fall_us)) {
                status_flags_ |= kTriggerOverrun;
                digitalWrite(channel.pin, LOW);
                channel.high = false;
                channel.next_rise_us += channel.period_us;
            }
        }
    }
}

void CameraTriggerScheduler::disableAll() {
    for (Channel& channel : channels_) {
        if (channel.enabled || channel.high) {
            digitalWrite(channel.pin, LOW);
        }
        channel = Channel{};
    }
    for (std::uint8_t pin : kCameraSyncPins) {
        digitalWrite(pin, LOW);
    }
}

bool CameraTriggerScheduler::isAllowedPin(std::int32_t pin) {
    for (std::uint8_t allowed : kCameraSyncPins) {
        if (pin == static_cast<std::int32_t>(allowed)) {
            return true;
        }
    }
    return false;
}

bool CameraTriggerScheduler::due(std::uint32_t now_us, std::uint32_t target_us) {
    return static_cast<std::int32_t>(now_us - target_us) >= 0;
}

std::uint32_t CameraTriggerScheduler::normalizePhase(
    std::int64_t phase_offset_us,
    std::uint32_t period_us) {
    if (period_us == 0u) {
        return 0;
    }
    std::int64_t phase = phase_offset_us % static_cast<std::int64_t>(period_us);
    if (phase < 0) {
        phase += static_cast<std::int64_t>(period_us);
    }
    return static_cast<std::uint32_t>(phase);
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

}  // namespace posest::firmware
