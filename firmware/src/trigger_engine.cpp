#include "trigger_engine.hpp"

#include <Arduino.h>

#include "board.hpp"
#include "timebase.hpp"

namespace gw_fw {
namespace {

constexpr uint32_t kPulseWidthUs = 100;
constexpr size_t kQueueSize = 64;

struct Group {
    bool configured = false;
    bool pulse_high = false;
    uint8_t slot = 0;
    uint8_t pin_mask = 0;
    uint32_t period_us = 0;
    uint32_t remainder = 0;
    uint32_t remainder_accum = 0;
    uint32_t rate_millihz = 0;
    uint32_t next_start = 0;
    uint32_t pulse_end = 0;
    uint32_t index = 0;
};

Group g_groups[gw_sync::kMaxGroups];
volatile bool g_armed = false;
volatile TriggerQueueEvent g_queue[kQueueSize];
volatile uint8_t g_head = 0;
volatile uint8_t g_tail = 0;
volatile uint32_t g_drops = 0;

bool due(uint32_t now, uint32_t deadline) {
    return static_cast<int32_t>(now - deadline) >= 0;
}

void enqueue(uint8_t slot, uint32_t index, uint64_t t_us) {
    const uint8_t next = static_cast<uint8_t>((g_head + 1u) % kQueueSize);
    if (next == g_tail) {
        ++g_drops;
        return;
    }
    g_queue[g_head].slot = slot;
    g_queue[g_head].index = index;
    g_queue[g_head].t_us = t_us;
    __DMB();
    g_head = next;
}

void advance(Group& group) {
    group.next_start += group.period_us;
    group.remainder_accum += group.remainder;
    if (group.remainder_accum >= group.rate_millihz) {
        group.remainder_accum -= group.rate_millihz;
        ++group.next_start;
    }
}

void schedule_next(uint32_t now) {
    if (!g_armed) {
        timebase::disable_compare();
        return;
    }
    uint32_t best = now + 0x7FFFFFFFu;
    int32_t best_delta = 0x7FFFFFFF;
    for (const auto& group : g_groups) {
        if (!group.configured) continue;
        const uint32_t deadline = group.pulse_high ? group.pulse_end
                                                   : group.next_start;
        int32_t delta = static_cast<int32_t>(deadline - now);
        if (delta < 2) delta = 2;
        if (delta < best_delta) {
            best_delta = delta;
            best = now + static_cast<uint32_t>(delta);
        }
    }
    timebase::set_compare(best);
}

}  // namespace

void trigger_begin() {
    board::initialise_outputs_idle();
    timebase::begin();
}

gw_sync::AckStatus trigger_set_config(const gw_sync::GroupConfig* groups,
                                      uint8_t count) {
    if (count > gw_sync::kMaxGroups || (count && !groups)) {
        return gw_sync::AckStatus::BadConfig;
    }
    Group candidate[gw_sync::kMaxGroups]{};
    uint8_t used_pins = 0;
    uint8_t used_slots = 0;
    for (uint8_t i = 0; i < count; ++i) {
        const auto& wire = groups[i];
        if (wire.slot >= gw_sync::kMaxGroups ||
            (used_slots & (1u << wire.slot)) || wire.pin_mask == 0 ||
            (wire.pin_mask & ~0x3Fu) || (wire.pin_mask & used_pins) ||
            wire.rate_millihz == 0 ||
            wire.rate_millihz > gw_sync::kMaxRateMilliHz) {
            return gw_sync::AckStatus::BadConfig;
        }
        used_slots |= static_cast<uint8_t>(1u << wire.slot);
        used_pins |= wire.pin_mask;
        Group& group = candidate[wire.slot];
        group.configured = true;
        group.slot = wire.slot;
        group.pin_mask = wire.pin_mask;
        group.rate_millihz = wire.rate_millihz;
        group.period_us = 1'000'000'000u / wire.rate_millihz;
        group.remainder = 1'000'000'000u % wire.rate_millihz;
        if (group.period_us <= kPulseWidthUs) return gw_sync::AckStatus::BadConfig;
    }

    noInterrupts();
    g_armed = false;
    timebase::disable_compare();
    board::set_output_mask_low(0x3F);
    for (size_t i = 0; i < gw_sync::kMaxGroups; ++i) g_groups[i] = candidate[i];
    interrupts();
    return gw_sync::AckStatus::Ok;
}

gw_sync::AckStatus trigger_arm() {
    // A lost ACK may make the host repeat ARM. Preserve the existing phase and
    // event indices so that retrying the command cannot create a trigger gap.
    if (g_armed) return gw_sync::AckStatus::Ok;
    bool any = false;
    const uint32_t now = timebase::now_us32();
    noInterrupts();
    for (auto& group : g_groups) {
        if (!group.configured) continue;
        any = true;
        group.pulse_high = false;
        group.remainder_accum = 0;
        group.index = 0;
        group.next_start = now + group.period_us;
    }
    if (any) {
        g_armed = true;
        schedule_next(now);
    }
    interrupts();
    return any ? gw_sync::AckStatus::Ok : gw_sync::AckStatus::NoConfig;
}

void trigger_stop() {
    noInterrupts();
    g_armed = false;
    timebase::disable_compare();
    board::set_output_mask_low(0x3F);
    for (auto& group : g_groups) group.pulse_high = false;
    interrupts();
}

gw_sync::AckStatus trigger_test_output(uint8_t logical_output) {
    if (logical_output < 1 || logical_output > gw_sync::kOutputCount || g_armed) {
        return logical_output < 1 || logical_output > gw_sync::kOutputCount
            ? gw_sync::AckStatus::BadConfig : gw_sync::AckStatus::Busy;
    }
    const uint8_t mask = static_cast<uint8_t>(1u << (logical_output - 1));
    board::set_output_mask_high(mask);
    delayMicroseconds(kPulseWidthUs);
    board::set_output_mask_low(mask);
    return gw_sync::AckStatus::Ok;
}

bool trigger_pop(TriggerQueueEvent& event) {
    if (g_tail == g_head) return false;
    event.slot = g_queue[g_tail].slot;
    event.index = g_queue[g_tail].index;
    event.t_us = g_queue[g_tail].t_us;
    __DMB();
    g_tail = static_cast<uint8_t>((g_tail + 1u) % kQueueSize);
    return true;
}

bool trigger_is_armed() { return g_armed; }
uint32_t trigger_drop_count() { return g_drops; }

void trigger_timer_isr() {
    const uint32_t now = timebase::now_us32();
    const uint64_t now64 = timebase::now_us();
    for (auto& group : g_groups) {
        if (!group.configured) continue;
        if (group.pulse_high && due(now, group.pulse_end)) {
            board::set_output_mask_low(group.pin_mask);
            group.pulse_high = false;
        }
        if (!group.pulse_high && due(now, group.next_start)) {
            board::set_output_mask_high(group.pin_mask);
            group.pulse_high = true;
            group.pulse_end = now + kPulseWidthUs;
            enqueue(group.slot, group.index++, now64);
            do {
                advance(group);
            } while (due(now, group.next_start));
        }
    }
    schedule_next(now);
}

}  // namespace gw_fw
