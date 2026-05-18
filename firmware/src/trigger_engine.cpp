#include "trigger_engine.h"

#include <string.h>

namespace gw_fw {

TriggerEngine* TriggerEngine::s_self_ = nullptr;

void TriggerEngine::isr_0() { if (s_self_) s_self_->on_group_fire(0); }
void TriggerEngine::isr_1() { if (s_self_) s_self_->on_group_fire(1); }
void TriggerEngine::isr_2() { if (s_self_) s_self_->on_group_fire(2); }
void TriggerEngine::isr_3() { if (s_self_) s_self_->on_group_fire(3); }

void TriggerEngine::begin() {
    s_self_ = this;
    for (int i = 0; i < kMaxOutputs; ++i) {
        pinMode(kOutputPins[i], OUTPUT);
        digitalWriteFast(kOutputPins[i], LOW);
    }
}

int TriggerEngine::find_group_by_name(const char* name) const {
    for (int i = 0; i < n_groups_; ++i) {
        if (strncmp(groups_[i].name, name, kMaxGroupNameLen + 1) == 0) return i;
    }
    return -1;
}

bool TriggerEngine::set_config(const char* name, float fps, uint8_t pin_mask,
                               const char*& err) {
    if (!name || !name[0]) { err = "missing name";          return false; }
    if (strlen(name) > kMaxGroupNameLen) { err = "name too long"; return false; }
    if (!(fps > 0.0f) || fps > 10000.0f) { err = "fps out of range"; return false; }
    if (pin_mask == 0 || (pin_mask & ~0x3F)) { err = "pin_mask invalid"; return false; }

    // Updating an armed group is unsafe (the IntervalTimer is running). Force
    // the host to STOP first.
    if (armed_) { err = "stop before reconfiguring"; return false; }

    int idx = find_group_by_name(name);
    if (idx < 0) {
        if (n_groups_ >= kMaxGroups) { err = "group table full"; return false; }
        idx = n_groups_++;
    }
    Group& g = groups_[idx];
    strncpy(g.name, name, kMaxGroupNameLen);
    g.name[kMaxGroupNameLen] = '\0';
    g.fps       = fps;
    g.pin_mask  = pin_mask;
    g.pulse_idx = 0;
    g.head      = g.tail = 0;
    return true;
}

void TriggerEngine::clear_config() {
    stop();
    for (int i = 0; i < n_groups_; ++i) {
        groups_[i].name[0]  = '\0';
        groups_[i].fps      = 0.0f;
        groups_[i].pin_mask = 0;
        groups_[i].pulse_idx = 0;
        groups_[i].head = groups_[i].tail = 0;
    }
    n_groups_ = 0;
}

bool TriggerEngine::arm(const char*& err) {
    if (n_groups_ == 0) { err = "no groups configured"; return false; }
    if (armed_) return true;

    static constexpr void (*kTrampolines[kMaxGroups])() = {
        &TriggerEngine::isr_0, &TriggerEngine::isr_1,
        &TriggerEngine::isr_2, &TriggerEngine::isr_3,
    };

    for (int i = 0; i < n_groups_; ++i) {
        Group& g = groups_[i];
        g.pulse_idx = 0;
        g.head = g.tail = 0;
        const uint32_t period_us = static_cast<uint32_t>(1'000'000.0f / g.fps + 0.5f);
        if (!g.timer.begin(kTrampolines[i], period_us)) {
            err = "IntervalTimer::begin failed";
            // Roll back any timers we already started.
            for (int j = 0; j < i; ++j) groups_[j].timer.end();
            return false;
        }
    }
    armed_ = true;
    return true;
}

void TriggerEngine::stop() {
    for (int i = 0; i < n_groups_; ++i) groups_[i].timer.end();
    for (int i = 0; i < kMaxOutputs; ++i) digitalWriteFast(kOutputPins[i], LOW);
    armed_ = false;
}

void TriggerEngine::on_group_fire(int i) {
    Group& g = groups_[i];

    // Latch the timestamp BEFORE driving the pins so we report what the
    // host actually saw arrive at the camera, not the time after the spin.
    const uint32_t t_us = micros();
    const uint32_t idx  = ++g.pulse_idx;

    // Rising edge — drive every pin in the mask HIGH simultaneously, hold
    // for kPulseWidthUs, then back to LOW. delayMicroseconds() is a busy
    // spin; 100 µs at 600 MHz is ~60k cycles, comfortable inside an ISR.
    const uint8_t mask = g.pin_mask;
    for (int p = 0; p < kMaxOutputs; ++p) {
        if (mask & (1u << p)) digitalWriteFast(kOutputPins[p], HIGH);
    }
    delayMicroseconds(kPulseWidthUs);
    for (int p = 0; p < kMaxOutputs; ++p) {
        if (mask & (1u << p)) digitalWriteFast(kOutputPins[p], LOW);
    }

    // SPSC enqueue. If the host is too slow to drain the ring we drop the
    // event silently (host will see a gap in `idx`, which is the same signal
    // a missed-trigger would produce on the camera side).
    const uint32_t head = g.head;
    const uint32_t next = (head + 1) & (kPulseRingSize - 1);
    if (next != g.tail) {
        g.ring[head] = PulseEvent{idx, t_us};
        g.head       = next;
    }
}

bool TriggerEngine::drain_event(int i, PulseEvent& out) {
    if (i < 0 || i >= n_groups_) return false;
    Group& g = groups_[i];
    const uint32_t tail = g.tail;
    if (tail == g.head) return false;
    out      = g.ring[tail];
    g.tail   = (tail + 1) & (kPulseRingSize - 1);
    return true;
}

}  // namespace gw_fw
