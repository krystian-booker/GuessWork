#pragma once

#include <Arduino.h>

// TriggerEngine drives up to kMaxGroups independent trigger groups on a
// Teensy 4.1. Each group owns its own IntervalTimer and a small SPSC ring
// of pulse events (rising-edge timestamps + monotonic indices) that the
// main loop drains and serializes over USB-CDC.
//
// kMaxGroups is bounded by the number of IntervalTimer hardware slots on
// the Teensy 4.x platform (4). Two groups (apriltag + vio) are the typical
// host configuration; four leaves comfortable headroom.

namespace gw_fw {

constexpr int kMaxGroups       = 4;
constexpr int kMaxOutputs      = 6;
constexpr int kMaxGroupNameLen = 16;     // not counting NUL terminator
constexpr int kPulseRingSize   = 32;     // power of two; SPSC mask = size-1

// Physical Teensy pins used for outputs 1..6. Keep these contiguous and
// PWM-capable for future flexibility; today they're driven with
// digitalWriteFast and don't need a hardware PWM block.
constexpr uint8_t kOutputPins[kMaxOutputs] = {2, 3, 4, 5, 6, 7};

// Width of every rising-edge pulse. The FLIR opto-isolated Line0 input has
// a ~10 µs detection threshold; 100 µs is well above that and well below
// the shortest 60 Hz period (16.667 ms).
constexpr uint32_t kPulseWidthUs = 100;

struct PulseEvent {
    uint32_t idx;     // group-local rising-edge counter, starts at 1 after arm()
    uint32_t t_us;    // micros() captured at the rising edge
};

class TriggerEngine {
public:
    // Configure pin modes, drive everything LOW. Must be called once in setup().
    void begin();

    // Replace (or add) a group with the given parameters. Returns false if the
    // table is full or the inputs are invalid. Pin numbers are 1..6 (matching
    // the host wire labels), packed into pin_mask (bit n-1 = pin n).
    bool set_config(const char* name, float fps, uint8_t pin_mask, const char*& err);

    // Stop arms (if any) and wipe the group table.
    void clear_config();

    // Start every configured group's IntervalTimer. Returns false if no groups
    // are configured. Safe to call when already armed (no-op then).
    bool arm(const char*& err);

    // Stop every IntervalTimer and drive all outputs LOW.
    void stop();

    bool is_armed() const { return armed_; }
    int  group_count() const { return n_groups_; }

    const char* group_name(int i) const     { return groups_[i].name; }
    float       group_fps(int i) const      { return groups_[i].fps; }
    uint8_t     group_pin_mask(int i) const { return groups_[i].pin_mask; }

    // Pop one pulse event from group `i`'s ring. Returns false when empty.
    // Safe to call from the main loop while ISRs continue to push.
    bool drain_event(int i, PulseEvent& out);

private:
    struct Group {
        char           name[kMaxGroupNameLen + 1] = {};
        float          fps      = 0.0f;
        uint8_t        pin_mask = 0;          // bit n-1 set iff pin n is in this group
        IntervalTimer  timer;
        volatile uint32_t pulse_idx = 0;       // monotonic, increments on rising edge
        volatile uint32_t head     = 0;        // ISR-side write index
        volatile uint32_t tail     = 0;        // main-loop read index
        PulseEvent        ring[kPulseRingSize] = {};
    };

    Group groups_[kMaxGroups];
    int   n_groups_ = 0;
    bool  armed_    = false;

    // IntervalTimer dispatch. C-style trampolines because IntervalTimer's
    // callback is a void(*)() with no user context.
    static TriggerEngine* s_self_;
    static void isr_0(); static void isr_1(); static void isr_2(); static void isr_3();
    static_assert(kMaxGroups == 4, "ISR trampoline table must match kMaxGroups");
    void on_group_fire(int i);

    int find_group_by_name(const char* name) const;
};

}  // namespace gw_fw
