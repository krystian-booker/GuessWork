#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "posest/MeasurementTypes.h"
#include "posest/Timestamp.h"

namespace posest {

struct TriggerStamp {
    std::uint64_t teensy_time_us{0};
    std::uint32_t trigger_sequence{0};
    Timestamp host_time{};
};

// Thread-safe per-camera ring of recent CameraTriggerEvent stamps. Lets
// camera producers stitch a Teensy-stamped shutter pulse onto each captured
// frame without competing with FusionService for the single MeasurementBus
// consumer slot.
class CameraTriggerCache {
public:
    explicit CameraTriggerCache(
        std::unordered_map<std::int32_t, std::string> pin_to_camera,
        std::chrono::milliseconds match_window = std::chrono::milliseconds(50),
        std::size_t per_camera_capacity = 32);

    // Called by TeensyService for every decoded CameraTriggerEvent. Events on
    // pins that aren't mapped to a known camera, and events whose status_flags
    // include kStatusUnsynchronizedTime, are silently skipped — without a
    // valid host↔teensy time sync the event's host_time can drift past the
    // match_window and produce wrong stamps.
    void recordEvent(const CameraTriggerEvent& ev);

    // Producer-side query: returns the most recent stamp whose host_time is
    // <= capture_time and within match_window of it; nullopt otherwise.
    std::optional<TriggerStamp> lookup(
        const std::string& camera_id, Timestamp capture_time) const;

    void clear();

private:
    mutable std::mutex mu_;
    std::unordered_map<std::string, std::deque<TriggerStamp>> per_camera_;
    std::unordered_map<std::int32_t, std::string> pin_to_camera_;
    std::chrono::milliseconds match_window_;
    std::size_t per_camera_capacity_;
};

}  // namespace posest
