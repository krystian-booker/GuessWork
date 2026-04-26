#pragma once

#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>

#include "posest/MeasurementTypes.h"

namespace posest {

// Thread-safe ring of recent ToF samples for the single VIO camera. Producers
// look up samples by trigger_sequence — the firmware tags each VL53L4CD
// ranging with the sequence number of the camera trigger that immediately
// preceded it, so the join is exact rather than time-window-fuzzy.
class ToFSampleCache {
public:
    explicit ToFSampleCache(std::string vio_camera_id, std::size_t capacity = 32);

    // Called by TeensyService for each decoded ToFSample. Samples carrying
    // kStatusUnsynchronizedTime still get cached — the host_time is unreliable
    // but trigger_sequence is sufficient for the lookup contract.
    void recordSample(const ToFSample& sample);

    // Returns the cached sample whose trigger_sequence equals the requested
    // sequence and which was recorded for the VIO camera; nullopt otherwise.
    std::optional<ToFSample> lookupBySequence(
        const std::string& camera_id,
        std::uint32_t trigger_sequence) const;

    // Clears the ring; primarily for tests and on stop/restart.
    void clear();

    const std::string& vioCameraId() const { return vio_camera_id_; }

private:
    mutable std::mutex mu_;
    std::string vio_camera_id_;
    std::deque<ToFSample> ring_;
    std::size_t capacity_;
};

}  // namespace posest
