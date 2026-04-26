#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <opencv2/core/mat.hpp>

namespace posest {

struct Frame {
    std::chrono::steady_clock::time_point capture_time{};
    std::uint64_t sequence{0};
    std::string camera_id;
    cv::Mat image;
    // Populated by ProducerBase when a CameraTriggerCache is wired and the
    // capture timestamp matches a recent Teensy-stamped trigger pulse.
    std::optional<std::uint64_t> teensy_time_us;
    std::optional<std::uint32_t> trigger_sequence;
    // Populated by ProducerBase when a ToFSampleCache is wired AND a matching
    // trigger_sequence was stamped above AND the cache holds a sample for that
    // sequence. distance_m has the host-side mounting offset applied (raw
    // chip reading lives on the bus-published ToFSample, not on Frame).
    std::optional<double> ground_distance_m;
    std::optional<std::uint32_t> tof_trigger_sequence;
};

using FramePtr = std::shared_ptr<const Frame>;

}  // namespace posest
