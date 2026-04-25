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
};

using FramePtr = std::shared_ptr<const Frame>;

}  // namespace posest
