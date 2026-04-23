#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <opencv2/core/mat.hpp>

namespace posest {

struct Frame {
    std::chrono::steady_clock::time_point capture_time{};
    std::uint64_t sequence{0};
    std::string camera_id;
    cv::Mat image;
};

using FramePtr = std::shared_ptr<const Frame>;

}  // namespace posest
