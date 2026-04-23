#include "posest/MockProducer.h"

#include <chrono>
#include <thread>
#include <utility>

#include <opencv2/core.hpp>

namespace posest::mock {

using namespace std::chrono;

MockProducer::MockProducer(MockProducerConfig cfg)
    : ProducerBase(cfg.id),
      cfg_(std::move(cfg)),
      period_(duration_cast<steady_clock::duration>(
          duration<double>(1.0 / cfg_.target_fps))) {}

bool MockProducer::captureOne(
    cv::Mat& out,
    std::optional<std::chrono::steady_clock::time_point>& out_capture_time) {
    auto now = steady_clock::now();
    steady_clock::time_point scheduled_capture;

    if (first_) {
        scheduled_capture = now;
        next_deadline_ = now + period_;
        first_ = false;
    } else {
        // If we've fallen behind (e.g. heavy system load), jump the schedule
        // forward so we don't burn CPU chasing missed deadlines.
        if (next_deadline_ < now) {
            scheduled_capture = now;
            next_deadline_ = now + period_;
        } else {
            std::this_thread::sleep_until(next_deadline_);
            scheduled_capture = next_deadline_;
            next_deadline_ += period_;
        }
    }

    out.create(cfg_.height, cfg_.width, CV_8UC1);
    out.setTo(cv::Scalar(static_cast<double>(frame_idx_ & 0xFF)));
    ++frame_idx_;

    // Stand in for a camera's native timestamp: we report the instant the
    // "shutter" fired (the scheduled deadline), not the post-capture return
    // time. When supply_timestamp is false, leave the optional empty so the
    // base falls back to steady_clock::now().
    if (cfg_.supply_timestamp) {
        out_capture_time = scheduled_capture;
    }
    return true;
}

}  // namespace posest::mock
