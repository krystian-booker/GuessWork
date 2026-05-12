#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>

#include "producer/spinnaker_producer.hpp"

namespace gw::server {

struct PipelineStats {
    bool     camera_connected = false;
    uint64_t frames_produced  = 0;
    uint64_t frames_dropped   = 0;
    uint64_t frames_incomplete = 0;
    double   fps_1s           = 0.0;
    double   uptime_s         = 0.0;
};

// Thread-safe sampler over a running SpinnakerProducer. Holds a small amount
// of state so it can compute a rolling 1-second FPS without burdening callers.
//
// snapshot() is safe to call from any thread; FPS is recomputed at most once
// per second. Between recomputations the cached value is returned.
class PipelineStatsView {
public:
    using Clock     = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    PipelineStatsView(const SpinnakerProducer& producer, TimePoint started_at);

    PipelineStats snapshot();

private:
    const SpinnakerProducer& producer_;
    const TimePoint          started_at_;

    std::mutex mu_;
    uint64_t   last_published_ = 0;
    TimePoint  last_sample_t_;
    double     cached_fps_ = 0.0;
};

}  // namespace gw::server
