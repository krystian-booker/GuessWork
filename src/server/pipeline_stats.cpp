#include "server/pipeline_stats.hpp"

namespace gw::server {

namespace {
constexpr auto kFpsWindow = std::chrono::milliseconds(1000);
}

PipelineStatsView::PipelineStatsView(const SpinnakerProducer& producer, TimePoint started_at)
    : producer_(producer)
    , started_at_(started_at)
    , last_sample_t_(started_at) {}

PipelineStats PipelineStatsView::snapshot() {
    const auto producer_stats = producer_.stats();
    const auto now            = Clock::now();

    std::lock_guard lock(mu_);

    const auto since_last = now - last_sample_t_;
    if (since_last >= kFpsWindow) {
        const auto seconds   = std::chrono::duration<double>(since_last).count();
        const auto delta_pub = producer_stats.total_published - last_published_;
        cached_fps_          = (seconds > 0.0) ? (static_cast<double>(delta_pub) / seconds) : 0.0;
        last_published_      = producer_stats.total_published;
        last_sample_t_       = now;
    }

    PipelineStats out;
    out.camera_connected  = true;  // producer was successfully started by main.
    out.frames_produced   = producer_stats.total_published;
    out.frames_dropped    = producer_stats.total_dropped;
    out.frames_incomplete = producer_stats.total_incomplete;
    out.fps_1s            = cached_fps_;
    out.uptime_s          = std::chrono::duration<double>(now - started_at_).count();
    return out;
}

}  // namespace gw::server
