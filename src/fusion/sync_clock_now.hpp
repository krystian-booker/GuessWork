#pragma once

#include <cstdint>
#include <optional>

namespace gw::fusion {

// Estimates the sync-controller clock at a given host steady-clock instant.
// Arrival latency biases the estimate slightly early, which safely shortens
// output extrapolation. A bounded EMA rejects delayed USB batches.
class SyncClockNowEstimator {
public:
    void feed(int64_t measurement_ns, int64_t host_arrival_ns);
    std::optional<int64_t> now(int64_t host_now_ns) const;

    bool healthy() const { return samples_ >= kWarmup; }
    double offset_ms() const { return offset_ns_ * 1e-6; }
    uint64_t samples() const { return samples_; }
    void reset();

private:
    static constexpr uint64_t kWarmup = 8;
    static constexpr double kAlpha = 0.05;
    static constexpr double kClampNs = 50e6;

    double offset_ns_ = 0.0;
    uint64_t samples_ = 0;
};

}  // namespace gw::fusion

