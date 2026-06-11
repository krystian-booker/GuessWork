#pragma once

#include <cstdint>
#include <optional>

namespace gw::fusion {

// Host-steady → Teensy-clock offset estimator, used by the fusion output
// thread to extrapolate the latest smoothed state to "Teensy-now".
//
// feed() takes any Teensy-domain measurement timestamp together with its
// host-side arrival time (steady-clock ns). offset = t_meas − host_arrival
// is EMA-smoothed (α = 0.05 after an 8-sample warmup) with a ±50 ms
// innovation clamp so one delayed batch can't yank the estimate. The
// feed-path latency makes the estimate slightly EARLY in the Teensy domain
// — which only shortens output extrapolation, never overshoots it.
//
// Pure: no clocks, no threads. The owner serializes calls and passes
// "host now" into now().
class TeensyNowEstimator {
public:
    void feed(int64_t t_meas_ns, int64_t host_arrival_ns);

    // Teensy-domain estimate of the given host-steady time; nullopt until
    // warmed up.
    std::optional<int64_t> now(int64_t host_now_ns) const;

    bool     healthy() const { return n_ >= kWarmup; }
    double   offset_ms() const { return offset_ns_ * 1e-6; }
    uint64_t samples() const { return n_; }

    void reset();

private:
    static constexpr uint64_t kWarmup     = 8;
    static constexpr double   kAlpha      = 0.05;
    static constexpr double   kClampNs    = 50e6;  // ±50 ms innovation clamp

    double   offset_ns_ = 0.0;  // EMA of t_meas − host_arrival
    uint64_t n_         = 0;
};

}  // namespace gw::fusion
