#pragma once

#include <cstdint>
#include <deque>
#include <optional>

namespace gw {

// Maps the robot controller's FPGA clock onto the Teensy clock from the
// (rio_time_us, teensy_arrival_us) pairs carried by every chassis-speeds
// sample. Pure — no system clocks, no threads; the owner serializes calls
// and passes "now" (latest Teensy-domain time it has seen) into healthy().
//
// Model: teensy_us ≈ rio_us + offset(t), with crystal drift ≤ ~100 ppm.
// Each pair satisfies arrival − rio = offset + transit, where transit is the
// controller's scheduling delay + CAN latency + ISR latency (0.1–5 ms,
// one-sided). Estimation:
//   - pairs are bucketed into 250 ms windows of rio time; each bucket keeps
//     min(arrival − rio), which rejects the one-sided transit jitter;
//   - a least-squares line through the last 24 bucket minima (6 s) gives
//     offset + drift.
// A constant floor bias (~one CAN frame + ISR, ≈150 µs) remains in the
// offset. Accepted: far below the usefulness threshold for 5.2 m/s odometry,
// and it cancels on the pose-downlink round trip (to_rio_us carries the same
// bias). See docs/can-protocol.md.
//
// Reset triggers (controller reboot / FPGA clock set): rio time jumping
// backward, or the measured delta stepping > 50 ms away from the fit while
// healthy. After a reset the estimator re-warms in ~1 s; callers fall back
// to arrival stamps while unhealthy.
class RioClockSync {
public:
    void feed(uint64_t rio_time_us, uint64_t teensy_arrival_us);

    // True iff the fit is usable: >= 4 completed buckets, residual RMS
    // <= 500 µs, |drift| <= 200 ppm, and the last pair fed is no older than
    // 500 ms relative to `now_teensy_us`.
    bool healthy(uint64_t now_teensy_us) const;

    // Map a controller timestamp into the Teensy domain (ns) and back.
    // Empty when the fit is not usable (same criteria as healthy() minus the
    // staleness check, which needs `now`).
    std::optional<uint64_t> to_teensy_ns(uint64_t rio_time_us) const;
    std::optional<uint64_t> to_rio_us(uint64_t teensy_ns) const;

    // Modeled offset (teensy − rio, µs) at the latest sample; 0 before the
    // first fit.
    double offset_us() const { return last_offset_us_; }
    double drift_ppm() const { return drift_ppm_; }

    uint64_t samples() const { return samples_; }  // pairs since last reset
    uint64_t resets() const { return resets_; }

    void reset();

private:
    struct Bucket {
        uint64_t rio_start_us = 0;  // bucket key (rio_time_us / kBucketUs)
        double   rio_mid_us   = 0;  // representative x for the fit
        double   min_delta_us = 0;  // min(arrival − rio) seen in the bucket
        uint32_t count        = 0;
    };

    static constexpr uint64_t kBucketUs       = 250'000;  // 250 ms
    static constexpr size_t   kMaxBuckets     = 24;       // 6 s window
    static constexpr size_t   kMinFitBuckets  = 4;        // ~1 s warm-up
    static constexpr double   kMaxResidualUs  = 500.0;
    static constexpr double   kMaxDriftPpm    = 200.0;
    static constexpr uint64_t kMaxFeedAgeUs   = 500'000;
    static constexpr uint64_t kBackwardJumpUs = 10'000;
    static constexpr double   kOffsetStepUs   = 50'000.0;

    void refit();
    bool fit_usable() const;
    // Modeled delta (teensy − rio, µs) at the given rio time.
    double offset_at(double rio_us) const;

    std::deque<Bucket> buckets_;
    bool   fit_valid_      = false;
    double fit_x0_us_      = 0;  // x origin for numeric conditioning
    double fit_intercept_  = 0;  // delta at x0
    double fit_slope_      = 0;  // d(delta)/d(rio_us)
    double fit_rms_us_     = 0;
    double drift_ppm_      = 0;
    double last_offset_us_ = 0;

    uint64_t last_rio_us_     = 0;
    uint64_t last_arrival_us_ = 0;
    uint64_t samples_         = 0;
    uint64_t resets_          = 0;
};

}  // namespace gw
