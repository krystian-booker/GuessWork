#pragma once

#include <cstdint>
#include <deque>
#include <optional>

namespace gw {

// Maps a remote clock onto a local clock from (remote_stamp, local_arrival)
// pairs carried by a periodic stream. Pure — no system clocks, no threads;
// the owner serializes calls and passes "now" (latest local-domain time it
// has seen) into healthy().
//
// Two instances cover the whole system (see docs/ethernet-protocol.md §time
// sync):
//   - RIO↔host:    remote = controller FPGA µs (UDP chassis-speeds packets),
//                  local = host monotonic µs stamped at recvfrom.
//   - Teensy↔host: remote = Teensy µs (IMU/TRIG telemetry stamps),
//                  local = host monotonic µs stamped at CDC read.
// Chaining the two maps any remote domain onto any other without a direct
// link between them.
//
// Model: local_us ≈ remote_us + offset(t), with crystal drift ≤ ~100 ppm.
// Each pair satisfies arrival − remote = offset + transit, where transit is
// scheduling + wire + stack latency (0.1–15 ms, strictly one-sided).
// Estimation:
//   - pairs are bucketed into 250 ms windows of remote time; each bucket
//     keeps min(arrival − remote), which rejects the one-sided jitter;
//   - a least-squares line through the last 24 bucket minima (6 s) gives
//     offset + drift.
// A constant floor bias (the minimum transit of the feeding path — ~150 µs
// for UDP on the robot LAN, ~0.5 ms for batched USB telemetry) remains in
// the offset. Accepted: far below the usefulness threshold for 5.2 m/s
// odometry, and the pose-downlink round trip carries the same bias in the
// opposite direction, cancelling on the wire.
//
// Reset triggers (remote reboot / clock set): remote time jumping backward,
// or the measured delta stepping > 50 ms away from the fit while healthy.
// After a reset the estimator re-warms in ~1 s; callers fall back to
// arrival stamps while unhealthy.
class ClockSync {
public:
    void feed(uint64_t remote_us, uint64_t local_arrival_us);

    // True iff the fit is usable: >= 4 completed buckets, residual RMS
    // <= 500 µs, |drift| <= 200 ppm, and the last pair fed is no older than
    // 500 ms relative to `now_local_us`.
    bool healthy(uint64_t now_local_us) const;

    // Map a remote timestamp into the local domain (ns) and back. Empty when
    // the fit is not usable (same criteria as healthy() minus the staleness
    // check, which needs `now`).
    std::optional<uint64_t> to_local_ns(uint64_t remote_us) const;
    std::optional<uint64_t> to_remote_us(uint64_t local_ns) const;

    // Modeled offset (local − remote, µs) at the latest sample; 0 before the
    // first fit.
    double offset_us() const { return last_offset_us_; }
    double drift_ppm() const { return drift_ppm_; }

    uint64_t samples() const { return samples_; }  // pairs since last reset
    uint64_t resets() const { return resets_; }

    void reset();

private:
    struct Bucket {
        uint64_t remote_start_us = 0;  // bucket key (remote_us / kBucketUs)
        double   remote_mid_us   = 0;  // representative x for the fit
        double   min_delta_us    = 0;  // min(arrival − remote) in the bucket
        uint32_t count           = 0;
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
    // Modeled delta (local − remote, µs) at the given remote time.
    double offset_at(double remote_us) const;

    std::deque<Bucket> buckets_;
    bool   fit_valid_      = false;
    double fit_x0_us_      = 0;  // x origin for numeric conditioning
    double fit_intercept_  = 0;  // delta at x0
    double fit_slope_      = 0;  // d(delta)/d(remote_us)
    double fit_rms_us_     = 0;
    double drift_ppm_      = 0;
    double last_offset_us_ = 0;

    uint64_t last_remote_us_  = 0;
    uint64_t last_arrival_us_ = 0;
    uint64_t samples_         = 0;
    uint64_t resets_          = 0;
};

}  // namespace gw
