#include "core/clock_sync.hpp"

#include <cmath>

namespace gw {

void ClockSync::feed(uint64_t remote_us, uint64_t local_arrival_us) {
    // Remote reboot / clock-set detection. Feeding streams are in-order
    // (50–400 Hz periodic sources), so any real backward movement is a
    // clock event, not reordering.
    if (samples_ > 0 && remote_us + kBackwardJumpUs < last_remote_us_) {
        reset();
        ++resets_;
    }
    const double delta =
        static_cast<double>(local_arrival_us) - static_cast<double>(remote_us);
    if (fit_valid_ &&
        std::abs(delta - offset_at(static_cast<double>(remote_us))) >
            kOffsetStepUs) {
        reset();
        ++resets_;
    }

    last_remote_us_     = remote_us;
    last_arrival_us_ = local_arrival_us;
    ++samples_;

    const uint64_t key = remote_us / kBucketUs;
    if (buckets_.empty() || buckets_.back().remote_start_us != key) {
        // The just-closed bucket changes the fit; recompute before opening a
        // new one so to_*()/healthy() always see the freshest usable line.
        Bucket b;
        b.remote_start_us = key;
        b.remote_mid_us   = static_cast<double>(key) * kBucketUs + kBucketUs / 2.0;
        b.min_delta_us = delta;
        b.count        = 1;
        buckets_.push_back(b);
        while (buckets_.size() > kMaxBuckets) buckets_.pop_front();
        refit();
    } else {
        Bucket& b = buckets_.back();
        if (delta < b.min_delta_us) b.min_delta_us = delta;
        ++b.count;
    }
    if (fit_valid_) {
        last_offset_us_ = offset_at(static_cast<double>(remote_us));
    }
}

void ClockSync::refit() {
    // Fit min_delta vs rio_mid over completed buckets only (the last bucket
    // is still accumulating and its minimum is biased high). Ordinary least
    // squares; x is re-origined to the first bucket to keep the products
    // well-conditioned (remote_us values are ~1e9+).
    const size_t n_total = buckets_.size();
    if (n_total < kMinFitBuckets + 1) {  // +1: last bucket excluded
        fit_valid_ = false;
        return;
    }
    const size_t n  = n_total - 1;
    const double x0 = buckets_.front().remote_mid_us;

    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (size_t i = 0; i < n; ++i) {
        const double x = buckets_[i].remote_mid_us - x0;
        const double y = buckets_[i].min_delta_us;
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
    }
    const double denom = static_cast<double>(n) * sxx - sx * sx;
    if (denom <= 0.0) {
        fit_valid_ = false;
        return;
    }
    fit_slope_     = (static_cast<double>(n) * sxy - sx * sy) / denom;
    fit_intercept_ = (sy - fit_slope_ * sx) / static_cast<double>(n);
    fit_x0_us_     = x0;
    drift_ppm_     = fit_slope_ * 1e6;

    double ss = 0;
    for (size_t i = 0; i < n; ++i) {
        const double x = buckets_[i].remote_mid_us - x0;
        const double r = buckets_[i].min_delta_us - (fit_intercept_ + fit_slope_ * x);
        ss += r * r;
    }
    fit_rms_us_ = std::sqrt(ss / static_cast<double>(n));
    fit_valid_  = true;
}

double ClockSync::offset_at(double remote_us) const {
    return fit_intercept_ + fit_slope_ * (remote_us - fit_x0_us_);
}

bool ClockSync::fit_usable() const {
    return fit_valid_ && fit_rms_us_ <= kMaxResidualUs &&
           std::abs(drift_ppm_) <= kMaxDriftPpm;
}

bool ClockSync::healthy(uint64_t now_local_us) const {
    if (!fit_usable()) return false;
    if (now_local_us > last_arrival_us_ + kMaxFeedAgeUs) return false;
    return true;
}

std::optional<uint64_t> ClockSync::to_local_ns(uint64_t remote_us) const {
    if (!fit_usable()) return std::nullopt;
    const double remote = static_cast<double>(remote_us);
    const double local_us = remote + offset_at(remote);
    if (local_us < 0.0) return std::nullopt;
    return static_cast<uint64_t>(local_us * 1000.0 + 0.5);
}

std::optional<uint64_t> ClockSync::to_remote_us(uint64_t local_ns) const {
    if (!fit_usable()) return std::nullopt;
    // Invert local = remote + intercept + slope·(remote − x0). The closed
    // form is exact; with |slope| ≤ 2e-4 the division is numerically benign.
    const double local_us = static_cast<double>(local_ns) * 1e-3;
    const double remote_us =
        (local_us - fit_intercept_ + fit_slope_ * fit_x0_us_) / (1.0 + fit_slope_);
    if (remote_us < 0.0) return std::nullopt;
    return static_cast<uint64_t>(remote_us + 0.5);
}

void ClockSync::reset() {
    buckets_.clear();
    fit_valid_      = false;
    fit_x0_us_      = 0;
    fit_intercept_  = 0;
    fit_slope_      = 0;
    fit_rms_us_     = 0;
    drift_ppm_      = 0;
    last_offset_us_ = 0;
    last_remote_us_     = 0;
    last_arrival_us_ = 0;
    samples_         = 0;
}

}  // namespace gw
