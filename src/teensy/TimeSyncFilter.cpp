#include "posest/teensy/TimeSyncFilter.h"

#include <algorithm>
#include <limits>

namespace posest::teensy {

bool TimeSyncFilter::update(const Sample& sample) {
    if (count_ >= kBootstrap) {
        std::uint64_t min_rtt = std::numeric_limits<std::uint64_t>::max();
        for (std::size_t i = 0; i < count_; ++i) {
            min_rtt = std::min(min_rtt, samples_[i].round_trip_us);
        }
        const std::uint64_t threshold =
            std::max<std::uint64_t>(min_rtt, kMinRttFloorUs) * 2u + 1000u;
        if (sample.round_trip_us > threshold) {
            ++rejected_;
            return false;
        }
    }

    if (count_ < kRingSize) {
        samples_[count_++] = sample;
    } else {
        samples_[head_] = sample;
        head_ = (head_ + 1u) % kRingSize;
    }

    if (accepted_ == 0u) {
        offset_ema_us_ = sample.offset_us;
    } else {
        offset_ema_us_ =
            offset_ema_us_ + (sample.offset_us - offset_ema_us_) / 8;
    }
    anchor_teensy_us_ = sample.teensy_midpoint_us;
    last_rtt_us_ = sample.round_trip_us;
    ++accepted_;
    established_ = true;
    recomputeRegression();
    return true;
}

void TimeSyncFilter::recomputeRegression() {
    skew_per_us_ = 0.0;
    if (count_ < 3u) {
        return;
    }
    double sum_x = 0.0;
    double sum_y = 0.0;
    for (std::size_t i = 0; i < count_; ++i) {
        sum_x += static_cast<double>(samples_[i].teensy_midpoint_us);
        sum_y += static_cast<double>(samples_[i].offset_us);
    }
    const double mean_x = sum_x / static_cast<double>(count_);
    const double mean_y = sum_y / static_cast<double>(count_);
    double num = 0.0;
    double den = 0.0;
    for (std::size_t i = 0; i < count_; ++i) {
        const double dx =
            static_cast<double>(samples_[i].teensy_midpoint_us) - mean_x;
        const double dy =
            static_cast<double>(samples_[i].offset_us) - mean_y;
        num += dx * dy;
        den += dx * dx;
    }
    if (den > 0.0) {
        skew_per_us_ = num / den;
    }
}

std::int64_t TimeSyncFilter::apply(std::int64_t teensy_time_us) const {
    if (!established_) {
        return teensy_time_us;
    }
    const double drift =
        skew_per_us_ * static_cast<double>(teensy_time_us - anchor_teensy_us_);
    const std::int64_t adjustment = static_cast<std::int64_t>(drift);
    return teensy_time_us + offset_ema_us_ + adjustment;
}

void TimeSyncFilter::reset() {
    samples_ = {};
    count_ = 0;
    head_ = 0;
    offset_ema_us_ = 0;
    anchor_teensy_us_ = 0;
    skew_per_us_ = 0.0;
    accepted_ = 0;
    rejected_ = 0;
    last_rtt_us_ = 0;
    established_ = false;
}

}  // namespace posest::teensy
