#include "fusion/sync_clock_now.hpp"

#include <algorithm>

namespace gw::fusion {

void SyncClockNowEstimator::feed(int64_t measurement_ns,
                                 int64_t host_arrival_ns) {
    const double offset = static_cast<double>(measurement_ns) -
                          static_cast<double>(host_arrival_ns);
    if (samples_ == 0) {
        offset_ns_ = offset;
    } else if (samples_ < kWarmup) {
        offset_ns_ += (offset - offset_ns_) /
                      static_cast<double>(samples_ + 1);
    } else {
        const double innovation =
            std::clamp(offset - offset_ns_, -kClampNs, kClampNs);
        offset_ns_ += kAlpha * innovation;
    }
    ++samples_;
}

std::optional<int64_t> SyncClockNowEstimator::now(int64_t host_now_ns) const {
    if (!healthy()) return std::nullopt;
    return host_now_ns + static_cast<int64_t>(offset_ns_);
}

void SyncClockNowEstimator::reset() {
    offset_ns_ = 0.0;
    samples_ = 0;
}

}  // namespace gw::fusion

