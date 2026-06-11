#include "fusion/teensy_now.hpp"

#include <algorithm>

namespace gw::fusion {

void TeensyNowEstimator::feed(int64_t t_meas_ns, int64_t host_arrival_ns) {
    const double offset =
        static_cast<double>(t_meas_ns) - static_cast<double>(host_arrival_ns);
    if (n_ == 0) {
        offset_ns_ = offset;
    } else if (n_ < kWarmup) {
        // Plain average during warmup — converges fast from the first sample.
        offset_ns_ += (offset - offset_ns_) / static_cast<double>(n_ + 1);
    } else {
        const double innovation =
            std::clamp(offset - offset_ns_, -kClampNs, kClampNs);
        offset_ns_ += kAlpha * innovation;
    }
    ++n_;
}

std::optional<int64_t> TeensyNowEstimator::now(int64_t host_now_ns) const {
    if (!healthy()) return std::nullopt;
    return host_now_ns + static_cast<int64_t>(offset_ns_);
}

void TeensyNowEstimator::reset() {
    offset_ns_ = 0.0;
    n_         = 0;
}

}  // namespace gw::fusion
