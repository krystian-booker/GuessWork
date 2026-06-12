#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace gw {

// Fixed 256-sample ring of millisecond latencies with last/p95 readout.
// NOT thread-safe — the owner serializes access (e.g. FusionSupervisor
// guards its instances with a mutex; FusionEngine is single-threaded).
class LatencyStats {
public:
    void add(double ms) {
        ring_[n_ % kRing] = ms;
        last_             = ms;
        ++n_;
    }

    double last_ms() const { return last_; }

    // 0 before the first sample. Index formula matches the fusion engine's
    // original hand-rolled ring so test expectations stay stable.
    double p95_ms() const {
        const size_t n = std::min<size_t>(n_, kRing);
        if (n == 0) return 0.0;
        std::array<double, kRing> sorted = ring_;
        std::sort(sorted.begin(), sorted.begin() + static_cast<ptrdiff_t>(n));
        return sorted[static_cast<size_t>(0.95 * static_cast<double>(n - 1))];
    }

    uint64_t count() const { return n_; }

    void reset() {
        ring_.fill(0.0);
        last_ = 0.0;
        n_    = 0;
    }

private:
    static constexpr size_t kRing = 256;

    std::array<double, kRing> ring_{};
    double   last_ = 0.0;
    uint64_t n_    = 0;
};

}  // namespace gw
