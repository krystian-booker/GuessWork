#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace posest::util {

// Fixed-capacity rolling histogram of latency samples in microseconds. Used
// to surface fusion-side per-stage timings (bus-pop → graph-update, graph
// update → publish) through DaemonHealth without holding an unbounded vector.
//
// Not thread-safe. Callers (FusionService) serialize access under their own
// mutex; the daemon's health thread reads via snapshot() while the worker
// thread writes via recordSample(), both under the same lock.
class LatencyHistogram {
public:
    struct Snapshot {
        std::size_t count{0};
        std::int64_t min_us{0};
        std::int64_t max_us{0};
        std::int64_t avg_us{0};
        std::int64_t p50_us{0};
        std::int64_t p95_us{0};
        std::int64_t p99_us{0};
    };

    explicit LatencyHistogram(std::size_t capacity = 1024)
        : capacity_(capacity == 0 ? 1u : capacity), samples_(capacity_, 0) {}

    void recordSample(std::int64_t value_us) {
        samples_[head_] = value_us;
        head_ = (head_ + 1u) % capacity_;
        if (count_ < capacity_) {
            ++count_;
        }
    }

    Snapshot snapshot() const {
        Snapshot s;
        s.count = count_;
        if (count_ == 0u) {
            return s;
        }
        // Sorting a copy keeps the rolling buffer in insertion order so
        // subsequent recordSample writes still index by `head_` correctly.
        std::vector<std::int64_t> sorted(
            samples_.begin(),
            samples_.begin() + static_cast<std::ptrdiff_t>(count_));
        std::sort(sorted.begin(), sorted.end());
        s.min_us = sorted.front();
        s.max_us = sorted.back();
        std::int64_t sum = 0;
        for (const auto v : sorted) {
            sum += v;
        }
        s.avg_us = sum / static_cast<std::int64_t>(count_);
        const auto pct = [&](double p) -> std::int64_t {
            const std::size_t idx = std::min(
                count_ - 1u,
                static_cast<std::size_t>(
                    p * static_cast<double>(count_)));
            return sorted[idx];
        };
        s.p50_us = pct(0.50);
        s.p95_us = pct(0.95);
        s.p99_us = pct(0.99);
        return s;
    }

    std::size_t count() const { return count_; }
    std::size_t capacity() const { return capacity_; }

    void clear() {
        count_ = 0;
        head_ = 0;
    }

private:
    std::size_t capacity_;
    std::vector<std::int64_t> samples_;
    std::size_t count_{0};
    std::size_t head_{0};
};

}  // namespace posest::util
