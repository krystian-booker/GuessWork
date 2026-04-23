#include "posest/MockConsumer.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <thread>
#include <utility>

namespace posest::mock {

using namespace std::chrono;

MockConsumer::MockConsumer(MockConsumerConfig cfg)
    : ConsumerBase(cfg.id), cfg_(std::move(cfg)) {
    samples_.reserve(4096);
}

void MockConsumer::process(const Frame& frame) {
    const auto now = steady_clock::now();
    const auto latency = duration_cast<microseconds>(now - frame.capture_time).count();

    {
        std::lock_guard<std::mutex> g(samples_mu_);
        samples_.push_back({frame.sequence, latency});
    }

    if (cfg_.processing_delay.count() > 0) {
        std::this_thread::sleep_for(cfg_.processing_delay);
    }
}

MockConsumerStats MockConsumer::snapshot() const {
    MockConsumerStats s;
    s.mailbox_drops = droppedByMailbox();

    std::vector<Sample> local;
    {
        std::lock_guard<std::mutex> g(samples_mu_);
        local = samples_;
    }

    s.frames_received = static_cast<std::uint64_t>(local.size());
    if (local.empty()) {
        return s;
    }

    // Sequence range + gaps. Samples arrive in producer order, but play it
    // safe: compute first/last via min/max over sequence numbers.
    auto [min_it, max_it] = std::minmax_element(
        local.begin(), local.end(),
        [](const Sample& a, const Sample& b) { return a.sequence < b.sequence; });
    s.first_sequence = min_it->sequence;
    s.last_sequence = max_it->sequence;
    const auto span = s.last_sequence - s.first_sequence + 1;
    s.sequence_gaps = span > s.frames_received ? span - s.frames_received : 0;

    // Latency stats.
    std::vector<std::int64_t> lat;
    lat.reserve(local.size());
    for (const auto& sample : local) {
        lat.push_back(sample.latency_us);
    }
    std::sort(lat.begin(), lat.end());
    s.min_latency_us = lat.front();
    s.max_latency_us = lat.back();
    long long sum = 0;
    for (auto v : lat) sum += v;
    s.avg_latency_us = sum / static_cast<long long>(lat.size());
    const size_t p95_idx = std::min(lat.size() - 1, static_cast<size_t>(lat.size() * 95 / 100));
    s.p95_latency_us = lat[p95_idx];

    return s;
}

}  // namespace posest::mock
