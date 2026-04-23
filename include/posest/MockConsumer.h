#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "posest/ConsumerBase.h"

namespace posest::mock {

struct MockConsumerConfig {
    std::string id = "mock_consumer";
    // Simulated per-frame processing time. Used by tests to force the
    // latest-only drop path.
    std::chrono::milliseconds processing_delay{0};
};

struct MockConsumerStats {
    std::uint64_t frames_received = 0;
    std::uint64_t mailbox_drops = 0;   // counted by LatestFrameSlot

    // Derived from received sequence numbers: (last - first + 1) - received.
    // Interpreted as "producer frames we never saw".
    std::uint64_t sequence_gaps = 0;
    std::uint64_t first_sequence = 0;
    std::uint64_t last_sequence = 0;

    // Latency = receive_time - Frame::capture_time, in microseconds.
    std::int64_t min_latency_us = 0;
    std::int64_t avg_latency_us = 0;
    std::int64_t p95_latency_us = 0;
    std::int64_t max_latency_us = 0;
};

class MockConsumer final : public ConsumerBase {
public:
    explicit MockConsumer(MockConsumerConfig cfg);

    MockConsumerStats snapshot() const;

protected:
    void process(const Frame& frame) override;

private:
    struct Sample {
        std::uint64_t sequence;
        std::int64_t latency_us;
    };

    MockConsumerConfig cfg_;

    mutable std::mutex samples_mu_;
    std::vector<Sample> samples_;
};

}  // namespace posest::mock
