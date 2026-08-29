#include <gtest/gtest.h>

#include <cstdint>

#include "fusion/sync_clock_now.hpp"

namespace gw::fusion {
namespace {

struct Lcg {
    uint64_t state = 0x9E3779B97F4A7C15ull;
    uint64_t next() {
        state = state * 6364136223846793005ull + 1442695040888963407ull;
        return state >> 33;
    }
    int64_t uniform(int64_t lo, int64_t hi) {
        return lo + static_cast<int64_t>(next() % static_cast<uint64_t>(hi - lo));
    }
};

constexpr int64_t kOffset = 5'000'000'000'000ll;

int64_t drive(SyncClockNowEstimator& estimator, Lcg& rng, int64_t& host_ns,
              int count, int64_t jitter_low_ns, int64_t jitter_high_ns) {
    int64_t last_host = host_ns;
    for (int i = 0; i < count; ++i) {
        const int64_t measurement = host_ns + kOffset;
        const int64_t arrival = host_ns + rng.uniform(jitter_low_ns,
                                                       jitter_high_ns);
        estimator.feed(measurement, arrival);
        last_host = host_ns;
        host_ns += 10'000'000;
    }
    return last_host;
}

}  // namespace

TEST(SyncClockNowTest, UnhealthyBeforeWarmup) {
    SyncClockNowEstimator estimator;
    Lcg rng;
    int64_t host = 0;
    drive(estimator, rng, host, 7, 1'000'000, 3'000'000);
    EXPECT_FALSE(estimator.healthy());
    EXPECT_FALSE(estimator.now(host).has_value());
}

TEST(SyncClockNowTest, ConvergesToConstantOffset) {
    SyncClockNowEstimator estimator;
    Lcg rng;
    int64_t host = 0;
    const int64_t last = drive(estimator, rng, host, 100, 1'000'000, 3'000'000);
    const auto now = estimator.now(last);
    ASSERT_TRUE(now.has_value());
    const int64_t error = *now - (last + kOffset);
    EXPECT_LE(error, 0);
    EXPECT_GT(error, -4'000'000);
}

TEST(SyncClockNowTest, OutlierIsClamped) {
    SyncClockNowEstimator estimator;
    Lcg rng;
    int64_t host = 0;
    drive(estimator, rng, host, 100, 1'000'000, 2'000'000);
    const double before = estimator.offset_ms();
    estimator.feed(host + kOffset, host + 500'000'000);
    EXPECT_LT(before - estimator.offset_ms(), 3.0);
}

TEST(SyncClockNowTest, TracksSlowDrift) {
    SyncClockNowEstimator estimator;
    Lcg rng;
    int64_t host = 0;
    for (int i = 0; i < 6000; ++i) {
        const int64_t drift = static_cast<int64_t>(host * 1e-5);
        estimator.feed(host + kOffset + drift,
                       host + rng.uniform(1'000'000, 2'000'000));
        host += 10'000'000;
    }
    const int64_t truth_offset = kOffset + static_cast<int64_t>(host * 1e-5);
    const auto now = estimator.now(host);
    ASSERT_TRUE(now.has_value());
    const int64_t error = *now - (host + truth_offset);
    EXPECT_GT(error, -4'000'000);
    EXPECT_LE(error, 1'000'000);
}

TEST(SyncClockNowTest, ResetClears) {
    SyncClockNowEstimator estimator;
    Lcg rng;
    int64_t host = 0;
    drive(estimator, rng, host, 50, 1'000'000, 2'000'000);
    ASSERT_TRUE(estimator.healthy());
    estimator.reset();
    EXPECT_FALSE(estimator.healthy());
    EXPECT_EQ(estimator.samples(), 0u);
    EXPECT_FALSE(estimator.now(host).has_value());
}

}  // namespace gw::fusion

