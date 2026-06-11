#include <gtest/gtest.h>

#include <cstdint>

#include "fusion/teensy_now.hpp"

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

constexpr int64_t kOffset = 5'000'000'000'000ll;  // teensy − host, 5000 s

// Feeds n samples at 100 Hz with the given feed-latency jitter range.
// teensy time = host + offset; arrival = host + latency.
int64_t drive(TeensyNowEstimator& e, Lcg& rng, int64_t& host_ns, int n,
              int64_t jit_lo_ns, int64_t jit_hi_ns) {
    int64_t last_host = host_ns;
    for (int i = 0; i < n; ++i) {
        const int64_t t_meas  = host_ns + kOffset;
        const int64_t arrival = host_ns + rng.uniform(jit_lo_ns, jit_hi_ns);
        e.feed(t_meas, arrival);
        last_host = host_ns;
        host_ns += 10'000'000;  // 10 ms
    }
    return last_host;
}

}  // namespace

TEST(TeensyNowTest, UnhealthyBeforeWarmup) {
    TeensyNowEstimator e;
    Lcg rng;
    int64_t host = 0;
    drive(e, rng, host, 7, 1'000'000, 3'000'000);  // < 8-sample warmup
    EXPECT_FALSE(e.healthy());
    EXPECT_FALSE(e.now(host).has_value());
}

TEST(TeensyNowTest, ConvergesToConstantOffset) {
    TeensyNowEstimator e;
    Lcg rng;
    int64_t host = 0;
    const int64_t last = drive(e, rng, host, 100, 1'000'000, 3'000'000);
    ASSERT_TRUE(e.healthy());
    const auto now = e.now(last);
    ASSERT_TRUE(now.has_value());
    // Truth at that host time is host + offset; the feed latency (1–3 ms)
    // biases the estimate EARLY by roughly the mean latency.
    const int64_t err = *now - (last + kOffset);
    EXPECT_LE(err, 0);            // never late
    EXPECT_GT(err, -4'000'000);   // within the latency band
}

TEST(TeensyNowTest, OutlierClamped) {
    TeensyNowEstimator e;
    Lcg rng;
    int64_t host = 0;
    drive(e, rng, host, 100, 1'000'000, 2'000'000);
    const double before = e.offset_ms();
    // One sample delayed by 500 ms (arrival way late → offset way low).
    e.feed(host + kOffset, host + 500'000'000);
    const double after = e.offset_ms();
    EXPECT_LT(before - after, 3.0);  // clamp limits the hit to α·50 ms = 2.5 ms
}

TEST(TeensyNowTest, TracksSlowDrift) {
    TeensyNowEstimator e;
    Lcg rng;
    int64_t host = 0;
    // 10 ppm drift over 60 s = 600 µs of offset movement.
    for (int i = 0; i < 6000; ++i) {
        const int64_t drift  = static_cast<int64_t>(host * 1e-5);
        const int64_t t_meas = host + kOffset + drift;
        e.feed(t_meas, host + rng.uniform(1'000'000, 2'000'000));
        host += 10'000'000;
    }
    const int64_t truth_offset = kOffset + static_cast<int64_t>(host * 1e-5);
    const auto now = e.now(host);
    ASSERT_TRUE(now.has_value());
    const int64_t err = *now - (host + truth_offset);
    EXPECT_GT(err, -4'000'000);  // tracked within the latency band + lag
    EXPECT_LE(err, 1'000'000);
}

TEST(TeensyNowTest, ResetClears) {
    TeensyNowEstimator e;
    Lcg rng;
    int64_t host = 0;
    drive(e, rng, host, 50, 1'000'000, 2'000'000);
    ASSERT_TRUE(e.healthy());
    e.reset();
    EXPECT_FALSE(e.healthy());
    EXPECT_EQ(e.samples(), 0u);
    EXPECT_FALSE(e.now(host).has_value());
}

}  // namespace gw::fusion
