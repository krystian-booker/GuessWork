#include <gtest/gtest.h>

#include "core/latency_stats.hpp"

namespace gw {

TEST(LatencyStatsTest, EmptyIsZero) {
    LatencyStats s;
    EXPECT_DOUBLE_EQ(s.last_ms(), 0.0);
    EXPECT_DOUBLE_EQ(s.p95_ms(), 0.0);
    EXPECT_EQ(s.count(), 0u);
}

TEST(LatencyStatsTest, SingleSample) {
    LatencyStats s;
    s.add(12.5);
    EXPECT_DOUBLE_EQ(s.last_ms(), 12.5);
    EXPECT_DOUBLE_EQ(s.p95_ms(), 12.5);
    EXPECT_EQ(s.count(), 1u);
}

TEST(LatencyStatsTest, P95OverRollingWindow) {
    LatencyStats s;
    // 1000 monotone samples: the ring holds the last 256 (744..999); p95 of
    // that window sits in its top decile.
    for (int i = 0; i < 1000; ++i) s.add(static_cast<double>(i));
    EXPECT_DOUBLE_EQ(s.last_ms(), 999.0);
    EXPECT_GE(s.p95_ms(), 744.0 + 0.9 * 255.0);
    EXPECT_LE(s.p95_ms(), 999.0);
    EXPECT_EQ(s.count(), 1000u);
}

TEST(LatencyStatsTest, ResetClears) {
    LatencyStats s;
    s.add(5.0);
    s.reset();
    EXPECT_DOUBLE_EQ(s.last_ms(), 0.0);
    EXPECT_DOUBLE_EQ(s.p95_ms(), 0.0);
    EXPECT_EQ(s.count(), 0u);
}

}  // namespace gw
