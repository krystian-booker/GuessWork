#include <cstdint>

#include <gtest/gtest.h>

#include "posest/util/LatencyHistogram.h"

namespace {

posest::util::LatencyHistogram::Snapshot snapshotOf(
    posest::util::LatencyHistogram& h) {
    return h.snapshot();
}

}  // namespace

TEST(LatencyHistogram, EmptySnapshotReturnsZeroCount) {
    posest::util::LatencyHistogram h;
    const auto snap = snapshotOf(h);
    EXPECT_EQ(snap.count, 0u);
    EXPECT_EQ(snap.min_us, 0);
    EXPECT_EQ(snap.max_us, 0);
    EXPECT_EQ(snap.p50_us, 0);
}

TEST(LatencyHistogram, ComputesPercentilesOnLinearSamples) {
    posest::util::LatencyHistogram h(100);
    for (std::int64_t i = 1; i <= 100; ++i) {
        h.recordSample(i);
    }
    const auto snap = snapshotOf(h);
    EXPECT_EQ(snap.count, 100u);
    EXPECT_EQ(snap.min_us, 1);
    EXPECT_EQ(snap.max_us, 100);
    EXPECT_EQ(snap.avg_us, 50);  // (1 + 100) * 100 / 2 / 100 = 50
    EXPECT_EQ(snap.p50_us, 51);  // index = floor(0.5 * 100) = 50 → sorted[50] = 51
    EXPECT_EQ(snap.p95_us, 96);  // index 95 → sorted[95] = 96
    EXPECT_EQ(snap.p99_us, 100); // index 99 → sorted[99] = 100
}

TEST(LatencyHistogram, RollsOverWhenCapacityExceeded) {
    // Capacity 4: only the last 4 samples should be retained.
    posest::util::LatencyHistogram h(4);
    h.recordSample(1);
    h.recordSample(2);
    h.recordSample(3);
    h.recordSample(4);
    h.recordSample(100);  // evicts 1
    h.recordSample(200);  // evicts 2
    const auto snap = snapshotOf(h);
    EXPECT_EQ(snap.count, 4u);
    EXPECT_EQ(snap.min_us, 3);
    EXPECT_EQ(snap.max_us, 200);
}

TEST(LatencyHistogram, ClearResetsCount) {
    posest::util::LatencyHistogram h(8);
    for (std::int64_t i = 0; i < 5; ++i) {
        h.recordSample(i);
    }
    EXPECT_EQ(h.count(), 5u);
    h.clear();
    EXPECT_EQ(h.count(), 0u);
    const auto snap = snapshotOf(h);
    EXPECT_EQ(snap.count, 0u);
}

TEST(LatencyHistogram, HandlesSingleSample) {
    posest::util::LatencyHistogram h;
    h.recordSample(42);
    const auto snap = snapshotOf(h);
    EXPECT_EQ(snap.count, 1u);
    EXPECT_EQ(snap.min_us, 42);
    EXPECT_EQ(snap.max_us, 42);
    EXPECT_EQ(snap.avg_us, 42);
    EXPECT_EQ(snap.p50_us, 42);
    EXPECT_EQ(snap.p99_us, 42);
}
