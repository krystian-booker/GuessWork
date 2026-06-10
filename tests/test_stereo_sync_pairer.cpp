#include <gtest/gtest.h>

#include <atomic>
#include <thread>

#include "vio/stereo_sync_consumer.hpp"

namespace gw::vio {

namespace {

SideFrame frame(int64_t t_ns, uint8_t marker = 0) {
    SideFrame f;
    f.t_ns   = t_ns;
    f.width  = 4;
    f.height = 4;
    f.pixels.assign(16, marker);
    return f;
}

}  // namespace

TEST(StereoSyncPairerTest, EqualStampsPair) {
    StereoSyncPairer p;
    p.push(StereoSyncPairer::kLeft, frame(100, 1));
    p.push(StereoSyncPairer::kRight, frame(100, 2));

    StereoPair out;
    ASSERT_TRUE(p.wait_pop(out));
    EXPECT_EQ(out.t_ns, 100);
    EXPECT_EQ(out.left.pixels[0], 1);
    EXPECT_EQ(out.right.pixels[0], 2);
    EXPECT_EQ(p.counters().paired, 1u);
    EXPECT_EQ(p.counters().dropped_unmatched, 0u);
}

TEST(StereoSyncPairerTest, RightBeforeLeftReorder) {
    StereoSyncPairer p;
    p.push(StereoSyncPairer::kRight, frame(100));
    p.push(StereoSyncPairer::kRight, frame(133));
    p.push(StereoSyncPairer::kLeft, frame(133));  // pairs with the 2nd right

    StereoPair out;
    ASSERT_TRUE(p.wait_pop(out));
    EXPECT_EQ(out.t_ns, 133);
    // The right frame at t=100 was older than the match → purged.
    EXPECT_EQ(p.counters().dropped_unmatched, 1u);
}

TEST(StereoSyncPairerTest, MissingSideEvictsAtCap) {
    StereoSyncPairer p;
    // Left side never arrives; right frames evict beyond the 2-slot cap.
    p.push(StereoSyncPairer::kRight, frame(100));
    p.push(StereoSyncPairer::kRight, frame(133));
    p.push(StereoSyncPairer::kRight, frame(166));
    p.push(StereoSyncPairer::kRight, frame(200));
    EXPECT_EQ(p.counters().dropped_unmatched, 2u);  // 100 and 133 evicted

    // A late left at an evicted stamp can no longer pair.
    p.push(StereoSyncPairer::kLeft, frame(100));
    EXPECT_EQ(p.counters().paired, 0u);
    // But a left at a buffered stamp pairs.
    p.push(StereoSyncPairer::kLeft, frame(200));
    StereoPair out;
    ASSERT_TRUE(p.wait_pop(out));
    EXPECT_EQ(out.t_ns, 200);
}

TEST(StereoSyncPairerTest, ZeroStampDropped) {
    StereoSyncPairer p;
    p.push(StereoSyncPairer::kLeft, frame(0));
    p.push(StereoSyncPairer::kRight, frame(0));
    EXPECT_EQ(p.counters().dropped_zero_ts, 2u);
    EXPECT_EQ(p.counters().paired, 0u);
}

TEST(StereoSyncPairerTest, MatchPurgesOlderOnBothSides) {
    StereoSyncPairer p;
    p.push(StereoSyncPairer::kLeft, frame(100));   // will go stale
    p.push(StereoSyncPairer::kRight, frame(133));
    p.push(StereoSyncPairer::kLeft, frame(133));   // pairs; left@100 purged

    StereoPair out;
    ASSERT_TRUE(p.wait_pop(out));
    EXPECT_EQ(out.t_ns, 133);
    EXPECT_EQ(p.counters().dropped_unmatched, 1u);

    // A right at the stale stamp finds nothing.
    p.push(StereoSyncPairer::kRight, frame(100));
    EXPECT_EQ(p.counters().paired, 1u);
}

TEST(StereoSyncPairerTest, PairQueueOverflowDropsOldest) {
    StereoSyncPairer p;
    for (int i = 1; i <= 6; ++i) {
        p.push(StereoSyncPairer::kLeft, frame(i * 33));
        p.push(StereoSyncPairer::kRight, frame(i * 33));
    }
    EXPECT_EQ(p.counters().paired, 6u);
    EXPECT_EQ(p.counters().dropped_pair_queue, 2u);  // cap 4

    StereoPair out;
    ASSERT_TRUE(p.wait_pop(out));
    EXPECT_EQ(out.t_ns, 3 * 33);  // pairs 1–2 were dropped
}

TEST(StereoSyncPairerTest, ShutdownWakesWaitPop) {
    StereoSyncPairer p;
    std::atomic<bool> returned_false{false};
    std::thread t([&] {
        StereoPair out;
        if (!p.wait_pop(out)) returned_false.store(true);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    p.shutdown();
    t.join();
    EXPECT_TRUE(returned_false.load());

    // Pushes after shutdown are no-ops.
    p.push(StereoSyncPairer::kLeft, frame(100));
    EXPECT_EQ(p.counters().paired, 0u);
}

TEST(StereoSyncPairerTest, ResetClearsBuffersKeepsCounters) {
    StereoSyncPairer p;
    p.push(StereoSyncPairer::kLeft, frame(100));
    p.push(StereoSyncPairer::kRight, frame(100));
    p.reset();

    StereoPair out;
    p.shutdown();
    EXPECT_FALSE(p.wait_pop(out));            // queue cleared
    EXPECT_EQ(p.counters().paired, 1u);       // counters survive
}

}  // namespace gw::vio
