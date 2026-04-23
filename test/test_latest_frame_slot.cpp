#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "posest/Frame.h"
#include "posest/LatestFrameSlot.h"

using namespace std::chrono_literals;
using posest::Frame;
using posest::FramePtr;
using posest::LatestFrameSlot;

namespace {

FramePtr makeFrame(std::uint64_t seq) {
    auto f = std::make_shared<Frame>();
    f->sequence = seq;
    f->capture_time = std::chrono::steady_clock::now();
    return f;
}

}  // namespace

TEST(LatestFrameSlot, PutThenTakeReturnsFrame) {
    LatestFrameSlot slot;
    slot.put(makeFrame(42));
    FramePtr f = slot.take();
    ASSERT_NE(f, nullptr);
    EXPECT_EQ(f->sequence, 42u);
    EXPECT_EQ(slot.droppedCount(), 0u);
}

TEST(LatestFrameSlot, PutOverwritesUnreadFrame) {
    LatestFrameSlot slot;
    slot.put(makeFrame(1));
    slot.put(makeFrame(2));
    slot.put(makeFrame(3));
    FramePtr f = slot.take();
    ASSERT_NE(f, nullptr);
    EXPECT_EQ(f->sequence, 3u);
    EXPECT_EQ(slot.droppedCount(), 2u);
}

TEST(LatestFrameSlot, TakeBlocksUntilPut) {
    LatestFrameSlot slot;
    std::atomic<bool> observed{false};
    std::thread taker([&] {
        FramePtr f = slot.take();
        ASSERT_NE(f, nullptr);
        EXPECT_EQ(f->sequence, 7u);
        observed.store(true);
    });

    std::this_thread::sleep_for(20ms);
    EXPECT_FALSE(observed.load()) << "take() should still be blocked";
    slot.put(makeFrame(7));
    taker.join();
    EXPECT_TRUE(observed.load());
}

TEST(LatestFrameSlot, ShutdownUnblocksTake) {
    LatestFrameSlot slot;
    std::thread taker([&] {
        FramePtr f = slot.take();
        EXPECT_EQ(f, nullptr);
    });
    std::this_thread::sleep_for(10ms);
    slot.shutdown();
    taker.join();
}

TEST(LatestFrameSlot, PutAfterShutdownIsDropped) {
    LatestFrameSlot slot;
    slot.shutdown();
    slot.put(makeFrame(1));
    FramePtr f = slot.take();
    EXPECT_EQ(f, nullptr);
}

TEST(LatestFrameSlot, ConcurrentPutTake1P1C) {
    // A producer thread puts N frames back-to-back. A consumer thread loops
    // take() until it sees seq == N-1. Invariants: every observed frame is
    // non-null and sequence numbers are monotonically non-decreasing.
    constexpr std::uint64_t kN = 50000;
    LatestFrameSlot slot;

    std::atomic<bool> stop{false};
    std::thread consumer([&] {
        std::uint64_t last = 0;
        bool seen_any = false;
        while (!stop.load(std::memory_order_acquire)) {
            FramePtr f = slot.take();
            if (!f) break;
            if (seen_any) {
                EXPECT_GE(f->sequence, last);
            }
            last = f->sequence;
            seen_any = true;
            if (last >= kN - 1) break;
        }
    });

    for (std::uint64_t i = 0; i < kN; ++i) {
        slot.put(makeFrame(i));
    }
    // Give consumer a chance to observe the final frame before we shut down.
    std::this_thread::sleep_for(10ms);
    stop.store(true, std::memory_order_release);
    slot.shutdown();
    consumer.join();

    // Most puts should be dropped (latest-only semantics); exact count is
    // timing-dependent, so we just assert it's plausible.
    EXPECT_GT(slot.droppedCount(), 0u);
    EXPECT_LT(slot.droppedCount(), kN);
}
