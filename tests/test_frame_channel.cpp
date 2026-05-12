#include <gtest/gtest.h>

#include <CoreVideo/CoreVideo.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>
#include <vector>

#include "core/clock.hpp"
#include "core/frame.hpp"
#include "core/frame_channel.hpp"
#include "core/frame_pool.hpp"

namespace gw {

namespace {

constexpr FrameFormat kMono64x64{
    .width        = 64,
    .height       = 64,
    .pixel_format = kCVPixelFormatType_OneComponent8,
};

// Stamp the first pixel of a frame with a known value (low byte of sequence).
// Used to verify that a consumer never reads a frame whose bytes have been
// torn by a publisher mid-overwrite.
void stamp_frame(Frame* f, uint8_t marker) {
    CVPixelBufferRef pb = f->pixel_buffer();
    CVPixelBufferLockBaseAddress(pb, 0);
    auto* base = static_cast<uint8_t*>(CVPixelBufferGetBaseAddress(pb));
    base[0]    = marker;
    CVPixelBufferUnlockBaseAddress(pb, 0);
}

uint8_t read_marker(Frame* f) {
    CVPixelBufferRef pb = f->pixel_buffer();
    CVPixelBufferLockBaseAddress(pb, kCVPixelBufferLock_ReadOnly);
    const auto*   base = static_cast<const uint8_t*>(CVPixelBufferGetBaseAddress(pb));
    const uint8_t v    = base[0];
    CVPixelBufferUnlockBaseAddress(pb, kCVPixelBufferLock_ReadOnly);
    return v;
}

// Helper: publish a fresh frame from the pool with the given sequence number.
// The frame is filled so frame->sequence() == seq and the first pixel == low byte of seq.
void publish_seq(FrameChannel& ch, FramePool& pool, uint64_t seq) {
    Frame* f = pool.acquire();
    ASSERT_NE(f, nullptr);
    f->set_sequence(seq);
    f->set_host_capture_ns(seq * 1'000'000);  // any monotonic value
    stamp_frame(f, static_cast<uint8_t>(seq));
    ch.publish(f);
}

}  // namespace

// ---------------------------------------------------------------------------
// Basic semantics
// ---------------------------------------------------------------------------

TEST(FrameChannelTest, ConsumerSeesPublishedFrame) {
    FramePool    pool(kMono64x64, 4);
    FrameChannel ch;
    auto         sub = ch.subscribe();
    ASSERT_NE(sub, nullptr);

    publish_seq(ch, pool, 1);

    Frame* f = ch.next_frame(sub);
    ASSERT_NE(f, nullptr);
    EXPECT_EQ(f->sequence(), 1u);
    EXPECT_EQ(read_marker(f), 1u);
    f->release();
}

TEST(FrameChannelTest, NewSubscriberSeesNextPublishOnly) {
    // A subscriber attached AFTER a publish should not see the earlier frame.
    FramePool    pool(kMono64x64, 4);
    FrameChannel ch;
    publish_seq(ch, pool, 1);     // published before any subscriber

    auto sub = ch.subscribe();    // attaches now
    publish_seq(ch, pool, 2);

    Frame* f = ch.next_frame(sub);
    ASSERT_NE(f, nullptr);
    EXPECT_EQ(f->sequence(), 2u);
    f->release();
}

TEST(FrameChannelTest, ConsumerBlocksUntilPublish) {
    FramePool    pool(kMono64x64, 4);
    FrameChannel ch;
    auto         sub = ch.subscribe();

    std::atomic<bool>     got{false};
    std::atomic<uint64_t> seen_seq{0};
    std::thread           t([&] {
        Frame* f = ch.next_frame(sub);
        if (f) {
            seen_seq.store(f->sequence());
            got.store(true);
            f->release();
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    EXPECT_FALSE(got.load()) << "consumer should still be blocked";
    publish_seq(ch, pool, 42);
    t.join();
    EXPECT_TRUE(got.load());
    EXPECT_EQ(seen_seq.load(), 42u);
}

TEST(FrameChannelTest, DetachWakesAndReturnsNull) {
    FramePool    pool(kMono64x64, 4);
    FrameChannel ch;
    auto         sub = ch.subscribe();

    std::atomic<bool> done{false};
    std::atomic<bool> returned_null{false};
    std::thread       t([&] {
        Frame* f = ch.next_frame(sub);
        returned_null.store(f == nullptr);
        if (f) f->release();
        done.store(true);
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    EXPECT_FALSE(done.load());

    ch.unsubscribe(sub);
    t.join();
    EXPECT_TRUE(done.load());
    EXPECT_TRUE(returned_null.load());
}

TEST(FrameChannelTest, MultipleConsumersAllSeeSamePublish) {
    FramePool    pool(kMono64x64, 4);
    FrameChannel ch;
    auto         a = ch.subscribe();
    auto         b = ch.subscribe();

    publish_seq(ch, pool, 7);

    Frame* fa = ch.next_frame(a);
    Frame* fb = ch.next_frame(b);
    ASSERT_NE(fa, nullptr);
    ASSERT_NE(fb, nullptr);
    EXPECT_EQ(fa->sequence(), 7u);
    EXPECT_EQ(fb->sequence(), 7u);
    EXPECT_EQ(fa, fb) << "both consumers should see the same Frame*";
    fa->release();
    fb->release();
}

// ---------------------------------------------------------------------------
// Latest-only / frame skipping
// ---------------------------------------------------------------------------

TEST(FrameChannelTest, SlowConsumerSkipsIntermediateFrames) {
    FramePool    pool(kMono64x64, 8);
    FrameChannel ch;
    auto         sub = ch.subscribe();

    // Publish 5 frames in a tight loop, before the consumer reads any.
    for (uint64_t i = 1; i <= 5; ++i) {
        publish_seq(ch, pool, i);
    }

    // The consumer should immediately get the LATEST frame, not the oldest.
    Frame* f = ch.next_frame(sub);
    ASSERT_NE(f, nullptr);
    EXPECT_EQ(f->sequence(), 5u) << "latest-only: oldest frames should be skipped";
    f->release();
}

TEST(FrameChannelTest, RecycledFramesReturnToPool) {
    FramePool    pool(kMono64x64, 3);
    FrameChannel ch;
    auto         sub = ch.subscribe();

    // Publish many more frames than the pool capacity. If frames did not
    // recycle, this would exhaust the pool.
    for (uint64_t i = 1; i <= 50; ++i) {
        publish_seq(ch, pool, i);
        Frame* f = ch.next_frame(sub);
        ASSERT_NE(f, nullptr) << "iteration " << i;
        EXPECT_GE(f->sequence(), i);  // could skip ahead in degenerate cases, but won't here
        f->release();
    }
}

// ---------------------------------------------------------------------------
// Concurrency / stress
// ---------------------------------------------------------------------------

TEST(FrameChannelTest, StressNoTornReadsAndMonotonicSequences) {
    constexpr uint32_t kPoolCapacity   = 8;
    constexpr int      kProduceCount   = 2000;
    constexpr int      kNumConsumers   = 3;

    FramePool    pool(kMono64x64, kPoolCapacity);
    FrameChannel ch;

    std::vector<FrameChannel::SubscriberHandle> subs;
    subs.reserve(kNumConsumers);
    for (int i = 0; i < kNumConsumers; ++i) subs.push_back(ch.subscribe());

    std::atomic<bool>             stop_consumers{false};
    std::atomic<int>              torn_reads{0};
    std::atomic<int>              regressions{0};
    std::vector<std::thread>      consumer_threads;
    std::vector<std::atomic<int>> received(kNumConsumers);
    for (auto& r : received) r.store(0);

    for (int ci = 0; ci < kNumConsumers; ++ci) {
        consumer_threads.emplace_back([&, ci] {
            uint64_t last_seq = 0;
            // Each consumer has a different processing delay.
            const auto delay = std::chrono::microseconds(50 * (ci + 1));
            while (!stop_consumers.load(std::memory_order_relaxed)) {
                Frame* f = ch.next_frame(subs[ci]);
                if (!f) break;
                const uint64_t seq    = f->sequence();
                const uint8_t  marker = read_marker(f);
                if (marker != static_cast<uint8_t>(seq)) {
                    torn_reads.fetch_add(1, std::memory_order_relaxed);
                }
                if (seq <= last_seq) {
                    regressions.fetch_add(1, std::memory_order_relaxed);
                }
                last_seq = seq;
                received[ci].fetch_add(1, std::memory_order_relaxed);
                f->release();
                std::this_thread::sleep_for(delay);
            }
        });
    }

    std::thread producer([&] {
        for (int i = 1; i <= kProduceCount; ++i) {
            Frame* f = pool.acquire();
            if (!f) {
                // Pool momentarily exhausted under contention - back off and retry.
                std::this_thread::yield();
                --i;
                continue;
            }
            f->set_sequence(static_cast<uint64_t>(i));
            f->set_host_capture_ns(Clock::now_ns());
            stamp_frame(f, static_cast<uint8_t>(i));
            ch.publish(f);
        }
    });

    producer.join();
    // Give consumers a moment to drain the last frame.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    stop_consumers.store(true);
    for (auto& sub : subs) ch.unsubscribe(sub);
    for (auto& t : consumer_threads) t.join();

    EXPECT_EQ(torn_reads.load(), 0) << "marker byte should match frame.sequence()";
    EXPECT_EQ(regressions.load(), 0) << "sequence numbers must be monotonic per consumer";
    for (int ci = 0; ci < kNumConsumers; ++ci) {
        EXPECT_GT(received[ci].load(), 0) << "consumer " << ci << " saw zero frames";
        EXPECT_LE(received[ci].load(), kProduceCount + 1);
    }
}

}  // namespace gw
