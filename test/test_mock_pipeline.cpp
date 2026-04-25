#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>

#include "posest/MockConsumer.h"
#include "posest/MockProducer.h"

using namespace std::chrono_literals;
using posest::mock::MockConsumer;
using posest::mock::MockConsumerConfig;
using posest::mock::MockProducer;
using posest::mock::MockProducerConfig;

namespace {

// Wall-clock tests are inherently timing-sensitive. We keep durations short
// enough to keep the suite fast but wide enough to tolerate CI jitter.
void runFor(std::chrono::milliseconds d) { std::this_thread::sleep_for(d); }

}  // namespace

TEST(MockPipeline, FastConsumerSeesMostFrames) {
    MockProducer prod(MockProducerConfig{.id = "cam", .target_fps = 60.0});
    auto fast = std::make_shared<MockConsumer>(MockConsumerConfig{.id = "fast"});
    prod.addConsumer(fast);

    fast->start();
    const auto t0 = std::chrono::steady_clock::now();
    prod.start();
    runFor(1000ms);
    prod.stop();
    const auto t1 = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(20ms);
    fast->stop();

    const double seconds = std::chrono::duration<double>(t1 - t0).count();
    const auto s = fast->snapshot();
    const double observed_fps = static_cast<double>(s.frames_received) / seconds;

    // Expect at least 80% of target FPS on a loaded CI; consumer is idle so
    // almost nothing should be dropped.
    EXPECT_GE(observed_fps, 60.0 * 0.8);
    EXPECT_LE(s.sequence_gaps, s.frames_received / 10);  // ≤10% gaps
}

TEST(MockPipeline, SlowConsumerDropsButProducerIsNotBlocked) {
    // Producer at 120 FPS, consumer sleeps 50ms per frame (=20 FPS max).
    // Assert: producer rate is close to target, and consumer observes gaps.
    MockProducer prod(MockProducerConfig{.id = "cam", .target_fps = 120.0});
    auto slow = std::make_shared<MockConsumer>(MockConsumerConfig{
        .id = "slow",
        .processing_delay = 50ms,
    });
    prod.addConsumer(slow);

    slow->start();
    const auto t0 = std::chrono::steady_clock::now();
    prod.start();
    runFor(1500ms);
    prod.stop();
    const auto t1 = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(80ms);
    slow->stop();

    const double seconds = std::chrono::duration<double>(t1 - t0).count();
    const double producer_fps = static_cast<double>(prod.producedCount()) / seconds;

    EXPECT_GE(producer_fps, 120.0 * 0.8)
        << "producer was throttled by slow consumer";

    const auto s = slow->snapshot();
    ASSERT_GT(s.frames_received, 0u);
    EXPECT_LT(static_cast<double>(s.frames_received) / seconds, 40.0)
        << "slow consumer shouldn't see more than ~20 FPS";
    EXPECT_GT(s.sequence_gaps, 0u) << "expected the latest-only slot to drop frames";
}

TEST(MockPipeline, FastAndSlowConsumersAreIndependent) {
    MockProducer prod(MockProducerConfig{.id = "cam", .target_fps = 120.0});
    auto fast = std::make_shared<MockConsumer>(MockConsumerConfig{.id = "fast"});
    auto slow = std::make_shared<MockConsumer>(MockConsumerConfig{
        .id = "slow",
        .processing_delay = 30ms,
    });
    prod.addConsumer(fast);
    prod.addConsumer(slow);

    fast->start();
    slow->start();
    const auto t0 = std::chrono::steady_clock::now();
    prod.start();
    runFor(1500ms);
    prod.stop();
    const auto t1 = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(50ms);
    fast->stop();
    slow->stop();

    const double seconds = std::chrono::duration<double>(t1 - t0).count();
    const auto fs = fast->snapshot();
    const auto ss = slow->snapshot();

    EXPECT_GT(fs.frames_received, ss.frames_received * 2)
        << "fast consumer should be unaffected by slow consumer";
    const double fast_fps = static_cast<double>(fs.frames_received) / seconds;
    EXPECT_GE(fast_fps, 120.0 * 0.7);
}

TEST(MockPipeline, StopJoinsAllThreadsWithinTimeout) {
    MockProducer prod(MockProducerConfig{.id = "cam", .target_fps = 240.0});
    auto c = std::make_shared<MockConsumer>(MockConsumerConfig{.id = "c"});
    prod.addConsumer(c);

    c->start();
    prod.start();
    runFor(200ms);

    const auto t0 = std::chrono::steady_clock::now();
    prod.stop();
    c->stop();
    const auto elapsed = std::chrono::steady_clock::now() - t0;

    // stop() on producer sleeps until current captureOne returns (which may
    // be mid-sleep-until, ≤ one period ≈ 4ms at 240 FPS). stop() on consumer
    // waits for current process() call (instant here). 500ms is generous.
    EXPECT_LT(elapsed, 500ms);
}

TEST(MockPipeline, HotPreviewAttachDoesNotDisturbPrimary) {
    // The motivating use case for dynamic subscription: a slow auxiliary
    // consumer (mimicking a JPEG-encoding preview) attaches mid-stream and
    // detaches later. The primary consumer's FPS must stay close to target
    // throughout, and the auxiliary must receive at least some frames.
    MockProducer prod(MockProducerConfig{.id = "cam", .target_fps = 60.0});
    auto primary = std::make_shared<MockConsumer>(MockConsumerConfig{.id = "primary"});
    auto aux = std::make_shared<MockConsumer>(MockConsumerConfig{
        .id = "aux",
        .processing_delay = 30ms,
    });
    prod.addConsumer(primary);

    primary->start();
    aux->start();
    const auto t0 = std::chrono::steady_clock::now();
    prod.start();

    runFor(400ms);
    prod.addConsumer(aux);
    runFor(400ms);
    EXPECT_TRUE(prod.removeConsumer(aux));
    runFor(400ms);

    prod.stop();
    const auto t1 = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(50ms);
    primary->stop();
    aux->stop();

    const double seconds = std::chrono::duration<double>(t1 - t0).count();
    const auto ps = primary->snapshot();
    const auto as = aux->snapshot();

    const double primary_fps =
        static_cast<double>(ps.frames_received) / seconds;
    EXPECT_GE(primary_fps, 60.0 * 0.7)
        << "primary should have run unimpeded across attach/detach";
    EXPECT_GT(as.frames_received, 0u)
        << "auxiliary attached mid-stream should have received frames";
}
