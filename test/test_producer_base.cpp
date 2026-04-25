#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "posest/ConsumerBase.h"
#include "posest/Frame.h"
#include "posest/MockProducer.h"
#include "posest/ProducerBase.h"

using namespace std::chrono_literals;

namespace {

// Trivial producer: emits `max` frames back-to-back, then signals EOS.
// Does NOT supply its own timestamp, so it exercises the fallback path.
class CountingProducer final : public posest::ProducerBase {
public:
    CountingProducer(std::string id, std::uint64_t max)
        : posest::ProducerBase(std::move(id)), max_(max) {}
    ~CountingProducer() override { stop(); }

protected:
    bool captureOne(
        cv::Mat& out,
        std::optional<std::chrono::steady_clock::time_point>& /*out_capture_time*/) override {
        if (emitted_ >= max_) return false;
        out.create(4, 4, CV_8UC1);
        out.setTo(cv::Scalar(0));
        ++emitted_;
        return true;
    }

private:
    std::uint64_t max_;
    std::uint64_t emitted_{0};
};

// Producer that stamps every frame with a fixed, subclass-chosen time_point.
// Used to verify the base passes the subclass-supplied timestamp through
// unchanged rather than overwriting it with its own now() call.
class StampedProducer final : public posest::ProducerBase {
public:
    StampedProducer(std::string id,
                    std::uint64_t max,
                    std::chrono::steady_clock::time_point stamp)
        : posest::ProducerBase(std::move(id)), max_(max), stamp_(stamp) {}
    ~StampedProducer() override { stop(); }

protected:
    bool captureOne(
        cv::Mat& out,
        std::optional<std::chrono::steady_clock::time_point>& out_capture_time) override {
        if (emitted_ >= max_) return false;
        out.create(4, 4, CV_8UC1);
        out.setTo(cv::Scalar(0));
        ++emitted_;
        out_capture_time = stamp_;
        return true;
    }

private:
    std::uint64_t max_;
    std::chrono::steady_clock::time_point stamp_;
    std::uint64_t emitted_{0};
};

struct Observation {
    std::uint64_t sequence;
    std::chrono::steady_clock::time_point capture_time;
    std::string camera_id;
};

class RecordingConsumer final : public posest::ConsumerBase {
public:
    explicit RecordingConsumer(std::string id) : posest::ConsumerBase(std::move(id)) {}
    ~RecordingConsumer() override { stop(); }

    std::vector<Observation> take() const {
        std::lock_guard<std::mutex> g(mu_);
        return obs_;
    }

protected:
    void process(const posest::Frame& f) override {
        std::lock_guard<std::mutex> g(mu_);
        obs_.push_back({f.sequence, f.capture_time, f.camera_id});
    }

private:
    mutable std::mutex mu_;
    std::vector<Observation> obs_;
};

}  // namespace

TEST(ProducerBase, TimestampsAreMonotonicAndNonzero) {
    CountingProducer prod("cam", 200);
    auto cons = std::make_shared<RecordingConsumer>("c");
    prod.addConsumer(cons);

    cons->start();
    ASSERT_EQ(prod.start(), posest::ProducerState::Running);

    // Wait for producer to exhaust its 200 frames; consumer sees at least 1
    // because the final put is never dropped.
    while (prod.producedCount() < 200) std::this_thread::sleep_for(1ms);
    prod.stop();
    std::this_thread::sleep_for(5ms);
    cons->stop();

    const auto obs = cons->take();
    ASSERT_FALSE(obs.empty());
    for (const auto& o : obs) {
        EXPECT_NE(o.capture_time, std::chrono::steady_clock::time_point{});
    }
    for (size_t i = 1; i < obs.size(); ++i) {
        EXPECT_LE(obs[i - 1].capture_time, obs[i].capture_time);
    }
}

TEST(ProducerBase, FanOutDeliversToAllConsumers) {
    CountingProducer prod("cam", 500);
    auto a = std::make_shared<RecordingConsumer>("a");
    auto b = std::make_shared<RecordingConsumer>("b");
    auto c = std::make_shared<RecordingConsumer>("c");
    prod.addConsumer(a);
    prod.addConsumer(b);
    prod.addConsumer(c);

    a->start();
    b->start();
    c->start();
    ASSERT_EQ(prod.start(), posest::ProducerState::Running);
    while (prod.producedCount() < 500) std::this_thread::sleep_for(1ms);
    prod.stop();
    std::this_thread::sleep_for(5ms);
    a->stop();
    b->stop();
    c->stop();

    for (auto* cc : {a.get(), b.get(), c.get()}) {
        const auto obs = cc->take();
        ASSERT_FALSE(obs.empty());
        EXPECT_EQ(obs.front().camera_id, "cam");
    }
}

TEST(ProducerBase, SequenceAssignedByProducerIsContiguous) {
    // The consumer may see gaps (latest-only slot), but the sequence values
    // it DOES observe must form a strictly increasing sub-sequence of
    // 0..N-1 without duplicates.
    CountingProducer prod("cam", 300);
    auto cons = std::make_shared<RecordingConsumer>("c");
    prod.addConsumer(cons);

    cons->start();
    ASSERT_EQ(prod.start(), posest::ProducerState::Running);
    while (prod.producedCount() < 300) std::this_thread::sleep_for(1ms);
    prod.stop();
    std::this_thread::sleep_for(5ms);
    cons->stop();

    const auto obs = cons->take();
    ASSERT_FALSE(obs.empty());
    for (size_t i = 1; i < obs.size(); ++i) {
        EXPECT_GT(obs[i].sequence, obs[i - 1].sequence);
    }
    EXPECT_LT(obs.back().sequence, 300u);
}

TEST(ProducerBase, SubclassSuppliedTimestampPassesThrough) {
    // Anchor the stamp in the past so we can unambiguously distinguish it
    // from any fallback stamp (which would be steady_clock::now()).
    const auto anchor =
        std::chrono::steady_clock::now() - std::chrono::seconds(10);

    StampedProducer prod("cam", 50, anchor);
    auto cons = std::make_shared<RecordingConsumer>("c");
    prod.addConsumer(cons);

    cons->start();
    ASSERT_EQ(prod.start(), posest::ProducerState::Running);
    while (prod.producedCount() < 50) std::this_thread::sleep_for(1ms);
    prod.stop();
    std::this_thread::sleep_for(5ms);
    cons->stop();

    const auto obs = cons->take();
    ASSERT_FALSE(obs.empty());
    for (const auto& o : obs) {
        EXPECT_EQ(o.capture_time, anchor)
            << "base must not overwrite subclass-supplied timestamp";
    }
}

// A producer whose first captureOne() succeeds, then the second throws. Used
// to verify ProducerBase's runLoop does not propagate exceptions.
class ThrowingProducer final : public posest::ProducerBase {
public:
    explicit ThrowingProducer(std::string id)
        : posest::ProducerBase(std::move(id)) {}
    ~ThrowingProducer() override { stop(); }

protected:
    bool captureOne(
        cv::Mat& out,
        std::optional<std::chrono::steady_clock::time_point>&) override {
        if (call_++ == 0) {
            out.create(4, 4, CV_8UC1);
            out.setTo(cv::Scalar(0));
            return true;
        }
        throw std::runtime_error("intentional capture failure");
    }

private:
    std::uint64_t call_{0};
};

TEST(ProducerBase, ThrowingCaptureOneLogsAndExitsCleanly) {
    ThrowingProducer prod("cam");
    auto cons = std::make_shared<RecordingConsumer>("c");
    prod.addConsumer(cons);
    cons->start();
    ASSERT_EQ(prod.start(), posest::ProducerState::Running);

    // The throw on the second capture should terminate the worker. Stopping
    // (which joins the worker) must complete without deadlocking.
    const auto t0 = std::chrono::steady_clock::now();
    while (prod.producedCount() < 1 &&
           std::chrono::steady_clock::now() - t0 < std::chrono::seconds(1)) {
        std::this_thread::sleep_for(1ms);
    }
    prod.stop();
    std::this_thread::sleep_for(5ms);
    cons->stop();

    EXPECT_GE(prod.producedCount(), 1u);
}

TEST(ProducerBase, StartStopRestartIsClean) {
    CountingProducer prod("cam", 100);
    ASSERT_EQ(prod.start(), posest::ProducerState::Running);
    prod.stop();
    // Re-entrancy: a second start/stop must work without leaking the worker.
    CountingProducer prod2("cam2", 100);
    ASSERT_EQ(prod2.start(), posest::ProducerState::Running);
    prod2.stop();
}

TEST(ProducerBase, FallbackStampsWithSteadyClockNowWhenSubclassOmits) {
    const auto before = std::chrono::steady_clock::now();
    CountingProducer prod("cam", 50);  // does NOT set out_capture_time
    auto cons = std::make_shared<RecordingConsumer>("c");
    prod.addConsumer(cons);

    cons->start();
    ASSERT_EQ(prod.start(), posest::ProducerState::Running);
    while (prod.producedCount() < 50) std::this_thread::sleep_for(1ms);
    prod.stop();
    std::this_thread::sleep_for(5ms);
    cons->stop();
    const auto after = std::chrono::steady_clock::now();

    const auto obs = cons->take();
    ASSERT_FALSE(obs.empty());
    for (const auto& o : obs) {
        EXPECT_NE(o.capture_time, std::chrono::steady_clock::time_point{});
        EXPECT_GE(o.capture_time, before);
        EXPECT_LE(o.capture_time, after);
    }
}

TEST(ProducerBase, HotAddConsumerSeesOnlyFramesAfterAttach) {
    // Attach a consumer after start() has already begun fanning out. Its first
    // observed sequence must be strictly greater than the producer's frame
    // count at the moment it was attached.
    posest::mock::MockProducer prod(
        posest::mock::MockProducerConfig{.id = "cam", .target_fps = 240.0});
    auto pre = std::make_shared<RecordingConsumer>("pre");
    prod.addConsumer(pre);

    pre->start();
    ASSERT_EQ(prod.start(), posest::ProducerState::Running);
    std::this_thread::sleep_for(50ms);

    const auto produced_at_attach = prod.producedCount();
    auto late = std::make_shared<RecordingConsumer>("late");
    late->start();
    prod.addConsumer(late);

    std::this_thread::sleep_for(100ms);
    prod.stop();
    std::this_thread::sleep_for(10ms);
    pre->stop();
    late->stop();

    const auto late_obs = late->take();
    ASSERT_FALSE(late_obs.empty()) << "hot-attached consumer received nothing";
    EXPECT_GE(late_obs.front().sequence, produced_at_attach)
        << "hot-attached consumer must not see frames captured before attach";
}

TEST(ProducerBase, HotRemoveConsumerStopsReceiving) {
    posest::mock::MockProducer prod(
        posest::mock::MockProducerConfig{.id = "cam", .target_fps = 240.0});
    auto a = std::make_shared<RecordingConsumer>("a");
    auto b = std::make_shared<RecordingConsumer>("b");
    prod.addConsumer(a);
    prod.addConsumer(b);

    a->start();
    b->start();
    ASSERT_EQ(prod.start(), posest::ProducerState::Running);
    std::this_thread::sleep_for(50ms);

    const bool removed = prod.removeConsumer(a);
    EXPECT_TRUE(removed);

    // Drain window: at most one frame may have been mid-fan-out when remove
    // returned, and one frame may sit in a's mailbox waiting for its worker.
    std::this_thread::sleep_for(50ms);
    const std::size_t a_count_after_drain = a->take().size();

    std::this_thread::sleep_for(200ms);
    prod.stop();
    std::this_thread::sleep_for(10ms);
    a->stop();
    b->stop();

    const auto a_final = a->take();
    const auto b_final = b->take();
    EXPECT_EQ(a_final.size(), a_count_after_drain)
        << "removed consumer kept receiving frames";
    EXPECT_GT(b_final.size(), a_count_after_drain + 30u)
        << "remaining consumer should have kept receiving";
}

TEST(ProducerBase, RemoveConsumerNotPresentReturnsFalse) {
    CountingProducer prod("cam", 1);
    auto a = std::make_shared<RecordingConsumer>("a");
    auto stranger = std::make_shared<RecordingConsumer>("stranger");
    prod.addConsumer(a);

    EXPECT_FALSE(prod.removeConsumer(stranger));
    EXPECT_TRUE(prod.removeConsumer(a));
    EXPECT_FALSE(prod.removeConsumer(a)) << "double-remove must report false";
}

TEST(ProducerBase, AddRemoveDuringCaptureDoesNotDeadlock) {
    posest::mock::MockProducer prod(
        posest::mock::MockProducerConfig{.id = "cam", .target_fps = 240.0});
    auto primary = std::make_shared<RecordingConsumer>("primary");
    prod.addConsumer(primary);

    primary->start();
    const auto t0 = std::chrono::steady_clock::now();
    ASSERT_EQ(prod.start(), posest::ProducerState::Running);

    std::atomic<bool> stop_churn{false};
    std::thread churn([&] {
        std::vector<std::shared_ptr<RecordingConsumer>> live;
        std::uint64_t i = 0;
        while (!stop_churn.load(std::memory_order_acquire)) {
            // Toggle between attach and detach.
            if ((i & 1u) == 0) {
                auto c = std::make_shared<RecordingConsumer>(
                    "transient_" + std::to_string(i));
                c->start();
                prod.addConsumer(c);
                live.push_back(c);
            } else if (!live.empty()) {
                auto c = live.back();
                live.pop_back();
                EXPECT_TRUE(prod.removeConsumer(c));
                c->stop();
            }
            ++i;
            std::this_thread::sleep_for(2ms);
        }
        for (auto& c : live) {
            EXPECT_TRUE(prod.removeConsumer(c));
            c->stop();
        }
    });

    std::this_thread::sleep_for(300ms);
    stop_churn.store(true, std::memory_order_release);
    churn.join();

    prod.stop();
    const auto elapsed = std::chrono::steady_clock::now() - t0;
    std::this_thread::sleep_for(10ms);
    primary->stop();

    const double seconds = std::chrono::duration<double>(elapsed).count();
    const double primary_fps =
        static_cast<double>(primary->take().size()) / seconds;
    // Concurrent add/remove must not throttle the producer below ~80% of target.
    EXPECT_GE(primary_fps, 240.0 * 0.5)
        << "subscriber churn starved the primary consumer";
}

TEST(ProducerBase, StateRunningWhileCapturing) {
    posest::mock::MockProducer prod(
        posest::mock::MockProducerConfig{.id = "cam", .target_fps = 240.0});
    EXPECT_EQ(prod.state(), posest::ProducerState::Idle);
    ASSERT_EQ(prod.start(), posest::ProducerState::Running);
    std::this_thread::sleep_for(20ms);
    EXPECT_EQ(prod.state(), posest::ProducerState::Running);
    prod.stop();
    EXPECT_EQ(prod.state(), posest::ProducerState::Idle);
}

TEST(ProducerBase, StateBecomesEndOfStreamWhenCaptureOneReturnsFalse) {
    // CountingProducer emits N frames then returns false. Verify that the
    // base transitions to EndOfStream on its own — without anyone calling
    // stop() — and that the public state reflects it.
    CountingProducer prod("cam", 5);
    auto cons = std::make_shared<RecordingConsumer>("c");
    prod.addConsumer(cons);

    cons->start();
    ASSERT_EQ(prod.start(), posest::ProducerState::Running);

    // Wait for the loop to exhaust its budget AND notice the false return.
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (prod.state() == posest::ProducerState::Running &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(1ms);
    }
    EXPECT_EQ(prod.state(), posest::ProducerState::EndOfStream);
    EXPECT_EQ(prod.producedCount(), 5u);

    // start() on a terminal-state producer must report the terminal state
    // back to the caller and remain a no-op.
    EXPECT_EQ(prod.start(), posest::ProducerState::EndOfStream);

    cons->stop();
}

TEST(ProducerBase, StateBecomesFailedOnException) {
    ThrowingProducer prod("cam");
    auto cons = std::make_shared<RecordingConsumer>("c");
    prod.addConsumer(cons);

    cons->start();
    ASSERT_EQ(prod.start(), posest::ProducerState::Running);

    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (prod.state() == posest::ProducerState::Running &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(1ms);
    }
    EXPECT_EQ(prod.state(), posest::ProducerState::Failed);
    EXPECT_EQ(prod.start(), posest::ProducerState::Failed)
        << "start() on a Failed producer must remain a no-op";

    cons->stop();
}

TEST(ProducerBase, StopJoinsCleanlyAfterEndOfStream) {
    // After the worker has exited on its own, stop() must still join
    // (the worker is joinable but already terminated) and reset to Idle
    // within a tight bound.
    CountingProducer prod("cam", 10);
    ASSERT_EQ(prod.start(), posest::ProducerState::Running);

    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (prod.state() == posest::ProducerState::Running &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(1ms);
    }
    ASSERT_EQ(prod.state(), posest::ProducerState::EndOfStream);

    const auto t0 = std::chrono::steady_clock::now();
    prod.stop();
    const auto elapsed = std::chrono::steady_clock::now() - t0;

    EXPECT_LT(elapsed, 100ms) << "stop() must not hang after EndOfStream";
    EXPECT_EQ(prod.state(), posest::ProducerState::Idle);
}
