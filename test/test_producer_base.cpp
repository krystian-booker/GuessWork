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
#include "posest/ProducerBase.h"

using namespace std::chrono_literals;

namespace {

// Trivial producer: emits `max` frames back-to-back, then signals EOS.
// Does NOT supply its own timestamp, so it exercises the fallback path.
class CountingProducer final : public posest::ProducerBase {
public:
    CountingProducer(std::string id, std::uint64_t max)
        : posest::ProducerBase(std::move(id)), max_(max) {}

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
    prod.start();

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
    prod.start();
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
    prod.start();
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
    prod.start();
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
    prod.start();

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
    prod.start();
    prod.stop();
    // Re-entrancy: a second start/stop must work without leaking the worker.
    CountingProducer prod2("cam2", 100);
    prod2.start();
    prod2.stop();
}

TEST(ProducerBase, FallbackStampsWithSteadyClockNowWhenSubclassOmits) {
    const auto before = std::chrono::steady_clock::now();
    CountingProducer prod("cam", 50);  // does NOT set out_capture_time
    auto cons = std::make_shared<RecordingConsumer>("c");
    prod.addConsumer(cons);

    cons->start();
    prod.start();
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
