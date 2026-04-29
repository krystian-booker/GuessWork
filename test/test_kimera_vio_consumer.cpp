#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "posest/CameraTriggerCache.h"
#include "posest/Frame.h"
#include "posest/MeasurementBus.h"
#include "posest/MeasurementTypes.h"
#include "posest/ProducerBase.h"
#include "posest/Timestamp.h"
#include "posest/ToFSampleCache.h"
#include "posest/vio/FakeVioBackend.h"
#include "posest/vio/KimeraVioConfig.h"
#include "posest/vio/KimeraVioConsumer.h"

using namespace std::chrono_literals;

namespace {

// 1:1 time converter — Teensy microseconds map directly to a
// steady_clock time point. Production binds this to
// TeensyService::timestampFromTeensyTime; in tests we bypass the
// time-sync filter entirely.
posest::Timestamp identityConverter(std::uint64_t teensy_time_us,
                                    posest::Timestamp /*fallback*/) {
    return posest::Timestamp{std::chrono::microseconds(teensy_time_us)};
}

posest::FramePtr makeFrame(std::uint64_t teensy_time_us,
                           std::uint64_t sequence,
                           std::optional<double> ground_distance_m) {
    auto f = std::make_shared<posest::Frame>();
    f->capture_time = posest::Timestamp{std::chrono::microseconds(teensy_time_us)};
    f->sequence = sequence;
    f->camera_id = "downward";
    f->image = cv::Mat::zeros(64, 64, CV_8UC1);
    f->teensy_time_us = teensy_time_us;
    f->ground_distance_m = ground_distance_m;
    return f;
}

// Poll-based drain so the test never blocks on take() when the
// consumer happens not to publish (e.g. tracking_ok=false).
std::vector<posest::VioMeasurement> drainVio(
    posest::MeasurementBus& out_bus,
    std::size_t expected,
    std::chrono::milliseconds budget = 500ms) {
    std::vector<posest::VioMeasurement> got;
    const auto deadline = std::chrono::steady_clock::now() + budget;
    while (got.size() < expected &&
           std::chrono::steady_clock::now() < deadline) {
        if (out_bus.size() == 0) {
            std::this_thread::sleep_for(2ms);
            continue;
        }
        auto m = out_bus.take();
        if (!m.has_value()) break;
        if (auto* v = std::get_if<posest::VioMeasurement>(&*m)) {
            got.push_back(*v);
        }
    }
    return got;
}

// Deliver a frame and give the consumer's drop-oldest mailbox time to
// observe it before another deliver() can replace it. ConsumerBase's
// LatestFrameSlot is intentionally drop-oldest (see CLAUDE.md) so this
// pacing is required at test boundaries — production producers run at
// >=30 Hz so the natural inter-frame gap covers this in practice.
void deliverPaced(posest::vio::KimeraVioConsumer& consumer,
                  posest::FramePtr frame) {
    consumer.deliver(std::move(frame));
    std::this_thread::sleep_for(20ms);
}

// Poll until `pred()` returns true or the deadline expires. Returns
// true on success. Use this for assertions that depend on the
// consumer's two background threads (frame worker A, IMU drainer B)
// having processed an event from the test thread — a fixed sleep is
// fragile when the binary's scheduler timing shifts (e.g. between
// debug and release builds, or after a dependency change). The
// 200 ms ceiling is generous: in practice the drainer + worker
// complete within a single millisecond on a quiet machine.
template <typename Pred>
bool waitFor(Pred pred,
             std::chrono::milliseconds budget = 200ms) {
    const auto deadline = std::chrono::steady_clock::now() + budget;
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) return true;
        std::this_thread::sleep_for(1ms);
    }
    return pred();
}

}  // namespace

// First frame must NOT emit a VioMeasurement (no T_prev). Subsequent
// frames produce a measurement per frame.
TEST(KimeraVioConsumer, SkipsFirstOutputAndPublishesSubsequent) {
    posest::MeasurementBus imu_bus(64);
    posest::MeasurementBus out_bus(64);

    auto backend = std::make_unique<posest::vio::FakeVioBackend>();
    auto* backend_ptr = backend.get();
    posest::vio::KimeraVioConsumer consumer(
        "vio", imu_bus, out_bus, &identityConverter,
        std::move(backend), {});
    consumer.start();

    // First frame — should be consumed, push to backend, but produce
    // no published measurement (skipped because no prior pose).
    deliverPaced(consumer, makeFrame(1'000, 0, 0.05));
    // Second frame — should publish a delta against frame 1.
    deliverPaced(consumer, makeFrame(2'000, 1, 0.05));

    auto got = drainVio(out_bus, 1);
    ASSERT_EQ(got.size(), 1u);
    EXPECT_TRUE(got[0].tracking_ok);
    EXPECT_EQ(got[0].camera_id, "vio");

    // The fake backend advances pose by 0.05 m forward + 0.01 rad yaw
    // per frame. Between t=1 and t=2 that's exactly one delta.
    EXPECT_NEAR(got[0].relative_motion.translation_m.x, 0.05, 1e-9);
    EXPECT_NEAR(got[0].relative_motion.rotation_rpy_rad.z, 0.01, 1e-9);

    // Stats sanity.
    auto s = consumer.stats();
    EXPECT_EQ(s.outputs_skipped_first, 1u);
    EXPECT_EQ(s.outputs_published, 1u);
    (void)backend_ptr;

    consumer.stop();
}

// tracking_ok=false outputs are dropped — no measurement on the bus,
// stats are bumped.
TEST(KimeraVioConsumer, DropsOutputWhenTrackingNotOk) {
    posest::MeasurementBus imu_bus(64);
    posest::MeasurementBus out_bus(64);

    posest::vio::FakeVioBackend::Config bcfg;
    bcfg.tracking_ok_after_n_frames = 10;  // Frames 1-10 not ok.
    auto backend = std::make_unique<posest::vio::FakeVioBackend>(bcfg);
    posest::vio::KimeraVioConsumer consumer(
        "vio", imu_bus, out_bus, &identityConverter,
        std::move(backend), {});
    consumer.start();

    for (std::uint64_t i = 0; i < 5; ++i) {
        deliverPaced(consumer, makeFrame(1'000 * (i + 1), i, 0.05));
    }
    std::this_thread::sleep_for(20ms);
    EXPECT_EQ(out_bus.size(), 0u);

    auto s = consumer.stats();
    EXPECT_EQ(s.outputs_published, 0u);
    EXPECT_GT(s.outputs_skipped_no_tracking, 0u);

    consumer.stop();
}

// Airborne frames inflate covariance diagonals to the configured cap.
TEST(KimeraVioConsumer, InflatesCovarianceWhenAirborne) {
    posest::MeasurementBus imu_bus(64);
    posest::MeasurementBus out_bus(64);

    posest::vio::KimeraVioConfig cfg;
    cfg.inflation_factor = 1.0e3;
    cfg.inflation_cap = 1.0e6;
    cfg.covariance_strategy =
        posest::vio::CovarianceStrategy::kAbsolute;  // pass-through

    auto backend = std::make_unique<posest::vio::FakeVioBackend>();
    posest::vio::KimeraVioConsumer consumer(
        "vio", imu_bus, out_bus, &identityConverter,
        std::move(backend), cfg);
    consumer.start();

    // Frame 0: grounded (sets last_kimera_pose; no publish).
    deliverPaced(consumer, makeFrame(1'000, 0, 0.05));
    // Frame 1: airborne — distance 0.30 m > 0.15 m threshold.
    deliverPaced(consumer, makeFrame(2'000, 1, 0.30));

    auto got = drainVio(out_bus, 1);
    ASSERT_EQ(got.size(), 1u);

    // FakeVioBackend emits sigma_translation_m = 0.01, so diagonal var
    // = 1e-4. Inflated by 1e3 → 0.1, well below the cap. Translation
    // diagonals are at indices 21, 28, 35 (rows/cols 3,4,5).
    EXPECT_NEAR(got[0].covariance[21], 1e-4 * 1e3, 1e-9);
    EXPECT_NEAR(got[0].covariance[28], 1e-4 * 1e3, 1e-9);
    EXPECT_NEAR(got[0].covariance[35], 1e-4 * 1e3, 1e-9);

    auto s = consumer.stats();
    EXPECT_EQ(s.outputs_inflated_airborne, 1u);

    consumer.stop();
}

// IMU samples published to the input bus reach the backend before
// shutdown.
TEST(KimeraVioConsumer, ForwardsImuSamplesToBackend) {
    posest::MeasurementBus imu_bus(64);
    posest::MeasurementBus out_bus(64);

    auto backend = std::make_unique<posest::vio::FakeVioBackend>();
    auto* backend_ptr = backend.get();
    posest::vio::KimeraVioConsumer consumer(
        "vio", imu_bus, out_bus, &identityConverter,
        std::move(backend), {});
    consumer.start();

    // Push 5 IMU samples whose teensy times bracket the next frame's.
    // The consumer compares each sample's teensy_time_us directly
    // against frame.teensy_time_us — no host-clock round trip.
    for (std::uint64_t i = 0; i < 5; ++i) {
        posest::ImuSample s;
        const std::uint64_t t = 500 + 100 * i;
        s.timestamp = posest::Timestamp{std::chrono::microseconds(t)};
        s.teensy_time_us = t;
        s.accel_mps2 = {0.0, 0.0, 9.81};
        imu_bus.publish(s);
    }

    // Wait for the drainer (thread B) to lift everything off the bus
    // into the consumer's imu_buffer_ BEFORE delivering the frame.
    // The frame worker drains imu_buffer_ once at the top of
    // process(); if the drainer hasn't finished yet, the worker only
    // sees the samples that happened to be in the buffer at the
    // instant it acquired the lock — and the test would race on a
    // partial count. In production this can't happen because IMU
    // arrives continuously; in a single-batch test we have to gate
    // explicitly. See deliverPaced for the worker-side equivalent.
    ASSERT_TRUE(waitFor([&] { return imu_bus.size() == 0; }));

    // Frame at t=2000 us; the consumer should drain everything <=2000us.
    deliverPaced(consumer, makeFrame(2'000, 0, 0.05));

    EXPECT_TRUE(waitFor(
        [&] { return backend_ptr->imuPushCount() >= 5u; }));
    EXPECT_GE(backend_ptr->framePushCount(), 1u);

    consumer.stop();
}

// Regression test for the wire-level fix: the backend must receive each
// sample's own teensy_time_us, not the frame's. Before the fix the
// consumer stamped every IMU push with frame.teensy_time_us because
// ImuSample dropped the field on the bus.
TEST(KimeraVioConsumer, ImuTimestampPropagatedToBackend) {
    posest::MeasurementBus imu_bus(64);
    posest::MeasurementBus out_bus(64);

    auto backend = std::make_unique<posest::vio::FakeVioBackend>();
    auto* backend_ptr = backend.get();
    posest::vio::KimeraVioConsumer consumer(
        "vio", imu_bus, out_bus, &identityConverter,
        std::move(backend), {});
    consumer.start();

    posest::ImuSample s;
    s.timestamp = posest::Timestamp{std::chrono::microseconds(1234)};
    s.teensy_time_us = 1234;
    s.accel_mps2 = {0.0, 0.0, 9.81};
    imu_bus.publish(s);

    // Same drainer-then-deliver gate as ForwardsImuSamplesToBackend;
    // see comment there.
    ASSERT_TRUE(waitFor([&] { return imu_bus.size() == 0; }));

    // Frame at t=5000 us — distinct from the sample's 1234, so the
    // assertion below would fail under the old work-around that
    // stamped IMU pushes with frame_teensy_us.
    deliverPaced(consumer, makeFrame(5'000, 0, 0.05));

    ASSERT_TRUE(waitFor(
        [&] { return backend_ptr->imuPushCount() >= 1u; }));
    EXPECT_EQ(backend_ptr->lastImuTeensyTimeUs(), 1234u);

    consumer.stop();
}

// applyConfig swaps live fields on the next process() iteration.
// inflation_factor is the cleanest dial to observe — kAbsolute strategy
// makes the measurement covariance equal to the backend's covariance,
// scaled by inflation_factor when airborne.
TEST(KimeraVioConsumer, ApplyConfigSwapsLiveFieldsOnNextFrame) {
    posest::MeasurementBus imu_bus(64);
    posest::MeasurementBus out_bus(64);

    posest::vio::KimeraVioConfig cfg;
    cfg.inflation_factor = 1.0e3;
    cfg.inflation_cap = 1.0e6;
    cfg.covariance_strategy =
        posest::vio::CovarianceStrategy::kAbsolute;

    auto backend = std::make_unique<posest::vio::FakeVioBackend>();
    posest::vio::KimeraVioConsumer consumer(
        "vio", imu_bus, out_bus, &identityConverter,
        std::move(backend), cfg);
    consumer.start();

    // Frame 0: grounded — sets last_kimera_pose (no publish).
    deliverPaced(consumer, makeFrame(1'000, 0, 0.05));
    // Frame 1: airborne — published with default inflation_factor.
    deliverPaced(consumer, makeFrame(2'000, 1, 0.30));

    auto first = drainVio(out_bus, 1);
    ASSERT_EQ(first.size(), 1u);
    EXPECT_NEAR(first[0].covariance[21], 1e-4 * 1e3, 1e-9);

    // Live edit from a side thread — same shape the WebService callback
    // uses on the daemon's web thread.
    posest::vio::KimeraVioConfig updated = cfg;
    updated.inflation_factor = 2.0e3;
    std::thread t(
        [&consumer, updated]() { consumer.applyConfig(updated); });
    t.join();

    // Frame 2: airborne again. drainPendingConfig swaps inflation_factor
    // before update(), so the published covariance reflects 2.0e3.
    deliverPaced(consumer, makeFrame(3'000, 2, 0.30));
    auto second = drainVio(out_bus, 1);
    ASSERT_EQ(second.size(), 1u);
    EXPECT_NEAR(second[0].covariance[21], 1e-4 * 2e3, 1e-9);

    auto s = consumer.stats();
    EXPECT_EQ(s.config_reloads_applied, 1u);
    EXPECT_EQ(s.config_reloads_structural_skipped, 0u);

    consumer.stop();
}

// Structural fields (param_dir, imu_buffer_capacity, camera_id) revert
// to the running config and bump config_reloads_structural_skipped.
// Live fields in the same applyConfig call still take effect.
TEST(KimeraVioConsumer, ApplyConfigRevertsStructuralFields) {
    posest::MeasurementBus imu_bus(64);
    posest::MeasurementBus out_bus(64);

    posest::vio::KimeraVioConfig cfg;
    cfg.param_dir = "/original";
    cfg.imu_buffer_capacity = 1024;
    cfg.camera_id = "vio";
    cfg.covariance_strategy =
        posest::vio::CovarianceStrategy::kAbsolute;
    cfg.inflation_factor = 1.0e3;

    auto backend = std::make_unique<posest::vio::FakeVioBackend>();
    posest::vio::KimeraVioConsumer consumer(
        "vio", imu_bus, out_bus, &identityConverter,
        std::move(backend), cfg);
    consumer.start();

    // Drive one frame so the consumer's worker has actually started
    // processing — keeps the structural-revert assertion deterministic.
    deliverPaced(consumer, makeFrame(1'000, 0, 0.05));

    posest::vio::KimeraVioConfig updated = cfg;
    updated.param_dir = "/changed";              // structural — reverted
    updated.imu_buffer_capacity = 4096;          // structural — reverted
    updated.camera_id = "different";             // structural — reverted
    updated.inflation_factor = 5.0e3;            // live — applied
    consumer.applyConfig(updated);

    // Next frame triggers drainPendingConfig.
    deliverPaced(consumer, makeFrame(2'000, 1, 0.30));
    auto got = drainVio(out_bus, 1);
    ASSERT_EQ(got.size(), 1u);
    // The live inflation_factor change took effect.
    EXPECT_NEAR(got[0].covariance[21], 1e-4 * 5e3, 1e-9);
    // The reverted camera_id stamp survived.
    EXPECT_EQ(got[0].camera_id, "vio");

    auto s = consumer.stats();
    EXPECT_EQ(s.config_reloads_applied, 1u);
    EXPECT_EQ(s.config_reloads_structural_skipped, 1u);

    consumer.stop();
}

#include "posest/pipelines/PipelineStats.h"
#include "posest/runtime/IVisionPipeline.h"

// CLAHE preprocessing is off by default — the consumer must not call any
// OpenCV image processing and the new counters stay at zero. This pins
// the default-behaviour contract: turning Phase 2 on must be explicit.
TEST(KimeraVioConsumer, ClaheOffByDefaultLeavesCountersAtZero) {
    posest::MeasurementBus imu_bus(64);
    posest::MeasurementBus out_bus(64);

    auto backend = std::make_unique<posest::vio::FakeVioBackend>();
    posest::vio::KimeraVioConsumer consumer(
        "vio", imu_bus, out_bus, &identityConverter,
        std::move(backend), {});
    consumer.start();

    deliverPaced(consumer, makeFrame(1'000, 0, 0.05));
    deliverPaced(consumer, makeFrame(2'000, 1, 0.05));
    auto got = drainVio(out_bus, 1);
    ASSERT_EQ(got.size(), 1u);

    auto s = consumer.stats();
    EXPECT_EQ(s.frames_clahe_applied, 0u);
    EXPECT_EQ(s.frames_clahe_skipped_low_texture, 0u);

    consumer.stop();
}

// preprocess_clahe + ir_led_enabled both on, no variance gate: every
// frame goes through CLAHE.
TEST(KimeraVioConsumer, ClaheAppliedWhenAllGatesOn) {
    posest::MeasurementBus imu_bus(64);
    posest::MeasurementBus out_bus(64);

    posest::vio::KimeraVioConfig cfg;
    cfg.preprocess_clahe = true;
    cfg.ir_led_enabled = true;
    cfg.clahe_min_variance_laplacian = 0.0;  // gate disabled

    auto backend = std::make_unique<posest::vio::FakeVioBackend>();
    posest::vio::KimeraVioConsumer consumer(
        "vio", imu_bus, out_bus, &identityConverter,
        std::move(backend), cfg);
    consumer.start();

    deliverPaced(consumer, makeFrame(1'000, 0, 0.05));
    deliverPaced(consumer, makeFrame(2'000, 1, 0.05));
    auto got = drainVio(out_bus, 1);
    ASSERT_EQ(got.size(), 1u);

    auto s = consumer.stats();
    EXPECT_EQ(s.frames_clahe_applied, 2u);
    EXPECT_EQ(s.frames_clahe_skipped_low_texture, 0u);

    consumer.stop();
}

// preprocess_clahe is on but ir_led_enabled is false → CLAHE skipped
// entirely. The counters stay at zero (skip is "not gated", not "gated
// off by texture") because the IR check short-circuits before CLAHE
// even considers the frame.
TEST(KimeraVioConsumer, ClaheSkippedWhenIrLedDisabled) {
    posest::MeasurementBus imu_bus(64);
    posest::MeasurementBus out_bus(64);

    posest::vio::KimeraVioConfig cfg;
    cfg.preprocess_clahe = true;
    cfg.ir_led_enabled = false;  // gate off
    cfg.clahe_min_variance_laplacian = 0.0;

    auto backend = std::make_unique<posest::vio::FakeVioBackend>();
    posest::vio::KimeraVioConsumer consumer(
        "vio", imu_bus, out_bus, &identityConverter,
        std::move(backend), cfg);
    consumer.start();

    deliverPaced(consumer, makeFrame(1'000, 0, 0.05));
    deliverPaced(consumer, makeFrame(2'000, 1, 0.05));
    auto got = drainVio(out_bus, 1);
    ASSERT_EQ(got.size(), 1u);

    auto s = consumer.stats();
    EXPECT_EQ(s.frames_clahe_applied, 0u);
    EXPECT_EQ(s.frames_clahe_skipped_low_texture, 0u);

    consumer.stop();
}

// makeFrame produces an all-zeros 64x64 image. variance-of-Laplacian on
// a uniform image is exactly 0, so any positive floor suppresses CLAHE
// and bumps frames_clahe_skipped_low_texture. A textured frame would
// pass the same gate; the test exercises the suppression path.
TEST(KimeraVioConsumer, ClaheTextureGateSuppressesFlatFrames) {
    posest::MeasurementBus imu_bus(64);
    posest::MeasurementBus out_bus(64);

    posest::vio::KimeraVioConfig cfg;
    cfg.preprocess_clahe = true;
    cfg.ir_led_enabled = true;
    cfg.clahe_min_variance_laplacian = 1.0;  // any positive floor works

    auto backend = std::make_unique<posest::vio::FakeVioBackend>();
    posest::vio::KimeraVioConsumer consumer(
        "vio", imu_bus, out_bus, &identityConverter,
        std::move(backend), cfg);
    consumer.start();

    deliverPaced(consumer, makeFrame(1'000, 0, 0.05));
    deliverPaced(consumer, makeFrame(2'000, 1, 0.05));
    auto got = drainVio(out_bus, 1);
    ASSERT_EQ(got.size(), 1u);

    auto s = consumer.stats();
    EXPECT_EQ(s.frames_clahe_applied, 0u);
    EXPECT_EQ(s.frames_clahe_skipped_low_texture, 2u);

    consumer.stop();
}

// Backend's landmark_count rides through to the consumer's stats and
// the EMA seeds from the first sample (no zero-drift bias). The default
// FakeVioBackend reports 20 landmarks per output, well above the
// default landmark_count_floor of 8, so the floor counter stays put.
TEST(KimeraVioConsumer, LandmarkCountTelemetryAndFloorCounter) {
    posest::MeasurementBus imu_bus(64);
    posest::MeasurementBus out_bus(64);

    posest::vio::KimeraVioConfig cfg;
    cfg.landmark_count_floor = 8;

    posest::vio::FakeVioBackend::Config bcfg;
    bcfg.landmark_count = 4;  // Below floor → counter ticks.

    auto backend = std::make_unique<posest::vio::FakeVioBackend>(bcfg);
    posest::vio::KimeraVioConsumer consumer(
        "vio", imu_bus, out_bus, &identityConverter,
        std::move(backend), cfg);
    consumer.start();

    // Three frames → first is skipped (no T_prev), the other two
    // publish and increment outputs_below_landmark_floor.
    deliverPaced(consumer, makeFrame(1'000, 0, 0.05));
    deliverPaced(consumer, makeFrame(2'000, 1, 0.05));
    deliverPaced(consumer, makeFrame(3'000, 2, 0.05));
    auto got = drainVio(out_bus, 2);
    ASSERT_EQ(got.size(), 2u);

    auto s = consumer.stats();
    EXPECT_EQ(s.outputs_published, 2u);
    EXPECT_EQ(s.last_landmark_count, 4);
    // EMA seeds from the first sample, then an alpha=0.1 update; with
    // a constant 4 input the average stays at 4.
    EXPECT_NEAR(s.landmark_count_avg, 4.0, 1e-9);
    EXPECT_EQ(s.outputs_below_landmark_floor, 2u);

    consumer.stop();
}

// pipelineStats() snapshots the consumer's KimeraVioStats and stamps
// pipeline_id from ConsumerBase::id(). last_output_age_ms is the
// watchdog signal: nullopt before the first publish, set afterwards.
// Surfaced via DaemonHealth::vio_pipelines once wired in Daemon.cpp.
TEST(KimeraVioConsumer, PipelineStatsSurfacesKimeraVioVariantAndStaleness) {
    posest::MeasurementBus imu_bus(64);
    posest::MeasurementBus out_bus(64);

    auto backend = std::make_unique<posest::vio::FakeVioBackend>();
    posest::vio::KimeraVioConsumer consumer(
        "vio_pipeline_stats_test", imu_bus, out_bus,
        &identityConverter, std::move(backend), {});

    // Before any output: variant alternative is KimeraVioStats but
    // last_output_age_ms is empty.
    {
        const auto v = consumer.pipelineStats();
        const auto* s = std::get_if<posest::vio::KimeraVioStats>(&v);
        ASSERT_NE(s, nullptr);
        EXPECT_EQ(s->pipeline_id, "vio_pipeline_stats_test");
        EXPECT_FALSE(s->last_output_age_ms.has_value());
    }

    consumer.start();
    deliverPaced(consumer, makeFrame(1'000, 0, 0.05));
    deliverPaced(consumer, makeFrame(2'000, 1, 0.05));
    auto got = drainVio(out_bus, 1);
    ASSERT_EQ(got.size(), 1u);

    // After a publish: last_output_age_ms is set and small.
    {
        const auto v = consumer.pipelineStats();
        const auto* s = std::get_if<posest::vio::KimeraVioStats>(&v);
        ASSERT_NE(s, nullptr);
        ASSERT_TRUE(s->last_output_age_ms.has_value());
        EXPECT_GE(*s->last_output_age_ms, 0);
        // 1 second is huge for a polling-only test on a quiet machine
        // and tight enough to catch a clock-domain regression.
        EXPECT_LT(*s->last_output_age_ms, 1000);
        EXPECT_EQ(s->outputs_published, 1u);
    }

    consumer.stop();
}

// One-shot test producer: emits exactly one frame at a caller-supplied
// capture_time, then returns false to signal EndOfStream. Defined inline
// because the existing MockProducer paces itself with sleep_until and
// can't guarantee a single-frame run that aligns with a pre-armed
// CameraTriggerCache entry within the 50 ms match window.
class OneShotProducer : public posest::ProducerBase {
public:
    OneShotProducer(std::string id, std::chrono::steady_clock::time_point t)
        : posest::ProducerBase(std::move(id)), capture_time_(t) {}
    ~OneShotProducer() override { stop(); }

protected:
    bool captureOne(
        cv::Mat& out,
        std::optional<std::chrono::steady_clock::time_point>& ts) override {
        if (emitted_) {
            return false;  // EndOfStream — runLoop joins and exits cleanly.
        }
        out = cv::Mat::zeros(64, 64, CV_8UC1);
        ts = capture_time_;
        emitted_ = true;
        return true;
    }

private:
    std::chrono::steady_clock::time_point capture_time_;
    bool emitted_{false};
};

// Pin number is arbitrary — the test owns the pin↔camera mapping and
// only one camera is involved.
constexpr std::int32_t kTestTeensyPin = 17;
constexpr std::uint32_t kTestTriggerSequence = 42;
constexpr std::uint64_t kTestTeensyTimeUs = 7'654'321;

// End-to-end coverage of ProducerBase's trigger+ToF cache join into
// Frame::ground_distance_m and the consumer's airborne_tracker. This is
// the integration glue that the daemon depends on (Daemon.cpp wires
// CameraProducer::setToFSampleCache + setTriggerCache) but no other test
// exercises top-to-bottom — test_tof_sample_cache.cpp is isolated and
// the rest of test_kimera_vio_consumer.cpp injects ground_distance_m
// synthetically into Frame.
TEST(KimeraVioConsumer, ProducerToFCacheReachesAirborneTracker) {
    posest::MeasurementBus imu_bus(64);
    posest::MeasurementBus out_bus(64);

    auto backend = std::make_unique<posest::vio::FakeVioBackend>();
    auto consumer = std::make_shared<posest::vio::KimeraVioConsumer>(
        "vio", imu_bus, out_bus, &identityConverter,
        std::move(backend), posest::vio::KimeraVioConfig{});

    // Pre-arm both caches with matching entries: a trigger pulse at
    // capture_time t0 (the producer emits one frame at this exact instant),
    // and a ToF reading carrying the same trigger_sequence at an airborne
    // distance so the AirborneTracker's state machine has something
    // observable to react to.
    const auto t0 = std::chrono::steady_clock::now();

    auto trigger_cache = std::make_shared<posest::CameraTriggerCache>(
        std::unordered_map<std::int32_t, std::string>{{kTestTeensyPin,
                                                       "downward"}});
    posest::CameraTriggerEvent ev{};
    ev.timestamp = t0;
    ev.teensy_time_us = kTestTeensyTimeUs;
    ev.pin = kTestTeensyPin;
    ev.trigger_sequence = kTestTriggerSequence;
    ev.status_flags = 0;  // Synced — recordEvent gates on this.
    trigger_cache->recordEvent(ev);

    auto tof_cache = std::make_shared<posest::ToFSampleCache>("downward");
    posest::ToFSample sample{};
    sample.timestamp = t0;
    sample.teensy_time_us = kTestTeensyTimeUs;
    sample.trigger_sequence = kTestTriggerSequence;
    sample.distance_m = 0.30;  // Above 0.15 m default → airborne.
    tof_cache->recordSample(sample);

    OneShotProducer producer("downward", t0);
    producer.setTriggerCache(trigger_cache);
    producer.setToFSampleCache(tof_cache);
    producer.addConsumer(consumer);

    consumer->start();
    ASSERT_EQ(producer.start(), posest::ProducerState::Running);

    // Wait for the consumer's frame worker to drain its mailbox. We
    // observe via stats() rather than the FakeVioBackend's framePushCount
    // because the consumer increments frames_pushed only after the
    // ground_distance_m → AirborneTracker → recordFrameSnapshot chain
    // has run, which is exactly what this test asserts.
    EXPECT_TRUE(waitFor([&] {
        return consumer->stats().frames_pushed >= 1u;
    }));

    auto s = consumer->stats();
    EXPECT_EQ(s.frames_pushed, 1u);
    // The cache hit is the load-bearing assertion: ground_distance_missing
    // counts AirborneTracker::update(nullopt) calls. Zero proves the
    // ToF cache lookup in ProducerBase::runLoop succeeded and the value
    // reached the consumer's airborne tracker on this frame.
    EXPECT_EQ(s.ground_distance_missing, 0u);

    producer.stop();
    consumer->stop();
}

// Counterpart: when no ToF sample is recorded for the active
// trigger_sequence, ProducerBase emits a frame with ground_distance_m
// unset and the AirborneTracker's missingCount() bumps. This confirms
// the missing-distance path is wired symmetrically — a misconfigured
// ToFSampleCache is observable from outside via stats().
TEST(KimeraVioConsumer, MissingToFSampleSurfacesAsMissingCount) {
    posest::MeasurementBus imu_bus(64);
    posest::MeasurementBus out_bus(64);

    auto backend = std::make_unique<posest::vio::FakeVioBackend>();
    auto consumer = std::make_shared<posest::vio::KimeraVioConsumer>(
        "vio", imu_bus, out_bus, &identityConverter,
        std::move(backend), posest::vio::KimeraVioConfig{});

    const auto t0 = std::chrono::steady_clock::now();

    auto trigger_cache = std::make_shared<posest::CameraTriggerCache>(
        std::unordered_map<std::int32_t, std::string>{{kTestTeensyPin,
                                                       "downward"}});
    posest::CameraTriggerEvent ev{};
    ev.timestamp = t0;
    ev.teensy_time_us = kTestTeensyTimeUs;
    ev.pin = kTestTeensyPin;
    ev.trigger_sequence = kTestTriggerSequence;
    ev.status_flags = 0;
    trigger_cache->recordEvent(ev);

    // ToF cache is constructed but no sample recorded — the lookup misses.
    auto tof_cache = std::make_shared<posest::ToFSampleCache>("downward");

    OneShotProducer producer("downward", t0);
    producer.setTriggerCache(trigger_cache);
    producer.setToFSampleCache(tof_cache);
    producer.addConsumer(consumer);

    consumer->start();
    ASSERT_EQ(producer.start(), posest::ProducerState::Running);

    EXPECT_TRUE(waitFor([&] {
        return consumer->stats().frames_pushed >= 1u;
    }));

    auto s = consumer->stats();
    EXPECT_EQ(s.frames_pushed, 1u);
    EXPECT_GE(s.ground_distance_missing, 1u);

    producer.stop();
    consumer->stop();
}

// Phase 3.2: mono_translation_scale_factor is YAML-driven but live —
// the consumer cycles backend->stop()/start() in place when it changes
// so Kimera re-parses the (already-repainted) BackendParams.yaml. The
// daemon's WebService callback emits the new YAML before invoking
// applyConfig; this unit test substitutes FakeVioBackend, so we
// observe the cycle via the config_reloads_backend_restarted counter
// and the post-restart "first frame" skip behaviour (Kimera's world
// frame resets after a stop/start, so the first output after restart
// has no prior pose to delta against).
TEST(KimeraVioConsumer, ApplyConfigRestartsBackendOnMonoTranslationScaleFactorChange) {
    posest::MeasurementBus imu_bus(64);
    posest::MeasurementBus out_bus(64);

    posest::vio::KimeraVioConfig cfg;
    cfg.mono_translation_scale_factor = 0.1;

    auto backend = std::make_unique<posest::vio::FakeVioBackend>();
    posest::vio::KimeraVioConsumer consumer(
        "vio", imu_bus, out_bus, &identityConverter,
        std::move(backend), cfg);
    consumer.start();

    // Frame 0 + 1 establish prior pose and produce one published
    // measurement (the second frame's delta against the first).
    deliverPaced(consumer, makeFrame(1'000, 0, 0.05));
    deliverPaced(consumer, makeFrame(2'000, 1, 0.05));
    auto pre = drainVio(out_bus, 1);
    ASSERT_EQ(pre.size(), 1u);

    posest::vio::KimeraVioConfig updated = cfg;
    updated.mono_translation_scale_factor = 0.5;
    consumer.applyConfig(updated);

    // Frame 2 triggers drainPendingConfig which performs the in-place
    // restart. last_kimera_pose_ resets, so this frame is the new
    // "first" — pushed to backend, no measurement published.
    deliverPaced(consumer, makeFrame(3'000, 2, 0.05));
    auto restart_first = drainVio(out_bus, 1, 50ms);
    EXPECT_EQ(restart_first.size(), 0u);

    // Frame 3 produces the first post-restart published measurement.
    deliverPaced(consumer, makeFrame(4'000, 3, 0.05));
    auto post = drainVio(out_bus, 1);
    ASSERT_EQ(post.size(), 1u);

    auto s = consumer.stats();
    EXPECT_EQ(s.config_reloads_applied, 1u);
    EXPECT_EQ(s.config_reloads_structural_skipped, 0u);
    EXPECT_EQ(s.config_reloads_backend_restarted, 1u);
    // The restart counts an extra outputs_skipped_first beyond the
    // initial bootstrap skip.
    EXPECT_GE(s.outputs_skipped_first, 2u);

    consumer.stop();
}

// ToF round-trip: Frame::ground_distance_m must propagate onto the
// emitted VioMeasurement via the snapshot lookup, so downstream factor
// graphs have a metric anchor for monocular scale recovery without
// re-querying the ToF cache on a different time cursor. The
// outputs_with_ground_distance counter is the operator's signal that
// the camera↔ToF time alignment reaches the output sink.
TEST(KimeraVioConsumer, GroundDistancePropagatesToVioMeasurement) {
    posest::MeasurementBus imu_bus(64);
    posest::MeasurementBus out_bus(64);

    auto backend = std::make_unique<posest::vio::FakeVioBackend>();
    posest::vio::KimeraVioConsumer consumer(
        "vio", imu_bus, out_bus, &identityConverter,
        std::move(backend), {});
    consumer.start();

    // Two grounded frames at nominal carpet-mount distance — well below
    // the 0.15 m airborne threshold so neither covariance inflation nor
    // a missing-count bump are at play. Frame 0 sets last_kimera_pose
    // (no publish); frame 1 publishes a delta against frame 0 and the
    // VioMeasurement must carry frame 1's ground_distance_m.
    deliverPaced(consumer, makeFrame(1'000, 0, 0.10));
    deliverPaced(consumer, makeFrame(2'000, 1, 0.12));

    auto got = drainVio(out_bus, 1);
    ASSERT_EQ(got.size(), 1u);
    ASSERT_TRUE(got[0].ground_distance_m.has_value());
    EXPECT_NEAR(*got[0].ground_distance_m, 0.12, 1e-9);

    auto s = consumer.stats();
    EXPECT_EQ(s.outputs_published, 1u);
    EXPECT_EQ(s.outputs_with_ground_distance, 1u);

    consumer.stop();
}

// Counterpart: when the source Frame has no ToF reading, the emitted
// VioMeasurement carries an empty ground_distance_m and the
// outputs_with_ground_distance counter does NOT increment. Pins the
// no-ToF path so a regression that always stamps "0.0" instead of
// nullopt is caught — downstream code distinguishes "ToF says zero"
// from "ToF unavailable".
TEST(KimeraVioConsumer, MissingGroundDistanceLeavesVioMeasurementEmpty) {
    posest::MeasurementBus imu_bus(64);
    posest::MeasurementBus out_bus(64);

    auto backend = std::make_unique<posest::vio::FakeVioBackend>();
    posest::vio::KimeraVioConsumer consumer(
        "vio", imu_bus, out_bus, &identityConverter,
        std::move(backend), {});
    consumer.start();

    // makeFrame's third arg threads through to Frame::ground_distance_m;
    // pass std::nullopt to simulate a ToFSampleCache miss for both frames.
    deliverPaced(consumer, makeFrame(1'000, 0, std::nullopt));
    deliverPaced(consumer, makeFrame(2'000, 1, std::nullopt));

    auto got = drainVio(out_bus, 1);
    ASSERT_EQ(got.size(), 1u);
    EXPECT_FALSE(got[0].ground_distance_m.has_value());

    auto s = consumer.stats();
    EXPECT_EQ(s.outputs_published, 1u);
    EXPECT_EQ(s.outputs_with_ground_distance, 0u);

    consumer.stop();
}

// Frames without a Teensy timestamp can't be aligned to IMU and must
// be dropped with the right counter bumped.
TEST(KimeraVioConsumer, DropsFramesMissingTeensyTimestamp) {
    posest::MeasurementBus imu_bus(64);
    posest::MeasurementBus out_bus(64);

    auto backend = std::make_unique<posest::vio::FakeVioBackend>();
    posest::vio::KimeraVioConsumer consumer(
        "vio", imu_bus, out_bus, &identityConverter,
        std::move(backend), {});
    consumer.start();

    auto f = std::make_shared<posest::Frame>();
    f->capture_time = std::chrono::steady_clock::now();
    f->image = cv::Mat::zeros(64, 64, CV_8UC1);
    // Note: teensy_time_us deliberately left unset.
    consumer.deliver(f);
    std::this_thread::sleep_for(50ms);

    auto s = consumer.stats();
    EXPECT_EQ(s.frames_pushed, 0u);
    EXPECT_GT(s.frames_dropped_backpressure, 0u);

    consumer.stop();
}
