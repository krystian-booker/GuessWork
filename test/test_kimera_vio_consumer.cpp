#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "posest/Frame.h"
#include "posest/MeasurementBus.h"
#include "posest/MeasurementTypes.h"
#include "posest/Timestamp.h"
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
    // Frame at t=2000 us; the consumer should drain everything <=2000us.
    deliverPaced(consumer, makeFrame(2'000, 0, 0.05));

    EXPECT_GE(backend_ptr->imuPushCount(), 5u);
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

    // Frame at t=5000 us — distinct from the sample's 1234, so the
    // assertion below would fail under the old work-around that
    // stamped IMU pushes with frame_teensy_us.
    deliverPaced(consumer, makeFrame(5'000, 0, 0.05));

    ASSERT_GE(backend_ptr->imuPushCount(), 1u);
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
