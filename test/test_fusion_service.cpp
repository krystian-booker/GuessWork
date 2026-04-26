#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <vector>

#include <gtest/gtest.h>

#include "posest/MeasurementBus.h"
#include "posest/fusion/FusionService.h"

using namespace std::chrono_literals;

namespace {

class RecordingSink final : public posest::fusion::IFusionOutputSink {
public:
    void publish(posest::FusedPoseEstimate estimate) override {
        std::lock_guard<std::mutex> g(mu);
        estimates.push_back(estimate);
    }

    std::mutex mu;
    std::vector<posest::FusedPoseEstimate> estimates;
};

bool covarianceIsFinite(const posest::FusedPoseEstimate& estimate) {
    for (double value : estimate.covariance) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    return true;
}

posest::fusion::FusionConfig makeTagFusionConfig() {
    return posest::fusion::FusionConfig{};
}

posest::AprilTagObservation makePriorObservation(
    const posest::Pose3d& field_to_robot,
    double sigma_translation = 0.1,
    double sigma_rotation = 0.05) {
    posest::AprilTagObservation observation;
    observation.camera_id = "cam0";
    observation.capture_time = std::chrono::steady_clock::now();
    observation.field_to_robot = field_to_robot;
    observation.solved_tag_count = 2;
    observation.reprojection_rms_px = 0.3;
    const double var_t = sigma_translation * sigma_translation;
    const double var_r = sigma_rotation * sigma_rotation;
    observation.covariance.fill(0.0);
    observation.covariance[0 * 6 + 0] = var_r;
    observation.covariance[1 * 6 + 1] = var_r;
    observation.covariance[2 * 6 + 2] = var_r;
    observation.covariance[3 * 6 + 3] = var_t;
    observation.covariance[4 * 6 + 4] = var_t;
    observation.covariance[5 * 6 + 5] = var_t;
    return observation;
}

posest::ChassisSpeedsSample makeChassis(
    posest::Timestamp timestamp,
    double vx_mps = 0.0,
    double vy_mps = 0.0,
    double omega_radps = 0.0) {
    posest::ChassisSpeedsSample sample;
    sample.timestamp = timestamp;
    sample.vx_mps = vx_mps;
    sample.vy_mps = vy_mps;
    sample.omega_radps = omega_radps;
    return sample;
}

posest::VioMeasurement makeVio(
    posest::Timestamp timestamp,
    bool tracking_ok,
    posest::Pose3d relative_motion = {},
    double sigma_translation = 0.05,
    double sigma_rotation = 0.02) {
    posest::VioMeasurement m;
    m.camera_id = "vio0";
    m.timestamp = timestamp;
    m.relative_motion = relative_motion;
    m.tracking_ok = tracking_ok;
    m.covariance.fill(0.0);
    const double var_t = sigma_translation * sigma_translation;
    const double var_r = sigma_rotation * sigma_rotation;
    m.covariance[0 * 6 + 0] = var_r;
    m.covariance[1 * 6 + 1] = var_r;
    m.covariance[2 * 6 + 2] = var_r;
    m.covariance[3 * 6 + 3] = var_t;
    m.covariance[4 * 6 + 4] = var_t;
    m.covariance[5 * 6 + 5] = var_t;
    return m;
}

}  // namespace

TEST(FusionService, ProcessesChassisSpeedsWithGtsamAndPublishesEstimate) {
    posest::MeasurementBus bus(8);
    posest::fusion::FusionService fusion(bus);
    auto sink = std::make_shared<RecordingSink>();
    fusion.addOutputSink(sink);

    fusion.start();

    const auto t0 = std::chrono::steady_clock::now();
    // Bootstrap at the origin.
    ASSERT_TRUE(bus.publish(makeChassis(t0)));
    // 5.0 m/s for 300 ms → x = 1.5 m. Δt stays under max_chassis_dt_seconds.
    ASSERT_TRUE(bus.publish(makeChassis(t0 + 300ms, 5.0)));

    std::this_thread::sleep_for(100ms);
    fusion.stop();

    const auto stats = fusion.stats();
    EXPECT_EQ(stats.measurements_processed, 2u);
    EXPECT_EQ(stats.stale_measurements, 0u);

    const auto latest = fusion.latestEstimate();
    ASSERT_TRUE(latest.has_value());
    EXPECT_NEAR(latest->field_to_robot.x_m, 1.5, 1e-6);
    EXPECT_NEAR(latest->field_to_robot.y_m, 0.0, 1e-6);
    EXPECT_TRUE(covarianceIsFinite(*latest));
    EXPECT_NE(latest->status_flags & posest::fusion::kFusionStatusVisionUnavailable, 0u);
    EXPECT_EQ(latest->status_flags & posest::fusion::kFusionStatusShockInflated, 0u);

    std::lock_guard<std::mutex> g(sink->mu);
    EXPECT_GE(sink->estimates.size(), 2u);
}

TEST(FusionService, UsesAggregatedAprilTagPoseAsSinglePrior) {
    posest::MeasurementBus bus(8);
    posest::fusion::FusionService fusion(bus, makeTagFusionConfig());
    fusion.start();

    auto observation = makePriorObservation(posest::Pose3d{
        .translation_m = {3.0, 0.0, 0.0},
        .rotation_rpy_rad = {0.0, 0.0, 0.0},
    });
    ASSERT_TRUE(bus.publish(observation));

    std::this_thread::sleep_for(50ms);
    fusion.stop();

    const auto latest = fusion.latestEstimate();
    ASSERT_TRUE(latest.has_value());
    EXPECT_NEAR(latest->field_to_robot.x_m, 3.0, 1e-3);
    EXPECT_NEAR(latest->field_to_robot.y_m, 0.0, 1e-3);
    EXPECT_EQ(latest->status_flags & posest::fusion::kFusionStatusVisionUnavailable, 0u);
    EXPECT_TRUE(covarianceIsFinite(*latest));
}

TEST(FusionService, RejectsNonPositiveDefiniteCovariance) {
    posest::MeasurementBus bus(8);
    posest::fusion::FusionService fusion(bus, makeTagFusionConfig());
    fusion.start();

    posest::AprilTagObservation observation;
    observation.camera_id = "cam0";
    observation.capture_time = std::chrono::steady_clock::now();
    observation.field_to_robot = posest::Pose3d{};
    observation.solved_tag_count = 2;
    observation.covariance.fill(0.0);
    ASSERT_TRUE(bus.publish(observation));

    std::this_thread::sleep_for(50ms);
    fusion.stop();

    const auto latest = fusion.latestEstimate();
    if (latest.has_value()) {
        EXPECT_NE(
            latest->status_flags & posest::fusion::kFusionStatusOptimizerError,
            0u);
    }
}

TEST(FusionService, MarksVisionUnavailableForEmptyTagObservationAfterInitialization) {
    posest::MeasurementBus bus(8);
    posest::fusion::FusionService fusion(bus, makeTagFusionConfig());
    fusion.start();

    const auto now = std::chrono::steady_clock::now();
    ASSERT_TRUE(bus.publish(makeChassis(now)));
    // 1.0 m/s for 100 ms → x = 0.1 m.
    ASSERT_TRUE(bus.publish(makeChassis(now + 100ms, 1.0)));

    posest::AprilTagObservation observation;
    observation.camera_id = "cam0";
    observation.capture_time = now + 200ms;
    ASSERT_TRUE(bus.publish(observation));

    std::this_thread::sleep_for(100ms);
    fusion.stop();

    const auto latest = fusion.latestEstimate();
    ASSERT_TRUE(latest.has_value());
    EXPECT_NEAR(latest->field_to_robot.x_m, 0.1, 1e-6);
    EXPECT_NE(latest->status_flags & posest::fusion::kFusionStatusVisionUnavailable, 0u);
}

TEST(FusionService, IgnoresUnsupportedMeasurements) {
    posest::MeasurementBus bus(8);
    posest::fusion::FusionService fusion(bus);
    fusion.start();

    // CameraTriggerEvent and ToFSample are not consumed by FusionService;
    // they should be dropped without affecting any counters.
    posest::CameraTriggerEvent trigger;
    trigger.timestamp = std::chrono::steady_clock::now();
    trigger.pin = 2;
    ASSERT_TRUE(bus.publish(trigger));

    posest::ToFSample tof;
    tof.timestamp = std::chrono::steady_clock::now();
    ASSERT_TRUE(bus.publish(tof));

    std::this_thread::sleep_for(20ms);
    fusion.stop();

    const auto stats = fusion.stats();
    EXPECT_EQ(stats.measurements_processed, 0u);
    EXPECT_EQ(stats.stale_measurements, 0u);
    EXPECT_FALSE(fusion.latestEstimate().has_value());
}

TEST(FusionService, RejectsStaleChassisSpeedsMeasurements) {
    posest::MeasurementBus bus(8);
    posest::fusion::FusionService fusion(bus);
    fusion.start();

    const auto now = std::chrono::steady_clock::now();
    ASSERT_TRUE(bus.publish(makeChassis(now)));
    ASSERT_TRUE(bus.publish(makeChassis(now - 1ms)));

    std::this_thread::sleep_for(50ms);
    fusion.stop();

    const auto stats = fusion.stats();
    EXPECT_EQ(stats.measurements_processed, 1u);
    EXPECT_EQ(stats.stale_measurements, 1u);
}

TEST(FusionService, InflatesCovarianceOnImuShock) {
    posest::MeasurementBus bus(8);
    posest::fusion::FusionService fusion(bus);
    fusion.start();

    const auto t0 = std::chrono::steady_clock::now();
    // Bootstrap.
    ASSERT_TRUE(bus.publish(makeChassis(t0)));

    // ImuSample with |a - g| ≈ 70 m/s², well above the 50 m/s² threshold.
    posest::ImuSample shock;
    shock.timestamp = t0 + 30ms;
    shock.accel_mps2 = {0.0, 0.0, 80.0};
    ASSERT_TRUE(bus.publish(shock));

    // ChassisSpeeds within the 50 ms window so the shock counts.
    ASSERT_TRUE(bus.publish(makeChassis(t0 + 40ms, 1.0)));

    std::this_thread::sleep_for(100ms);
    fusion.stop();

    const auto latest = fusion.latestEstimate();
    ASSERT_TRUE(latest.has_value());
    EXPECT_NE(latest->status_flags & posest::fusion::kFusionStatusShockInflated, 0u);
}

TEST(FusionService, InflatesCovarianceOnFreefall) {
    posest::MeasurementBus bus(8);
    posest::fusion::FusionService fusion(bus);
    fusion.start();

    const auto t0 = std::chrono::steady_clock::now();
    ASSERT_TRUE(bus.publish(makeChassis(t0)));

    // |a| ≈ 0.5 m/s² — below the 3 m/s² free-fall threshold.
    posest::ImuSample airborne;
    airborne.timestamp = t0 + 30ms;
    airborne.accel_mps2 = {0.0, 0.0, 0.5};
    ASSERT_TRUE(bus.publish(airborne));

    ASSERT_TRUE(bus.publish(makeChassis(t0 + 40ms, 1.0)));

    std::this_thread::sleep_for(100ms);
    fusion.stop();

    const auto latest = fusion.latestEstimate();
    ASSERT_TRUE(latest.has_value());
    EXPECT_NE(latest->status_flags & posest::fusion::kFusionStatusShockInflated, 0u);
}

TEST(FusionService, DoesNotInflateCovarianceBelowThreshold) {
    posest::MeasurementBus bus(8);
    posest::fusion::FusionService fusion(bus);
    fusion.start();

    const auto t0 = std::chrono::steady_clock::now();
    ASSERT_TRUE(bus.publish(makeChassis(t0)));

    // Normal driving: |a| ≈ 11 m/s², |a - g| ≈ 1.2 m/s². Neither branch fires.
    posest::ImuSample steady;
    steady.timestamp = t0 + 30ms;
    steady.accel_mps2 = {0.0, 0.0, 11.0};
    ASSERT_TRUE(bus.publish(steady));

    ASSERT_TRUE(bus.publish(makeChassis(t0 + 40ms, 1.0)));

    std::this_thread::sleep_for(100ms);
    fusion.stop();

    const auto latest = fusion.latestEstimate();
    ASSERT_TRUE(latest.has_value());
    EXPECT_EQ(latest->status_flags & posest::fusion::kFusionStatusShockInflated, 0u);
}

TEST(FusionService, SkipsFactorOnLargeChassisGap) {
    posest::MeasurementBus bus(8);
    posest::fusion::FusionService fusion(bus);
    fusion.start();

    const auto t0 = std::chrono::steady_clock::now();
    ASSERT_TRUE(bus.publish(makeChassis(t0)));
    // Δt = 600 ms exceeds max_chassis_dt_seconds (default 0.5 s).
    ASSERT_TRUE(bus.publish(makeChassis(t0 + 600ms, 5.0)));

    std::this_thread::sleep_for(100ms);
    fusion.stop();

    const auto latest = fusion.latestEstimate();
    ASSERT_TRUE(latest.has_value());
    EXPECT_NE(latest->status_flags & posest::fusion::kFusionStatusChassisGap, 0u);
    // Pose stayed at origin because the BetweenFactor was skipped.
    EXPECT_NEAR(latest->field_to_robot.x_m, 0.0, 1e-6);
}

TEST(FusionConfig, BuildFusionConfigIgnoresRuntimeConfig) {
    posest::runtime::RuntimeConfig runtime_config;
    runtime_config.calibrations.push_back({
        .camera_id = "cam0",
        .version = "v1",
        .active = true,
    });
    const auto config = posest::fusion::buildFusionConfig(runtime_config);
    EXPECT_GT(config.chassis_sigmas[0], 0.0);
    EXPECT_GT(config.origin_prior_sigmas[0], 0.0);
}

// ---------- Per-type monotonicity ----------

TEST(FusionService, AcceptsAprilTagAfterChassisInTheirOwnTimeline) {
    posest::MeasurementBus bus(8);
    posest::fusion::FusionService fusion(bus);
    fusion.start();

    const auto t0 = std::chrono::steady_clock::now();
    // Bootstrap with chassis at t=10 ms, then publish an AprilTag observation
    // stamped at t=5 ms. With per-type cursors the AprilTag is accepted; with
    // a single global cursor it would have been silently dropped.
    ASSERT_TRUE(bus.publish(makeChassis(t0)));
    ASSERT_TRUE(bus.publish(makeChassis(t0 + 10ms, 1.0)));
    auto tag = makePriorObservation({{0.0, 0.0, 0.0}, {2.0, 0.5, 0.0}});
    tag.capture_time = t0 + 5ms;
    ASSERT_TRUE(bus.publish(tag));

    std::this_thread::sleep_for(100ms);
    fusion.stop();

    const auto stats = fusion.stats();
    EXPECT_EQ(stats.stale_measurements, 0u);
    // 2 chassis + 1 tag = 3 accepted measurements.
    EXPECT_EQ(stats.measurements_processed, 3u);
}

// ---------- VIO ingestion ----------

TEST(FusionService, IngestsVioMeasurementWhenTrackingOk) {
    posest::MeasurementBus bus(8);
    posest::fusion::FusionConfig config;
    config.enable_vio = true;
    posest::fusion::FusionService fusion(bus, config);
    auto sink = std::make_shared<RecordingSink>();
    fusion.addOutputSink(sink);
    fusion.start();

    const auto t0 = std::chrono::steady_clock::now();
    // Bootstrap the key chain via chassis so the VIO BetweenFactor has an
    // anchor (mirrors the production order: chassis arrives at ~100 Hz, VIO
    // at ~30 Hz, both gated on the time-sync filter).
    ASSERT_TRUE(bus.publish(makeChassis(t0)));
    posest::Pose3d delta;
    delta.translation_m = {1.0, 0.0, 0.0};
    ASSERT_TRUE(bus.publish(makeVio(t0 + 50ms, /*tracking_ok=*/true, delta)));

    std::this_thread::sleep_for(100ms);
    fusion.stop();

    const auto stats = fusion.stats();
    EXPECT_EQ(stats.measurements_vio_processed, 1u);
    EXPECT_EQ(stats.measurements_vio_skipped_no_tracking, 0u);

    const auto latest = fusion.latestEstimate();
    ASSERT_TRUE(latest.has_value());
    // Origin + 1 m relative motion → x ≈ 1.0.
    EXPECT_NEAR(latest->field_to_robot.x_m, 1.0, 1e-6);
}

TEST(FusionService, SkipsVioMeasurementWhenTrackingNotOk) {
    posest::MeasurementBus bus(8);
    posest::fusion::FusionConfig config;
    config.enable_vio = true;
    posest::fusion::FusionService fusion(bus, config);
    auto sink = std::make_shared<RecordingSink>();
    fusion.addOutputSink(sink);
    fusion.start();

    const auto t0 = std::chrono::steady_clock::now();
    ASSERT_TRUE(bus.publish(makeChassis(t0)));
    // Wait for the chassis bootstrap estimate to land in the sink before
    // capturing the baseline; otherwise the assertion races the worker.
    std::this_thread::sleep_for(50ms);
    std::size_t sink_baseline = 0;
    {
        std::lock_guard<std::mutex> g(sink->mu);
        sink_baseline = sink->estimates.size();
    }
    ASSERT_TRUE(bus.publish(makeVio(t0 + 50ms, /*tracking_ok=*/false)));

    std::this_thread::sleep_for(50ms);
    fusion.stop();

    const auto stats = fusion.stats();
    EXPECT_EQ(stats.measurements_vio_processed, 0u);
    EXPECT_EQ(stats.measurements_vio_skipped_no_tracking, 1u);
    // No new estimate published for the placeholder-style VIO sample.
    std::lock_guard<std::mutex> g(sink->mu);
    EXPECT_EQ(sink->estimates.size(), sink_baseline);
}

TEST(FusionService, SkipsVioMeasurementWhenDisabledByConfig) {
    posest::MeasurementBus bus(8);
    // enable_vio left at default (false) — the dispatch path must bail out
    // before incrementing the no-tracking counter.
    posest::fusion::FusionService fusion(bus);
    fusion.start();

    const auto t0 = std::chrono::steady_clock::now();
    ASSERT_TRUE(bus.publish(makeChassis(t0)));
    ASSERT_TRUE(bus.publish(makeVio(t0 + 50ms, /*tracking_ok=*/true)));

    std::this_thread::sleep_for(50ms);
    fusion.stop();

    const auto stats = fusion.stats();
    EXPECT_EQ(stats.measurements_vio_processed, 0u);
    EXPECT_EQ(stats.measurements_vio_skipped_no_tracking, 0u);
}
