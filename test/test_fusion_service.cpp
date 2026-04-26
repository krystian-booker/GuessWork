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

TEST(FusionService, RecordsPerStageLatencyInStats) {
    // Phase E: every chassis sample that reaches the graph should accumulate
    // one sample on graph_update_us and one on publish_us. IMU samples
    // bypass the graph and don't show up here.
    posest::MeasurementBus bus(8);
    posest::fusion::FusionService fusion(bus);
    fusion.start();

    const auto t0 = std::chrono::steady_clock::now();
    ASSERT_TRUE(bus.publish(makeChassis(t0)));
    ASSERT_TRUE(bus.publish(makeChassis(t0 + 50ms, 1.0)));
    ASSERT_TRUE(bus.publish(makeChassis(t0 + 100ms, 1.0)));

    std::this_thread::sleep_for(100ms);
    fusion.stop();

    const auto stats = fusion.stats();
    EXPECT_GE(stats.graph_update_us.count, 3u);
    EXPECT_GE(stats.publish_us.count, 3u);
    // A chassis update spends most of its time in ISAM2; publish into a no-op
    // sink (only latest_estimate_) is far cheaper. Allow a generous bound to
    // tolerate CI jitter while still catching a swap.
    EXPECT_LE(stats.publish_us.avg_us, stats.graph_update_us.avg_us + 1000);
    EXPECT_GT(stats.graph_factor_count, 0u);
    EXPECT_GT(stats.graph_variable_count, 0u);
    EXPECT_GT(stats.last_update_wall_clock_us, 0);
}

TEST(FusionService, InflatesCovarianceOnRioReportedSlipWithQuietImu) {
    // Phase B: a chassis sample with kChassisStatusSlip in status_flags must
    // inflate noise even when the IMU is reading nominal (no shock, no
    // free-fall). This catches wheel spin on low-traction surfaces, which
    // the IMU cannot see directly.
    posest::MeasurementBus bus(8);
    posest::fusion::FusionService fusion(bus);
    fusion.start();

    const auto t0 = std::chrono::steady_clock::now();
    ASSERT_TRUE(bus.publish(makeChassis(t0)));

    // Quiet IMU at gravity — neither shock nor free-fall fires.
    posest::ImuSample steady;
    steady.timestamp = t0 + 30ms;
    steady.accel_mps2 = {0.0, 0.0, 9.81};
    ASSERT_TRUE(bus.publish(steady));

    auto slipping = makeChassis(t0 + 40ms, 1.0);
    slipping.status_flags = posest::kChassisStatusSlip;
    ASSERT_TRUE(bus.publish(slipping));

    std::this_thread::sleep_for(100ms);
    fusion.stop();

    const auto latest = fusion.latestEstimate();
    ASSERT_TRUE(latest.has_value());
    EXPECT_NE(latest->status_flags & posest::fusion::kFusionStatusSlipReported, 0u);
    EXPECT_NE(latest->status_flags & posest::fusion::kFusionStatusDegradedInput, 0u);
    EXPECT_EQ(latest->status_flags & posest::fusion::kFusionStatusShockInflated, 0u);
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

TEST(FusionConfig, BuildFusionConfigMapsFromRuntimeFusion) {
    // Phase A: buildFusionConfig must round-trip every tunable from the
    // RuntimeConfig.fusion sub-struct. This locks in the field-for-field
    // mapping so future Phase B/C additions can't silently regress.
    posest::runtime::RuntimeConfig runtime_config;
    runtime_config.fusion.chassis_sigmas = {0.07, 0.08, 0.09, 0.011, 0.012, 0.013};
    runtime_config.fusion.huber_k = 1.7;
    runtime_config.fusion.shock_threshold_mps2 = 47.5;
    runtime_config.fusion.enable_imu_preintegration = true;
    runtime_config.fusion.max_keyframe_dt_seconds = 0.025;
    runtime_config.fusion.persisted_bias = {1, 2, 3, 4, 5, 6};
    runtime_config.fusion.marginalize_keyframe_window = 750;
    runtime_config.fusion.enable_vio = true;
    runtime_config.fusion.vio_default_sigmas = {0.04, 0.05, 0.06, 0.015, 0.016, 0.017};

    const auto config = posest::fusion::buildFusionConfig(runtime_config);
    EXPECT_DOUBLE_EQ(config.chassis_sigmas[0], 0.07);
    EXPECT_DOUBLE_EQ(config.chassis_sigmas[5], 0.013);
    EXPECT_DOUBLE_EQ(config.huber_k, 1.7);
    EXPECT_DOUBLE_EQ(config.shock_threshold_mps2, 47.5);
    EXPECT_TRUE(config.enable_imu_preintegration);
    EXPECT_DOUBLE_EQ(config.max_keyframe_dt_seconds, 0.025);
    EXPECT_DOUBLE_EQ(config.persisted_bias[0], 1.0);
    EXPECT_EQ(config.marginalize_keyframe_window, 750u);
    EXPECT_TRUE(config.enable_vio);
    EXPECT_DOUBLE_EQ(config.vio_default_sigmas[0], 0.04);
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

// ---------- Phase C: IMU preintegration ----------

namespace {

posest::fusion::FusionConfig makeImuPreintegrationConfig() {
    posest::fusion::FusionConfig cfg;
    cfg.enable_imu_preintegration = true;
    // Tighten the bias-calibration window so tests don't have to feed half
    // a second of fake IMU; the bound is enforced by the validator at >=0.5
    // for the SQLite-loaded path, but the in-process struct accepts any
    // positive value.
    cfg.bias_calibration_seconds = 0.05;  // 50 ms is plenty for tests
    cfg.max_keyframe_dt_seconds = 0.020;
    cfg.max_imu_gap_seconds = 0.100;
    return cfg;
}

posest::ImuSample makeStationaryImu(posest::Timestamp t) {
    posest::ImuSample s;
    s.timestamp = t;
    // |a| = 9.80665 m/s² along +Z; matches the default gravity_local_mps2.
    s.accel_mps2 = {0.0, 0.0, 9.80665};
    s.gyro_radps = {0.0, 0.0, 0.0};
    return s;
}

}  // namespace

TEST(FusionServiceImu, BiasCalibrationCompletesAndAdvancesToAwaitingFieldFix) {
    posest::MeasurementBus bus(64);
    posest::fusion::FusionService fusion(bus, makeImuPreintegrationConfig());
    fusion.start();

    // Bootstrap: chassis at zero (stationary) and a window of stationary IMU
    // samples that exceeds bias_calibration_seconds. Then a final chassis
    // sample after the calibration window so the published estimate
    // reflects the post-calibration state (kAwaitingFieldFix).
    const auto t0 = std::chrono::steady_clock::now();
    ASSERT_TRUE(bus.publish(makeChassis(t0)));
    for (int i = 0; i < 30; ++i) {
        ASSERT_TRUE(bus.publish(makeStationaryImu(t0 + std::chrono::milliseconds(i * 5))));
    }
    // 200 ms post-bootstrap > bias_calibration_seconds (50 ms) — by now the
    // calibration is done and the next chassis sample will report the new
    // init state in its status_flags.
    ASSERT_TRUE(bus.publish(makeChassis(t0 + 250ms, 0.0)));

    std::this_thread::sleep_for(150ms);
    fusion.stop();

    const auto stats = fusion.stats();
    EXPECT_GE(stats.bias_calibrations_completed, 1u);
    // No vision yet, so we should still be in the awaiting-field-fix state.
    const auto latest = fusion.latestEstimate();
    ASSERT_TRUE(latest.has_value());
    EXPECT_NE(latest->status_flags & posest::fusion::kFusionStatusAwaitingFieldFix, 0u);
}

TEST(FusionServiceImu, FieldFixAdvancesToRunningAndPopulatesVelocity) {
    posest::MeasurementBus bus(64);
    posest::fusion::FusionService fusion(bus, makeImuPreintegrationConfig());
    fusion.start();

    const auto t0 = std::chrono::steady_clock::now();
    ASSERT_TRUE(bus.publish(makeChassis(t0)));
    // Stationary IMU window completes calibration → kAwaitingFieldFix.
    for (int i = 0; i < 30; ++i) {
        ASSERT_TRUE(bus.publish(makeStationaryImu(t0 + std::chrono::milliseconds(i * 5))));
    }
    // First vision observation snaps origin and transitions kAwaitingFieldFix
    // → kRunning. After this, makeEstimate() should populate velocity.
    auto vision = makePriorObservation({{1.0, 0.5, 0.0}, {0.0, 0.0, 0.0}}, 0.05, 0.02);
    vision.capture_time = t0 + 200ms;
    ASSERT_TRUE(bus.publish(vision));

    // Drive a couple of keyframes worth of chassis + IMU at t0+200ms onward
    // so the running graph emits estimates with velocity.
    for (int i = 1; i <= 5; ++i) {
        ASSERT_TRUE(bus.publish(makeStationaryImu(t0 + 200ms + std::chrono::milliseconds(i * 5))));
    }
    ASSERT_TRUE(bus.publish(makeChassis(t0 + 250ms, 0.5)));

    std::this_thread::sleep_for(150ms);
    fusion.stop();

    const auto latest = fusion.latestEstimate();
    ASSERT_TRUE(latest.has_value());
    EXPECT_EQ(latest->status_flags & posest::fusion::kFusionStatusAwaitingFieldFix, 0u);
    // Velocity should be populated once kRunning.
    EXPECT_TRUE(latest->velocity.has_value());
}

TEST(FusionServiceImu, RejectsOutOfOrderImuSamples) {
    posest::MeasurementBus bus(64);
    posest::fusion::FusionService fusion(bus, makeImuPreintegrationConfig());
    fusion.start();

    const auto t0 = std::chrono::steady_clock::now();
    ASSERT_TRUE(bus.publish(makeChassis(t0)));
    // Calibration window.
    for (int i = 0; i < 30; ++i) {
        ASSERT_TRUE(bus.publish(makeStationaryImu(t0 + std::chrono::milliseconds(i * 5))));
    }
    // Field fix.
    auto vision = makePriorObservation({{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, 0.05, 0.02);
    vision.capture_time = t0 + 200ms;
    ASSERT_TRUE(bus.publish(vision));

    // Drive forward past the field fix.
    ASSERT_TRUE(bus.publish(makeStationaryImu(t0 + 250ms)));
    // Now publish an IMU sample with a timestamp earlier than the previous
    // accepted one — the per-type cursor in FusionService will reject it
    // before it reaches the backend (out-of-order is enforced both by the
    // service-level cursor and the backend's last_imu_time_).
    posest::ImuSample stale;
    stale.timestamp = t0 + 100ms;  // older than 250ms
    stale.accel_mps2 = {0.0, 0.0, 9.80665};
    ASSERT_TRUE(bus.publish(stale));

    std::this_thread::sleep_for(100ms);
    fusion.stop();

    const auto stats = fusion.stats();
    // Either path counts the rejection — service-level stale_measurements
    // OR backend-level imu_out_of_order. At least one must fire.
    EXPECT_GE(stats.stale_measurements + stats.imu_out_of_order, 1u);
}
