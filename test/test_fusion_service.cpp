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

}  // namespace

TEST(FusionService, ProcessesWheelOdometryWithGtsamAndPublishesEstimate) {
    posest::MeasurementBus bus(8);
    posest::fusion::FusionService fusion(bus);
    auto sink = std::make_shared<RecordingSink>();
    fusion.addOutputSink(sink);

    fusion.start();

    posest::WheelOdometrySample odom;
    odom.timestamp = std::chrono::steady_clock::now();
    odom.chassis_delta.x_m = 1.5;
    ASSERT_TRUE(bus.publish(odom));

    std::this_thread::sleep_for(50ms);
    fusion.stop();

    const auto stats = fusion.stats();
    EXPECT_EQ(stats.measurements_processed, 1u);
    EXPECT_EQ(stats.stale_measurements, 0u);

    const auto latest = fusion.latestEstimate();
    ASSERT_TRUE(latest.has_value());
    EXPECT_NEAR(latest->field_to_robot.x_m, 1.5, 1e-6);
    EXPECT_NEAR(latest->field_to_robot.y_m, 0.0, 1e-6);
    EXPECT_TRUE(covarianceIsFinite(*latest));
    EXPECT_NE(latest->status_flags & posest::fusion::kFusionStatusVisionUnavailable, 0u);

    std::lock_guard<std::mutex> g(sink->mu);
    ASSERT_EQ(sink->estimates.size(), 1u);
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
    posest::WheelOdometrySample odom;
    odom.timestamp = now;
    odom.chassis_delta.x_m = 1.0;
    ASSERT_TRUE(bus.publish(odom));

    posest::AprilTagObservation observation;
    observation.camera_id = "cam0";
    observation.capture_time = now + 1ms;
    ASSERT_TRUE(bus.publish(observation));

    std::this_thread::sleep_for(50ms);
    fusion.stop();

    const auto latest = fusion.latestEstimate();
    ASSERT_TRUE(latest.has_value());
    EXPECT_NEAR(latest->field_to_robot.x_m, 1.0, 1e-6);
    EXPECT_NE(latest->status_flags & posest::fusion::kFusionStatusVisionUnavailable, 0u);
}

TEST(FusionService, IgnoresUnsupportedMeasurements) {
    posest::MeasurementBus bus(8);
    posest::fusion::FusionService fusion(bus);
    fusion.start();

    posest::ImuSample imu;
    imu.timestamp = std::chrono::steady_clock::now();
    ASSERT_TRUE(bus.publish(imu));

    std::this_thread::sleep_for(20ms);
    fusion.stop();

    const auto stats = fusion.stats();
    EXPECT_EQ(stats.measurements_processed, 0u);
    EXPECT_EQ(stats.stale_measurements, 0u);
    EXPECT_FALSE(fusion.latestEstimate().has_value());
}

TEST(FusionService, RejectsStaleMeasurements) {
    posest::MeasurementBus bus(8);
    posest::fusion::FusionService fusion(bus);
    fusion.start();

    const auto now = std::chrono::steady_clock::now();
    posest::WheelOdometrySample newer;
    newer.timestamp = now;
    posest::WheelOdometrySample older;
    older.timestamp = now - 1ms;

    ASSERT_TRUE(bus.publish(newer));
    ASSERT_TRUE(bus.publish(older));

    std::this_thread::sleep_for(50ms);
    fusion.stop();

    const auto stats = fusion.stats();
    EXPECT_EQ(stats.measurements_processed, 1u);
    EXPECT_EQ(stats.stale_measurements, 1u);
}

TEST(FusionConfig, BuildFusionConfigIgnoresRuntimeConfig) {
    posest::runtime::RuntimeConfig runtime_config;
    runtime_config.calibrations.push_back({
        .camera_id = "cam0",
        .version = "v1",
        .active = true,
    });
    const auto config = posest::fusion::buildFusionConfig(runtime_config);
    EXPECT_GT(config.wheel_sigmas[0], 0.0);
    EXPECT_GT(config.origin_prior_sigmas[0], 0.0);
}
