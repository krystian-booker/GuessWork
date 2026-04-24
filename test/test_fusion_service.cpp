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
    posest::fusion::FusionConfig config;
    config.field_to_tags.emplace(
        1,
        posest::Pose3d{
            .translation_m = {5.0, 0.0, 0.0},
            .rotation_rpy_rad = {0.0, 0.0, 0.0},
        });
    config.camera_to_robot.emplace(
        "cam0",
        posest::Pose3d{
            .translation_m = {0.0, 0.0, 0.0},
            .rotation_rpy_rad = {0.0, 0.0, 0.0},
        });
    return config;
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

TEST(FusionService, UsesAprilTagMeasurementAsAbsolutePosePrior) {
    posest::MeasurementBus bus(8);
    posest::fusion::FusionService fusion(bus, makeTagFusionConfig());
    fusion.start();

    posest::AprilTagObservation observation;
    observation.camera_id = "cam0";
    observation.capture_time = std::chrono::steady_clock::now();
    posest::AprilTagDetection detection;
    detection.tag_id = 1;
    detection.camera_to_tag = posest::Pose3d{
        .translation_m = {2.0, 0.0, 0.0},
        .rotation_rpy_rad = {0.0, 0.0, 0.0},
    };
    observation.detections.push_back(detection);
    ASSERT_TRUE(bus.publish(observation));

    std::this_thread::sleep_for(50ms);
    fusion.stop();

    const auto latest = fusion.latestEstimate();
    ASSERT_TRUE(latest.has_value());
    EXPECT_NEAR(latest->field_to_robot.x_m, 3.0, 1e-6);
    EXPECT_NEAR(latest->field_to_robot.y_m, 0.0, 1e-6);
    EXPECT_EQ(latest->status_flags & posest::fusion::kFusionStatusVisionUnavailable, 0u);
    EXPECT_TRUE(covarianceIsFinite(*latest));
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

TEST(FusionConfig, BuildsActiveFieldTagsAndCameraExtrinsicsFromRuntimeConfig) {
    posest::runtime::RuntimeConfig runtime_config;
    posest::runtime::FieldLayoutConfig layout;
    layout.id = "field";
    layout.name = "Field";
    layout.source_file_path = "field.json";
    layout.field_length_m = 16.0;
    layout.field_width_m = 8.0;
    layout.tags.push_back({
        .tag_id = 1,
        .field_to_tag = {
            .translation_m = {2.0, 3.0, 0.5},
            .rotation_rpy_rad = {0.0, 0.0, 1.0},
        },
    });
    runtime_config.field_layouts.push_back(layout);
    runtime_config.active_field_layout_id = "field";
    runtime_config.calibrations.push_back({
        .camera_id = "cam0",
        .version = "v1",
        .active = true,
    });
    runtime_config.camera_extrinsics.push_back({
        .camera_id = "cam0",
        .version = "v1",
        .camera_to_robot = {
            .translation_m = {0.1, 0.2, 0.3},
            .rotation_rpy_rad = {0.0, 0.0, 0.0},
        },
    });

    const auto config = posest::fusion::buildFusionConfig(runtime_config);

    ASSERT_EQ(config.field_to_tags.size(), 1u);
    ASSERT_EQ(config.camera_to_robot.size(), 1u);
    EXPECT_DOUBLE_EQ(config.field_to_tags.at(1).translation_m.y, 3.0);
    EXPECT_DOUBLE_EQ(config.camera_to_robot.at("cam0").translation_m.x, 0.1);
}
