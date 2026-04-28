#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "posest/config/CalibrationParsers.h"

namespace {

std::filesystem::path fixturePath(const std::string& name) {
    return std::filesystem::path(POSEST_TEST_DATA_DIR) / "kalibr" / name;
}

}  // namespace

TEST(KalibrCameraResultsParser, ExtractsRmsForOneCamera) {
    const auto metrics = posest::config::parseKalibrCameraResults(
        fixturePath("results-cam_pass.txt"));
    ASSERT_EQ(metrics.size(), 1u);
    ASSERT_TRUE(metrics.count("cam0"));
    // RMS = hypot(0.181675, 0.194522) ≈ 0.26617
    EXPECT_NEAR(metrics.at("cam0").reprojection_rms_px,
                std::hypot(0.181675, 0.194522), 1e-9);
    EXPECT_EQ(metrics.at("cam0").observation_count, 0);
}

TEST(KalibrCameraResultsParser, ExtractsRmsForBothCameras) {
    const auto metrics = posest::config::parseKalibrCameraResults(
        fixturePath("results-cam_two_cameras.txt"));
    ASSERT_EQ(metrics.size(), 2u);
    EXPECT_NEAR(metrics.at("cam0").reprojection_rms_px,
                std::hypot(0.20, 0.22), 1e-9);
    EXPECT_NEAR(metrics.at("cam1").reprojection_rms_px,
                std::hypot(0.30, 0.34), 1e-9);
}

TEST(KalibrCameraResultsParser, FailedRunReportsAboveGateRms) {
    const auto metrics = posest::config::parseKalibrCameraResults(
        fixturePath("results-cam_fail.txt"));
    ASSERT_TRUE(metrics.count("cam0"));
    // hypot(1.5, 1.7) ≈ 2.265 — well above the 1.0 px gate.
    EXPECT_NEAR(metrics.at("cam0").reprojection_rms_px, 2.26715, 1e-4);
}

TEST(KalibrCameraResultsParser, ThrowsOnMissingFile) {
    EXPECT_THROW(
        posest::config::parseKalibrCameraResults(
            fixturePath("does_not_exist.txt")),
        std::runtime_error);
}

TEST(KalibrCameraImuResultsParser, ExtractsAllThreeMetrics) {
    const auto metrics = posest::config::parseKalibrCameraImuResults(
        fixturePath("results-imucam_pass.txt"));
    ASSERT_EQ(metrics.size(), 1u);
    ASSERT_TRUE(metrics.count("cam0"));
    EXPECT_DOUBLE_EQ(metrics.at("cam0").reprojection_rms_px, 0.41);
    EXPECT_DOUBLE_EQ(metrics.at("cam0").gyro_rms_radps, 0.0021);
    EXPECT_DOUBLE_EQ(metrics.at("cam0").accel_rms_mps2, 0.07);
}

TEST(KalibrCameraImuResultsParser, LeavesMissingMetricsAtZero) {
    const auto metrics = posest::config::parseKalibrCameraImuResults(
        fixturePath("results-imucam_missing_metrics.txt"));
    ASSERT_TRUE(metrics.count("cam0"));
    EXPECT_DOUBLE_EQ(metrics.at("cam0").reprojection_rms_px, 0.41);
    // Gyro/accel sections are absent; broadcast is a no-op.
    EXPECT_DOUBLE_EQ(metrics.at("cam0").gyro_rms_radps, 0.0);
    EXPECT_DOUBLE_EQ(metrics.at("cam0").accel_rms_mps2, 0.0);
}

TEST(KalibrAllCamerasParser, ReturnsBothCamerasPlusOneBaseline) {
    std::unordered_map<std::string, std::string> topic_to_id{
        {"/posest/cam0/image_raw", "left_cam"},
        {"/posest/cam1/image_raw", "right_cam"},
    };
    const auto bundle = posest::config::parseKalibrAllCameras(
        fixturePath("camchain_two_cameras.yaml"),
        topic_to_id, "v1", "2026-04-27T00:00:00Z");

    ASSERT_EQ(bundle.cameras.size(), 2u);
    EXPECT_EQ(bundle.cameras[0].camera_id, "left_cam");
    EXPECT_DOUBLE_EQ(bundle.cameras[0].fx, 400.1);
    EXPECT_TRUE(bundle.cameras[0].active);
    EXPECT_EQ(bundle.cameras[0].version, "v1");
    EXPECT_EQ(bundle.cameras[1].camera_id, "right_cam");
    EXPECT_DOUBLE_EQ(bundle.cameras[1].fx, 402.0);

    ASSERT_EQ(bundle.cam_to_cam.size(), 1u);
    EXPECT_EQ(bundle.cam_to_cam[0].reference_camera_id, "left_cam");
    EXPECT_EQ(bundle.cam_to_cam[0].target_camera_id, "right_cam");
    EXPECT_EQ(bundle.cam_to_cam[0].version, "v1");
    EXPECT_DOUBLE_EQ(
        bundle.cam_to_cam[0].target_in_reference.translation_m.x, 0.10);
    EXPECT_DOUBLE_EQ(
        bundle.cam_to_cam[0].target_in_reference.translation_m.y, 0.0);
}

TEST(KalibrAllCamerasParser, SingleCameraReturnsZeroBaselines) {
    std::unordered_map<std::string, std::string> topic_to_id{
        {"/posest/cam0/image_raw", "only_cam"},
    };
    const auto bundle = posest::config::parseKalibrAllCameras(
        fixturePath("camchain_single_camera.yaml"),
        topic_to_id, "v1", "2026-04-27T00:00:00Z");
    ASSERT_EQ(bundle.cameras.size(), 1u);
    EXPECT_EQ(bundle.cameras[0].camera_id, "only_cam");
    EXPECT_TRUE(bundle.cam_to_cam.empty());
}

TEST(KalibrAllCamerasParser, ThrowsOnUnknownTopic) {
    std::unordered_map<std::string, std::string> topic_to_id{
        {"/posest/cam0/image_raw", "left_cam"},
        // cam1's rostopic intentionally absent.
    };
    EXPECT_THROW(
        posest::config::parseKalibrAllCameras(
            fixturePath("camchain_two_cameras.yaml"),
            topic_to_id, "v1", "2026-04-27T00:00:00Z"),
        std::invalid_argument);
}

TEST(KalibrAllCamerasParser, ThrowsOnPartialResult) {
    std::unordered_map<std::string, std::string> topic_to_id{
        {"/posest/cam0/image_raw", "left_cam"},
        {"/posest/cam1/image_raw", "right_cam"},
    };
    // Single-camera fixture against a two-camera request — fail-safe.
    EXPECT_THROW(
        posest::config::parseKalibrAllCameras(
            fixturePath("camchain_single_camera.yaml"),
            topic_to_id, "v1", "2026-04-27T00:00:00Z"),
        std::runtime_error);
}
