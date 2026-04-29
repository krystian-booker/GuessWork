// test_kimera_backend.cpp — smoke test for the real Kimera-VIO adapter.
//
// COMPILED ONLY WHEN POSEST_BUILD_VIO IS ON. On Linux without
// POSEST_BUILD_VIO, this file is excluded from the build and the
// FakeVioBackend tests in test_kimera_vio_consumer.cpp are the
// cross-platform integration coverage.
//
// The two cases below are first-line proof that:
//   1. Constructing → start() → stop() exercises the spin thread
//      lifecycle without hanging or leaking, and the Kimera API
//      surfaces (VioParams ctor, MonoImuPipeline ctor, register
//      callback, shutdown) compile + link against the installed
//      Kimera version.
//   2. tryPushFrame / tryPushImu return false before start() and
//      true while running — the trivial running-flag contract that
//      KimeraVioConsumer relies on.
//
// This is NOT a correctness test for Kimera's trajectory; it does
// not exercise convergence. Trajectory correctness is validated
// end-to-end via the daemon + real cameras, which lives outside
// this test suite.

#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "posest/runtime/RuntimeConfig.h"
#include "posest/vio/IVioBackend.h"
#include "posest/vio/KimeraParamWriter.h"

namespace posest::vio {

// Forward-declared factory; lives in KimeraBackend.cpp behind the
// POSEST_BUILD_VIO gate.
std::unique_ptr<IVioBackend> makeKimeraBackend(const std::string& param_dir);

}  // namespace posest::vio

namespace {

std::filesystem::path makeTempParamDir(const std::string& tag) {
    const auto stamp =
        std::chrono::steady_clock::now().time_since_epoch().count();
    auto p = std::filesystem::temp_directory_path() /
             ("posest_kimera_backend_" + tag + "_" + std::to_string(stamp));
    std::filesystem::create_directories(p);
    return p;
}

posest::runtime::RuntimeConfig makeVioReadyConfig() {
    posest::runtime::RuntimeConfig config;

    posest::CameraConfig cam;
    cam.id = "downward";
    cam.type = "v4l2";
    cam.device = "/dev/video0";
    cam.enabled = true;
    cam.format.width = 640;
    cam.format.height = 480;
    cam.format.fps = 120.0;
    cam.format.pixel_format = "mjpeg";
    config.cameras.push_back(cam);

    posest::runtime::CameraCalibrationConfig calib;
    calib.camera_id = "downward";
    calib.version = "v1";
    calib.active = true;
    calib.source_file_path = "/tmp/calib.yaml";
    calib.created_at = "2026-04-27T00:00:00Z";
    calib.image_width = 640;
    calib.image_height = 480;
    calib.camera_model = "pinhole";
    calib.distortion_model = "radtan";
    calib.fx = 500.0;
    calib.fy = 501.0;
    calib.cx = 320.0;
    calib.cy = 240.0;
    calib.distortion_coefficients = {0.1, -0.05, 0.001, -0.001};
    config.calibrations.push_back(calib);

    posest::runtime::CameraImuCalibrationConfig cam_imu;
    cam_imu.camera_id = "downward";
    cam_imu.version = "imu-v1";
    cam_imu.active = true;
    cam_imu.source_file_path = "/tmp/imucam.yaml";
    cam_imu.created_at = "2026-04-27T00:01:00Z";
    cam_imu.camera_to_imu = {{0.01, 0.02, 0.03}, {0.0, 0.0, 0.0}};
    cam_imu.imu_to_camera = {{-0.01, -0.02, -0.03}, {0.0, 0.0, 0.0}};
    cam_imu.time_shift_s = 0.004;
    config.camera_imu_calibrations.push_back(cam_imu);

    config.vio.enabled = true;
    config.vio.vio_camera_id = "downward";

    return config;
}

class KimeraBackendSmokeFixture : public ::testing::Test {
protected:
    void SetUp() override {
        param_dir_ = makeTempParamDir(test_info_->name());
        const auto cfg = makeVioReadyConfig();
        // Lay down all four YAMLs Kimera expects (FrontendParams,
        // BackendParams, ImuParams with IMUtoBodyT_BS + random walks
        // appended, LeftCameraParams with intrinsics + T_BS). This
        // mirrors what the daemon does at boot.
        posest::vio::emitKimeraParamYamls(cfg, param_dir_.string());
    }

    void TearDown() override {
        std::error_code ec;
        std::filesystem::remove_all(param_dir_, ec);
    }

    std::filesystem::path param_dir_;
};

TEST_F(KimeraBackendSmokeFixture, ConstructsAndShutsDownCleanly) {
    auto backend = posest::vio::makeKimeraBackend(param_dir_.string());
    ASSERT_NE(backend, nullptr);

    backend->setOutputCallback(
        [](const posest::vio::VioBackendOutput&) { /* drop */ });

    backend->start();

    // Give the spin thread a moment to be live.
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    backend->stop();
    // Idempotent: a second stop is a no-op.
    backend->stop();
}

TEST_F(KimeraBackendSmokeFixture, PushesRejectedBeforeStartAcceptedAfter) {
    auto backend = posest::vio::makeKimeraBackend(param_dir_.string());
    ASSERT_NE(backend, nullptr);

    cv::Mat image(480, 640, CV_8UC1, cv::Scalar(0));
    const Eigen::Vector3d accel{0.0, 0.0, 9.81};
    const Eigen::Vector3d gyro{0.0, 0.0, 0.0};

    // Before start: backend should refuse pushes.
    EXPECT_FALSE(backend->tryPushFrame(1000ULL, image));
    EXPECT_FALSE(backend->tryPushImu(1000ULL, accel, gyro));

    backend->start();

    // While running: pushes are accepted (queue may still drop later,
    // but the backend should not flatly refuse).
    EXPECT_TRUE(backend->tryPushFrame(2000ULL, image));
    EXPECT_TRUE(backend->tryPushImu(2000ULL, accel, gyro));

    backend->stop();

    // After stop: refuses again.
    EXPECT_FALSE(backend->tryPushFrame(3000ULL, image));
    EXPECT_FALSE(backend->tryPushImu(3000ULL, accel, gyro));
}

}  // namespace
