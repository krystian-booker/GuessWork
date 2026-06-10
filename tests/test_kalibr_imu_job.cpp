#include <gtest/gtest.h>

#include <string>

#include "calibration/calibration_store.hpp"
#include "server/kalibr_imu_job.hpp"

namespace gw::server {

namespace {

constexpr const char* kPairImucam = R"(cam0:
  T_cam_imu:
  - [1.0, 0.0, 0.0, 0.01]
  - [0.0, 1.0, 0.0, 0.02]
  - [0.0, 0.0, 1.0, 0.03]
  - [0.0, 0.0, 0.0, 1.0]
  camera_model: pinhole
  distortion_coeffs: [-0.28, 0.07, 0.0002, -0.00003]
  distortion_model: radtan
  intrinsics: [458.6, 457.3, 367.2, 248.4]
  resolution: [752, 480]
  rostopic: /cam0/image_raw
  timeshift_cam_imu: 1.0e-05
cam1:
  T_cam_imu:
  - [1.0, 0.0, 0.0, 0.04]
  - [0.0, 1.0, 0.0, 0.05]
  - [0.0, 0.0, 1.0, 0.06]
  - [0.0, 0.0, 0.0, 1.0]
  T_cn_cnm1:
  - [1.0, 0.0, 0.0, -0.11]
  - [0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0]
  camera_model: pinhole
  distortion_coeffs: [-0.28, 0.07, -0.0001, -0.00003]
  distortion_model: radtan
  intrinsics: [457.6, 456.1, 380.0, 255.2]
  resolution: [752, 480]
  rostopic: /cam1/image_raw
  timeshift_cam_imu: 2.0e-05
)";

}  // namespace

TEST(BuildImuExtrinsicsPayloadsTest, SplitsPairIntoCam0KeyedPayloads) {
    const auto payloads = build_imu_extrinsics_payloads(kPairImucam, 2);
    ASSERT_EQ(payloads.size(), 2u);

    // Every payload re-parses as a single cam0-keyed document.
    const auto p0 = gw::calib::parse_camchain(payloads[0]);
    ASSERT_EQ(p0.cameras.size(), 1u);
    EXPECT_EQ(p0.cameras[0].first, "cam0");
    ASSERT_TRUE(p0.cameras[0].second.imu.has_value());
    EXPECT_DOUBLE_EQ(p0.cameras[0].second.imu->T_cam_imu[0][3], 0.01);
    EXPECT_FALSE(p0.cameras[0].second.imu->T_cn_cnm1.has_value());

    const auto p1 = gw::calib::parse_camchain(payloads[1]);
    ASSERT_EQ(p1.cameras.size(), 1u);
    EXPECT_EQ(p1.cameras[0].first, "cam0");  // re-keyed from cam1
    ASSERT_TRUE(p1.cameras[0].second.imu.has_value());
    EXPECT_DOUBLE_EQ(p1.cameras[0].second.imu->T_cam_imu[0][3], 0.04);
    // The cam-cam transform survives the re-key (vio_right keeps its baseline).
    ASSERT_TRUE(p1.cameras[0].second.imu->T_cn_cnm1.has_value());
    EXPECT_DOUBLE_EQ((*p1.cameras[0].second.imu->T_cn_cnm1)[0][3], -0.11);
}

TEST(BuildImuExtrinsicsPayloadsTest, ThrowsOnCameraCountMismatch) {
    EXPECT_THROW(build_imu_extrinsics_payloads(kPairImucam, 1),
                 gw::calib::CalibrationParseError);
    EXPECT_THROW(build_imu_extrinsics_payloads(kPairImucam, 3),
                 gw::calib::CalibrationParseError);
}

TEST(BuildImuExtrinsicsPayloadsTest, ThrowsWhenExtrinsicsMissing) {
    // A plain intrinsics camchain (no T_cam_imu) is not a valid imucam result.
    constexpr const char* kIntrinsicsOnly = R"(cam0:
  camera_model: pinhole
  distortion_coeffs: [0, 0, 0, 0]
  distortion_model: radtan
  intrinsics: [458.6, 457.3, 367.2, 248.4]
  resolution: [752, 480]
)";
    EXPECT_THROW(build_imu_extrinsics_payloads(kIntrinsicsOnly, 1),
                 gw::calib::CalibrationParseError);
}

}  // namespace gw::server
