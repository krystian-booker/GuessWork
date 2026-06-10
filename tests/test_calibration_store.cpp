#include <gtest/gtest.h>

#include <string>

#include "calibration/calibration_store.hpp"

namespace gw::calib {

namespace {

// Real-shaped Kalibr intrinsics camchain (kalibr_calibrate_cameras output)
// with the guesswork_meta enrichment our KalibrJob appends.
constexpr const char* kIntrinsicsCamchain = R"(cam0:
  camera_model: pinhole
  distortion_coeffs: [-0.286, 0.073, 0.0002, -0.00003]
  distortion_model: radtan
  intrinsics: [1465.3, 1466.1, 1023.5, 767.2]
  resolution: [2048, 1536]
  rostopic: /cam0/image_raw
guesswork_meta:
  reprojection_error_px: 0.214301
  reprojection_error_u_px: 0.201122
  reprojection_error_v_px: 0.226901
)";

// Real-shaped camchain-imucam (kalibr_calibrate_imu_camera output) for a
// stereo pair: cam0 has T_cam_imu + timeshift, cam1 additionally T_cn_cnm1.
constexpr const char* kImucamCamchain = R"(cam0:
  T_cam_imu:
  - [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975]
  - [0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768]
  - [-0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949]
  - [0.0, 0.0, 0.0, 1.0]
  camera_model: pinhole
  distortion_coeffs: [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05]
  distortion_model: radtan
  intrinsics: [458.654, 457.296, 367.215, 248.375]
  resolution: [752, 480]
  rostopic: /cam0/image_raw
  timeshift_cam_imu: 5.6375620963e-05
cam1:
  T_cam_imu:
  - [0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556]
  - [0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024]
  - [-0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038]
  - [0.0, 0.0, 0.0, 1.0]
  T_cn_cnm1:
  - [0.999997256477, 0.002312067192, 0.000376008102, -0.110073808127]
  - [-0.002317135723, 0.999898048507, 0.014089835846, 0.000399121547]
  - [-0.000343393121, -0.014090668452, 0.999900662638, -0.000853702503]
  - [0.0, 0.0, 0.0, 1.0]
  camera_model: pinhole
  distortion_coeffs: [-0.28368365, 0.07451284, -0.00010473, -3.55590700e-05]
  distortion_model: radtan
  intrinsics: [457.587, 456.134, 379.999, 255.238]
  resolution: [752, 480]
  rostopic: /cam1/image_raw
  timeshift_cam_imu: 5.1234567890e-05
)";

}  // namespace

TEST(CalibrationStoreTest, ParsesIntrinsicsCamchainIgnoringMeta) {
    const auto chain = parse_camchain(kIntrinsicsCamchain);
    ASSERT_EQ(chain.cameras.size(), 1u);
    EXPECT_EQ(chain.cameras[0].first, "cam0");

    const auto& in = chain.cameras[0].second.intrinsics;
    EXPECT_EQ(in.camera_model, "pinhole");
    EXPECT_EQ(in.distortion_model, "radtan");
    EXPECT_DOUBLE_EQ(in.intrinsics[0], 1465.3);
    EXPECT_DOUBLE_EQ(in.intrinsics[3], 767.2);
    EXPECT_DOUBLE_EQ(in.distortion_coeffs[0], -0.286);
    EXPECT_EQ(in.resolution[0], 2048u);
    EXPECT_EQ(in.resolution[1], 1536u);
    EXPECT_EQ(in.rostopic, "/cam0/image_raw");
    EXPECT_FALSE(chain.cameras[0].second.imu.has_value());
}

TEST(CalibrationStoreTest, ParsesImucamCamchainWithExtrinsics) {
    const auto chain = parse_camchain(kImucamCamchain);
    ASSERT_EQ(chain.cameras.size(), 2u);
    EXPECT_EQ(chain.cameras[0].first, "cam0");
    EXPECT_EQ(chain.cameras[1].first, "cam1");

    const auto& imu0 = chain.cameras[0].second.imu;
    ASSERT_TRUE(imu0.has_value());
    EXPECT_DOUBLE_EQ(imu0->T_cam_imu[0][0], 0.0148655429818);
    EXPECT_DOUBLE_EQ(imu0->T_cam_imu[1][3], -0.064676986768);
    EXPECT_DOUBLE_EQ(imu0->T_cam_imu[3][3], 1.0);
    EXPECT_DOUBLE_EQ(imu0->timeshift_cam_imu, 5.6375620963e-05);
    EXPECT_FALSE(imu0->T_cn_cnm1.has_value());

    const auto& imu1 = chain.cameras[1].second.imu;
    ASSERT_TRUE(imu1.has_value());
    ASSERT_TRUE(imu1->T_cn_cnm1.has_value());
    // Stereo baseline lives in (*T_cn_cnm1)[0][3].
    EXPECT_DOUBLE_EQ((*imu1->T_cn_cnm1)[0][3], -0.110073808127);
}

TEST(CalibrationStoreTest, RoundTripsThroughSerializer) {
    const auto chain = parse_camchain(kImucamCamchain);
    const auto yaml  = serialize_camchain(chain);
    const auto again = parse_camchain(yaml);

    ASSERT_EQ(again.cameras.size(), chain.cameras.size());
    for (size_t c = 0; c < chain.cameras.size(); ++c) {
        const auto& a = chain.cameras[c].second;
        const auto& b = again.cameras[c].second;
        EXPECT_EQ(a.intrinsics.camera_model, b.intrinsics.camera_model);
        for (int i = 0; i < 4; ++i) {
            EXPECT_DOUBLE_EQ(a.intrinsics.intrinsics[i], b.intrinsics.intrinsics[i]);
            EXPECT_DOUBLE_EQ(a.intrinsics.distortion_coeffs[i],
                             b.intrinsics.distortion_coeffs[i]);
        }
        EXPECT_EQ(a.intrinsics.resolution, b.intrinsics.resolution);
        ASSERT_EQ(a.imu.has_value(), b.imu.has_value());
        if (a.imu) {
            EXPECT_DOUBLE_EQ(a.imu->timeshift_cam_imu, b.imu->timeshift_cam_imu);
            for (int r = 0; r < 4; ++r) {
                for (int k = 0; k < 4; ++k) {
                    EXPECT_DOUBLE_EQ(a.imu->T_cam_imu[r][k], b.imu->T_cam_imu[r][k]);
                }
            }
            EXPECT_EQ(a.imu->T_cn_cnm1.has_value(), b.imu->T_cn_cnm1.has_value());
        }
    }
}

TEST(CalibrationStoreTest, SerializeSingleCameraRekeysToCam0) {
    const auto chain = parse_camchain(kImucamCamchain);
    // Take cam1 (which has T_cn_cnm1) and re-key it.
    const auto doc   = serialize_single_camera(chain.cameras[1].second);
    const auto again = parse_camchain(doc);

    ASSERT_EQ(again.cameras.size(), 1u);
    EXPECT_EQ(again.cameras[0].first, "cam0");
    ASSERT_TRUE(again.cameras[0].second.imu.has_value());
    EXPECT_TRUE(again.cameras[0].second.imu->T_cn_cnm1.has_value());
    EXPECT_DOUBLE_EQ(again.cameras[0].second.intrinsics.intrinsics[0], 457.587);
}

TEST(CalibrationStoreTest, ThrowsOnMalformedShapes) {
    EXPECT_THROW(parse_camchain("not: a camchain"), CalibrationParseError);
    EXPECT_THROW(parse_camchain("cam0: 12"), CalibrationParseError);
    EXPECT_THROW(parse_camchain("- a\n- list\n"), CalibrationParseError);
    EXPECT_THROW(parse_camchain(": bad yaml ["), CalibrationParseError);

    // Wrong arity in intrinsics.
    EXPECT_THROW(parse_camchain(R"(cam0:
  camera_model: pinhole
  intrinsics: [1.0, 2.0, 3.0]
  distortion_model: radtan
  distortion_coeffs: [0, 0, 0, 0]
  resolution: [10, 10]
)"),
                 CalibrationParseError);

    // 3-row T_cam_imu.
    EXPECT_THROW(parse_camchain(R"(cam0:
  camera_model: pinhole
  intrinsics: [1.0, 2.0, 3.0, 4.0]
  distortion_model: radtan
  distortion_coeffs: [0, 0, 0, 0]
  resolution: [10, 10]
  T_cam_imu:
  - [1, 0, 0, 0]
  - [0, 1, 0, 0]
  - [0, 0, 0, 1]
)"),
                 CalibrationParseError);
}

}  // namespace gw::calib
