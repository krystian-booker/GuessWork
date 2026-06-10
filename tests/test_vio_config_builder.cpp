#include <gtest/gtest.h>

#include "apriltag/pose_math.hpp"
#include "calibration/calibration_store.hpp"
#include "vio/vio_config_builder.hpp"

namespace gw::vio {

namespace {

constexpr const char* kLeftImucam = R"(cam0:
  T_cam_imu:
  - [0.0, -1.0, 0.0, 0.01]
  - [1.0, 0.0, 0.0, -0.02]
  - [0.0, 0.0, 1.0, 0.03]
  - [0.0, 0.0, 0.0, 1.0]
  camera_model: pinhole
  distortion_coeffs: [-0.28, 0.07, 0.0002, -0.00003]
  distortion_model: radtan
  intrinsics: [1465.3, 1466.1, 1023.5, 767.2]
  resolution: [2048, 1536]
  rostopic: /cam0/image_raw
  timeshift_cam_imu: 4.2e-05
guesswork_meta:
  session_id: 20260610-101010-123
  source_cam_index: 0
  timeshift_cam_imu_s: 4.2e-05
  reprojection_error_mean_px: 0.21
  reprojection_error_median_px: 0.18
  reprojection_error_std_px: 0.35
)";

constexpr const char* kRightImucamEqui = R"(cam0:
  T_cam_imu:
  - [1.0, 0.0, 0.0, -0.05]
  - [0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0]
  camera_model: pinhole
  distortion_coeffs: [0.02, -0.005, 0.001, -0.0002]
  distortion_model: equidistant
  intrinsics: [1450.0, 1451.0, 1020.0, 770.0]
  resolution: [2048, 1536]
  rostopic: /cam1/image_raw
  timeshift_cam_imu: 4.5e-05
)";

VioImuNoise test_noise() {
    return {2.4e-4, 2.7e-5, 1.7e-3, 4.4e-4};
}

gw::calib::CamchainEntry entry_of(const char* yaml) {
    return gw::calib::parse_camchain(yaml).cameras.front().second;
}

}  // namespace

TEST(VioConfigBuilderTest, GoldenRunnerConfig) {
    VioTuning tuning;
    tuning.num_pts        = 175;
    tuning.fast_threshold = 25;
    tuning.downsample     = true;

    const auto cfg = build_runner_config(entry_of(kLeftImucam),
                                         entry_of(kRightImucamEqui),
                                         test_noise(), tuning);

    // Left: radtan, intrinsics verbatim (builder never halves — the
    // VioManagerOptions mapping does, matching upstream's loader).
    EXPECT_FALSE(cfg.left.equidistant);
    EXPECT_DOUBLE_EQ(cfg.left.fxfycxcy[0], 1465.3);
    EXPECT_DOUBLE_EQ(cfg.left.fxfycxcy[3], 767.2);
    EXPECT_DOUBLE_EQ(cfg.left.dist[0], -0.28);
    EXPECT_EQ(cfg.left.wh[0], 2048u);
    EXPECT_EQ(cfg.left.wh[1], 1536u);
    EXPECT_DOUBLE_EQ(cfg.left.T_cam_imu[0][1], -1.0);
    EXPECT_DOUBLE_EQ(cfg.left.T_cam_imu[0][3], 0.01);

    // Right: equidistant flagged.
    EXPECT_TRUE(cfg.right.equidistant);
    EXPECT_DOUBLE_EQ(cfg.right.T_cam_imu[0][3], -0.05);

    // Noise passthrough (Kalibr continuous-time units, no inflation here).
    EXPECT_DOUBLE_EQ(cfg.sigma_w, 2.4e-4);
    EXPECT_DOUBLE_EQ(cfg.sigma_wb, 2.7e-5);
    EXPECT_DOUBLE_EQ(cfg.sigma_a, 1.7e-3);
    EXPECT_DOUBLE_EQ(cfg.sigma_ab, 4.4e-4);

    // Time offset = LEFT camera's timeshift.
    EXPECT_DOUBLE_EQ(cfg.calib_camimu_dt, 4.2e-05);

    EXPECT_EQ(cfg.num_pts, 175);
    EXPECT_EQ(cfg.fast_threshold, 25);
    EXPECT_TRUE(cfg.downsample);
    EXPECT_EQ(cfg.max_clone_size, 11);
}

TEST(VioConfigBuilderTest, ThrowsWithoutExtrinsicsOrNoise) {
    constexpr const char* kIntrinsicsOnly = R"(cam0:
  camera_model: pinhole
  distortion_coeffs: [0, 0, 0, 0]
  distortion_model: radtan
  intrinsics: [1465.3, 1466.1, 1023.5, 767.2]
  resolution: [2048, 1536]
)";
    EXPECT_THROW(build_runner_config(entry_of(kIntrinsicsOnly),
                                     entry_of(kRightImucamEqui), test_noise(), {}),
                 VioConfigError);

    VioImuNoise unset;
    EXPECT_THROW(build_runner_config(entry_of(kLeftImucam),
                                     entry_of(kRightImucamEqui), unset, {}),
                 VioConfigError);
}

TEST(VioConfigBuilderTest, CovConversionMatchesHandComputedJPJt) {
    // R_GtoI = 90° about Z. J = blkdiag(I, R).
    const gw::apriltag::Mat3 R = {{{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}}};

    // P with distinct diagonal + one cross term.
    std::array<double, 36> P{};
    const double dθ[3] = {0.01, 0.02, 0.03};
    const double dp[3] = {0.5, 1.0, 2.0};
    for (int i = 0; i < 3; ++i) P[i * 6 + i] = dθ[i];
    for (int i = 0; i < 3; ++i) P[(3 + i) * 6 + (3 + i)] = dp[i];
    P[0 * 6 + 3] = P[3 * 6 + 0] = 0.1;  // cross θx↔px

    const auto S = cov_ov_to_body_tangent(R, P);

    // Rotation block unchanged.
    EXPECT_DOUBLE_EQ(S[0], 0.01);
    EXPECT_DOUBLE_EQ(S[7], 0.02);
    EXPECT_DOUBLE_EQ(S[14], 0.03);

    // Σtt = R·diag(dp)·Rᵀ: with this R, global px variance lands on body x'
    // = (0,1,0)·p... compute: R row0=(0,1,0) → var = dp_y = 1.0;
    // row1=(-1,0,0) → dp_x = 0.5; row2 → dp_z = 2.0.
    EXPECT_NEAR(S[3 * 6 + 3], 1.0, 1e-12);
    EXPECT_NEAR(S[4 * 6 + 4], 0.5, 1e-12);
    EXPECT_NEAR(S[5 * 6 + 5], 2.0, 1e-12);

    // Cross block: Σθt = Pθp·Rᵀ. Pθp row0 = (0.1, 0, 0);
    // (Pθp·Rᵀ)[0][c] = Σ_k Pθp[0][k]·R[c][k] → c=0: R[0][0]*0.1 = 0;
    // c=1: R[1][0]*0.1 = -0.1; c=2: 0.
    EXPECT_NEAR(S[0 * 6 + 3], 0.0, 1e-12);
    EXPECT_NEAR(S[0 * 6 + 4], -0.1, 1e-12);
    // Symmetry.
    EXPECT_NEAR(S[4 * 6 + 0], -0.1, 1e-12);

    // Identity rotation = identity transform.
    const auto S_id = cov_ov_to_body_tangent(
        {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}, P);
    for (int i = 0; i < 36; ++i) EXPECT_DOUBLE_EQ(S_id[i], P[i]);
}

TEST(VioConfigBuilderTest, GuessworkMetaParses) {
    const auto meta = gw::calib::parse_guesswork_meta(kLeftImucam);
    ASSERT_TRUE(meta.has_value());
    ASSERT_TRUE(meta->reprojection_error_std_px.has_value());
    EXPECT_DOUBLE_EQ(*meta->reprojection_error_std_px, 0.35);
    ASSERT_TRUE(meta->session_id.has_value());
    EXPECT_EQ(*meta->session_id, "20260610-101010-123");
    ASSERT_TRUE(meta->source_cam_index.has_value());
    EXPECT_EQ(*meta->source_cam_index, 0);

    // No block → nullopt.
    EXPECT_FALSE(gw::calib::parse_guesswork_meta(kRightImucamEqui).has_value());
}

}  // namespace gw::vio
