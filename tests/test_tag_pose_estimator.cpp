#include <gtest/gtest.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <cmath>
#include <random>
#include <vector>

#include "apriltag/field_layout.hpp"
#include "apriltag/pose_math.hpp"
#include "apriltag/tag_pose_estimator.hpp"

namespace gw::apriltag {

namespace {

// ---------------------------------------------------------------------------
// Synthetic ground-truth machinery: choose camera + transforms, project
// field-frame tag corners through the full distortion model, add pixel
// noise, and feed the estimator. No hardware, fully deterministic.
// ---------------------------------------------------------------------------

PinholeCamera fixture_camera() {
    PinholeCamera cam;
    cam.fxfycxcy = {1100.0, 1100.0, 1024.0, 768.0};
    cam.model    = PinholeCamera::Dist::kRadTan;
    cam.d        = {-0.18, 0.07, 1e-4, 1e-4};
    cam.width    = 2048;
    cam.height   = 1536;
    return cam;
}

// Wall of tags at x = 10 facing toward blue (−X, i.e. toward the camera),
// various y/z. Quaternion (0,0,0,1) = 180° about Z.
FieldLayout fixture_layout() {
    FieldLayout layout;
    layout.length_m = 16.541;
    layout.width_m  = 8.069;
    const Mat3 R = quat_wxyz_to_mat3(0, 0, 0, 1);
    const std::array<std::array<double, 3>, 4> positions = {{
        {10.0, 3.0, 1.0}, {10.0, 4.0, 1.2}, {10.0, 5.0, 0.9}, {10.0, 3.6, 1.6}}};
    for (int i = 0; i < 4; ++i) {
        layout.tags.push_back({i + 1, mat4_from_rt(R, positions[i])});
    }
    return layout;
}

// Camera at `eye` looking along field +X with +Z up: cam axes in field are
// X=(0,−1,0) (right), Y=(0,0,−1) (down), Z=(1,0,0) (forward).
Mat4 t_field_cam_looking_plus_x(const std::array<double, 3>& eye) {
    const Mat3 R = {{{0, 0, 1}, {-1, 0, 0}, {0, -1, 0}}};  // columns = cam axes
    return mat4_from_rt(R, eye);
}

// Bench extrinsics: deliberately non-identity so a chain bug can't cancel.
Mat4 fixture_t_cam_imu() {
    return retract(mat4_identity(), {0.02, -0.03, 0.05, 0.10, -0.06, 0.04});
}
Mat4 fixture_t_robot_imu() {
    return retract(mat4_identity(), {0.0, 0.0, M_PI / 2, 0.20, 0.05, 0.30});
}

struct Scene {
    PinholeCamera  cam   = fixture_camera();
    PreparedLayout layout;
    Mat4           T_cam_robot   = mat4_identity();
    Mat4           T_field_robot = mat4_identity();  // ground truth
    Mat4           T_cam_field   = mat4_identity();
};

Scene make_scene(const std::array<double, 3>& cam_eye = {7.0, 4.0, 0.5}) {
    Scene s;
    s.layout = prepare_layout(fixture_layout());

    const Mat4 T_field_cam = t_field_cam_looking_plus_x(cam_eye);
    s.T_cam_field          = mat4_inverse_se3(T_field_cam);

    const Mat4 T_cam_imu   = fixture_t_cam_imu();
    const Mat4 T_robot_imu = fixture_t_robot_imu();
    s.T_cam_robot = mat4_mul(T_cam_imu, mat4_inverse_se3(T_robot_imu));
    // Ground truth via the exact chain the estimator inverts.
    s.T_field_robot = mat4_mul(T_field_cam, s.T_cam_robot);
    return s;
}

// Projects one tag's field corners through the camera WITH distortion.
TagObservation project_tag(const Scene& s, int tag_id, std::mt19937& rng,
                           double noise_sigma_px) {
    const auto& fc = s.layout.corners.at(tag_id);
    std::vector<cv::Point3d> obj;
    for (const auto& c : fc) obj.emplace_back(c[0], c[1], c[2]);

    cv::Matx33d R;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) R(r, c) = s.T_cam_field[r][c];
    cv::Vec3d rvec;
    cv::Rodrigues(R, rvec);
    const cv::Vec3d tvec{s.T_cam_field[0][3], s.T_cam_field[1][3],
                         s.T_cam_field[2][3]};

    const cv::Matx33d K{s.cam.fxfycxcy[0], 0, s.cam.fxfycxcy[2],
                        0, s.cam.fxfycxcy[1], s.cam.fxfycxcy[3],
                        0, 0, 1};
    const cv::Mat D(1, 4, CV_64F, const_cast<double*>(s.cam.d.data()));
    std::vector<cv::Point2d> px;
    cv::projectPoints(obj, rvec, tvec, K, D, px);

    std::normal_distribution<double> noise(0.0, noise_sigma_px);
    TagObservation obs;
    obs.id              = tag_id;
    obs.decision_margin = 80.0;
    for (int i = 0; i < 4; ++i) {
        obs.corners_px[i] = {px[i].x + (noise_sigma_px > 0 ? noise(rng) : 0.0),
                             px[i].y + (noise_sigma_px > 0 ? noise(rng) : 0.0)};
    }
    return obs;
}

double translation_err_m(const Mat4& a, const Mat4& b) {
    double s = 0;
    for (int r = 0; r < 3; ++r) s += (a[r][3] - b[r][3]) * (a[r][3] - b[r][3]);
    return std::sqrt(s);
}

double rotation_err_rad(const Mat4& a, const Mat4& b) {
    const auto xi = log_se3(mat4_mul(mat4_inverse_se3(a), b));
    return std::sqrt(xi[0] * xi[0] + xi[1] * xi[1] + xi[2] * xi[2]);
}

}  // namespace

TEST(TagPoseEstimatorTest, MultiTagAccuracy) {
    const Scene  s = make_scene();
    std::mt19937 rng{42};

    std::vector<TagObservation> obs;
    for (int id = 1; id <= 4; ++id) obs.push_back(project_tag(s, id, rng, 0.5));

    EstimatorConfig cfg;
    cfg.sigma_px = 0.5;
    const auto r = estimate_robot_pose(obs, s.cam, s.layout, s.T_cam_robot, cfg);
    ASSERT_TRUE(r.pose.has_value()) << "skip=" << to_string(r.skip);
    EXPECT_EQ(r.pose->n_tags, 4u);
    EXPECT_EQ(r.pose->tag_ids.size(), 4u);
    EXPECT_LT(translation_err_m(r.pose->T_field_robot, s.T_field_robot), 0.05);
    EXPECT_LT(rotation_err_rad(r.pose->T_field_robot, s.T_field_robot),
              1.0 * M_PI / 180.0);
    EXPECT_LT(r.pose->mean_reproj_err_px, 2.0);
}

// Covariance consistency: NEES = ξᵀΣ⁻¹ξ with ξ = log(T_gt⁻¹ · T_est) (the
// body-tangent error matching the covariance convention) must be χ²₆-
// distributed if Σ is honest. Mean over 500 draws ∈ [5.0, 7.5]
// (E[χ²₆] = 6, sd of the mean = √(12/500) ≈ 0.155; bounds are ±4σ plus
// linearization slack), and >95 % of draws below the 99th percentile.
TEST(TagPoseEstimatorTest, CovarianceConsistencyNees) {
    const Scene  s = make_scene();
    std::mt19937 rng{42};

    EstimatorConfig cfg;
    cfg.sigma_px = 0.5;

    constexpr int       kDraws = 500;
    std::vector<double> nees;
    nees.reserve(kDraws);
    for (int draw = 0; draw < kDraws; ++draw) {
        std::vector<TagObservation> obs;
        for (int id = 1; id <= 4; ++id) obs.push_back(project_tag(s, id, rng, 0.5));
        const auto r = estimate_robot_pose(obs, s.cam, s.layout, s.T_cam_robot, cfg);
        ASSERT_TRUE(r.pose.has_value());

        const auto xi = log_se3(mat4_mul(mat4_inverse_se3(s.T_field_robot),
                                         r.pose->T_field_robot));
        cv::Matx66d cov;
        for (int i = 0; i < 36; ++i) cov(i / 6, i % 6) = r.pose->cov[i];
        cv::Matx66d info;
        cv::invert(cov, info, cv::DECOMP_SVD);
        double v = 0;
        for (int a = 0; a < 6; ++a)
            for (int b = 0; b < 6; ++b) v += xi[a] * info(a, b) * xi[b];
        nees.push_back(v);
    }

    double mean = 0;
    int    below_99 = 0;
    for (double v : nees) {
        mean += v;
        if (v < 16.81) ++below_99;  // χ²₆ 99th percentile
    }
    mean /= kDraws;
    EXPECT_GT(mean, 5.0) << "covariance too conservative";
    EXPECT_LT(mean, 7.5) << "covariance overconfident";
    EXPECT_GT(static_cast<double>(below_99) / kDraws, 0.95);
}

TEST(TagPoseEstimatorTest, SingleTagObliqueAccuracy) {
    // View the tag wall from the side so the single-tag solve is well-
    // conditioned (~35° off the tag normal).
    const Scene  s = make_scene({7.0, 1.8, 0.9});
    std::mt19937 rng{7};

    std::vector<TagObservation> obs{project_tag(s, 1, rng, 0.5)};
    EstimatorConfig cfg;
    cfg.sigma_px = 0.5;
    const auto r = estimate_robot_pose(obs, s.cam, s.layout, s.T_cam_robot, cfg);
    ASSERT_TRUE(r.pose.has_value()) << "skip=" << to_string(r.skip);
    EXPECT_EQ(r.pose->n_tags, 1u);
    EXPECT_LT(translation_err_m(r.pose->T_field_robot, s.T_field_robot), 0.08);
    EXPECT_LT(rotation_err_rad(r.pose->T_field_robot, s.T_field_robot),
              2.0 * M_PI / 180.0);
}

TEST(TagPoseEstimatorTest, SingleTagUsesLayoutTagSize) {
    // Regression: the single-tag IPPE object square must be built from the
    // layout's tag size, not a config default. A 20 cm bench layout solved
    // against the hardcoded 6.5 in square gives a pose scaled by the size
    // ratio (~17% range error here).
    Scene s  = make_scene({7.0, 1.8, 0.9});
    s.layout = prepare_layout(fixture_layout(), /*tag_size_m=*/0.20);
    std::mt19937 rng{11};

    std::vector<TagObservation> obs{project_tag(s, 1, rng, 0.3)};
    EstimatorConfig cfg;
    cfg.sigma_px = 0.3;
    const auto r = estimate_robot_pose(obs, s.cam, s.layout, s.T_cam_robot, cfg);
    ASSERT_TRUE(r.pose.has_value()) << "skip=" << to_string(r.skip);
    EXPECT_EQ(r.pose->n_tags, 1u);
    // Bound chosen to separate ordinary single-tag noise (~9 cm at this
    // viewpoint) from the size-mismatch failure (0.1651/0.20 range scaling
    // ≈ 0.5 m at ~3.3 m).
    EXPECT_LT(translation_err_m(r.pose->T_field_robot, s.T_field_robot), 0.15);
    EXPECT_LT(rotation_err_rad(r.pose->T_field_robot, s.T_field_robot),
              2.0 * M_PI / 180.0);
}

TEST(TagPoseEstimatorTest, NearFrontoParallelSingleTagIsAmbiguous) {
    // Camera nearly dead-on in front of tag 1 at 4 m (a couple of degrees
    // off-axis — exactly fronto-parallel degenerates to a single IPPE
    // solution): the planar two-fold ambiguity is strong, both solutions
    // explain the noisy corners about equally well → must be gated.
    const Scene  s = make_scene({6.0, 3.1, 1.03});
    std::mt19937 rng{99};

    std::vector<TagObservation> obs{project_tag(s, 1, rng, 0.5)};
    const auto r = estimate_robot_pose(obs, s.cam, s.layout, s.T_cam_robot);
    EXPECT_FALSE(r.pose.has_value());
    EXPECT_EQ(r.skip, SkipReason::kAmbiguous);
}

TEST(TagPoseEstimatorTest, EquidistantModelPath) {
    Scene s    = make_scene();
    s.cam.model = PinholeCamera::Dist::kEquidistant;
    s.cam.d     = {0.02, -0.005, 0.001, -0.0002};

    // Project through the fisheye model to generate ground truth.
    std::mt19937 rng{11};
    std::vector<TagObservation> obs;
    for (int id = 1; id <= 4; ++id) {
        const auto& fc = s.layout.corners.at(id);
        std::vector<cv::Point3d> obj;
        for (const auto& c : fc) obj.emplace_back(c[0], c[1], c[2]);
        cv::Matx33d R;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) R(r, c) = s.T_cam_field[r][c];
        cv::Vec3d rvec;
        cv::Rodrigues(R, rvec);
        const cv::Vec3d tvec{s.T_cam_field[0][3], s.T_cam_field[1][3],
                             s.T_cam_field[2][3]};
        const cv::Matx33d K{s.cam.fxfycxcy[0], 0, s.cam.fxfycxcy[2],
                            0, s.cam.fxfycxcy[1], s.cam.fxfycxcy[3],
                            0, 0, 1};
        std::vector<cv::Point2d> px;
        cv::fisheye::projectPoints(obj, px, rvec, tvec, K,
                                   cv::Vec4d(s.cam.d[0], s.cam.d[1], s.cam.d[2],
                                             s.cam.d[3]));
        std::normal_distribution<double> noise(0.0, 0.5);
        TagObservation o;
        o.id              = id;
        o.decision_margin = 80.0;
        for (int i = 0; i < 4; ++i) o.corners_px[i] = {px[i].x + noise(rng),
                                                       px[i].y + noise(rng)};
        obs.push_back(o);
    }

    EstimatorConfig cfg;
    cfg.sigma_px = 0.5;
    const auto r = estimate_robot_pose(obs, s.cam, s.layout, s.T_cam_robot, cfg);
    ASSERT_TRUE(r.pose.has_value()) << "skip=" << to_string(r.skip);
    EXPECT_LT(translation_err_m(r.pose->T_field_robot, s.T_field_robot), 0.05);
}

TEST(TagPoseEstimatorTest, UnknownTagsOnlySkips) {
    const Scene  s = make_scene();
    std::mt19937 rng{5};
    auto obs = project_tag(s, 1, rng, 0.5);
    obs.id   = 99;  // not in the layout
    const auto r =
        estimate_robot_pose({obs}, s.cam, s.layout, s.T_cam_robot);
    EXPECT_FALSE(r.pose.has_value());
    EXPECT_EQ(r.skip, SkipReason::kNoKnownTags);
}

TEST(TagPoseEstimatorTest, CorruptedCornerFailsReprojGate) {
    const Scene  s = make_scene();
    std::mt19937 rng{5};
    std::vector<TagObservation> obs;
    for (int id = 1; id <= 4; ++id) obs.push_back(project_tag(s, id, rng, 0.5));
    obs[2].corners_px[1][0] += 40.0;  // gross outlier on one corner
    const auto r = estimate_robot_pose(obs, s.cam, s.layout, s.T_cam_robot);
    EXPECT_FALSE(r.pose.has_value());
    EXPECT_EQ(r.skip, SkipReason::kHighReprojErr);
}

TEST(TagPoseEstimatorTest, MultiTagCovarianceTighterThanSingle) {
    const Scene  s = make_scene({7.0, 1.8, 0.9});
    std::mt19937 rng{21};

    EstimatorConfig cfg;
    cfg.sigma_px = 0.5;
    // This test compares covariance scaling, not gating — disable the
    // ambiguity gate so the single-tag solve always goes through.
    cfg.ambiguity_min_ratio = 1.0;
    std::vector<TagObservation> all;
    for (int id = 1; id <= 4; ++id) all.push_back(project_tag(s, id, rng, 0.5));
    const auto multi  = estimate_robot_pose(all, s.cam, s.layout, s.T_cam_robot, cfg);
    const auto single = estimate_robot_pose({all[0]}, s.cam, s.layout,
                                            s.T_cam_robot, cfg);
    ASSERT_TRUE(multi.pose.has_value());
    ASSERT_TRUE(single.pose.has_value());
    // Compare translation-block traces.
    double tr_multi = 0, tr_single = 0;
    for (int i = 3; i < 6; ++i) {
        tr_multi  += multi.pose->cov[i * 6 + i];
        tr_single += single.pose->cov[i * 6 + i];
    }
    EXPECT_LT(tr_multi, tr_single);
}

TEST(TagPoseEstimatorTest, SingleTagRangeAccuracy) {
    const Scene  s = make_scene({7.0, 1.8, 0.9});
    std::mt19937 rng{13};
    const auto   obs = project_tag(s, 1, rng, 0.3);

    const auto range = single_tag_range_m(obs, s.cam, s.layout.tag_size_m);
    ASSERT_TRUE(range.has_value());

    // Ground-truth camera→tag-center distance.
    const auto& T = s.layout.by_id.at(1).T_field_tag;
    const Mat4  T_field_cam = mat4_inverse_se3(s.T_cam_field);
    double      d2          = 0;
    for (int r = 0; r < 3; ++r) {
        const double diff = T[r][3] - T_field_cam[r][3];
        d2 += diff * diff;
    }
    const double gt = std::sqrt(d2);
    EXPECT_NEAR(*range, gt, gt * 0.02);  // within 2 %
}

}  // namespace gw::apriltag
