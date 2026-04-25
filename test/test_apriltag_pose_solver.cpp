#include <array>
#include <cmath>
#include <vector>

#include <apriltag/apriltag.h>
#include <apriltag/common/image_u8.h>
#include <apriltag/common/zarray.h>
#include <apriltag/tag36h11.h>
#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "posest/pipelines/AprilTagPipeline.h"
#include "posest/pipelines/AprilTagPoseSolver.h"

namespace {

constexpr double kFx = 500.0;
constexpr double kFy = 500.0;
constexpr double kCx = 320.0;
constexpr double kCy = 320.0;
constexpr double kTagSize = 0.1;

cv::Matx33d eulerXYZ(double roll, double pitch, double yaw) {
    const double cr = std::cos(roll), sr = std::sin(roll);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cyw = std::cos(yaw), syw = std::sin(yaw);
    return {
        cyw * cp, cyw * sp * sr - syw * cr, cyw * sp * cr + syw * sr,
        syw * cp, syw * sp * sr + cyw * cr, syw * sp * cr - cyw * sr,
        -sp,      cp * sr,                  cp * cr,
    };
}

cv::Affine3d poseToAffine(const posest::Pose3d& p) {
    return cv::Affine3d(
        eulerXYZ(p.rotation_rpy_rad.x, p.rotation_rpy_rad.y, p.rotation_rpy_rad.z),
        cv::Vec3d(p.translation_m.x, p.translation_m.y, p.translation_m.z));
}

std::array<cv::Point3d, 4> tagFrameCorners(double size) {
    const double h = size * 0.5;
    return {{
        cv::Point3d(-h,  h, 0.0),
        cv::Point3d( h,  h, 0.0),
        cv::Point3d( h, -h, 0.0),
        cv::Point3d(-h, -h, 0.0),
    }};
}

posest::pipelines::AprilTagCameraCalibration makePinholeCalibration(
    std::vector<double> dist = {},
    std::string model = {}) {
    posest::pipelines::AprilTagCameraCalibration c;
    c.fx = kFx;
    c.fy = kFy;
    c.cx = kCx;
    c.cy = kCy;
    c.distortion_model = std::move(model);
    c.distortion_coefficients = std::move(dist);
    return c;
}

cv::Mat distMatFromVector(const std::vector<double>& v) {
    if (v.empty()) {
        return {};
    }
    cv::Mat D(static_cast<int>(v.size()), 1, CV_64F);
    for (std::size_t i = 0; i < v.size(); ++i) {
        D.at<double>(static_cast<int>(i)) = v[i];
    }
    return D;
}

// Project a 3D field point to image pixels using pinhole + radtan distortion
// (or empty distortion -> pinhole only).
cv::Point2d projectPinholeOrRadtan(
    const cv::Vec3d& p_field,
    const cv::Affine3d& cam_T_field,
    const cv::Mat& K,
    const cv::Mat& dist) {
    std::vector<cv::Point3d> obj{cv::Point3d(p_field[0], p_field[1], p_field[2])};
    std::vector<cv::Point2d> img;
    cv::Mat rvec;
    cv::Rodrigues(cv::Mat(cam_T_field.rotation()), rvec);
    cv::Mat tvec = cv::Mat(cam_T_field.translation()).clone();
    cv::projectPoints(obj, rvec, tvec, K, dist, img);
    return img.front();
}

cv::Point2d projectFisheye(
    const cv::Vec3d& p_field,
    const cv::Affine3d& cam_T_field,
    const cv::Mat& K,
    const cv::Mat& dist) {
    std::vector<cv::Point3d> obj{cv::Point3d(p_field[0], p_field[1], p_field[2])};
    std::vector<cv::Point2d> img;
    cv::Mat rvec;
    cv::Rodrigues(cv::Mat(cam_T_field.rotation()), rvec);
    cv::Mat tvec = cv::Mat(cam_T_field.translation()).clone();
    cv::fisheye::projectPoints(obj, img, rvec, tvec, K, dist);
    return img.front();
}

// Build a solver TagInput by placing a tag at field_to_tag, projecting its 4
// corners through the given camera (cam_T_field, intrinsics + distortion), and
// returning an input with raw=nullptr (multi-tag SQPNP path doesn't need raw).
posest::pipelines::AprilTagPoseSolveInput::TagInput makeProjectedTagInput(
    int tag_id,
    const posest::Pose3d& field_to_tag,
    const cv::Affine3d& cam_T_field,
    const cv::Mat& K,
    const cv::Mat& dist,
    bool fisheye = false) {
    posest::pipelines::AprilTagPoseSolveInput::TagInput in;
    in.tag_id = tag_id;
    in.field_to_tag = field_to_tag;
    in.raw = nullptr;
    const auto field_T_tag = poseToAffine(field_to_tag);
    const auto corners = tagFrameCorners(kTagSize);
    for (std::size_t i = 0; i < 4; ++i) {
        const cv::Vec3d p_field =
            field_T_tag * cv::Vec3d(corners[i].x, corners[i].y, corners[i].z);
        const cv::Point2d uv = fisheye
            ? projectFisheye(p_field, cam_T_field, K, dist)
            : projectPinholeOrRadtan(p_field, cam_T_field, K, dist);
        in.image_corners_px[i] = {uv.x, uv.y};
    }
    return in;
}

posest::pipelines::AprilTagPoseSolveInput makeBaseInput(
    posest::pipelines::AprilTagCameraCalibration calibration,
    posest::Pose3d camera_to_robot = {}) {
    posest::pipelines::AprilTagPoseSolveInput input;
    input.calibration = std::move(calibration);
    input.camera_to_robot = camera_to_robot;
    input.tag_size_m = kTagSize;
    input.covariance = posest::pipelines::AprilTagCovarianceTuning{};
    return input;
}

cv::Mat makeIntrinsicMat() {
    return (cv::Mat_<double>(3, 3) << kFx, 0.0, kCx, 0.0, kFy, kCy, 0.0, 0.0, 1.0);
}

}  // namespace

TEST(AprilTagPoseSolver, MultiTagSqpnpRecoversIdentityPose) {
    auto input = makeBaseInput(makePinholeCalibration());
    const cv::Affine3d cam_T_field = cv::Affine3d::Identity();
    const cv::Mat K = makeIntrinsicMat();

    const posest::Pose3d tag1{.translation_m = {-0.3, 0.0, 1.0}};
    const posest::Pose3d tag2{.translation_m = {0.3, 0.0, 1.0}};
    input.tags.push_back(makeProjectedTagInput(1, tag1, cam_T_field, K, {}));
    input.tags.push_back(makeProjectedTagInput(2, tag2, cam_T_field, K, {}));

    const auto out = posest::pipelines::solveAprilTagPose(input);
    ASSERT_TRUE(out.field_to_robot.has_value());
    EXPECT_EQ(out.solved_tag_count, 2);
    EXPECT_NEAR(out.field_to_robot->translation_m.x, 0.0, 1e-3);
    EXPECT_NEAR(out.field_to_robot->translation_m.y, 0.0, 1e-3);
    EXPECT_NEAR(out.field_to_robot->translation_m.z, 0.0, 1e-3);
    EXPECT_NEAR(out.field_to_robot->rotation_rpy_rad.x, 0.0, 1e-3);
    EXPECT_NEAR(out.field_to_robot->rotation_rpy_rad.y, 0.0, 1e-3);
    EXPECT_NEAR(out.field_to_robot->rotation_rpy_rad.z, 0.0, 1e-3);
    EXPECT_LT(out.reprojection_rms_px, 0.5);
    // Diagonal covariance positive.
    for (int i = 0; i < 6; ++i) {
        EXPECT_GT(out.covariance[static_cast<std::size_t>(i * 6 + i)], 0.0);
    }
}

TEST(AprilTagPoseSolver, MultiTagSqpnpRecoversTranslatedRobot) {
    auto input = makeBaseInput(makePinholeCalibration());
    // Camera (== robot) at field position (0.5, 0, 0), no rotation.
    const cv::Affine3d field_T_camera(
        cv::Matx33d::eye(), cv::Vec3d(0.5, 0.0, 0.0));
    const cv::Affine3d cam_T_field = field_T_camera.inv();
    const cv::Mat K = makeIntrinsicMat();

    const posest::Pose3d tag1{.translation_m = {0.2, 0.0, 1.0}};
    const posest::Pose3d tag2{.translation_m = {0.8, 0.0, 1.0}};
    input.tags.push_back(makeProjectedTagInput(1, tag1, cam_T_field, K, {}));
    input.tags.push_back(makeProjectedTagInput(2, tag2, cam_T_field, K, {}));

    const auto out = posest::pipelines::solveAprilTagPose(input);
    ASSERT_TRUE(out.field_to_robot.has_value());
    EXPECT_NEAR(out.field_to_robot->translation_m.x, 0.5, 1e-3);
    EXPECT_NEAR(out.field_to_robot->translation_m.y, 0.0, 1e-3);
    EXPECT_NEAR(out.field_to_robot->translation_m.z, 0.0, 1e-3);
}

TEST(AprilTagPoseSolver, MultiTagSqpnpHandlesRadtanDistortion) {
    const std::vector<double> dist{0.05, -0.02, 0.0, 0.0, 0.0};
    auto input = makeBaseInput(makePinholeCalibration(dist, "radtan"));
    const cv::Affine3d cam_T_field = cv::Affine3d::Identity();
    const cv::Mat K = makeIntrinsicMat();
    const cv::Mat D = distMatFromVector(dist);

    const posest::Pose3d tag1{.translation_m = {-0.3, 0.0, 1.0}};
    const posest::Pose3d tag2{.translation_m = {0.3, 0.0, 1.0}};
    input.tags.push_back(makeProjectedTagInput(1, tag1, cam_T_field, K, D));
    input.tags.push_back(makeProjectedTagInput(2, tag2, cam_T_field, K, D));

    const auto out = posest::pipelines::solveAprilTagPose(input);
    ASSERT_TRUE(out.field_to_robot.has_value());
    EXPECT_NEAR(out.field_to_robot->translation_m.x, 0.0, 5e-3);
    EXPECT_NEAR(out.field_to_robot->translation_m.y, 0.0, 5e-3);
    EXPECT_NEAR(out.field_to_robot->translation_m.z, 0.0, 5e-3);
}

TEST(AprilTagPoseSolver, MultiTagSqpnpHandlesEquidistant) {
    const std::vector<double> dist{0.01, -0.005, 0.001, 0.0};
    auto input = makeBaseInput(makePinholeCalibration(dist, "equidistant"));
    const cv::Affine3d cam_T_field = cv::Affine3d::Identity();
    const cv::Mat K = makeIntrinsicMat();
    const cv::Mat D = distMatFromVector(dist);

    const posest::Pose3d tag1{.translation_m = {-0.3, 0.0, 1.0}};
    const posest::Pose3d tag2{.translation_m = {0.3, 0.0, 1.0}};
    input.tags.push_back(
        makeProjectedTagInput(1, tag1, cam_T_field, K, D, /*fisheye=*/true));
    input.tags.push_back(
        makeProjectedTagInput(2, tag2, cam_T_field, K, D, /*fisheye=*/true));

    const auto out = posest::pipelines::solveAprilTagPose(input);
    ASSERT_TRUE(out.field_to_robot.has_value());
    EXPECT_NEAR(out.field_to_robot->translation_m.x, 0.0, 5e-3);
    EXPECT_NEAR(out.field_to_robot->translation_m.y, 0.0, 5e-3);
    EXPECT_NEAR(out.field_to_robot->translation_m.z, 0.0, 5e-3);
}

TEST(AprilTagPoseSolver, EmptyInputProducesEmptyOutput) {
    auto input = makeBaseInput(makePinholeCalibration());
    const auto out = posest::pipelines::solveAprilTagPose(input);
    EXPECT_FALSE(out.field_to_robot.has_value());
    EXPECT_EQ(out.solved_tag_count, 0);
}

TEST(AprilTagPoseSolver, NoCalibrationProducesEmptyOutput) {
    posest::pipelines::AprilTagPoseSolveInput input;
    // intrinsics fx=fy=0 by default, treated as missing.
    input.tag_size_m = kTagSize;
    posest::pipelines::AprilTagPoseSolveInput::TagInput t;
    t.tag_id = 0;
    t.image_corners_px = {{ {0,0}, {1,0}, {1,1}, {0,1} }};
    input.tags.push_back(t);

    const auto out = posest::pipelines::solveAprilTagPose(input);
    EXPECT_FALSE(out.field_to_robot.has_value());
}

TEST(AprilTagPoseSolver, CovarianceScalesQuadraticallyWithDistance) {
    auto base = makeBaseInput(makePinholeCalibration());
    const cv::Mat K = makeIntrinsicMat();
    const cv::Affine3d cam_T_field = cv::Affine3d::Identity();

    // Near case: 2 tags at z = 1.0
    auto near = base;
    near.tags.push_back(makeProjectedTagInput(
        1, posest::Pose3d{.translation_m = {-0.3, 0.0, 1.0}}, cam_T_field, K, {}));
    near.tags.push_back(makeProjectedTagInput(
        2, posest::Pose3d{.translation_m = {0.3, 0.0, 1.0}}, cam_T_field, K, {}));

    // Far case: 2 tags at z = 2.0 (twice as far)
    auto far = base;
    far.tags.push_back(makeProjectedTagInput(
        1, posest::Pose3d{.translation_m = {-0.6, 0.0, 2.0}}, cam_T_field, K, {}));
    far.tags.push_back(makeProjectedTagInput(
        2, posest::Pose3d{.translation_m = {0.6, 0.0, 2.0}}, cam_T_field, K, {}));

    const auto out_near = posest::pipelines::solveAprilTagPose(near);
    const auto out_far = posest::pipelines::solveAprilTagPose(far);
    ASSERT_TRUE(out_near.field_to_robot.has_value());
    ASSERT_TRUE(out_far.field_to_robot.has_value());

    const double trans_near = out_near.covariance[3 * 6 + 3];
    const double trans_far = out_far.covariance[3 * 6 + 3];
    // Distance factor scales like (d/ref)^2, so going 1.0 m -> 2.0 m should
    // increase distance_factor from 1 to 4 => sigma scales 4x => variance 16x.
    EXPECT_GT(trans_far, trans_near * 8.0);
}

TEST(AprilTagPoseSolver, CovarianceScalesWithReprojectionRms) {
    auto base = makeBaseInput(makePinholeCalibration());
    // Lower the rms reference so any small residual amplifies the covariance.
    base.covariance.reference_rms_px = 0.01;
    const cv::Mat K = makeIntrinsicMat();
    const cv::Affine3d cam_T_field = cv::Affine3d::Identity();

    auto clean = base;
    clean.tags.push_back(makeProjectedTagInput(
        1, posest::Pose3d{.translation_m = {-0.3, 0.0, 1.0}}, cam_T_field, K, {}));
    clean.tags.push_back(makeProjectedTagInput(
        2, posest::Pose3d{.translation_m = {0.3, 0.0, 1.0}}, cam_T_field, K, {}));

    auto noisy = clean;
    // Apply a non-rigid perturbation pattern that no rigid pose can absorb:
    // alternating x/y shifts on corners of both tags.
    const std::array<cv::Point2d, 4> shifts{{
        {3.0, 0.0}, {-3.0, 0.0}, {0.0, 3.0}, {0.0, -3.0},
    }};
    for (auto& t : noisy.tags) {
        for (std::size_t i = 0; i < 4; ++i) {
            t.image_corners_px[i].x += shifts[i].x;
            t.image_corners_px[i].y += shifts[i].y;
        }
    }

    const auto out_clean = posest::pipelines::solveAprilTagPose(clean);
    const auto out_noisy = posest::pipelines::solveAprilTagPose(noisy);
    ASSERT_TRUE(out_clean.field_to_robot.has_value());
    ASSERT_TRUE(out_noisy.field_to_robot.has_value());
    EXPECT_GT(out_noisy.reprojection_rms_px, out_clean.reprojection_rms_px);
    EXPECT_GT(
        out_noisy.covariance[3 * 6 + 3],
        out_clean.covariance[3 * 6 + 3]);
}

TEST(AprilTagPoseSolver, SingleTagPathReportsAmbiguityFromOrthogonalIteration) {
    // Render a tag, run libapriltag detector, take the detection, feed it into
    // the solver via the single-tag (orthogonal-iteration) path.
    apriltag_family_t* family = tag36h11_create();
    image_u8_t* rendered = apriltag_to_image(family, 0u);
    cv::Mat tag(
        rendered->height, rendered->width, CV_8UC1, rendered->buf,
        static_cast<std::size_t>(rendered->stride));
    cv::Mat scaled;
    cv::resize(tag, scaled, cv::Size(200, 200), 0.0, 0.0, cv::INTER_NEAREST);
    cv::Mat canvas(640, 640, CV_8UC1, cv::Scalar(255));
    scaled.copyTo(canvas(cv::Rect(220, 220, 200, 200)));

    apriltag_detector_t* detector = apriltag_detector_create();
    apriltag_detector_add_family(detector, family);
    detector->quad_decimate = 1.0f;

    image_u8_t image{
        canvas.cols, canvas.rows,
        static_cast<int32_t>(canvas.step[0]), canvas.data,
    };
    zarray_t* detections = apriltag_detector_detect(detector, &image);
    ASSERT_GE(zarray_size(detections), 1);
    apriltag_detection_t* det = nullptr;
    zarray_get(detections, 0, &det);
    ASSERT_NE(det, nullptr);

    auto input = makeBaseInput(makePinholeCalibration());
    posest::pipelines::AprilTagPoseSolveInput::TagInput t;
    t.tag_id = det->id;
    for (std::size_t i = 0; i < 4; ++i) {
        t.image_corners_px[i] = {det->p[i][0], det->p[i][1]};
    }
    t.field_to_tag = posest::Pose3d{.translation_m = {0.0, 0.0, 1.0}};
    t.raw = det;
    input.tags.push_back(t);

    const auto out = posest::pipelines::solveAprilTagPose(input);
    EXPECT_EQ(out.solved_tag_count, out.field_to_robot.has_value() ? 1 : 0);
    ASSERT_TRUE(out.single_tag_ambiguity.has_value());
    EXPECT_GE(*out.single_tag_ambiguity, 0.0);
    EXPECT_LE(*out.single_tag_ambiguity, 1.0);

    apriltag_detections_destroy(detections);
    apriltag_detector_destroy(detector);
    image_u8_destroy(rendered);
    tag36h11_destroy(family);
}

TEST(AprilTagPoseSolver, SingleTagDropsWhenThresholdBelowReportedAmbiguity) {
    apriltag_family_t* family = tag36h11_create();
    image_u8_t* rendered = apriltag_to_image(family, 0u);
    cv::Mat tag(
        rendered->height, rendered->width, CV_8UC1, rendered->buf,
        static_cast<std::size_t>(rendered->stride));
    cv::Mat scaled;
    cv::resize(tag, scaled, cv::Size(200, 200), 0.0, 0.0, cv::INTER_NEAREST);
    cv::Mat canvas(640, 640, CV_8UC1, cv::Scalar(255));
    scaled.copyTo(canvas(cv::Rect(220, 220, 200, 200)));

    apriltag_detector_t* detector = apriltag_detector_create();
    apriltag_detector_add_family(detector, family);
    detector->quad_decimate = 1.0f;

    image_u8_t image{
        canvas.cols, canvas.rows,
        static_cast<int32_t>(canvas.step[0]), canvas.data,
    };
    zarray_t* detections = apriltag_detector_detect(detector, &image);
    ASSERT_GE(zarray_size(detections), 1);
    apriltag_detection_t* det = nullptr;
    zarray_get(detections, 0, &det);
    ASSERT_NE(det, nullptr);

    auto base = makeBaseInput(makePinholeCalibration());
    posest::pipelines::AprilTagPoseSolveInput::TagInput tag_input;
    tag_input.tag_id = det->id;
    for (std::size_t i = 0; i < 4; ++i) {
        tag_input.image_corners_px[i] = {det->p[i][0], det->p[i][1]};
    }
    tag_input.field_to_tag = posest::Pose3d{.translation_m = {0.0, 0.0, 1.0}};
    tag_input.raw = det;

    // Loose threshold: orthogonal iteration's pose should always be emitted.
    auto loose_input = base;
    loose_input.covariance.ambiguity_drop_threshold = 1.0;
    loose_input.tags.push_back(tag_input);
    const auto loose = posest::pipelines::solveAprilTagPose(loose_input);
    ASSERT_TRUE(loose.single_tag_ambiguity.has_value());
    const double observed_ambiguity = *loose.single_tag_ambiguity;

    if (observed_ambiguity > 1e-6) {
        // Tight threshold below the observed ambiguity: pose must be dropped.
        auto tight_input = base;
        tight_input.covariance.ambiguity_drop_threshold = observed_ambiguity / 2.0;
        tight_input.tags.push_back(tag_input);
        const auto tight = posest::pipelines::solveAprilTagPose(tight_input);
        EXPECT_FALSE(tight.field_to_robot.has_value());
        ASSERT_TRUE(tight.single_tag_ambiguity.has_value());
        EXPECT_GT(*tight.single_tag_ambiguity, tight_input.covariance.ambiguity_drop_threshold);
    } else {
        // libapriltag found no second hypothesis (unambiguous pose). The drop
        // path can't be exercised; just confirm the pose came through.
        EXPECT_TRUE(loose.field_to_robot.has_value());
    }

    apriltag_detections_destroy(detections);
    apriltag_detector_destroy(detector);
    image_u8_destroy(rendered);
    tag36h11_destroy(family);
}
