#include "posest/pipelines/AprilTagPoseSolver.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/common/matd.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace posest::pipelines {

namespace {

bool isEquidistantModel(const std::string& model) {
    return model == "equidistant" || model == "fisheye" ||
           model == "kannala_brandt" || model == "kannala-brandt";
}

cv::Matx33d eulerXYZToMatrix(double roll, double pitch, double yaw) {
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    return {
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp,     cp * sr,                cp * cr,
    };
}

Vec3 matrixToEulerXYZ(const cv::Matx33d& R) {
    const double r00 = R(0, 0);
    const double r10 = R(1, 0);
    const double r20 = R(2, 0);
    const double r21 = R(2, 1);
    const double r22 = R(2, 2);
    const double r11 = R(1, 1);
    const double r12 = R(1, 2);

    const double sy = std::sqrt(r00 * r00 + r10 * r10);
    if (sy < 1e-9) {
        return {std::atan2(-r12, r11), std::atan2(-r20, sy), 0.0};
    }
    return {std::atan2(r21, r22), std::atan2(-r20, sy), std::atan2(r10, r00)};
}

cv::Affine3d pose3dToAffine(const Pose3d& pose) {
    const cv::Matx33d R = eulerXYZToMatrix(
        pose.rotation_rpy_rad.x, pose.rotation_rpy_rad.y, pose.rotation_rpy_rad.z);
    const cv::Vec3d t(pose.translation_m.x, pose.translation_m.y, pose.translation_m.z);
    return cv::Affine3d(R, t);
}

Pose3d affineToPose3d(const cv::Affine3d& aff) {
    const cv::Matx33d R = aff.rotation();
    const cv::Vec3d t = aff.translation();
    Pose3d out;
    out.translation_m = {t[0], t[1], t[2]};
    out.rotation_rpy_rad = matrixToEulerXYZ(R);
    return out;
}

Pose3d apriltagPoseToPose3d(const apriltag_pose_t& pose) {
    cv::Matx33d R;
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned j = 0; j < 3; ++j) {
            R(static_cast<int>(i), static_cast<int>(j)) = matd_get(pose.R, i, j);
        }
    }
    Pose3d out;
    out.translation_m = {
        matd_get(pose.t, 0, 0),
        matd_get(pose.t, 1, 0),
        matd_get(pose.t, 2, 0),
    };
    out.rotation_rpy_rad = matrixToEulerXYZ(R);
    return out;
}

cv::Mat makeIntrinsicMatrix(const AprilTagCameraCalibration& calibration) {
    cv::Mat K = (cv::Mat_<double>(3, 3) <<
        calibration.fx, 0.0, calibration.cx,
        0.0, calibration.fy, calibration.cy,
        0.0, 0.0, 1.0);
    return K;
}

cv::Mat makeDistCoeffs(const AprilTagCameraCalibration& calibration) {
    if (calibration.distortion_coefficients.empty()) {
        return {};
    }
    cv::Mat D(static_cast<int>(calibration.distortion_coefficients.size()), 1, CV_64F);
    for (std::size_t i = 0; i < calibration.distortion_coefficients.size(); ++i) {
        D.at<double>(static_cast<int>(i)) = calibration.distortion_coefficients[i];
    }
    return D;
}

std::array<cv::Point3d, 4> cornersInTagFrame(double tag_size_m) {
    const double half = tag_size_m * 0.5;
    return {{
        cv::Point3d(-half,  half, 0.0),
        cv::Point3d( half,  half, 0.0),
        cv::Point3d( half, -half, 0.0),
        cv::Point3d(-half, -half, 0.0),
    }};
}

void writeCovarianceDiagonal(
    std::array<double, 36>& out,
    double sigma_rot,
    double sigma_trans) {
    out.fill(0.0);
    const double var_r = sigma_rot * sigma_rot;
    const double var_t = sigma_trans * sigma_trans;
    out[0 * 6 + 0] = var_r;
    out[1 * 6 + 1] = var_r;
    out[2 * 6 + 2] = var_r;
    out[3 * 6 + 3] = var_t;
    out[4 * 6 + 4] = var_t;
    out[5 * 6 + 5] = var_t;
}

double smoothTagCountMult(int tag_count, double single_mult, double floor_mult, double decay_k) {
    const double N = static_cast<double>(std::max(1, tag_count));
    return floor_mult + (single_mult - floor_mult) *
        std::exp(-(N - 1.0) / std::max(1e-6, decay_k));
}

struct CovarianceFactors {
    double sigma_t{0.0};
    double sigma_r_base{0.0};
    double rot_multiplier{1.0};
};

CovarianceFactors covarianceFactors(
    double mean_distance_m,
    double rms_px,
    int tag_count,
    const AprilTagCovarianceTuning& tuning) {
    const double ref_d = std::max(tuning.reference_distance_m, 1e-9);
    const double ref_rms = std::max(tuning.reference_rms_px, 1e-9);
    const double distance_factor = std::max(
        1.0,
        (mean_distance_m * mean_distance_m) / (ref_d * ref_d));
    const double rms_factor = std::max(1.0, rms_px / ref_rms);
    const double trans_count_mult = smoothTagCountMult(
        tag_count,
        tuning.single_tag_translation_mult,
        tuning.well_spread_floor_mult,
        tuning.multi_tag_decay_k);
    const double rot_count_mult = smoothTagCountMult(
        tag_count,
        tuning.single_tag_rotation_mult,
        tuning.well_spread_floor_mult,
        tuning.multi_tag_decay_k);
    CovarianceFactors out;
    out.sigma_t =
        tuning.base_sigma_translation_m * distance_factor * rms_factor * trans_count_mult;
    out.sigma_r_base =
        tuning.base_sigma_rotation_rad * distance_factor * rms_factor;
    out.rot_multiplier = rot_count_mult;
    return out;
}

void computeCovariance(
    AprilTagPoseSolveOutput& out,
    double mean_distance_m,
    double rms_px,
    int tag_count,
    const AprilTagCovarianceTuning& tuning) {
    const auto f = covarianceFactors(mean_distance_m, rms_px, tag_count, tuning);
    writeCovarianceDiagonal(out.covariance, f.sigma_r_base * f.rot_multiplier, f.sigma_t);
}


bool intrinsicsArePresent(const AprilTagCameraCalibration& calibration) {
    return calibration.fx > 0.0 && calibration.fy > 0.0;
}

AprilTagPoseSolveOutput solveMultiTag(const AprilTagPoseSolveInput& input) {
    AprilTagPoseSolveOutput out;
    const auto tag_corners = cornersInTagFrame(input.tag_size_m);

    std::vector<cv::Point3d> object_points;
    std::vector<cv::Point2d> image_points;
    object_points.reserve(input.tags.size() * 4);
    image_points.reserve(input.tags.size() * 4);

    for (const auto& tag : input.tags) {
        const cv::Affine3d field_T_tag = pose3dToAffine(tag.field_to_tag);
        for (std::size_t c = 0; c < 4; ++c) {
            const cv::Vec3d p_field = field_T_tag * cv::Vec3d(
                tag_corners[c].x, tag_corners[c].y, tag_corners[c].z);
            object_points.emplace_back(p_field[0], p_field[1], p_field[2]);
            image_points.emplace_back(
                tag.image_corners_px[c].x,
                tag.image_corners_px[c].y);
        }
    }

    const cv::Mat K = makeIntrinsicMatrix(input.calibration);
    const cv::Mat D = makeDistCoeffs(input.calibration);

    std::vector<cv::Point2d> solve_image_points = image_points;
    cv::Mat solve_dist_coeffs = D;
    if (isEquidistantModel(input.calibration.distortion_model)) {
        if (D.empty()) {
            return out;
        }
        std::vector<cv::Point2d> undistorted;
        cv::fisheye::undistortPoints(image_points, undistorted, K, D, cv::noArray(), K);
        solve_image_points = std::move(undistorted);
        solve_dist_coeffs = cv::Mat();
    }

    cv::Mat rvec, tvec;
    bool ok = false;
    try {
        ok = cv::solvePnP(
            object_points,
            solve_image_points,
            K,
            solve_dist_coeffs,
            rvec,
            tvec,
            false,
            cv::SOLVEPNP_SQPNP);
    } catch (const cv::Exception&) {
        ok = false;
    }
    if (!ok) {
        return out;
    }

    try {
        cv::solvePnPRefineLM(
            object_points, solve_image_points, K, solve_dist_coeffs, rvec, tvec);
    } catch (const cv::Exception&) {
        // Keep the SQPNP result on refinement failure.
    }

    std::vector<cv::Point2d> projected;
    cv::projectPoints(object_points, rvec, tvec, K, solve_dist_coeffs, projected);
    double sum_sq = 0.0;
    for (std::size_t i = 0; i < projected.size(); ++i) {
        const double dx = projected[i].x - solve_image_points[i].x;
        const double dy = projected[i].y - solve_image_points[i].y;
        sum_sq += dx * dx + dy * dy;
    }
    const double rms_px = std::sqrt(sum_sq / static_cast<double>(projected.size()));

    cv::Matx33d Rmat;
    cv::Rodrigues(rvec, Rmat);
    const cv::Vec3d t(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

    double sum_z = 0.0;
    for (const auto& obj : object_points) {
        const cv::Vec3d p_field(obj.x, obj.y, obj.z);
        const cv::Vec3d p_cam = Rmat * p_field + t;
        sum_z += p_cam[2];
    }
    const double mean_distance_m = std::max(
        0.0, sum_z / static_cast<double>(object_points.size()));

    const cv::Affine3d cam_T_field(Rmat, t);
    const cv::Affine3d field_T_camera = cam_T_field.inv();
    const cv::Affine3d camera_T_robot = pose3dToAffine(input.camera_to_robot);
    const cv::Affine3d field_T_robot = field_T_camera * camera_T_robot;

    out.field_to_robot = affineToPose3d(field_T_robot);
    out.reprojection_rms_px = rms_px;
    out.solved_tag_count = static_cast<int>(input.tags.size());
    computeCovariance(out, mean_distance_m, rms_px, out.solved_tag_count, input.covariance);
    return out;
}

AprilTagPoseSolveOutput solveSingleTag(const AprilTagPoseSolveInput& input) {
    AprilTagPoseSolveOutput out;
    if (input.tags.size() != 1 || input.tags.front().raw == nullptr) {
        return out;
    }
    const auto& tag = input.tags.front();

    apriltag_detection_info_t info{};
    info.det = tag.raw;
    info.tagsize = input.tag_size_m;
    info.fx = input.calibration.fx;
    info.fy = input.calibration.fy;
    info.cx = input.calibration.cx;
    info.cy = input.calibration.cy;

    apriltag_pose_t pose1{};
    apriltag_pose_t pose2{};
    double err1 = 0.0;
    double err2 = 0.0;
    estimate_tag_pose_orthogonal_iteration(&info, &err1, &pose1, &err2, &pose2, 50);

    if (!pose1.R || !pose1.t) {
        if (pose1.R) matd_destroy(pose1.R);
        if (pose1.t) matd_destroy(pose1.t);
        if (pose2.R) matd_destroy(pose2.R);
        if (pose2.t) matd_destroy(pose2.t);
        return out;
    }

    const bool has_alternate = (pose2.R && pose2.t && err2 > 1e-12);
    const double ambiguity = has_alternate ? std::clamp(err1 / err2, 0.0, 1.0) : 0.0;
    out.single_tag_ambiguity = ambiguity;

    const Pose3d camera_to_tag = apriltagPoseToPose3d(pose1);

    matd_destroy(pose1.R);
    matd_destroy(pose1.t);
    if (pose2.R) matd_destroy(pose2.R);
    if (pose2.t) matd_destroy(pose2.t);

    if (ambiguity > input.covariance.ambiguity_drop_threshold) {
        return out;
    }

    const cv::Affine3d cam_T_tag = pose3dToAffine(camera_to_tag);
    const cv::Affine3d tag_T_camera = cam_T_tag.inv();
    const cv::Affine3d field_T_tag = pose3dToAffine(tag.field_to_tag);
    const cv::Affine3d field_T_camera = field_T_tag * tag_T_camera;
    const cv::Affine3d camera_T_robot = pose3dToAffine(input.camera_to_robot);
    const cv::Affine3d field_T_robot = field_T_camera * camera_T_robot;

    const auto tag_corners = cornersInTagFrame(input.tag_size_m);
    const cv::Matx33d K_mat(
        input.calibration.fx, 0.0, input.calibration.cx,
        0.0, input.calibration.fy, input.calibration.cy,
        0.0, 0.0, 1.0);
    double sum_sq = 0.0;
    int valid_corners = 0;
    for (std::size_t c = 0; c < 4; ++c) {
        const cv::Vec3d p_local(tag_corners[c].x, tag_corners[c].y, tag_corners[c].z);
        const cv::Vec3d p_cam = cam_T_tag * p_local;
        if (p_cam[2] <= 0.0) {
            continue;
        }
        const double u = K_mat(0, 0) * (p_cam[0] / p_cam[2]) + K_mat(0, 2);
        const double v = K_mat(1, 1) * (p_cam[1] / p_cam[2]) + K_mat(1, 2);
        const double dx = u - tag.image_corners_px[c].x;
        const double dy = v - tag.image_corners_px[c].y;
        sum_sq += dx * dx + dy * dy;
        ++valid_corners;
    }
    const double rms_px = (valid_corners > 0)
        ? std::sqrt(sum_sq / static_cast<double>(valid_corners))
        : 0.0;
    const double mean_distance_m = std::max(0.0, camera_to_tag.translation_m.z);

    out.field_to_robot = affineToPose3d(field_T_robot);
    out.reprojection_rms_px = rms_px;
    out.solved_tag_count = 1;
    computeSingleTagCovariance(
        out, mean_distance_m, rms_px, input.covariance, field_T_camera);
    return out;
}

}  // namespace

void computeSingleTagCovariance(
    AprilTagPoseSolveOutput& out,
    double mean_distance_m,
    double rms_px,
    const AprilTagCovarianceTuning& tuning,
    const cv::Affine3d& field_T_camera) {
    const auto f = covarianceFactors(mean_distance_m, rms_px, /*tag_count=*/1, tuning);
    out.covariance.fill(0.0);

    const double var_xy = f.sigma_r_base * f.sigma_r_base;
    const double var_z = (f.sigma_r_base * f.rot_multiplier) *
                         (f.sigma_r_base * f.rot_multiplier);
    const cv::Matx33d R = field_T_camera.rotation();
    for (int i = 0; i < 3; ++i) {
        const double r0 = R(i, 0);
        const double r1 = R(i, 1);
        const double r2 = R(i, 2);
        out.covariance[static_cast<std::size_t>(i * 6 + i)] =
            r0 * r0 * var_xy + r1 * r1 * var_xy + r2 * r2 * var_z;
    }

    const double var_t = f.sigma_t * f.sigma_t;
    out.covariance[3 * 6 + 3] = var_t;
    out.covariance[4 * 6 + 4] = var_t;
    out.covariance[5 * 6 + 5] = var_t;
}

AprilTagPoseSolveOutput solveAprilTagPose(const AprilTagPoseSolveInput& input) {
    if (input.tags.empty()) {
        return {};
    }
    if (!intrinsicsArePresent(input.calibration)) {
        return {};
    }
    if (input.tags.size() == 1) {
        return solveSingleTag(input);
    }
    return solveMultiTag(input);
}

}  // namespace posest::pipelines
