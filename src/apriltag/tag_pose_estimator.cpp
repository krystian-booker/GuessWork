#include "apriltag/tag_pose_estimator.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <cmath>

// ---------------------------------------------------------------------------
// Frame conventions (normative — getting any of these wrong silently corrupts
// every pose; the synthetic tests in tests/test_tag_pose_estimator.cpp and
// the hardware bench check are the arbiters):
//
//   T_a_b maps points in frame b into frame a (matches Kalibr's T_cam_imu).
//
//   field : FRC field frame — NWU, blue origin, +X toward red, +Z up.
//   tag   : WPILib tag frame — origin at tag center, +X out of the face
//           toward the viewer, +Y = viewer's right, +Z up.
//   tagcv : IPPE_SQUARE object frame — +X viewer's right, +Y viewer's up,
//           +Z out of the face toward the camera.
//   cam   : OpenCV camera frame — +X right, +Y down, +Z forward.
//
//   apriltag detector corners det->p[0..3] = BL, BR, TR, TL (image sense).
//   IPPE_SQUARE's object frame is IMAGE-ALIGNED (verified empirically
//   against OpenCV 4.13: identity rotation + zero error for a fronto-
//   parallel square): +X viewer's right, +Y viewer's DOWN, +Z into the
//   scene (out the back of the tag). The required object points
//     (−s/2,+s/2,0), (+s/2,+s/2,0), (+s/2,−s/2,0), (−s/2,−s/2,0)
//   are therefore BL, BR, TR, TL — det->p[0..3] pairs DIRECTLY, no reorder.
//
//   Basis change tagcv → tag (pure rotation, shared origin): the tag-frame
//   axes expressed in tagcv coordinates are X=(0,0,−1) (out of the face,
//   toward the viewer), Y=(1,0,0) (viewer's right), Z=(0,−1,0) (up):
//     R_tagcv_tag = [0 1 0; 0 0 −1; −1 0 0]
//
//   Chains:
//     multi-tag : solvePnP(field corners) → T_cam_field directly
//     single-tag: T_cam_field = T_cam_tagcv · T_tagcv_tag · inverse(T_field_tag)
//     output    : T_field_robot = inverse(T_cam_field) · T_cam_robot
// ---------------------------------------------------------------------------

namespace gw::apriltag {

namespace {

cv::Matx33d intrinsic_matrix(const PinholeCamera& cam) {
    return {cam.fxfycxcy[0], 0.0, cam.fxfycxcy[2],
            0.0, cam.fxfycxcy[1], cam.fxfycxcy[3],
            0.0, 0.0, 1.0};
}

// Undistort pixel corners and re-project through K (P=K) so the outputs
// stay in pixel units; every solve afterwards uses K with zero distortion.
std::vector<cv::Point2d> undistort_corners(const std::vector<cv::Point2d>& px,
                                           const PinholeCamera&            cam) {
    const cv::Matx33d K = intrinsic_matrix(cam);
    const cv::Mat     D(1, 4, CV_64F, const_cast<double*>(cam.d.data()));
    std::vector<cv::Point2d> out;
    if (cam.model == PinholeCamera::Dist::kRadTan) {
        cv::undistortPoints(px, out, K, D, cv::noArray(), K);
    } else {
        cv::fisheye::undistortPoints(px, out, K, D, cv::noArray(), K);
    }
    return out;
}

Mat4 mat4_from_rvec_tvec(const cv::Vec3d& rvec, const cv::Vec3d& tvec) {
    cv::Matx33d R;
    cv::Rodrigues(rvec, R);
    Mat4 T = mat4_identity();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) T[r][c] = R(r, c);
        T[r][3] = tvec[r];
    }
    return T;
}

void rvec_tvec_from_mat4(const Mat4& T, cv::Vec3d& rvec, cv::Vec3d& tvec) {
    cv::Matx33d R;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) R(r, c) = T[r][c];
    cv::Rodrigues(R, rvec);
    tvec = {T[0][3], T[1][3], T[2][3]};
}

double mean_reproj_err_px(const std::vector<cv::Point3d>& obj,
                          const std::vector<cv::Point2d>& img,
                          const Mat4& T_cam_obj, const cv::Matx33d& K) {
    cv::Vec3d rvec, tvec;
    rvec_tvec_from_mat4(T_cam_obj, rvec, tvec);
    std::vector<cv::Point2d> proj;
    cv::projectPoints(obj, rvec, tvec, K, cv::noArray(), proj);
    double sum = 0;
    for (size_t i = 0; i < obj.size(); ++i) {
        sum += cv::norm(proj[i] - img[i]);
    }
    return sum / static_cast<double>(obj.size());
}

// Numeric Jacobian of the corner projections w.r.t. a right (body-frame)
// perturbation of T_field_robot, evaluated at the solution. Σ = σ²(JᵀJ)⁻¹.
// Computed directly in the output tangent space — no adjoint chaining from
// OpenCV's rvec parameterization to get wrong.
std::optional<std::array<double, 36>> pose_covariance(
    const std::vector<cv::Point3d>& obj_field,
    const Mat4& T_field_robot, const Mat4& T_robot_cam,
    const cv::Matx33d& K, double sigma_px) {
    constexpr double kEps = 1e-5;
    const int        n2   = static_cast<int>(obj_field.size()) * 2;

    cv::Mat J(n2, 6, CV_64F);
    for (int axis = 0; axis < 6; ++axis) {
        std::array<std::vector<cv::Point2d>, 2> proj;
        for (int s = 0; s < 2; ++s) {
            Vec6 xi{};
            xi[axis] = (s == 0 ? +kEps : -kEps);
            const Mat4 T_field_cam =
                mat4_mul(retract(T_field_robot, xi), T_robot_cam);
            const Mat4 T_cam_field = mat4_inverse_se3(T_field_cam);
            cv::Vec3d rvec, tvec;
            rvec_tvec_from_mat4(T_cam_field, rvec, tvec);
            cv::projectPoints(obj_field, rvec, tvec, K, cv::noArray(), proj[s]);
        }
        for (size_t i = 0; i < obj_field.size(); ++i) {
            J.at<double>(static_cast<int>(2 * i), axis) =
                (proj[0][i].x - proj[1][i].x) / (2 * kEps);
            J.at<double>(static_cast<int>(2 * i) + 1, axis) =
                (proj[0][i].y - proj[1][i].y) / (2 * kEps);
        }
    }

    const cv::Mat JtJ = J.t() * J;
    cv::Mat       inv;
    // SVD inversion + condition check: a single tag seen nearly fronto-
    // parallel can leave the yaw/translation subspace ill-conditioned.
    if (cv::invert(JtJ, inv, cv::DECOMP_SVD) == 0.0) {
        return std::nullopt;
    }

    std::array<double, 36> out{};
    const double s2 = sigma_px * sigma_px;
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c) out[r * 6 + c] = s2 * inv.at<double>(r, c);
    return out;
}

}  // namespace

const char* to_string(SkipReason r) {
    switch (r) {
        case SkipReason::kNone:          return "none";
        case SkipReason::kNoKnownTags:   return "no_known_tags";
        case SkipReason::kAmbiguous:     return "ambiguous";
        case SkipReason::kHighReprojErr: return "high_reproj_err";
        case SkipReason::kSolveFailed:   return "solve_failed";
    }
    return "unknown";
}

EstimateResult estimate_robot_pose(const std::vector<TagObservation>& observations,
                                   const PinholeCamera&               camera,
                                   const PreparedLayout&              layout,
                                   const Mat4&                        T_cam_robot,
                                   const EstimatorConfig&             cfg) {
    EstimateResult result;

    // Collect observations of tags that exist in the layout.
    std::vector<const TagObservation*> known;
    for (const auto& obs : observations) {
        if (layout.corners.count(obs.id)) known.push_back(&obs);
    }
    if (known.empty()) {
        result.skip = SkipReason::kNoKnownTags;
        return result;
    }

    const cv::Matx33d K = intrinsic_matrix(camera);

    // Field-frame object points + undistorted image points, det->p order.
    std::vector<cv::Point3d> obj_field;
    std::vector<cv::Point2d> img_raw;
    std::vector<int32_t>     tag_ids;
    obj_field.reserve(known.size() * 4);
    img_raw.reserve(known.size() * 4);
    for (const auto* obs : known) {
        const auto& fc = layout.corners.at(obs->id);
        for (int i = 0; i < 4; ++i) {
            obj_field.emplace_back(fc[i][0], fc[i][1], fc[i][2]);
            img_raw.emplace_back(obs->corners_px[i][0], obs->corners_px[i][1]);
        }
        tag_ids.push_back(obs->id);
    }
    const std::vector<cv::Point2d> img = undistort_corners(img_raw, camera);

    Mat4 T_cam_field;
    try {
        if (known.size() >= 2) {
            // Multi-tag: SQPNP over all field-frame corners, LM refinement.
            cv::Vec3d rvec, tvec;
            if (!cv::solvePnP(obj_field, img, K, cv::noArray(), rvec, tvec,
                              /*useExtrinsicGuess=*/false, cv::SOLVEPNP_SQPNP)) {
                result.skip = SkipReason::kSolveFailed;
                return result;
            }
            cv::solvePnPRefineLM(obj_field, img, K, cv::noArray(), rvec, tvec);
            T_cam_field = mat4_from_rvec_tvec(rvec, tvec);
        } else {
            // Single tag: IPPE_SQUARE with the planar ambiguity gate.
            // Object points in the solver's image-aligned frame are
            // BL, BR, TR, TL — exactly det->p order, no reorder.
            const auto* obs = known.front();
            // The object square must match the size the layout's field-frame
            // corner table was built with, or the two frames disagree.
            const double h  = layout.tag_size_m / 2.0;
            const std::vector<cv::Point3d> obj_tagcv = {
                {-h, +h, 0}, {+h, +h, 0}, {+h, -h, 0}, {-h, -h, 0}};  // BL BR TR TL
            const std::vector<cv::Point2d> img_sq = {img[0], img[1], img[2], img[3]};

            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::Mat                errs;
            const int n = cv::solvePnPGeneric(
                obj_tagcv, img_sq, K, cv::noArray(), rvecs, tvecs,
                /*useExtrinsicGuess=*/false, cv::SOLVEPNP_IPPE_SQUARE,
                cv::noArray(), cv::noArray(), errs);
            if (n < 1) {
                result.skip = SkipReason::kSolveFailed;
                return result;
            }
            if (n >= 2) {
                // OpenCV returns the per-solution errors as CV_32F or CV_64F
                // depending on version/path — read whichever arrived.
                const auto err_at = [&errs](int i) {
                    return errs.type() == CV_64F
                               ? errs.at<double>(i)
                               : static_cast<double>(errs.at<float>(i));
                };
                const double e0 = err_at(0);
                const double e1 = err_at(1);
                if (e0 > 0.0 && e1 / e0 < cfg.ambiguity_min_ratio) {
                    result.skip = SkipReason::kAmbiguous;
                    return result;
                }
            }

            const Mat4 T_cam_tagcv = mat4_from_rvec_tvec(rvecs[0], tvecs[0]);
            const Mat4 T_tagcv_tag = mat4_from_rt(
                {{{0, 1, 0}, {0, 0, -1}, {-1, 0, 0}}}, {0, 0, 0});
            const Mat4 T_field_tag = layout.by_id.at(obs->id).T_field_tag;
            T_cam_field = mat4_mul(mat4_mul(T_cam_tagcv, T_tagcv_tag),
                                   mat4_inverse_se3(T_field_tag));
        }
    } catch (const cv::Exception&) {
        result.skip = SkipReason::kSolveFailed;
        return result;
    }

    const double reproj = mean_reproj_err_px(obj_field, img, T_cam_field, K);
    if (reproj > cfg.max_mean_reproj_px) {
        result.skip = SkipReason::kHighReprojErr;
        return result;
    }

    // T_field_robot = inverse(T_cam_field) · T_cam_robot
    const Mat4 T_field_robot =
        mat4_mul(mat4_inverse_se3(T_cam_field), T_cam_robot);
    const Mat4 T_robot_cam = mat4_inverse_se3(T_cam_robot);

    const auto cov = pose_covariance(obj_field, T_field_robot, T_robot_cam, K,
                                     cfg.sigma_px);
    if (!cov) {
        result.skip = SkipReason::kSolveFailed;
        return result;
    }

    PoseEstimate est;
    est.T_field_robot      = T_field_robot;
    est.cov                = *cov;
    est.mean_reproj_err_px = reproj;
    est.n_tags             = static_cast<uint32_t>(known.size());
    est.tag_ids            = std::move(tag_ids);
    result.pose            = std::move(est);
    return result;
}

std::optional<double> single_tag_range_m(const TagObservation& obs,
                                         const PinholeCamera&  camera,
                                         double                tag_size_m) {
    const cv::Matx33d K = intrinsic_matrix(camera);
    std::vector<cv::Point2d> raw;
    for (const auto& c : obs.corners_px) raw.emplace_back(c[0], c[1]);
    const auto img = undistort_corners(raw, camera);

    const double h = tag_size_m / 2.0;
    const std::vector<cv::Point3d> obj_tagcv = {
        {-h, +h, 0}, {+h, +h, 0}, {+h, -h, 0}, {-h, -h, 0}};  // BL BR TR TL
    const std::vector<cv::Point2d> img_sq = {img[0], img[1], img[2], img[3]};

    try {
        cv::Vec3d rvec, tvec;
        if (!cv::solvePnP(obj_tagcv, img_sq, K, cv::noArray(), rvec, tvec,
                          false, cv::SOLVEPNP_IPPE_SQUARE)) {
            return std::nullopt;
        }
        return cv::norm(tvec);
    } catch (const cv::Exception&) {
        return std::nullopt;
    }
}

}  // namespace gw::apriltag
