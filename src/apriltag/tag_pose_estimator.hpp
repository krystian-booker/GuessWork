#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

#include "apriltag/field_layout.hpp"
#include "apriltag/pose_math.hpp"

// Pure tag-pose estimation: detected tag corners + camera intrinsics +
// field layout + extrinsics chain → field-frame robot pose with covariance.
// No I/O, no threads, no apriltag-library types — fully unit-testable with
// synthetic projections. OpenCV is an implementation detail (cpp only).

namespace gw::apriltag {

struct PinholeCamera {
    std::array<double, 4> fxfycxcy{};  // fu fv pu pv, pixels
    enum class Dist : uint8_t { kRadTan, kEquidistant } model = Dist::kRadTan;
    std::array<double, 4> d{};         // radtan: k1 k2 p1 p2; equi: k1..k4
    uint32_t              width  = 0;  // expected frame geometry
    uint32_t              height = 0;
};

struct TagObservation {
    int                                  id = 0;
    // Pixel corners in detector order: p[0..3] = BL, BR, TR, TL.
    std::array<std::array<double, 2>, 4> corners_px{};
    double                               decision_margin = 0.0;
};

struct EstimatorConfig {
    double tag_size_m          = kFrcTagSizeM;
    double sigma_px            = 0.7;   // corner noise, decimation-adjusted
    double ambiguity_min_ratio = 2.0;   // single-tag: reject if err1/err0 below
    double max_mean_reproj_px  = 2.0;   // global sanity gate
};

enum class SkipReason : uint8_t {
    kNone,
    kNoKnownTags,    // no observation matched the field layout
    kAmbiguous,      // single-tag IPPE solutions too close to disambiguate
    kHighReprojErr,  // solution failed the mean-reprojection gate
    kSolveFailed,    // PnP failed or covariance not recoverable
};

struct PoseEstimate {
    Mat4                   T_field_robot = mat4_identity();
    // Row-major 6×6, tangent [ω, t], right/body perturbation (GTSAM Pose3).
    std::array<double, 36> cov{};
    double                 mean_reproj_err_px = 0.0;
    uint32_t               n_tags             = 0;
    std::vector<int32_t>   tag_ids;
};

struct EstimateResult {
    std::optional<PoseEstimate> pose;
    SkipReason                  skip = SkipReason::kNone;
};

// T_cam_robot maps robot-frame points into the camera frame
// (= T_cam_imu · inverse(T_robot_imu); constant per camera).
EstimateResult estimate_robot_pose(const std::vector<TagObservation>& observations,
                                   const PinholeCamera&               camera,
                                   const PreparedLayout&              layout,
                                   const Mat4&                        T_cam_robot,
                                   const EstimatorConfig&             cfg = {});

// Intrinsics-only range to a single tag (camera→tag-center distance) for
// bench validation and status display — works without any extrinsics or
// field layout. nullopt when the solve fails.
std::optional<double> single_tag_range_m(const TagObservation& obs,
                                         const PinholeCamera&  camera,
                                         double                tag_size_m = kFrcTagSizeM);

const char* to_string(SkipReason r);

}  // namespace gw::apriltag
