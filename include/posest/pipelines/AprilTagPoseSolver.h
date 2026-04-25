#pragma once

#include <array>
#include <optional>
#include <vector>

#include <opencv2/core/affine.hpp>

#include "posest/MeasurementTypes.h"
#include "posest/pipelines/AprilTagPipeline.h"

// Forward-declare libapriltag's detection struct so callers don't need the
// libapriltag headers transitively.
struct apriltag_detection;
using apriltag_detection_t = apriltag_detection;

namespace posest::pipelines {

struct AprilTagPoseSolveInput {
    struct TagInput {
        int tag_id{0};
        std::array<Vec2, 4> image_corners_px{};
        Pose3d field_to_tag;
        // Borrowed pointer; valid for the duration of solveAprilTagPose's call
        // (libapriltag detections are freed by the caller after the solve).
        apriltag_detection_t* raw{nullptr};
    };

    std::vector<TagInput> tags;
    AprilTagCameraCalibration calibration;
    Pose3d camera_to_robot;
    double tag_size_m{0.1651};
    AprilTagCovarianceTuning covariance;
};

struct AprilTagPoseSolveOutput {
    std::optional<Pose3d> field_to_robot;
    std::array<double, 36> covariance{};
    double reprojection_rms_px{0.0};
    int solved_tag_count{0};
    std::optional<double> single_tag_ambiguity;
};

AprilTagPoseSolveOutput solveAprilTagPose(const AprilTagPoseSolveInput& input);

// Single-tag covariance helper exposed for testing. Builds Σ_cam_R with the
// optical-axis (camera +Z) variance inflated by `single_tag_rotation_mult²`,
// rotates to world frame via `field_T_camera`, and writes the diagonal of the
// resulting 3×3 rotational sub-block plus a uniform translational diagonal
// (using the smooth tag-count multiplier evaluated at N=1).
void computeSingleTagCovariance(
    AprilTagPoseSolveOutput& out,
    double mean_distance_m,
    double rms_px,
    const AprilTagCovarianceTuning& tuning,
    const cv::Affine3d& field_T_camera);

}  // namespace posest::pipelines
