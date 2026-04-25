#pragma once

#include <array>
#include <optional>
#include <vector>

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

}  // namespace posest::pipelines
