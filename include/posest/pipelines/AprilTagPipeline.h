#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "posest/MeasurementTypes.h"
#include "posest/pipelines/VisionPipelineBase.h"
#include "posest/runtime/PipelineConfig.h"

struct apriltag_detector;
struct apriltag_family;

namespace posest::pipelines {

struct AprilTagCameraCalibration {
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    std::string distortion_model;
    std::vector<double> distortion_coefficients;
};

struct AprilTagCovarianceTuning {
    double base_sigma_translation_m{0.02};
    double base_sigma_rotation_rad{0.02};
    double reference_distance_m{1.0};
    double reference_rms_px{1.0};
    double single_tag_translation_mult{1.5};
    double single_tag_rotation_mult{5.0};
    double ambiguity_drop_threshold{0.4};
    double multi_tag_decay_k{2.0};
    double well_spread_floor_mult{0.5};
};

struct AprilTagPipelineConfig {
    std::string family = "tag36h11";
    int nthreads = 4;
    double quad_decimate = 1.0;
    double quad_sigma = 0.0;
    bool refine_edges = true;
    double decode_sharpening = 0.25;
    bool debug = false;
    std::string calibration_version;
    std::string field_layout_id;
    double tag_size_m{0.1651};
    std::unordered_map<std::string, AprilTagCameraCalibration> camera_calibrations;
    std::unordered_map<int, posest::Pose3d> field_to_tags;
    std::unordered_map<std::string, posest::Pose3d> camera_to_robot;
    AprilTagCovarianceTuning covariance;
};

AprilTagPipelineConfig parseAprilTagPipelineConfig(
    const runtime::PipelineConfig& pipeline_config);

class AprilTagPipeline final : public VisionPipelineBase {
public:
    AprilTagPipeline(
        std::string id,
        IMeasurementSink& sink,
        AprilTagPipelineConfig config = {});
    ~AprilTagPipeline() override;

protected:
    void processFrame(const Frame& frame) override;

private:
    AprilTagPipelineConfig config_;
    apriltag_detector* detector_{nullptr};
    apriltag_family* family_{nullptr};
    void (*family_destroy_)(apriltag_family*){nullptr};
};

}  // namespace posest::pipelines
