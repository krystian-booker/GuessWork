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
};

}  // namespace posest::pipelines
