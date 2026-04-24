#pragma once

#include <memory>
#include <string>

#include "posest/pipelines/VisionPipelineBase.h"
#include "posest/runtime/PipelineConfig.h"

struct apriltag_detector;
struct apriltag_family;

namespace posest::pipelines {

struct AprilTagPipelineConfig {
    std::string family = "tag36h11";
    int nthreads = 1;
    double quad_decimate = 2.0;
    double quad_sigma = 0.0;
    bool refine_edges = true;
    double decode_sharpening = 0.25;
    bool debug = false;
    std::string calibration_version;
    std::string field_layout_id;
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
