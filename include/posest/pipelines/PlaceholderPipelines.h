#pragma once

#include "posest/pipelines/VisionPipelineBase.h"

namespace posest::pipelines {

class PlaceholderAprilTagPipeline final : public VisionPipelineBase {
public:
    PlaceholderAprilTagPipeline(std::string id, IMeasurementSink& sink);

protected:
    void processFrame(const Frame& frame) override;
};

class PlaceholderVioPipeline final : public VisionPipelineBase {
public:
    PlaceholderVioPipeline(std::string id, IMeasurementSink& sink);

protected:
    void processFrame(const Frame& frame) override;
};

}  // namespace posest::pipelines
