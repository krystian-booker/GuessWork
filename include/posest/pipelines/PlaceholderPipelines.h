#pragma once

#include "posest/pipelines/VisionPipelineBase.h"

namespace posest::pipelines {

class PlaceholderAprilTagPipeline final : public VisionPipelineBase {
public:
    PlaceholderAprilTagPipeline(std::string id, IMeasurementSink& sink);
    // Required by the ConsumerBase contract: stop the worker thread before
    // this leaf's members go out of scope (~ConsumerBase aborts otherwise).
    ~PlaceholderAprilTagPipeline() override { stop(); }

protected:
    void processFrame(const Frame& frame) override;
};

class PlaceholderVioPipeline final : public VisionPipelineBase {
public:
    PlaceholderVioPipeline(std::string id, IMeasurementSink& sink);
    ~PlaceholderVioPipeline() override { stop(); }

protected:
    void processFrame(const Frame& frame) override;
};

}  // namespace posest::pipelines
