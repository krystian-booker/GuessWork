#pragma once

#include <string>

#include "posest/ConsumerBase.h"
#include "posest/Frame.h"
#include "posest/MeasurementBus.h"
#include "posest/runtime/IVisionPipeline.h"

namespace posest::pipelines {

// Vision pipelines are just consumers that also publish measurements. The
// thread + drop-oldest mailbox plumbing is inherited from ConsumerBase; this
// base only adds the type tag and the measurement sink reference.
class VisionPipelineBase : public ConsumerBase, public runtime::IVisionPipeline {
public:
    VisionPipelineBase(std::string id, std::string type, IMeasurementSink& sink);
    ~VisionPipelineBase() override = default;

    VisionPipelineBase(const VisionPipelineBase&) = delete;
    VisionPipelineBase& operator=(const VisionPipelineBase&) = delete;

    const std::string& type() const override { return type_; }

protected:
    IMeasurementSink& measurementSink() { return sink_; }
    virtual void processFrame(const Frame& frame) = 0;

private:
    void process(const Frame& frame) final { processFrame(frame); }

    std::string type_;
    IMeasurementSink& sink_;
};

}  // namespace posest::pipelines
