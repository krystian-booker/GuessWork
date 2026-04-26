#pragma once

#include <string>

#include "posest/IFrameConsumer.h"
#include "posest/pipelines/PipelineStats.h"

namespace posest::runtime {

// Virtual inheritance of IFrameConsumer is intentional: pipelines::VisionPipelineBase
// combines this with ConsumerBase (which also extends IFrameConsumer), and
// needs a single shared subobject. Do not drop the `virtual` keyword without
// also updating ConsumerBase.
class IVisionPipeline : public virtual IFrameConsumer {
public:
    ~IVisionPipeline() override = default;

    virtual const std::string& type() const = 0;

    // Polymorphic telemetry hook. Default = monostate (no stats); concrete
    // pipelines override with their own stats variant alternative. Surfaces
    // through DaemonController::refreshHealth without dynamic_pointer_cast
    // per concrete pipeline type.
    virtual pipelines::PipelineStatsValue pipelineStats() const {
        return pipelines::PipelineStatsValue{};
    }
};

}  // namespace posest::runtime
