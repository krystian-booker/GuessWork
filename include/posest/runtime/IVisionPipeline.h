#pragma once

#include <string>

#include "posest/IFrameConsumer.h"

namespace posest::runtime {

// Virtual inheritance of IFrameConsumer is intentional: pipelines::VisionPipelineBase
// combines this with ConsumerBase (which also extends IFrameConsumer), and
// needs a single shared subobject. Do not drop the `virtual` keyword without
// also updating ConsumerBase.
class IVisionPipeline : public virtual IFrameConsumer {
public:
    ~IVisionPipeline() override = default;

    virtual const std::string& type() const = 0;
};

}  // namespace posest::runtime
