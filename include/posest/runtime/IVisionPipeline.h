#pragma once

#include <string>

#include "posest/IFrameConsumer.h"

namespace posest::runtime {

class IVisionPipeline : public virtual IFrameConsumer {
public:
    ~IVisionPipeline() override = default;

    virtual const std::string& type() const = 0;
};

}  // namespace posest::runtime
