#pragma once

#include <memory>
#include <string>

#include "posest/IFrameConsumer.h"

namespace posest {

// Interface for a frame source (camera, file, mock, etc.).
//
// A producer owns its capture thread and fans frames out to any number of
// registered consumers. Wiring (addConsumer) must happen before start().
class IFrameProducer {
public:
    virtual ~IFrameProducer() = default;

    virtual const std::string& id() const = 0;

    virtual void addConsumer(std::shared_ptr<IFrameConsumer> consumer) = 0;

    virtual void start() = 0;
    virtual void stop() = 0;
};

}  // namespace posest
