#pragma once

#include <string>

#include "posest/Frame.h"

namespace posest {

// Interface for anything that wants to receive frames from a producer.
//
// deliver() is invoked from the producer's capture thread and MUST be
// non-blocking. Concrete implementations should typically forward into a
// LatestFrameSlot (see ConsumerBase) so slow processing can never stall
// the producer.
class IFrameConsumer {
public:
    virtual ~IFrameConsumer() = default;

    virtual const std::string& id() const = 0;

    virtual void start() = 0;
    virtual void stop() = 0;

    virtual void deliver(FramePtr frame) = 0;
};

}  // namespace posest
