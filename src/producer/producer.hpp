#pragma once

#include <string_view>

#include "core/frame_channel.hpp"
#include "core/frame_format.hpp"

namespace gw {

// Interface implemented by every camera driver. A producer owns its output
// FrameChannel and a worker thread that fills frames from its hardware/source.
class IProducer {
public:
    virtual ~IProducer() = default;

    virtual std::string_view name()    const = 0;
    virtual FrameFormat      format()  const = 0;
    virtual FrameChannel&    channel()       = 0;

    // start() initializes the underlying source and launches the capture
    // thread. Returns when streaming is live; throws on failure (no camera
    // detected, unsupported pixel format, etc.).
    virtual void start() = 0;

    // stop() requests the capture thread to exit, joins it, and releases SDK
    // resources. Safe to call from any thread; safe to call if start() was
    // never called or already stopped.
    virtual void stop() = 0;
};

}  // namespace gw
