#pragma once

#include <string_view>

#include "core/frame_channel.hpp"

namespace gw {

// Interface implemented by every consumer. A consumer subscribes to one
// FrameChannel and processes the frames it pulls. The consumer owns its own
// worker thread; attach() starts it, detach() stops and joins it.
class IConsumer {
public:
    virtual ~IConsumer() = default;

    virtual std::string_view name() const = 0;

    // attach() subscribes to the channel and starts the consumer's worker
    // thread. Throws on initialization failure (e.g. unable to create Metal
    // pipeline). Safe to call only once per consumer instance.
    virtual void attach(FrameChannel& ch) = 0;

    // detach() requests the worker to stop, unsubscribes from the channel
    // (which wakes any blocked next_frame call), and joins the thread.
    // Idempotent: safe to call multiple times.
    virtual void detach() = 0;
};

}  // namespace gw
