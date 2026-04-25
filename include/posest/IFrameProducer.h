#pragma once

#include <memory>
#include <string>

#include "posest/IFrameConsumer.h"

namespace posest {

// Lifecycle state reported by IFrameProducer::state().
//
//   Idle        — never started, or stopped cleanly via stop().
//   Running     — capture loop is active.
//   EndOfStream — captureOne() returned false; capture loop exited on its own
//                 (e.g. file-replay end, camera unplugged). The worker thread
//                 has terminated; stop() will join it without hanging.
//   Failed      — captureOne() (or another part of the capture loop) raised
//                 an exception. Same lifecycle as EndOfStream.
//
// EndOfStream and Failed are terminal: a producer in either state cannot be
// restarted. Construct a new producer instead.
enum class ProducerState {
    Idle,
    Running,
    EndOfStream,
    Failed,
};

// Interface for a frame source (camera, file, mock, etc.).
//
// A producer owns its capture thread and fans frames out to any number of
// registered consumers. addConsumer / removeConsumer are safe to call at any
// time, including while the producer is running — frames captured after the
// call are delivered to the new subscriber set. Consumer identity is by
// shared_ptr equality.
class IFrameProducer {
public:
    virtual ~IFrameProducer() = default;

    virtual const std::string& id() const = 0;

    virtual void addConsumer(std::shared_ptr<IFrameConsumer> consumer) = 0;

    // Detach a previously-added consumer. Returns true if found and removed.
    // Frames already pushed into the consumer's mailbox are not recalled; the
    // caller still owns the consumer's lifecycle (stop() before destruction).
    [[nodiscard]] virtual bool removeConsumer(
        const std::shared_ptr<IFrameConsumer>& consumer) = 0;

    // Returns the new state. On success returns Running. If the producer was
    // already Running, returns Running. If it has reached a terminal state
    // (EndOfStream / Failed) without an intervening stop(), the call is a
    // no-op and the terminal state is returned — the caller MUST construct
    // a new producer rather than attempting to re-arm this one. The
    // [[nodiscard]] enforces that the outcome is observed.
    [[nodiscard]] virtual ProducerState start() = 0;
    virtual void stop() = 0;

    // Current lifecycle state. Safe to call from any thread.
    virtual ProducerState state() const = 0;
};

}  // namespace posest
