#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>

#include "posest/Frame.h"

namespace posest {

// Single-slot mailbox with drop-oldest semantics.
//
// put(): never blocks on the consumer. If an unread frame is pending, it is
//        dropped and replaced by the new one (dropped_count is bumped).
// take(): blocks until a frame is available or shutdown() is called. Returns
//         nullptr only on shutdown.
// shutdown(): unblocks any waiting take() and causes future take() calls to
//             return nullptr immediately.
//
// This is the core latency primitive: the producer is never stalled, and the
// consumer always processes the newest available frame when it is ready.
class LatestFrameSlot {
public:
    LatestFrameSlot() = default;
    LatestFrameSlot(const LatestFrameSlot&) = delete;
    LatestFrameSlot& operator=(const LatestFrameSlot&) = delete;

    void put(FramePtr frame);
    FramePtr take();
    void shutdown();

    std::uint64_t droppedCount() const;

private:
    mutable std::mutex mu_;
    std::condition_variable cv_;
    FramePtr pending_;
    // Diagnostic counter only — not part of any synchronization, so it lives
    // outside the mutex to keep droppedCount() lock-free for telemetry callers.
    std::atomic<std::uint64_t> dropped_{0};
    bool shut_{false};
};

}  // namespace posest
