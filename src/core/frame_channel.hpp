#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include "core/frame.hpp"

namespace gw {

// Single-producer, multi-consumer "latest-only" frame distribution channel.
//
// One channel per producer. Consumers subscribe to obtain a SubscriberHandle
// and then call next_frame() to receive frames in pull-with-blocking style.
//
// Semantics:
//   - At any time the channel holds at most one published frame (the latest).
//     Each publish atomically replaces the previous one.
//   - A consumer that lags behind multiple publishes will see only the newest
//     frame on its next next_frame() call. Intermediate frames are skipped at
//     no cost: they are returned to the pool as soon as the channel evicts
//     them and no other consumer is still holding them.
//   - publish() and next_frame() are both lock-free on the hot path. The only
//     synchronization that takes a lock is subscribe()/unsubscribe(), which
//     is a slow path.
//
// Memory safety:
//   - The producer publishes a Frame* whose refcount it has just bumped via
//     pool.acquire(). The channel takes that single reference.
//   - When the channel exchanges in a new frame, it releases its old one.
//   - When a consumer next_frame()s, it retains the frame; release is the
//     caller's responsibility.
//
// Detach:
//   - unsubscribe() marks the subscriber detached and wakes it. A consumer
//     blocked in next_frame() returns nullptr, allowing graceful thread exit.
class FrameChannel {
public:
    struct Subscriber {
        std::atomic<uint64_t> last_seen_seq{0};
        std::atomic<bool>     attached{true};

        // Per-subscriber wake token. Bumped + notified when a new frame is
        // published or when the subscriber is detached. Each next_frame()
        // captures the token before sleeping; the notify changes the token's
        // observed value so the sleeping thread wakes.
        std::atomic<uint64_t> wait_token{0};
    };
    using SubscriberHandle = std::shared_ptr<Subscriber>;

    FrameChannel();
    ~FrameChannel();

    FrameChannel(const FrameChannel&)            = delete;
    FrameChannel& operator=(const FrameChannel&) = delete;

    // Producer-side. Takes ownership of one reference on `f`.
    // After publish() returns, the producer must NOT touch `f` again.
    // The frame's sequence and timestamps must already be set.
    void publish(Frame* f);

    // Consumer-side. Subscribe before calling next_frame().
    SubscriberHandle subscribe();
    void             unsubscribe(SubscriberHandle h);

    // Block until a frame with sequence > h->last_seen_seq is available, then
    // return it with refcount bumped (caller must release()). Returns nullptr
    // if the subscriber has been detached.
    Frame* next_frame(SubscriberHandle h);

private:
    using SubscriberList = std::vector<std::shared_ptr<Subscriber>>;

    // Returns a snapshot of the current subscriber list. Lock-free hot path.
    std::shared_ptr<const SubscriberList> snapshot_subs() const;

    // Tries to retain whatever is currently in latest_, performing the
    // safe-retain protocol (post-load verification). Returns nullptr if no
    // published frame is currently available.
    Frame* try_acquire_latest();

    std::atomic<Frame*>   latest_{nullptr};
    std::atomic<uint64_t> latest_seq_{0};

    mutable std::mutex                    subs_mu_;  // protects writes to subs_
    std::shared_ptr<const SubscriberList> subs_;     // read via shared_ptr atomic ops
};

}  // namespace gw
