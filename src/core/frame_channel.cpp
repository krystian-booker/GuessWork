#include "core/frame_channel.hpp"

#include <algorithm>

namespace gw {

FrameChannel::FrameChannel() : subs_(std::make_shared<SubscriberList>()) {}

FrameChannel::~FrameChannel() {
    // Drop any remaining published frame so its slot returns to its pool.
    Frame* f = latest_.exchange(nullptr, std::memory_order_acq_rel);
    if (f) f->release();
}

std::shared_ptr<const FrameChannel::SubscriberList> FrameChannel::snapshot_subs() const {
    return std::atomic_load_explicit(&subs_, std::memory_order_acquire);
}

void FrameChannel::publish(Frame* f) {
    Frame* old = latest_.exchange(f, std::memory_order_acq_rel);
    // latest_seq_ ordering: the consumer's wait-then-check uses this atomic to
    // detect "is there a new frame". Storing AFTER the exchange guarantees that
    // any consumer that sees the new latest_seq_ also sees the new latest_.
    latest_seq_.store(f->sequence(), std::memory_order_release);
    if (old) old->release();

    // Wake any subscriber whose last_seen_seq lags behind this frame.
    auto subs = snapshot_subs();
    if (!subs) return;
    const uint64_t new_seq = f->sequence();
    for (const auto& s : *subs) {
        if (!s->attached.load(std::memory_order_acquire)) continue;
        if (s->last_seen_seq.load(std::memory_order_relaxed) < new_seq) {
            s->wait_token.fetch_add(1, std::memory_order_release);
            s->wait_token.notify_one();
        }
    }
}

FrameChannel::SubscriberHandle FrameChannel::subscribe() {
    auto sub = std::make_shared<Subscriber>();
    // Seed last_seen_seq with the current latest so a fresh subscriber only
    // gets frames published AFTER it attached.
    sub->last_seen_seq.store(latest_seq_.load(std::memory_order_acquire),
                             std::memory_order_relaxed);

    std::lock_guard lk(subs_mu_);
    auto next = std::make_shared<SubscriberList>(*subs_);
    next->push_back(sub);
    std::atomic_store_explicit(&subs_,
                               std::shared_ptr<const SubscriberList>(std::move(next)),
                               std::memory_order_release);
    return sub;
}

void FrameChannel::unsubscribe(SubscriberHandle h) {
    if (!h) return;
    h->attached.store(false, std::memory_order_release);
    h->wait_token.fetch_add(1, std::memory_order_release);
    h->wait_token.notify_all();

    std::lock_guard lk(subs_mu_);
    auto next = std::make_shared<SubscriberList>();
    next->reserve(subs_->size());
    for (const auto& s : *subs_) {
        if (s.get() != h.get()) next->push_back(s);
    }
    std::atomic_store_explicit(&subs_,
                               std::shared_ptr<const SubscriberList>(std::move(next)),
                               std::memory_order_release);
}

Frame* FrameChannel::try_acquire_latest() {
    while (true) {
        Frame* f = latest_.load(std::memory_order_acquire);
        if (!f) return nullptr;
        if (!f->try_retain()) {
            // The frame's slot was just recycled (refcount briefly 0). Spin
            // and retry — by the next iteration latest_ will have advanced
            // (or rolled to the same slot with a newer fill).
            continue;
        }
        // Post-load verification: latest_ must still point at f. If it has
        // moved on, the slot may have been re-acquired by the producer and
        // is currently being filled with new data; releasing here keeps the
        // refcount bumped only briefly and the slot still alive.
        if (latest_.load(std::memory_order_acquire) == f) {
            return f;
        }
        f->release();
        // loop
    }
}

Frame* FrameChannel::next_frame(SubscriberHandle h) {
    if (!h) return nullptr;
    while (true) {
        if (!h->attached.load(std::memory_order_acquire)) return nullptr;

        const uint64_t seen = h->last_seen_seq.load(std::memory_order_relaxed);
        if (latest_seq_.load(std::memory_order_acquire) > seen) {
            Frame* f = try_acquire_latest();
            if (f) {
                const uint64_t fseq = f->sequence();
                if (fseq > seen) {
                    h->last_seen_seq.store(fseq, std::memory_order_release);
                    return f;
                }
                // fseq <= seen: this happens only if we raced into a frame whose
                // sequence we'd already advanced past via a previous return. Drop
                // and retry to fetch a fresher one.
                f->release();
                continue;
            }
            // Latest is null (channel destroyed?) or we raced; loop and re-check.
            continue;
        }

        // No frame newer than seen. Capture token, double-check, then wait.
        const uint64_t token = h->wait_token.load(std::memory_order_acquire);
        if (!h->attached.load(std::memory_order_acquire)) return nullptr;
        if (latest_seq_.load(std::memory_order_acquire) > seen) continue;
        h->wait_token.wait(token, std::memory_order_acquire);
    }
}

}  // namespace gw
