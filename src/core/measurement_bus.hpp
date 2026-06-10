#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>

namespace gw {

// Multi-subscriber measurement distribution bus for low-rate sensor data
// (IMU samples, tag poses, VIO odometry, chassis speeds — ≤ ~400 Hz).
//
// Unlike FrameChannel this is NOT latest-only: every subscriber gets every
// published value, buffered in a bounded per-subscriber ring. When a slow
// subscriber's ring fills, the oldest entries are dropped (and counted) —
// laggards lose history, never block the publisher.
//
// Plain mutex + condition_variable; rates are far too low to justify
// lock-free machinery. The subscribe/unsubscribe ergonomics mirror
// FrameChannel so consumers look familiar.
template <typename T>
class MeasurementBus {
public:
    struct Subscriber {
        explicit Subscriber(size_t cap) : capacity(cap) {}
        const size_t  capacity;
        std::deque<T> queue;     // guarded by the bus mutex
        uint64_t      dropped = 0;
        bool          attached = true;
    };
    using SubscriberHandle = std::shared_ptr<Subscriber>;

    MeasurementBus() = default;
    MeasurementBus(const MeasurementBus&)            = delete;
    MeasurementBus& operator=(const MeasurementBus&) = delete;

    SubscriberHandle subscribe(size_t capacity = 1024) {
        auto h = std::make_shared<Subscriber>(capacity);
        std::lock_guard lk(mu_);
        subs_.push_back(h);
        return h;
    }

    // Marks the subscriber detached and wakes any blocked wait_pop().
    void unsubscribe(const SubscriberHandle& h) {
        if (!h) return;
        {
            std::lock_guard lk(mu_);
            h->attached = false;
            std::erase(subs_, h);
        }
        cv_.notify_all();
    }

    void publish(const T& value) {
        {
            std::lock_guard lk(mu_);
            for (auto& s : subs_) {
                if (s->queue.size() >= s->capacity) {
                    s->queue.pop_front();
                    ++s->dropped;
                }
                s->queue.push_back(value);
            }
        }
        cv_.notify_all();
    }

    // Non-blocking pop. Returns false when the queue is empty.
    bool try_pop(const SubscriberHandle& h, T& out) {
        std::lock_guard lk(mu_);
        if (h->queue.empty()) return false;
        out = std::move(h->queue.front());
        h->queue.pop_front();
        return true;
    }

    // Blocking pop. Returns false only when the subscriber is detached
    // (graceful thread-exit signal, like FrameChannel::next_frame).
    bool wait_pop(const SubscriberHandle& h, T& out) {
        std::unique_lock lk(mu_);
        cv_.wait(lk, [&] { return !h->attached || !h->queue.empty(); });
        if (!h->attached) return false;
        out = std::move(h->queue.front());
        h->queue.pop_front();
        return true;
    }

    // Drops accumulated for a subscriber (slow-consumer signal).
    uint64_t dropped(const SubscriberHandle& h) const {
        std::lock_guard lk(mu_);
        return h->dropped;
    }

private:
    mutable std::mutex            mu_;
    std::condition_variable       cv_;
    std::vector<SubscriberHandle> subs_;
};

}  // namespace gw
