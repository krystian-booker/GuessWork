#include "posest/LatestFrameSlot.h"

#include <utility>

namespace posest {

void LatestFrameSlot::put(FramePtr frame) {
    {
        std::lock_guard<std::mutex> g(mu_);
        if (shut_) {
            return;
        }
        if (pending_) {
            dropped_.fetch_add(1, std::memory_order_relaxed);
        }
        pending_ = std::move(frame);
    }
    cv_.notify_one();
}

FramePtr LatestFrameSlot::take() {
    std::unique_lock<std::mutex> g(mu_);
    cv_.wait(g, [this] { return pending_ || shut_; });
    if (pending_) {
        FramePtr out = std::move(pending_);
        pending_.reset();
        return out;
    }
    return nullptr;
}

void LatestFrameSlot::shutdown() {
    {
        std::lock_guard<std::mutex> g(mu_);
        shut_ = true;
    }
    cv_.notify_all();
}

std::uint64_t LatestFrameSlot::droppedCount() const {
    return dropped_.load(std::memory_order_relaxed);
}

}  // namespace posest
