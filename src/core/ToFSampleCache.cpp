#include "posest/ToFSampleCache.h"

#include <utility>

namespace posest {

ToFSampleCache::ToFSampleCache(std::string vio_camera_id, std::size_t capacity)
    : vio_camera_id_(std::move(vio_camera_id)),
      capacity_(capacity == 0 ? 1 : capacity) {}

void ToFSampleCache::recordSample(const ToFSample& sample) {
    std::lock_guard<std::mutex> g(mu_);
    ring_.push_back(sample);
    while (ring_.size() > capacity_) {
        ring_.pop_front();
    }
}

std::optional<ToFSample> ToFSampleCache::lookupBySequence(
    const std::string& camera_id,
    std::uint32_t trigger_sequence) const {
    if (camera_id != vio_camera_id_) {
        return std::nullopt;
    }
    std::lock_guard<std::mutex> g(mu_);
    // Walk newest-to-oldest; match on trigger_sequence.
    for (auto rit = ring_.rbegin(); rit != ring_.rend(); ++rit) {
        if (rit->trigger_sequence == trigger_sequence) {
            return *rit;
        }
    }
    return std::nullopt;
}

void ToFSampleCache::clear() {
    std::lock_guard<std::mutex> g(mu_);
    ring_.clear();
}

}  // namespace posest
