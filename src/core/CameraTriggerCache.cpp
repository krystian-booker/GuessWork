#include "posest/CameraTriggerCache.h"

#include <utility>

#include "posest/teensy/Protocol.h"

namespace posest {

CameraTriggerCache::CameraTriggerCache(
    std::unordered_map<std::int32_t, std::string> pin_to_camera,
    std::chrono::milliseconds match_window,
    std::size_t per_camera_capacity)
    : pin_to_camera_(std::move(pin_to_camera)),
      match_window_(match_window),
      per_camera_capacity_(per_camera_capacity == 0 ? 1 : per_camera_capacity) {}

void CameraTriggerCache::recordEvent(const CameraTriggerEvent& ev) {
    if ((ev.status_flags & teensy::kStatusUnsynchronizedTime) != 0u) {
        return;
    }
    const auto pin_it = pin_to_camera_.find(ev.pin);
    if (pin_it == pin_to_camera_.end()) {
        return;
    }
    const std::string& camera_id = pin_it->second;

    std::lock_guard<std::mutex> g(mu_);
    auto& ring = per_camera_[camera_id];
    ring.push_back(TriggerStamp{ev.teensy_time_us, ev.trigger_sequence, ev.timestamp});
    // Prune events older than 2*match_window from the just-arrived event so
    // the deque stays bounded even when the per-camera capacity is large.
    const Timestamp cutoff = ev.timestamp - 2 * match_window_;
    while (!ring.empty() && ring.front().host_time < cutoff) {
        ring.pop_front();
    }
    while (ring.size() > per_camera_capacity_) {
        ring.pop_front();
    }
}

std::optional<TriggerStamp> CameraTriggerCache::lookup(
    const std::string& camera_id, Timestamp capture_time) const {
    std::lock_guard<std::mutex> g(mu_);
    const auto it = per_camera_.find(camera_id);
    if (it == per_camera_.end() || it->second.empty()) {
        return std::nullopt;
    }
    const auto& ring = it->second;
    // Walk newest-to-oldest; return the first stamp at or before capture_time
    // and within match_window.
    for (auto rit = ring.rbegin(); rit != ring.rend(); ++rit) {
        if (rit->host_time > capture_time) {
            continue;
        }
        if (capture_time - rit->host_time > match_window_) {
            return std::nullopt;
        }
        return *rit;
    }
    return std::nullopt;
}

void CameraTriggerCache::clear() {
    std::lock_guard<std::mutex> g(mu_);
    per_camera_.clear();
}

}  // namespace posest
