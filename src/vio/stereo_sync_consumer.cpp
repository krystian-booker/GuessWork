#include "vio/stereo_sync_consumer.hpp"

#include <CoreVideo/CoreVideo.h>

#include <cstring>
#include <utility>

#include "core/frame.hpp"
#include "core/mono8_copy.hpp"

namespace gw::vio {

// ---------------------------------------------------------------------------
// StereoSyncPairer
// ---------------------------------------------------------------------------

void StereoSyncPairer::push(Side side, SideFrame f) {
    std::lock_guard lk(mu_);
    if (shutdown_) return;

    if (f.t_ns == 0) {
        ++counters_.dropped_zero_ts;
        return;
    }

    auto& mine   = sides_[side];
    auto& theirs = sides_[side == kLeft ? kRight : kLeft];

    // Exact-equality match against the other side (shared trigger group ⇒
    // identical pulse stamps).
    bool matched = false;
    for (const auto& other : theirs) {
        if (other.t_ns == f.t_ns) {
            StereoPair pair;
            pair.t_ns = f.t_ns;
            if (side == kLeft) {
                pair.left  = std::move(f);
                pair.right = other;
            } else {
                pair.left  = other;
                pair.right = std::move(f);
            }
            // Everything at or before the matched stamp is now stale on
            // BOTH sides — a frame older than a formed pair can never pair.
            const auto purge = [&](std::deque<SideFrame>& q) {
                while (!q.empty() && q.front().t_ns <= pair.t_ns) {
                    if (q.front().t_ns != pair.t_ns) ++counters_.dropped_unmatched;
                    q.pop_front();
                }
            };
            purge(mine);
            purge(theirs);

            if (pairs_.size() >= kPairCap) {
                pairs_.pop_front();
                ++counters_.dropped_pair_queue;
            }
            pairs_.push_back(std::move(pair));
            ++counters_.paired;
            matched = true;
            break;
        }
    }

    if (!matched) {
        mine.push_back(std::move(f));
        if (mine.size() > kSideCap) {
            mine.pop_front();
            ++counters_.dropped_unmatched;
        }
        return;  // nothing new for the consumer
    }
    cv_.notify_one();
}

bool StereoSyncPairer::wait_pop(StereoPair& out) {
    std::unique_lock lk(mu_);
    cv_.wait(lk, [&] { return shutdown_ || !pairs_.empty(); });
    if (pairs_.empty()) return false;  // shutdown and drained
    out = std::move(pairs_.front());
    pairs_.pop_front();
    return true;
}

void StereoSyncPairer::shutdown() {
    {
        std::lock_guard lk(mu_);
        shutdown_ = true;
    }
    cv_.notify_all();
}

void StereoSyncPairer::reset() {
    std::lock_guard lk(mu_);
    sides_[0].clear();
    sides_[1].clear();
    pairs_.clear();
}

StereoSyncPairer::Counters StereoSyncPairer::counters() const {
    std::lock_guard lk(mu_);
    return counters_;
}

// ---------------------------------------------------------------------------
// VioFeederConsumer
// ---------------------------------------------------------------------------

VioFeederConsumer::VioFeederConsumer(StereoSyncPairer::Side            side,
                                     std::shared_ptr<StereoSyncPairer> pairer,
                                     bool                              rotate_180)
    : side_(side), pairer_(std::move(pairer)), rotate_180_(rotate_180) {}

VioFeederConsumer::~VioFeederConsumer() {
    detach();
}

void VioFeederConsumer::attach(gw::FrameChannel& ch) {
    if (running_.load()) return;
    channel_ = &ch;
    sub_     = ch.subscribe();
    running_.store(true, std::memory_order_release);
    worker_ = std::thread([this] { run(); });
}

void VioFeederConsumer::detach() {
    if (!running_.exchange(false)) return;
    if (channel_ && sub_) channel_->unsubscribe(sub_);
    if (worker_.joinable()) worker_.join();
    sub_.reset();
    channel_ = nullptr;
}

void VioFeederConsumer::run() {
    while (running_.load(std::memory_order_acquire)) {
        gw::Frame* f = channel_->next_frame(sub_);
        if (!f) break;  // detached

        SideFrame sf;
        sf.t_ns   = static_cast<int64_t>(f->camera_ts_ns());
        sf.width  = f->width();
        sf.height = f->height();

        // Zero stamp = no Teensy pulse matched; the pairer would drop it
        // anyway — skip the copy entirely.
        if (sf.t_ns == 0) {
            f->release();
            continue;
        }

        bool copied = false;
        CVPixelBufferRef pb = f->pixel_buffer();
        if (pb &&
            CVPixelBufferLockBaseAddress(pb, kCVPixelBufferLock_ReadOnly) ==
                kCVReturnSuccess) {
            const auto* base = static_cast<const uint8_t*>(
                CVPixelBufferGetBaseAddress(pb));
            const size_t stride = CVPixelBufferGetBytesPerRow(pb);
            if (base) {
                const size_t bytes =
                    static_cast<size_t>(sf.width) * static_cast<size_t>(sf.height);
                sf.pixels.resize(bytes);
                gw::copy_mono8(sf.pixels.data(), base, stride,
                               sf.width, sf.height, rotate_180_);
                copied = true;
            }
            CVPixelBufferUnlockBaseAddress(pb, kCVPixelBufferLock_ReadOnly);
        }
        f->release();

        if (!copied) continue;
        frames_copied_.fetch_add(1, std::memory_order_relaxed);
        pairer_->push(side_, std::move(sf));
    }
}

}  // namespace gw::vio
