#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string_view>
#include <thread>
#include <vector>

#include "consumer/consumer.hpp"
#include "core/frame_channel.hpp"

// Stereo frame synchronization for VIO.
//
// The two VIO cameras share one Teensy trigger group, so a stereo pair has
// IDENTICAL camera_ts_ns pulse stamps — pairing is exact-equality matching,
// not nearest-neighbour. Frames whose camera_ts_ns is 0 (no pulse matched —
// the producer's fallback path) are dropped: they can't be paired and their
// clock domain is wrong for the IMU anyway.
//
// Lifecycle: one VioFeederConsumer per camera rides the CameraSupervisor
// slot lifecycle (the ConsumerFactory mechanism — attach on producer start,
// detach before producer destruction). The StereoSyncPairer is owned by
// VioSupervisor and OUTLIVES the feeders: a USB unplug detaches one feeder,
// pairs naturally stop forming, and the VIO runner idles until frames
// resume. Public interfaces carry pixels as std::vector<uint8_t> — no
// OpenCV in any header here.

namespace gw::vio {

struct SideFrame {
    int64_t              t_ns   = 0;  // Teensy pulse stamp
    uint32_t             width  = 0;
    uint32_t             height = 0;
    std::vector<uint8_t> pixels;      // tightly packed Mono8
};

struct StereoPair {
    int64_t   t_ns = 0;
    SideFrame left;
    SideFrame right;
};

class StereoSyncPairer {
public:
    enum Side : int { kLeft = 0, kRight = 1 };

    // Thread-safe; called from the two feeder pull threads.
    void push(Side side, SideFrame f);

    // Blocking; returns false after shutdown() (graceful consumer exit).
    bool wait_pop(StereoPair& out);

    // Wakes wait_pop with false. Idempotent; push() becomes a no-op after,
    // until reset() revives the pairer.
    void shutdown();

    // Clears buffered frames/pairs (counters survive) and clears any prior
    // shutdown() so the pairer is usable again. Used on runner rebuild so a
    // new VIO session doesn't start on stale frames.
    void reset();

    struct Counters {
        uint64_t paired             = 0;
        uint64_t dropped_zero_ts    = 0;  // no Teensy pulse matched the frame
        uint64_t dropped_unmatched  = 0;  // evicted before a partner arrived
        uint64_t dropped_pair_queue = 0;  // consumer too slow, oldest pair lost
    };
    Counters counters() const;

private:
    // Reorder slack: at most 2 buffered frames per side (~2 frame periods
    // of staleness at the configured fps) before the oldest is evicted.
    static constexpr size_t kSideCap = 2;
    // Output slack for the runner (drop-oldest beyond this).
    static constexpr size_t kPairCap = 4;

    mutable std::mutex      mu_;
    std::condition_variable cv_;
    std::deque<SideFrame>   sides_[2];
    std::deque<StereoPair>  pairs_;
    Counters                counters_;
    bool                    shutdown_ = false;
};

// Per-camera factory product: fast-copies frames out of the camera's
// latest-only FrameChannel into the shared pairer. The copy pattern (lock,
// stride-aware memcpy, release immediately) mirrors
// MultiTopicBagRecorder::run_pull.
class VioFeederConsumer final : public gw::IConsumer {
public:
    // `rotate_180`: camera physically mounted upside-down — the copy reverses
    // rows+columns so OpenVINS sees the pair in one consistent roll (its KLT
    // stereo matcher cannot associate features across a 180° relative roll).
    // MUST match the calibration recordings (the bag recorders flip with the
    // same flag) — the calibration's pixel frame is the flipped one.
    VioFeederConsumer(StereoSyncPairer::Side          side,
                      std::shared_ptr<StereoSyncPairer> pairer,
                      bool                            rotate_180 = false);
    ~VioFeederConsumer() override;

    VioFeederConsumer(const VioFeederConsumer&)            = delete;
    VioFeederConsumer& operator=(const VioFeederConsumer&) = delete;

    std::string_view name() const override {
        return side_ == StereoSyncPairer::kLeft ? "vio-feeder-left"
                                                : "vio-feeder-right";
    }
    void attach(gw::FrameChannel& ch) override;
    void detach() override;  // does NOT shut the pairer down

    uint64_t frames_copied() const {
        return frames_copied_.load(std::memory_order_relaxed);
    }

private:
    void run();

    const StereoSyncPairer::Side      side_;
    std::shared_ptr<StereoSyncPairer> pairer_;
    const bool                        rotate_180_;

    gw::FrameChannel*                  channel_ = nullptr;
    gw::FrameChannel::SubscriberHandle sub_;
    std::thread                        worker_;
    std::atomic<bool>                  running_{false};
    std::atomic<uint64_t>              frames_copied_{0};
};

}  // namespace gw::vio
