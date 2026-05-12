#pragma once

#include <CoreVideo/CoreVideo.h>

#include <atomic>
#include <cstdint>
#include <string>
#include <string_view>

#include "core/frame_format.hpp"

namespace gw {

class FramePool;

// An intrusively-refcounted holder for one captured image plus metadata.
//
// Lifecycle:
//   - Frames are owned by a FramePool. The pool pre-allocates them with a
//     fixed format and recycles them via a free list.
//   - acquire() on the pool returns a Frame with refcount == 1, ready for the
//     producer to fill.
//   - Once published into a FrameChannel, retain()/release() balance every
//     consumer reference. When refcount drops to 0 the frame goes back to its
//     pool's free list (its CVPixelBuffer is NOT destroyed).
//   - When the pool is destroyed, all frames (and their CVPixelBuffers) are
//     released regardless of refcount.
//
// Thread-safety:
//   - retain()/release() are safe to call from any thread.
//   - Metadata setters are intended for the producer thread between acquire()
//     and publish(); they are NOT safe to call after the frame is visible to
//     consumers.
//   - Metadata getters are safe once the frame has been published (memory
//     visibility is established by the publish's release-store on the channel).
class Frame {
public:
    Frame(const Frame&)            = delete;
    Frame& operator=(const Frame&) = delete;
    Frame(Frame&&)                 = delete;
    Frame& operator=(Frame&&)      = delete;

    // Public destructor only to satisfy std::unique_ptr<Frame> ownership inside
    // FramePool. Frames cannot be constructed outside the pool (private ctor).
    ~Frame();

    // --- Image metadata ---
    uint32_t         width()         const { return width_; }
    uint32_t         height()        const { return height_; }
    OSType           pixel_format()  const { return pixel_format_; }
    size_t           bytes_per_row() const;
    CVPixelBufferRef pixel_buffer()  const { return buffer_; }

    // --- Per-frame metadata (producer writes between acquire and publish) ---
    void             set_sequence(uint64_t s)          { sequence_ = s; }
    void             set_host_capture_ns(uint64_t ns)  { host_capture_ns_ = ns; }
    void             set_camera_ts_ns(uint64_t ns)     { camera_ts_ns_ = ns; }
    void             set_producer_id(std::string_view id) { producer_id_.assign(id); }

    uint64_t         sequence()        const { return sequence_; }
    uint64_t         host_capture_ns() const { return host_capture_ns_; }
    uint64_t         camera_ts_ns()    const { return camera_ts_ns_; }
    std::string_view producer_id()     const { return producer_id_; }

    // --- Refcount ---
    void retain();
    void release();
    // Atomically increment refcount only if it's currently non-zero. Used by
    // FrameChannel's consumer pull path to safely retain a frame whose pool
    // slot may have been recycled between the channel's pointer load and this
    // retain. Returns true on success; false means the frame has been recycled
    // and the caller must re-load latest_ and retry.
    bool     try_retain();
    uint32_t refcount() const { return refcount_.load(std::memory_order_acquire); }

private:
    friend class FramePool;

    Frame(FramePool& pool, CVPixelBufferRef buffer);

    // Reset per-frame state at acquire time. Called by the pool.
    void reset_for_acquire();

    std::atomic<uint32_t> refcount_{0};
    FramePool*            pool_;
    CVPixelBufferRef      buffer_;   // CFRetain'd in ctor, CFRelease'd in dtor

    uint32_t width_;
    uint32_t height_;
    OSType   pixel_format_;

    uint64_t    sequence_        = 0;
    uint64_t    host_capture_ns_ = 0;
    uint64_t    camera_ts_ns_    = 0;
    std::string producer_id_;
};

}  // namespace gw
