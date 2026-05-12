#pragma once

#include <CoreVideo/CoreVideo.h>

#include <atomic>
#include <cstdint>
#include <string>
#include <string_view>

#include "core/frame_format.hpp"

namespace gw {

// An intrusively-refcounted holder for one captured image plus metadata.
//
// Ownership model:
//   - Frames are owned by some pool / source that constructs them and registers
//     a recycle callback. When the refcount drops back to 0 the callback is
//     invoked so the owner can return the underlying buffer to its free list
//     (FramePool) or release a held resource (e.g. a Spinnaker ImagePtr in
//     SpinnakerUserBufferPool's DMA mode).
//   - The recycle hook is type-erased so core/frame.hpp stays free of SDK
//     dependencies — owners pass their `this` pointer as `void* owner` and a
//     static thunk that casts it back.
//   - aux_data_ is an opaque void* slot the producer can use to associate
//     out-of-band state with the frame (e.g. an in-flight Spinnaker ImagePtr).
//     core/ never interprets it.
//
// Thread-safety:
//   - retain()/release() are safe to call from any thread.
//   - Metadata setters and set_aux_data() are intended for the producer thread
//     between checkout and publish; they are NOT safe to call after the frame
//     is visible to consumers.
//   - Metadata getters are safe once the frame has been published (memory
//     visibility is established by the publish's release-store on the channel).
class Frame {
public:
    using RecycleFn = void (*)(void* owner, Frame* f);

    Frame(const Frame&)            = delete;
    Frame& operator=(const Frame&) = delete;
    Frame(Frame&&)                 = delete;
    Frame& operator=(Frame&&)      = delete;

    // Construct a frame bound to the given recycle owner + callback. The buffer
    // is CFRetained; ownership is shared with the caller, who is responsible
    // for CFRelease'ing their reference. Frame's destructor CFRelease's its own.
    Frame(void* owner, RecycleFn recycle, CVPixelBufferRef buffer);

    // Public only to satisfy std::unique_ptr<Frame> ownership inside pools.
    ~Frame();

    // --- Image metadata ---
    uint32_t         width()         const { return width_; }
    uint32_t         height()        const { return height_; }
    OSType           pixel_format()  const { return pixel_format_; }
    size_t           bytes_per_row() const;
    CVPixelBufferRef pixel_buffer()  const { return buffer_; }

    // --- Per-frame metadata (producer writes between checkout and publish) ---
    void             set_sequence(uint64_t s)          { sequence_ = s; }
    void             set_host_capture_ns(uint64_t ns)  { host_capture_ns_ = ns; }
    void             set_camera_ts_ns(uint64_t ns)     { camera_ts_ns_ = ns; }
    void             set_producer_id(std::string_view id) { producer_id_.assign(id); }

    uint64_t         sequence()        const { return sequence_; }
    uint64_t         host_capture_ns() const { return host_capture_ns_; }
    uint64_t         camera_ts_ns()    const { return camera_ts_ns_; }
    std::string_view producer_id()     const { return producer_id_; }

    // --- Owner-managed opaque side-channel (e.g. Spinnaker ImagePtr*) ---
    void  set_aux_data(void* p) { aux_data_.store(p, std::memory_order_release); }
    void* aux_data() const      { return aux_data_.load(std::memory_order_acquire); }

    // --- Refcount ---
    void retain();
    void release();
    // Atomically increment refcount only if it's currently non-zero. Used by
    // FrameChannel's consumer pull path to safely retain a frame whose
    // slot may have been recycled between the channel's pointer load and this
    // retain. Returns true on success; false means the frame has been recycled
    // and the caller must re-load latest_ and retry.
    bool     try_retain();
    uint32_t refcount() const { return refcount_.load(std::memory_order_acquire); }

    // Reset metadata fields and bring refcount from 0 → 1. The owner calls
    // this when handing the frame out to a producer. Not thread-safe; only the
    // owner should call it, and only when the frame is currently recycled.
    void reset_for_acquire();

private:
    std::atomic<uint32_t> refcount_{0};
    void*                 owner_;
    RecycleFn             recycle_;
    CVPixelBufferRef      buffer_;   // CFRetain'd in ctor, CFRelease'd in dtor
    std::atomic<void*>    aux_data_{nullptr};

    uint32_t width_;
    uint32_t height_;
    OSType   pixel_format_;

    uint64_t    sequence_        = 0;
    uint64_t    host_capture_ns_ = 0;
    uint64_t    camera_ts_ns_    = 0;
    std::string producer_id_;
};

}  // namespace gw
