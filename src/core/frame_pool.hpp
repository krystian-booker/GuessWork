#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <vector>

#include "core/frame.hpp"
#include "core/frame_format.hpp"

namespace gw {

// A fixed-capacity pool of pre-allocated, IOSurface-backed Frames.
//
// Allocation strategy:
//   - Each Frame wraps its own CVPixelBufferRef, created with IOSurface backing
//     so it can be shared zero-copy across CPU/GPU/Neural Engine.
//   - The pool owns the Frame objects with stable pointers (no move/reallocation).
//   - acquire() linearly scans an atomic "in pool" flag per frame. Capacity is
//     small (typically pool_size = K_consumers + 3), so the scan is effectively
//     constant time.
//   - release_to_pool() is called by Frame::release() when the refcount reaches
//     zero, returning the frame to the free list for re-use.
class FramePool {
public:
    FramePool(FrameFormat format, uint32_t capacity);
    ~FramePool();

    FramePool(const FramePool&)            = delete;
    FramePool& operator=(const FramePool&) = delete;

    // Acquire a free frame. On success, the returned Frame has refcount == 1 and
    // all per-frame metadata reset. Returns nullptr if the pool is exhausted.
    Frame* acquire();

    // Called by Frame::release() when refcount drops to zero. Public so the
    // friendship surface stays small; production code should never call this.
    void return_to_pool(Frame* f);

    uint32_t           capacity()    const { return capacity_; }
    uint32_t           free_count()  const;
    const FrameFormat& format()      const { return format_; }

private:
    struct Slot {
        std::unique_ptr<Frame> frame;
        std::atomic<bool>      in_pool{true};
    };

    FrameFormat                        format_;
    uint32_t                           capacity_;
    std::vector<std::unique_ptr<Slot>> slots_;
    std::atomic<size_t>                hint_{0};
};

}  // namespace gw
