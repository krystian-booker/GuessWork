#pragma once

#include <CoreVideo/CoreVideo.h>

#include <cstdint>
#include <memory>
#include <vector>

// Spinnaker workaround: pre-include the SDK's platform header and neutralize
// its [[deprecated]]+__attribute__ macro that recent Apple Clang rejects.
#include <SpinnakerPlatform.h>
#undef  SPINNAKER_DEPRECATED_CLASS
#define SPINNAKER_DEPRECATED_CLASS(msg) class SPINNAKER_API

#include <ImagePtr.h>

#include "core/frame.hpp"
#include "core/frame_format.hpp"

namespace gw {

// A fixed-set pool of IOSurface-backed CVPixelBuffers registered with Spinnaker
// as user-supplied DMA targets (SetBufferOwnership(USER) + SetUserBuffers).
//
// Unlike FramePool, the pool does not manage a free list — Spinnaker owns the
// cycle. We just route: when GetNextImage returns an ImagePtr we find the slot
// whose base address matches img->GetData(), bind the held ImagePtr to that
// Frame, and hand the Frame to the channel. When the Frame's refcount drops to
// 0, the held ImagePtr is released, which returns the buffer to Spinnaker's
// internal queue for the next DMA.
class SpinnakerUserBufferPool {
public:
    // capacity: number of buffers (≥ 3 required by Spinnaker for NewestOnly).
    // spinnaker_payload_size: PayloadSize node value from the camera. Each
    //   buffer's reported size is rounded up to the 1024-byte USB3 grain.
    // Throws if the IOSurface base address isn't 1024-byte aligned or the
    // CVPixelBuffer's data area is smaller than the camera's payload.
    SpinnakerUserBufferPool(FrameFormat fmt,
                            uint32_t    capacity,
                            uint64_t    spinnaker_payload_size);
    ~SpinnakerUserBufferPool();

    SpinnakerUserBufferPool(const SpinnakerUserBufferPool&)            = delete;
    SpinnakerUserBufferPool& operator=(const SpinnakerUserBufferPool&) = delete;

    uint32_t           capacity()    const { return static_cast<uint32_t>(slots_.size()); }
    const FrameFormat& format()      const { return format_; }
    uint64_t           buffer_size() const { return buffer_size_; }
    size_t             bytes_per_row() const;

    // Returns the base-address array to pass to Spinnaker's SetUserBuffers.
    std::vector<void*> base_addresses() const;

    // Take ownership of an in-flight ImagePtr just returned by GetNextImage.
    // Looks up the matching slot by base-address. On success, the returned
    // Frame has refcount = 1 and aux_data set to a heap-allocated copy of img.
    // Returns nullptr if no slot's base address matches (a Spinnaker bug or
    // a pointer not from our pool).
    Frame* checkout(Spinnaker::ImagePtr img);

private:
    struct Slot;

    static void recycle_thunk(void* owner, Frame* f);
    void        return_buffer(Frame* f);

    FrameFormat                          format_;
    uint64_t                             buffer_size_;
    std::vector<std::unique_ptr<Slot>>   slots_;
};

}  // namespace gw
