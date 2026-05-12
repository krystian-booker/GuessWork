#include "producer/spinnaker_user_buffer_pool.hpp"

// The header pulled in <ImagePtr.h> with the macro workaround already applied.
// Pulling Spinnaker.h here gets us the full SDK (Image::Release etc.).
#include <Spinnaker.h>

#include <CoreVideo/CoreVideo.h>

#include <cstdint>
#include <new>
#include <stdexcept>
#include <string>

#include "core/iosurface_buffer.hpp"

namespace gw {

namespace {

constexpr uintptr_t kUsb3Alignment = 1024;

inline bool is_aligned(const void* p, uintptr_t alignment) {
    return (reinterpret_cast<uintptr_t>(p) & (alignment - 1)) == 0;
}

inline uint64_t round_up(uint64_t v, uint64_t align) {
    return ((v + align - 1) / align) * align;
}

}  // namespace

struct SpinnakerUserBufferPool::Slot {
    std::unique_ptr<Frame>  frame;
    CVPixelBufferRef        pixel_buffer = nullptr;
    void*                   base_addr    = nullptr;
    // Owned heap allocation so its address can be stashed in Frame::aux_data_
    // without exposing Spinnaker types to core/. nullptr when the buffer is
    // back in Spinnaker's hands.
    Spinnaker::ImagePtr*    in_flight    = nullptr;
};

SpinnakerUserBufferPool::SpinnakerUserBufferPool(FrameFormat fmt,
                                                 uint32_t    capacity,
                                                 uint64_t    spinnaker_payload_size)
    : format_(fmt),
      buffer_size_(round_up(spinnaker_payload_size, kUsb3Alignment)) {
    if (capacity == 0) {
        throw std::invalid_argument("SpinnakerUserBufferPool capacity must be > 0");
    }
    slots_.reserve(capacity);
    for (uint32_t i = 0; i < capacity; ++i) {
        CVPixelBufferRef pb = make_iosurface_pixel_buffer(fmt);

        // Lock once and keep locked for the pool's lifetime: this both
        // establishes a stable CPU-visible base address that Spinnaker can DMA
        // into and prevents IOSurface eviction.
        if (CVPixelBufferLockBaseAddress(pb, 0) != kCVReturnSuccess) {
            CFRelease(pb);
            throw std::runtime_error("CVPixelBufferLockBaseAddress failed");
        }

        void* base = CVPixelBufferGetBaseAddress(pb);
        if (base == nullptr) {
            CVPixelBufferUnlockBaseAddress(pb, 0);
            CFRelease(pb);
            throw std::runtime_error("CVPixelBufferGetBaseAddress returned null");
        }
        if (!is_aligned(base, kUsb3Alignment)) {
            CVPixelBufferUnlockBaseAddress(pb, 0);
            CFRelease(pb);
            throw std::runtime_error(
                "IOSurface base address is not 1024-byte aligned (required for USB3)");
        }

        // Verify the CVPixelBuffer can hold a full Spinnaker payload.
        // For Mono8 with no extra padding the stride equals the width and
        // bytes_per_row * height equals the Spinnaker payload; pool's buffer_size_
        // (1024-rounded payload) is the *minimum* — IOSurface may give us slightly
        // more (alignment slack), and that is fine.
        const size_t bytes_per_row = CVPixelBufferGetBytesPerRow(pb);
        const size_t cv_size       = bytes_per_row * fmt.height;
        if (cv_size < spinnaker_payload_size) {
            CVPixelBufferUnlockBaseAddress(pb, 0);
            CFRelease(pb);
            throw std::runtime_error(
                "IOSurface buffer is smaller than camera PayloadSize (" +
                std::to_string(cv_size) + " < " +
                std::to_string(spinnaker_payload_size) + ")");
        }

        auto slot          = std::make_unique<Slot>();
        slot->pixel_buffer = pb;     // owned (CFRelease in dtor)
        slot->base_addr    = base;
        slot->frame        = std::make_unique<Frame>(this, &recycle_thunk, pb);
        slots_.push_back(std::move(slot));
    }
}

SpinnakerUserBufferPool::~SpinnakerUserBufferPool() {
    // The producer must have called EndAcquisition before destroying us — if
    // any slot still has an in-flight ImagePtr at this point, release it.
    for (auto& slot : slots_) {
        if (slot->in_flight) {
            try {
                (*slot->in_flight)->Release();
            } catch (...) {
            }
            delete slot->in_flight;
            slot->in_flight = nullptr;
        }
        if (slot->pixel_buffer) {
            CVPixelBufferUnlockBaseAddress(slot->pixel_buffer, 0);
            CFRelease(slot->pixel_buffer);
        }
    }
}

size_t SpinnakerUserBufferPool::bytes_per_row() const {
    if (slots_.empty()) return 0;
    return CVPixelBufferGetBytesPerRow(slots_.front()->pixel_buffer);
}

std::vector<void*> SpinnakerUserBufferPool::base_addresses() const {
    std::vector<void*> out;
    out.reserve(slots_.size());
    for (const auto& slot : slots_) {
        out.push_back(slot->base_addr);
    }
    return out;
}

Frame* SpinnakerUserBufferPool::checkout(Spinnaker::ImagePtr img) {
    void* data = img->GetData();
    for (auto& slot : slots_) {
        if (slot->base_addr != data) continue;

        // Spinnaker shouldn't hand us the same slot twice without us releasing
        // first, but guard defensively: if there's already an in-flight ImagePtr
        // here, release the older one before overwriting (preserves Spinnaker's
        // internal accounting).
        if (slot->in_flight) {
            (*slot->in_flight)->Release();
            delete slot->in_flight;
            slot->in_flight = nullptr;
        }

        slot->in_flight = new Spinnaker::ImagePtr(std::move(img));
        slot->frame->reset_for_acquire();
        slot->frame->set_aux_data(slot->in_flight);
        return slot->frame.get();
    }
    return nullptr;
}

void SpinnakerUserBufferPool::recycle_thunk(void* owner, Frame* f) {
    static_cast<SpinnakerUserBufferPool*>(owner)->return_buffer(f);
}

void SpinnakerUserBufferPool::return_buffer(Frame* f) {
    // Recover the ImagePtr stashed in checkout(), release it, and clear
    // the slot. Linear scan over a small N (≤ 8 in practice).
    auto* held = static_cast<Spinnaker::ImagePtr*>(f->aux_data());
    if (held) {
        try {
            (*held)->Release();
        } catch (...) {
        }
        delete held;
    }
    f->set_aux_data(nullptr);

    for (auto& slot : slots_) {
        if (slot->frame.get() == f) {
            slot->in_flight = nullptr;
            return;
        }
    }
    // Not finding our own slot is a programming error.
    std::terminate();
}

}  // namespace gw
