#include "core/frame_pool.hpp"

#include <stdexcept>

#include "core/iosurface_buffer.hpp"

namespace gw {

namespace {

void recycle_thunk(void* owner, Frame* f) {
    static_cast<FramePool*>(owner)->return_to_pool(f);
}

}  // namespace

FramePool::FramePool(FrameFormat format, uint32_t capacity)
    : format_(format), capacity_(capacity) {
    if (capacity == 0) {
        throw std::invalid_argument("FramePool capacity must be > 0");
    }
    slots_.reserve(capacity);
    for (uint32_t i = 0; i < capacity; ++i) {
        CVPixelBufferRef pb   = make_iosurface_pixel_buffer(format);
        auto             slot = std::make_unique<Slot>();
        // Frame's ctor CFRetains the buffer; we own the original ref and CFRelease here.
        slot->frame = std::make_unique<Frame>(this, &recycle_thunk, pb);
        CFRelease(pb);
        slot->in_pool.store(true, std::memory_order_release);
        slots_.push_back(std::move(slot));
    }
}

FramePool::~FramePool() = default;

Frame* FramePool::acquire() {
    const size_t n     = slots_.size();
    const size_t start = hint_.load(std::memory_order_relaxed) % n;
    for (size_t i = 0; i < n; ++i) {
        const size_t idx      = (start + i) % n;
        bool         expected = true;
        if (slots_[idx]->in_pool.compare_exchange_strong(expected,
                                                         false,
                                                         std::memory_order_acq_rel,
                                                         std::memory_order_relaxed)) {
            slots_[idx]->frame->reset_for_acquire();
            hint_.store((idx + 1) % n, std::memory_order_relaxed);
            return slots_[idx]->frame.get();
        }
    }
    return nullptr;
}

void FramePool::return_to_pool(Frame* f) {
    // Linear search to find the slot (capacity is small).
    for (auto& slot : slots_) {
        if (slot->frame.get() == f) {
            slot->in_pool.store(true, std::memory_order_release);
            return;
        }
    }
    // Slot not found is a programming error: a Frame was associated with a pool
    // it doesn't belong to. Aborting here is the safest behavior.
    std::terminate();
}

uint32_t FramePool::free_count() const {
    uint32_t n = 0;
    for (const auto& slot : slots_) {
        if (slot->in_pool.load(std::memory_order_acquire)) ++n;
    }
    return n;
}

}  // namespace gw
