#include "core/frame.hpp"

namespace gw {

Frame::Frame(void* owner, RecycleFn recycle, CVPixelBufferRef buffer)
    : owner_(owner),
      recycle_(recycle),
      buffer_(buffer),
      width_(static_cast<uint32_t>(CVPixelBufferGetWidth(buffer))),
      height_(static_cast<uint32_t>(CVPixelBufferGetHeight(buffer))),
      pixel_format_(CVPixelBufferGetPixelFormatType(buffer)) {
    CFRetain(buffer_);
}

Frame::~Frame() {
    if (buffer_) {
        CFRelease(buffer_);
    }
}

size_t Frame::bytes_per_row() const {
    return CVPixelBufferGetBytesPerRow(buffer_);
}

void Frame::reset_for_acquire() {
    sequence_        = 0;
    host_capture_ns_ = 0;
    camera_ts_ns_    = 0;
    producer_id_.clear();
    aux_data_.store(nullptr, std::memory_order_relaxed);
    refcount_.store(1, std::memory_order_release);
}

void Frame::retain() {
    refcount_.fetch_add(1, std::memory_order_acq_rel);
}

bool Frame::try_retain() {
    uint32_t c = refcount_.load(std::memory_order_acquire);
    while (c > 0) {
        if (refcount_.compare_exchange_weak(c,
                                            c + 1,
                                            std::memory_order_acq_rel,
                                            std::memory_order_acquire)) {
            return true;
        }
    }
    return false;
}

void Frame::release() {
    const uint32_t prev = refcount_.fetch_sub(1, std::memory_order_acq_rel);
    if (prev == 1) {
        recycle_(owner_, this);
    }
}

}  // namespace gw
