#pragma once

#include <CoreVideo/CoreVideo.h>

#include <cstdint>

namespace gw::encoder {

// Converts a Mono8 (single-plane 8-bit grayscale) CVPixelBuffer into a fresh
// NV12 (bi-planar 4:2:0 YpCbCr) CVPixelBuffer at the configured output size:
//
//   - Y plane is the source bytes resampled to (out_w, out_h) via vImage.
//   - UV plane is uniformly 128 (chroma-neutral, i.e. grayscale).
//
// The output uses full-range chroma sampling
// (kCVPixelFormatType_420YpCbCr8BiPlanarFullRange) so we don't have to remap
// the Mono8 Y values. Output buffers come from an internal CVPixelBufferPool;
// callers receive a CFRetain'd reference and must CFRelease it.
class Mono8ToNv12 {
public:
    Mono8ToNv12(uint32_t out_w, uint32_t out_h);
    ~Mono8ToNv12();

    Mono8ToNv12(const Mono8ToNv12&)            = delete;
    Mono8ToNv12& operator=(const Mono8ToNv12&) = delete;

    // Returns a retained CVPixelBufferRef in NV12 format. Returns nullptr on
    // failure (e.g. pool exhausted, source not Mono8). The caller owns the
    // returned reference.
    CVPixelBufferRef convert(CVPixelBufferRef mono8_src);

    uint32_t out_width()  const { return out_w_; }
    uint32_t out_height() const { return out_h_; }

private:
    uint32_t              out_w_;
    uint32_t              out_h_;
    CVPixelBufferPoolRef  pool_      = nullptr;
    void*                 scale_tmp_ = nullptr;
    size_t                scale_tmp_size_ = 0;
};

}  // namespace gw::encoder
