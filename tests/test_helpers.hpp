#pragma once

#include <CoreVideo/CoreVideo.h>

#include <cstdint>

namespace gw::test {

// Build a Mono8 CVPixelBuffer with a row-major synthetic pattern. The seq
// parameter shifts the pattern so successive frames in a sequence differ
// (useful for keyframe / inter-frame coverage in encoder tests).
inline CVPixelBufferRef make_mono8(uint32_t w, uint32_t h, uint32_t seq = 0) {
    CVPixelBufferRef pb = nullptr;
    if (CVPixelBufferCreate(kCFAllocatorDefault, w, h,
                            kCVPixelFormatType_OneComponent8,
                            nullptr, &pb) != kCVReturnSuccess) {
        return nullptr;
    }
    CVPixelBufferLockBaseAddress(pb, 0);
    auto*  base   = static_cast<uint8_t*>(CVPixelBufferGetBaseAddress(pb));
    size_t stride = CVPixelBufferGetBytesPerRow(pb);
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            base[y * stride + x] = static_cast<uint8_t>((x + y + seq * 3) & 0xFF);
        }
    }
    CVPixelBufferUnlockBaseAddress(pb, 0);
    return pb;
}

}  // namespace gw::test
