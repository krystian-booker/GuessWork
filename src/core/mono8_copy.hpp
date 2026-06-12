#pragma once

#include <cstdint>
#include <cstring>

namespace gw {

// Copies a Mono8 plane out of a (possibly row-padded) source into a tight
// width*height destination, optionally rotated 180° (rows and columns both
// reversed — the layout produced by an upside-down-mounted camera viewed
// upright).
//
// Used by every consumer that copies frames out of the IOSurface (VIO feeder,
// calibration bag recorders): for VIO-role cameras mounted at 180° the flip
// happens HERE, in the copy that already exists, so OpenVINS and the Kalibr
// recordings share one upright pixel frame at no extra pass. The zero-copy
// consumers (AprilTag, preview encoder) keep the sensor-native frame — they
// are orientation-agnostic by design (docs/pose_pipeline.md).
inline void copy_mono8(uint8_t*       dst,
                       const uint8_t* src,
                       size_t         src_stride,
                       uint32_t       width,
                       uint32_t       height,
                       bool           rotate_180) {
    if (!rotate_180) {
        if (src_stride == width) {
            std::memcpy(dst, src, static_cast<size_t>(width) * height);
            return;
        }
        for (uint32_t y = 0; y < height; ++y) {
            std::memcpy(dst + static_cast<size_t>(y) * width,
                        src + static_cast<size_t>(y) * src_stride, width);
        }
        return;
    }
    // 180°: source row y, left-to-right → destination row (H-1-y),
    // right-to-left. The reversal loop auto-vectorizes (NEON byte-reverse).
    for (uint32_t y = 0; y < height; ++y) {
        const uint8_t* s = src + static_cast<size_t>(y) * src_stride;
        uint8_t* d = dst + static_cast<size_t>(height - 1 - y) * width + (width - 1);
        for (uint32_t x = 0; x < width; ++x) {
            *d-- = s[x];
        }
    }
}

}  // namespace gw
