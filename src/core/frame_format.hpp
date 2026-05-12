#pragma once

#include <CoreVideo/CoreVideo.h>

#include <cstdint>

namespace gw {

struct FrameFormat {
    uint32_t width;
    uint32_t height;
    OSType   pixel_format;   // CoreVideo FourCC, e.g. kCVPixelFormatType_OneComponent8
};

inline bool operator==(const FrameFormat& a, const FrameFormat& b) {
    return a.width == b.width && a.height == b.height && a.pixel_format == b.pixel_format;
}
inline bool operator!=(const FrameFormat& a, const FrameFormat& b) { return !(a == b); }

}  // namespace gw
