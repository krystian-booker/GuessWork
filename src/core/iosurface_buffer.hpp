#pragma once

#include <CoreFoundation/CoreFoundation.h>
#include <CoreVideo/CoreVideo.h>

#include <stdexcept>
#include <string>

#include "core/frame_format.hpp"

namespace gw {

// Create one IOSurface-backed CVPixelBuffer with the given format.
//
// Returns a CFRetain'd buffer the caller is responsible for CFRelease'ing.
// Buffers are aligned to 64-byte row strides (cache-line + NEON friendly) and
// backed by an IOSurface so they can be shared zero-copy between CPU/GPU/NE on
// Apple Silicon and used as DMA targets via Spinnaker's user-buffer API.
//
// Throws std::runtime_error on allocation failure.
inline CVPixelBufferRef make_iosurface_pixel_buffer(const FrameFormat& fmt) {
    CFMutableDictionaryRef iosurface_props =
        CFDictionaryCreateMutable(kCFAllocatorDefault,
                                  0,
                                  &kCFTypeDictionaryKeyCallBacks,
                                  &kCFTypeDictionaryValueCallBacks);

    CFMutableDictionaryRef attrs =
        CFDictionaryCreateMutable(kCFAllocatorDefault,
                                  0,
                                  &kCFTypeDictionaryKeyCallBacks,
                                  &kCFTypeDictionaryValueCallBacks);
    CFDictionarySetValue(attrs, kCVPixelBufferIOSurfacePropertiesKey, iosurface_props);

    const int   align    = 64;
    CFNumberRef alignRef = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &align);
    CFDictionarySetValue(attrs, kCVPixelBufferBytesPerRowAlignmentKey, alignRef);
    CFRelease(alignRef);

    CVPixelBufferRef buffer = nullptr;
    const CVReturn   r      = CVPixelBufferCreate(kCFAllocatorDefault,
                                           fmt.width,
                                           fmt.height,
                                           fmt.pixel_format,
                                           attrs,
                                           &buffer);

    CFRelease(attrs);
    CFRelease(iosurface_props);

    if (r != kCVReturnSuccess || buffer == nullptr) {
        throw std::runtime_error("CVPixelBufferCreate failed");
    }
    return buffer;
}

}  // namespace gw
