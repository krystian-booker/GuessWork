#include "encoder/mono8_to_nv12.hpp"

#include <Accelerate/Accelerate.h>
#include <CoreFoundation/CoreFoundation.h>

#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>

namespace gw::encoder {

namespace {

CFNumberRef cfnum_int(int32_t v) {
    return CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &v);
}

CFDictionaryRef make_pixel_buffer_attrs(uint32_t w, uint32_t h, OSType pix_fmt) {
    CFMutableDictionaryRef d = CFDictionaryCreateMutable(
        kCFAllocatorDefault, 0,
        &kCFTypeDictionaryKeyCallBacks,
        &kCFTypeDictionaryValueCallBacks);

    CFNumberRef cf_fmt = cfnum_int(static_cast<int32_t>(pix_fmt));
    CFNumberRef cf_w   = cfnum_int(static_cast<int32_t>(w));
    CFNumberRef cf_h   = cfnum_int(static_cast<int32_t>(h));
    CFDictionarySetValue(d, kCVPixelBufferPixelFormatTypeKey, cf_fmt);
    CFDictionarySetValue(d, kCVPixelBufferWidthKey,            cf_w);
    CFDictionarySetValue(d, kCVPixelBufferHeightKey,           cf_h);

    // Backing IOSurfaces so VideoToolbox can zero-copy these later if desired.
    CFMutableDictionaryRef io_surface_props = CFDictionaryCreateMutable(
        kCFAllocatorDefault, 0,
        &kCFTypeDictionaryKeyCallBacks,
        &kCFTypeDictionaryValueCallBacks);
    CFDictionarySetValue(d, kCVPixelBufferIOSurfacePropertiesKey, io_surface_props);

    CFRelease(cf_fmt);
    CFRelease(cf_w);
    CFRelease(cf_h);
    CFRelease(io_surface_props);
    return d;
}

}  // namespace

Mono8ToNv12::Mono8ToNv12(uint32_t out_w, uint32_t out_h)
    : out_w_(out_w), out_h_(out_h) {
    if (out_w_ == 0 || out_h_ == 0 || (out_w_ & 1u) || (out_h_ & 1u)) {
        throw std::invalid_argument(
            "Mono8ToNv12: output dimensions must be even and non-zero");
    }

    CFDictionaryRef pb_attrs = make_pixel_buffer_attrs(
        out_w_, out_h_, kCVPixelFormatType_420YpCbCr8BiPlanarFullRange);

    const CVReturn r = CVPixelBufferPoolCreate(
        kCFAllocatorDefault,
        /*poolAttributes=*/nullptr,
        pb_attrs,
        &pool_);
    CFRelease(pb_attrs);

    if (r != kCVReturnSuccess || !pool_) {
        throw std::runtime_error("CVPixelBufferPoolCreate failed: " + std::to_string(r));
    }
}

Mono8ToNv12::~Mono8ToNv12() {
    if (pool_) CFRelease(pool_);
    if (scale_tmp_) std::free(scale_tmp_);
}

CVPixelBufferRef Mono8ToNv12::convert(CVPixelBufferRef src) {
    if (!src) return nullptr;
    if (CVPixelBufferGetPixelFormatType(src) != kCVPixelFormatType_OneComponent8) {
        return nullptr;
    }

    CVPixelBufferRef dst = nullptr;
    if (CVPixelBufferPoolCreatePixelBuffer(kCFAllocatorDefault, pool_, &dst) !=
            kCVReturnSuccess || !dst) {
        return nullptr;
    }

    if (CVPixelBufferLockBaseAddress(src, kCVPixelBufferLock_ReadOnly) !=
            kCVReturnSuccess) {
        CFRelease(dst);
        return nullptr;
    }
    if (CVPixelBufferLockBaseAddress(dst, 0) != kCVReturnSuccess) {
        CVPixelBufferUnlockBaseAddress(src, kCVPixelBufferLock_ReadOnly);
        CFRelease(dst);
        return nullptr;
    }

    vImage_Buffer vi_src{
        .data     = CVPixelBufferGetBaseAddress(src),
        .height   = CVPixelBufferGetHeight(src),
        .width    = CVPixelBufferGetWidth(src),
        .rowBytes = CVPixelBufferGetBytesPerRow(src),
    };
    vImage_Buffer vi_y{
        .data     = CVPixelBufferGetBaseAddressOfPlane(dst, 0),
        .height   = CVPixelBufferGetHeightOfPlane(dst, 0),
        .width    = CVPixelBufferGetWidthOfPlane(dst, 0),
        .rowBytes = CVPixelBufferGetBytesPerRowOfPlane(dst, 0),
    };

    if (!scale_tmp_) {
        const vImage_Error need = vImageScale_Planar8(
            &vi_src, &vi_y, nullptr, kvImageGetTempBufferSize);
        if (need > 0) {
            scale_tmp_size_ = static_cast<size_t>(need);
            scale_tmp_      = std::malloc(scale_tmp_size_);
        }
    }

    const vImage_Error err = vImageScale_Planar8(
        &vi_src, &vi_y, scale_tmp_, kvImageNoFlags);

    // UV is uniformly 128 for chroma-neutral grayscale. Pool buffers retain
    // their bytes across checkout/return, so memset only on first sight per
    // buffer — saves ~600 KB/frame at 1280x960.
    static const CFStringRef kUvInitKey = CFSTR("gw.UvInit");
    CFTypeRef marker = CVBufferCopyAttachment(dst, kUvInitKey, nullptr);
    if (!marker) {
        auto*  uv     = static_cast<uint8_t*>(CVPixelBufferGetBaseAddressOfPlane(dst, 1));
        size_t uv_h   = CVPixelBufferGetHeightOfPlane(dst, 1);
        size_t uv_row = CVPixelBufferGetBytesPerRowOfPlane(dst, 1);
        std::memset(uv, 128, uv_h * uv_row);
        CVBufferSetAttachment(dst, kUvInitKey, kCFBooleanTrue,
                              kCVAttachmentMode_ShouldPropagate);
    } else {
        CFRelease(marker);
    }

    CVPixelBufferUnlockBaseAddress(dst, 0);
    CVPixelBufferUnlockBaseAddress(src, kCVPixelBufferLock_ReadOnly);

    if (err != kvImageNoError) {
        CFRelease(dst);
        return nullptr;
    }
    return dst;
}

}  // namespace gw::encoder
