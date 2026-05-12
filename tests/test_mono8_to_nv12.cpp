#include "encoder/mono8_to_nv12.hpp"
#include "test_helpers.hpp"

#include <CoreVideo/CoreVideo.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>

using gw::test::make_mono8;

TEST(Mono8ToNv12, ProducesNv12OutputAtRequestedSize) {
    CVPixelBufferRef src = make_mono8(64, 48);
    ASSERT_NE(src, nullptr);

    gw::encoder::Mono8ToNv12 conv(32, 24);
    CVPixelBufferRef out = conv.convert(src);
    ASSERT_NE(out, nullptr);

    EXPECT_EQ(CVPixelBufferGetWidth(out),  32u);
    EXPECT_EQ(CVPixelBufferGetHeight(out), 24u);
    EXPECT_EQ(CVPixelBufferGetPlaneCount(out), 2u);
    EXPECT_EQ(CVPixelBufferGetPixelFormatType(out),
              kCVPixelFormatType_420YpCbCr8BiPlanarFullRange);

    CFRelease(out);
    CVPixelBufferRelease(src);
}

TEST(Mono8ToNv12, UvPlaneIsUniform128ForGrayscale) {
    CVPixelBufferRef src = make_mono8(32, 32);
    ASSERT_NE(src, nullptr);

    gw::encoder::Mono8ToNv12 conv(16, 16);
    CVPixelBufferRef out = conv.convert(src);
    ASSERT_NE(out, nullptr);

    CVPixelBufferLockBaseAddress(out, kCVPixelBufferLock_ReadOnly);
    auto*  uv     = static_cast<uint8_t*>(CVPixelBufferGetBaseAddressOfPlane(out, 1));
    size_t stride = CVPixelBufferGetBytesPerRowOfPlane(out, 1);
    size_t rows   = CVPixelBufferGetHeightOfPlane(out, 1);
    size_t cols   = CVPixelBufferGetWidthOfPlane(out, 1) * 2;  // 2 bytes per chroma sample

    // UV plane should be exactly 128 in every byte (neutral chroma).
    for (size_t r = 0; r < rows; ++r) {
        for (size_t c = 0; c < cols; ++c) {
            ASSERT_EQ(uv[r * stride + c], 128u)
                << "uv mismatch at row " << r << " col " << c;
        }
    }
    CVPixelBufferUnlockBaseAddress(out, kCVPixelBufferLock_ReadOnly);

    CFRelease(out);
    CVPixelBufferRelease(src);
}

TEST(Mono8ToNv12, YPlaneSamplesMatchAtIdentityScale) {
    // When out size == in size, the Y plane should be approximately the source
    // bytes (vImage with no resampling is effectively a copy).
    CVPixelBufferRef src = make_mono8(16, 16);
    ASSERT_NE(src, nullptr);

    gw::encoder::Mono8ToNv12 conv(16, 16);
    CVPixelBufferRef out = conv.convert(src);
    ASSERT_NE(out, nullptr);

    CVPixelBufferLockBaseAddress(src, kCVPixelBufferLock_ReadOnly);
    CVPixelBufferLockBaseAddress(out, kCVPixelBufferLock_ReadOnly);

    auto*  src_base   = static_cast<const uint8_t*>(CVPixelBufferGetBaseAddress(src));
    size_t src_stride = CVPixelBufferGetBytesPerRow(src);
    auto*  y_base     = static_cast<const uint8_t*>(CVPixelBufferGetBaseAddressOfPlane(out, 0));
    size_t y_stride   = CVPixelBufferGetBytesPerRowOfPlane(out, 0);

    for (uint32_t y = 0; y < 16; ++y) {
        for (uint32_t x = 0; x < 16; ++x) {
            EXPECT_EQ(y_base[y * y_stride + x], src_base[y * src_stride + x])
                << "y mismatch at (" << x << "," << y << ")";
        }
    }

    CVPixelBufferUnlockBaseAddress(out, kCVPixelBufferLock_ReadOnly);
    CVPixelBufferUnlockBaseAddress(src, kCVPixelBufferLock_ReadOnly);

    CFRelease(out);
    CVPixelBufferRelease(src);
}
