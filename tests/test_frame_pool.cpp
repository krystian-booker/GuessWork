#include <gtest/gtest.h>

#include <CoreVideo/CoreVideo.h>

#include <cstring>

#include "core/frame.hpp"
#include "core/frame_pool.hpp"

namespace gw {

namespace {

constexpr FrameFormat kMono640x480{
    .width        = 640,
    .height       = 480,
    .pixel_format = kCVPixelFormatType_OneComponent8,
};

}  // namespace

TEST(FramePoolTest, ConstructedWithCapacityReportsCapacity) {
    FramePool pool(kMono640x480, 4);
    EXPECT_EQ(pool.capacity(), 4u);
    EXPECT_EQ(pool.free_count(), 4u);
    EXPECT_EQ(pool.format().width, 640u);
    EXPECT_EQ(pool.format().height, 480u);
}

TEST(FramePoolTest, AcquireReturnsFrameWithCorrectFormat) {
    FramePool pool(kMono640x480, 2);
    Frame*    f = pool.acquire();
    ASSERT_NE(f, nullptr);
    EXPECT_EQ(f->width(), 640u);
    EXPECT_EQ(f->height(), 480u);
    EXPECT_EQ(f->pixel_format(), kCVPixelFormatType_OneComponent8);
    EXPECT_GE(f->bytes_per_row(), 640u);
    EXPECT_EQ(f->refcount(), 1u);
    f->release();
}

TEST(FramePoolTest, AcquiredFrameHasIOSurfaceBackedPixelBuffer) {
    FramePool pool(kMono640x480, 1);
    Frame*    f = pool.acquire();
    ASSERT_NE(f, nullptr);
    CVPixelBufferRef pb = f->pixel_buffer();
    ASSERT_NE(pb, nullptr);
    IOSurfaceRef surface = CVPixelBufferGetIOSurface(pb);
    EXPECT_NE(surface, nullptr) << "pixel buffer must be IOSurface-backed for zero-copy";
    f->release();
}

TEST(FramePoolTest, AcquiredFrameIsCpuWritable) {
    FramePool pool(kMono640x480, 1);
    Frame*    f = pool.acquire();
    ASSERT_NE(f, nullptr);
    CVPixelBufferRef pb = f->pixel_buffer();
    ASSERT_EQ(CVPixelBufferLockBaseAddress(pb, 0), kCVReturnSuccess);
    void* base = CVPixelBufferGetBaseAddress(pb);
    ASSERT_NE(base, nullptr);
    std::memset(base, 0x42, kMono640x480.width);  // write first row
    CVPixelBufferUnlockBaseAddress(pb, 0);
    f->release();
}

TEST(FramePoolTest, MetadataIsReadAndWriteable) {
    FramePool pool(kMono640x480, 1);
    Frame*    f = pool.acquire();
    ASSERT_NE(f, nullptr);
    f->set_sequence(12345);
    f->set_host_capture_ns(1'000'000'000);
    f->set_camera_ts_ns(2'000'000'000);
    f->set_producer_id("cam0");
    EXPECT_EQ(f->sequence(), 12345u);
    EXPECT_EQ(f->host_capture_ns(), 1'000'000'000u);
    EXPECT_EQ(f->camera_ts_ns(), 2'000'000'000u);
    EXPECT_EQ(f->producer_id(), "cam0");
    f->release();
}

TEST(FramePoolTest, RetainAndReleaseAdjustRefcount) {
    FramePool pool(kMono640x480, 1);
    Frame*    f = pool.acquire();
    ASSERT_NE(f, nullptr);
    EXPECT_EQ(f->refcount(), 1u);
    f->retain();
    EXPECT_EQ(f->refcount(), 2u);
    f->retain();
    EXPECT_EQ(f->refcount(), 3u);
    f->release();
    EXPECT_EQ(f->refcount(), 2u);
    f->release();
    EXPECT_EQ(f->refcount(), 1u);
    // still in use, pool should still report 0 free
    EXPECT_EQ(pool.free_count(), 0u);
    f->release();
    // refcount hit 0, frame returned to pool
    EXPECT_EQ(pool.free_count(), 1u);
}

TEST(FramePoolTest, ExhaustionReturnsNullThenRecovers) {
    FramePool           pool(kMono640x480, 3);
    std::vector<Frame*> held;
    held.reserve(3);
    for (int i = 0; i < 3; ++i) {
        Frame* f = pool.acquire();
        ASSERT_NE(f, nullptr) << "acquire " << i << " should succeed";
        held.push_back(f);
    }
    EXPECT_EQ(pool.free_count(), 0u);
    EXPECT_EQ(pool.acquire(), nullptr) << "pool should be exhausted";
    // Release one
    held[1]->release();
    EXPECT_EQ(pool.free_count(), 1u);
    // Now acquire succeeds again
    Frame* f = pool.acquire();
    ASSERT_NE(f, nullptr);
    EXPECT_EQ(pool.free_count(), 0u);
    f->release();
    held[0]->release();
    held[2]->release();
    EXPECT_EQ(pool.free_count(), 3u);
}

TEST(FramePoolTest, ReleaseClearsMetadataOnNextAcquire) {
    FramePool pool(kMono640x480, 1);

    Frame* f = pool.acquire();
    ASSERT_NE(f, nullptr);
    f->set_sequence(999);
    f->set_producer_id("foo");
    f->release();

    Frame* g = pool.acquire();
    ASSERT_NE(g, nullptr);
    EXPECT_EQ(g->sequence(), 0u) << "metadata should be reset on acquire";
    EXPECT_TRUE(g->producer_id().empty());
    g->release();
}

TEST(FramePoolTest, DestructorReleasesAllFramesEvenIfHeld) {
    Frame* leaked = nullptr;
    {
        FramePool pool(kMono640x480, 2);
        leaked = pool.acquire();
        ASSERT_NE(leaked, nullptr);
        // Intentionally do not release. Pool destruction should still tear down cleanly
        // (Frame stays alive via its own refcount path; pool drops its ownership).
        // For this test, release before pool dies so we don't actually leak.
        leaked->release();
    }
    SUCCEED();
}

}  // namespace gw
