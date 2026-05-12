#include "encoder/h264_encoder.hpp"
#include "encoder/mono8_to_nv12.hpp"
#include "test_helpers.hpp"

#include <CoreVideo/CoreVideo.h>
#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <vector>

TEST(H264Encoder, ProducesValidAnnexBStreamWithKeyframes) {
    using namespace gw::encoder;

    constexpr uint32_t W   = 320;
    constexpr uint32_t H   = 240;
    constexpr uint32_t FPS = 30;
    constexpr uint32_t BR  = 1'000'000;

    std::mutex            mu;
    std::vector<uint8_t>  all_bytes;        // accumulated stream (for optional dump)
    std::atomic<int>      frame_count{0};
    std::atomic<int>      keyframe_count{0};

    {
        H264Encoder enc(W, H, FPS, BR,
            [&](std::vector<uint8_t> annex_b, bool keyframe, uint64_t /*pts*/) {
                ASSERT_GE(annex_b.size(), 4u);
                EXPECT_EQ(annex_b[0], 0x00);
                EXPECT_EQ(annex_b[1], 0x00);
                EXPECT_EQ(annex_b[2], 0x00);
                EXPECT_EQ(annex_b[3], 0x01);

                if (keyframe) keyframe_count.fetch_add(1);
                frame_count.fetch_add(1);

                std::lock_guard lk(mu);
                all_bytes.insert(all_bytes.end(), annex_b.begin(), annex_b.end());
            });

        Mono8ToNv12 conv(W, H);

        for (int i = 0; i < 30; ++i) {
            CVPixelBufferRef src  = gw::test::make_mono8(W, H, static_cast<uint32_t>(i));
            CVPixelBufferRef nv12 = conv.convert(src);
            ASSERT_NE(nv12, nullptr);
            enc.encode(nv12, static_cast<uint64_t>(i) * (1'000'000 / FPS));
            CFRelease(nv12);
            CFRelease(src);
        }
    }  // ~H264Encoder calls VTCompressionSessionCompleteFrames; all callbacks have fired.

    EXPECT_GE(frame_count.load(),    25);
    EXPECT_GE(keyframe_count.load(), 1);

    // Optional dump: set GW_H264_DUMP=/path/to/file.h264 to inspect with ffprobe.
    if (const char* path = std::getenv("GW_H264_DUMP")) {
        if (FILE* f = std::fopen(path, "wb")) {
            std::lock_guard lk(mu);
            std::fwrite(all_bytes.data(), 1, all_bytes.size(), f);
            std::fclose(f);
            std::fprintf(stderr, "H264Encoder: wrote %zu bytes to %s\n",
                         all_bytes.size(), path);
        }
    }
}
