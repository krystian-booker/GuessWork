#include <gtest/gtest.h>

#include <CoreVideo/CoreVideo.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "consumer/recording_consumer.hpp"
#include "core/frame.hpp"
#include "core/frame_channel.hpp"
#include "core/frame_pool.hpp"

namespace gw {

namespace {

constexpr FrameFormat kMono32x32{
    .width        = 32,
    .height       = 32,
    .pixel_format = kCVPixelFormatType_OneComponent8,
};

// Fill every pixel of a Mono8 frame with a marker byte, so a corrupt PNG
// produces a clearly-wrong file size.
void fill_frame(Frame* f, uint8_t marker) {
    CVPixelBufferRef pb = f->pixel_buffer();
    CVPixelBufferLockBaseAddress(pb, 0);
    auto*        base   = static_cast<uint8_t*>(CVPixelBufferGetBaseAddress(pb));
    const size_t stride = CVPixelBufferGetBytesPerRow(pb);
    const size_t h      = CVPixelBufferGetHeight(pb);
    const size_t w      = CVPixelBufferGetWidth(pb);
    for (size_t y = 0; y < h; ++y) {
        std::fill_n(base + y * stride, w, marker);
    }
    CVPixelBufferUnlockBaseAddress(pb, 0);
}

void publish_seq(FrameChannel& ch, FramePool& pool, uint64_t seq) {
    Frame* f = pool.acquire();
    ASSERT_NE(f, nullptr);
    f->set_sequence(seq);
    f->set_host_capture_ns(seq * 1'000'000);  // 1 ms apart
    fill_frame(f, static_cast<uint8_t>(seq & 0xFFu));
    ch.publish(f);
}

class RecordingConsumerTest : public ::testing::Test {
protected:
    void SetUp() override {
        root_ = std::filesystem::temp_directory_path() /
                ("gw_record_test_" + std::to_string(::getpid()) + "_" +
                 ::testing::UnitTest::GetInstance()->current_test_info()->name());
        std::filesystem::remove_all(root_);
    }
    void TearDown() override {
        std::filesystem::remove_all(root_);
    }

    // Spin until `predicate()` returns true or `timeout` elapses. Returns
    // whether the predicate was satisfied.
    template <typename Pred>
    bool wait_until(Pred pred, std::chrono::milliseconds timeout) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (pred()) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return pred();
    }

    std::filesystem::path root_;
};

}  // namespace

TEST_F(RecordingConsumerTest, AttachCreatesEuRoCLayout) {
    RecordingConsumer rec(root_);
    FramePool         pool(kMono32x32, 4);
    FrameChannel      ch;
    rec.attach(ch);

    EXPECT_TRUE(std::filesystem::exists(root_ / "mav0" / "cam0" / "data"));
    EXPECT_TRUE(std::filesystem::exists(root_ / "mav0" / "cam0" / "data.csv"));

    rec.detach();
}

TEST_F(RecordingConsumerTest, WritesPngPerPublishedFrame) {
    RecordingConsumer rec(root_);
    FramePool         pool(kMono32x32, 4);
    FrameChannel      ch;
    rec.attach(ch);

    // Publish a few frames, one at a time, giving the worker time to drain so
    // FrameChannel's latest-only doesn't subsume them.
    for (uint64_t seq = 1; seq <= 3; ++seq) {
        publish_seq(ch, pool, seq);
        ASSERT_TRUE(wait_until(
            [&] { return rec.frames_written() >= seq; },
            std::chrono::milliseconds(500)));
    }
    rec.detach();

    EXPECT_EQ(rec.frames_written(), 3u);

    const auto data_dir = root_ / "mav0" / "cam0" / "data";
    for (uint64_t seq = 1; seq <= 3; ++seq) {
        const auto p = data_dir / (std::to_string(seq * 1'000'000) + ".png");
        EXPECT_TRUE(std::filesystem::exists(p)) << "missing " << p;
        EXPECT_GT(std::filesystem::file_size(p), 50u)  // a real PNG, not zero bytes
            << "tiny " << p;
    }
}

TEST_F(RecordingConsumerTest, CsvMirrorsWrittenFrames) {
    RecordingConsumer rec(root_);
    FramePool         pool(kMono32x32, 4);
    FrameChannel      ch;
    rec.attach(ch);

    for (uint64_t seq = 1; seq <= 3; ++seq) {
        publish_seq(ch, pool, seq);
        ASSERT_TRUE(wait_until(
            [&] { return rec.frames_written() >= seq; },
            std::chrono::milliseconds(500)));
    }
    rec.detach();

    std::ifstream f(root_ / "mav0" / "cam0" / "data.csv");
    ASSERT_TRUE(f.is_open());
    std::vector<std::string> lines;
    for (std::string line; std::getline(f, line);) lines.push_back(line);
    ASSERT_EQ(lines.size(), 4u);  // header + 3 rows
    EXPECT_EQ(lines[0], "#timestamp [ns],filename");
    EXPECT_EQ(lines[1], "1000000,1000000.png");
    EXPECT_EQ(lines[2], "2000000,2000000.png");
    EXPECT_EQ(lines[3], "3000000,3000000.png");
}

TEST_F(RecordingConsumerTest, DetachIsIdempotent) {
    RecordingConsumer rec(root_);
    FrameChannel      ch;
    rec.attach(ch);
    rec.detach();
    EXPECT_NO_THROW(rec.detach());
}

}  // namespace gw
