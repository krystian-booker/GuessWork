#include <gtest/gtest.h>

#include <CoreVideo/CoreVideo.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

#include "consumer/rosbag_recording_consumer.hpp"
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
    f->set_host_capture_ns(seq * 1'000'000);
    fill_frame(f, static_cast<uint8_t>(seq & 0xFFu));
    ch.publish(f);
}

class RosbagRecordingConsumerTest : public ::testing::Test {
protected:
    void SetUp() override {
        root_ = std::filesystem::temp_directory_path() /
                ("gw_rosbag_consumer_test_" + std::to_string(::getpid()) + "_" +
                 ::testing::UnitTest::GetInstance()->current_test_info()->name());
        std::filesystem::remove_all(root_);

        target_ = std::filesystem::temp_directory_path() /
                  ("gw_rosbag_consumer_target_" + std::to_string(::getpid()) + ".yaml");
        std::ofstream(target_) << "target_type: 'aprilgrid'\n";
    }
    void TearDown() override {
        std::filesystem::remove_all(root_);
        std::filesystem::remove(target_);
    }

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
    std::filesystem::path target_;
};

}  // namespace

TEST_F(RosbagRecordingConsumerTest, AttachWritesBagAndTargetYaml) {
    RosbagRecordingConsumer rec(root_, target_);
    FramePool               pool(kMono32x32, 4);
    FrameChannel            ch;
    rec.attach(ch);

    EXPECT_TRUE(std::filesystem::exists(root_ / "calibration.bag"));
    EXPECT_TRUE(std::filesystem::exists(root_ / "target.yaml"));

    rec.detach();
}

TEST_F(RosbagRecordingConsumerTest, WritesEachPublishedFrame) {
    RosbagRecordingConsumer rec(root_, target_);
    FramePool               pool(kMono32x32, 4);
    FrameChannel            ch;
    rec.attach(ch);

    for (uint64_t seq = 1; seq <= 3; ++seq) {
        publish_seq(ch, pool, seq);
        ASSERT_TRUE(wait_until(
            [&] { return rec.frames_written() >= seq; },
            std::chrono::milliseconds(500)));
    }
    rec.detach();

    EXPECT_EQ(rec.frames_written(), 3u);

    // Bag should be a valid ROS1 v2.0 file (magic header present).
    std::ifstream f(root_ / "calibration.bag", std::ios::binary);
    ASSERT_TRUE(f.is_open());
    std::string magic(13, '\0');
    f.read(magic.data(), 13);
    EXPECT_EQ(magic, "#ROSBAG V2.0\n");
}

TEST_F(RosbagRecordingConsumerTest, DetachIsIdempotent) {
    RosbagRecordingConsumer rec(root_, target_);
    FrameChannel            ch;
    rec.attach(ch);
    rec.detach();
    EXPECT_NO_THROW(rec.detach());
}

}  // namespace gw
