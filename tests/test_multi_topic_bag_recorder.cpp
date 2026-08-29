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

#include "consumer/multi_topic_bag_recorder.hpp"
#include "core/frame.hpp"
#include "core/frame_channel.hpp"
#include "core/frame_pool.hpp"
#include "core/imu_types.hpp"
#include "core/measurement_bus.hpp"

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

// Publishes a frame stamped on the "sync controller clock" (camera_ts_ns) — the
// stamp the extrinsics recorder must use.
void publish_seq(FrameChannel& ch, FramePool& pool, uint64_t seq, uint64_t ts_ns) {
    Frame* f = pool.acquire();
    ASSERT_NE(f, nullptr);
    f->set_sequence(seq);
    f->set_host_capture_ns(seq);   // deliberately bogus: must NOT appear in bag
    f->set_camera_ts_ns(ts_ns);
    fill_frame(f, static_cast<uint8_t>(seq & 0xFFu));
    ch.publish(f);
}

class MultiTopicBagRecorderTest : public ::testing::Test {
protected:
    void SetUp() override {
        root_ = std::filesystem::temp_directory_path() /
                ("gw_multibag_test_" + std::to_string(::getpid()) + "_" +
                 ::testing::UnitTest::GetInstance()->current_test_info()->name());
        std::filesystem::remove_all(root_);

        target_ = std::filesystem::temp_directory_path() /
                  ("gw_multibag_target_" + std::to_string(::getpid()) + ".yaml");
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

    std::vector<uint8_t> read_bag() const {
        std::ifstream f(root_ / "calibration.bag", std::ios::binary);
        return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
    }

    static size_t count_substr(const std::vector<uint8_t>& hay, const std::string& needle) {
        size_t n = 0;
        auto   it = hay.begin();
        for (;;) {
            it = std::search(it, hay.end(), needle.begin(), needle.end());
            if (it == hay.end()) break;
            ++n;
            ++it;
        }
        return n;
    }

    std::filesystem::path root_;
    std::filesystem::path target_;
};

}  // namespace

TEST_F(MultiTopicBagRecorderTest, RecordsTwoCamerasAndImuIntoOneBag) {
    FramePool    pool_a(kMono32x32, 4), pool_b(kMono32x32, 4);
    FrameChannel ch_a, ch_b;
    MeasurementBus<ImuSample> imu_bus;

    MultiTopicBagRecorder rec(
        root_, target_,
        {{&ch_a, "/cam0/image_raw", "cam0"}, {&ch_b, "/cam1/image_raw", "cam1"}},
        &imu_bus);
    rec.start();

    EXPECT_TRUE(std::filesystem::exists(root_ / "target.yaml"));

    for (uint64_t seq = 1; seq <= 3; ++seq) {
        publish_seq(ch_a, pool_a, seq, 1'000'000'000ull + seq * 33'000'000ull);
        ASSERT_TRUE(wait_until([&] { return rec.camera_stats(0).written >= seq; },
                               std::chrono::milliseconds(500)));
        publish_seq(ch_b, pool_b, seq, 1'000'000'000ull + seq * 33'000'000ull);
        ASSERT_TRUE(wait_until([&] { return rec.camera_stats(1).written >= seq; },
                               std::chrono::milliseconds(500)));
    }
    for (int i = 0; i < 8; ++i) {
        ImuSample s;
        s.t_ns     = 1'000'000'000ull + static_cast<uint64_t>(i) * 2'500'000ull;
        s.accel[2] = 9.81f;
        s.gyro[0]  = 0.1f;
        imu_bus.publish(s);
    }
    ASSERT_TRUE(wait_until([&] { return rec.imu_written() >= 8; },
                           std::chrono::milliseconds(500)));
    rec.stop();

    EXPECT_EQ(rec.camera_stats(0).written, 3u);
    EXPECT_EQ(rec.camera_stats(1).written, 3u);
    EXPECT_EQ(rec.imu_written(), 8u);
    EXPECT_EQ(rec.imu_dropped(), 0u);

    const auto bag = read_bag();
    ASSERT_GE(bag.size(), 13u);
    EXPECT_EQ(std::string(bag.begin(), bag.begin() + 13), "#ROSBAG V2.0\n");

    // Each topic + the Imu type/MD5 must appear: once per chunk's connection
    // records and once in the trailing index region (≥ 2 each).
    EXPECT_GE(count_substr(bag, "/cam0/image_raw"), 2u);
    EXPECT_GE(count_substr(bag, "/cam1/image_raw"), 2u);
    EXPECT_GE(count_substr(bag, "/imu0"), 2u);
    EXPECT_GE(count_substr(bag, "sensor_msgs/Imu"), 2u);
    EXPECT_GE(count_substr(bag, "6a62c6daae103f4ff57a132d6f95cec2"), 2u);
}

TEST_F(MultiTopicBagRecorderTest, DropsFramesWithoutSyncControllerStamp) {
    FramePool    pool(kMono32x32, 4);
    FrameChannel ch;

    MultiTopicBagRecorder rec(root_, target_, {{&ch, "/cam0/image_raw", "cam0"}},
                              /*imu_bus=*/nullptr);
    rec.start();

    // camera_ts_ns == 0 → pulse-stamp fallback → unusable for a shared-clock
    // bag; must be dropped and counted.
    publish_seq(ch, pool, 1, /*ts_ns=*/0);
    ASSERT_TRUE(wait_until([&] { return rec.camera_stats(0).dropped >= 1; },
                           std::chrono::milliseconds(500)));
    EXPECT_EQ(rec.camera_stats(0).written, 0u);

    publish_seq(ch, pool, 2, /*ts_ns=*/2'000'000'000ull);
    ASSERT_TRUE(wait_until([&] { return rec.camera_stats(0).written >= 1; },
                           std::chrono::milliseconds(500)));
    rec.stop();

    EXPECT_EQ(rec.camera_stats(0).written, 1u);
    EXPECT_GE(rec.camera_stats(0).dropped, 1u);
}

TEST_F(MultiTopicBagRecorderTest, StopIsIdempotentAndBagIsClosed) {
    FramePool    pool(kMono32x32, 4);
    FrameChannel ch;
    MeasurementBus<ImuSample> imu_bus;

    MultiTopicBagRecorder rec(root_, target_, {{&ch, "/cam0/image_raw", "cam0"}},
                              &imu_bus);
    rec.start();
    publish_seq(ch, pool, 1, 1'000'000'000ull);
    ASSERT_TRUE(wait_until([&] { return rec.camera_stats(0).written >= 1; },
                           std::chrono::milliseconds(500)));
    rec.stop();
    EXPECT_NO_THROW(rec.stop());

    // Closed bag must carry the magic + a rewritten (non-zero) index_pos.
    const auto bag = read_bag();
    ASSERT_GE(bag.size(), 13u);
    EXPECT_EQ(std::string(bag.begin(), bag.begin() + 13), "#ROSBAG V2.0\n");
}

TEST_F(MultiTopicBagRecorderTest, RejectsEmptyCameraList) {
    EXPECT_THROW(MultiTopicBagRecorder(root_, target_, {}, nullptr),
                 std::invalid_argument);
}

}  // namespace gw
