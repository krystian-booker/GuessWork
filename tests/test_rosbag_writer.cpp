#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "consumer/rosbag_writer.hpp"

namespace gw {

namespace {

class RosbagWriterTest : public ::testing::Test {
protected:
    void SetUp() override {
        path_ = std::filesystem::temp_directory_path() /
                ("gw_rosbag_test_" + std::to_string(::getpid()) + "_" +
                 ::testing::UnitTest::GetInstance()->current_test_info()->name() +
                 ".bag");
        std::filesystem::remove(path_);
    }
    void TearDown() override {
        std::filesystem::remove(path_);
    }

    std::vector<uint8_t> read_all() const {
        std::ifstream f(path_, std::ios::binary);
        return {std::istreambuf_iterator<char>(f),
                std::istreambuf_iterator<char>()};
    }

    std::filesystem::path path_;
};

}  // namespace

TEST_F(RosbagWriterTest, EmitsRos1MagicHeader) {
    RosbagWriter w(path_);
    w.open();
    w.close();

    const auto bytes = read_all();
    ASSERT_GE(bytes.size(), 13u);
    const std::string magic(bytes.begin(), bytes.begin() + 13);
    EXPECT_EQ(magic, "#ROSBAG V2.0\n");
}

TEST_F(RosbagWriterTest, WritesEachAddedImage) {
    RosbagWriter w(path_, "/cam0/image_raw", "cam0");
    w.open();

    const uint32_t W = 16;
    const uint32_t H = 8;
    std::vector<uint8_t> pixels(W * H);
    for (int i = 0; i < 3; ++i) {
        std::fill(pixels.begin(), pixels.end(), static_cast<uint8_t>(i + 1));
        w.add_mono8_image(/*ts_ns=*/(i + 1) * 1'000'000'000ull, W, H, pixels.data());
    }
    w.close();

    EXPECT_EQ(w.messages_written(), 3u);

    const auto bytes = read_all();
    // Sanity: the bag must include the canonical sensor_msgs/Image MD5 (once
    // per connection record; we emit two of them — one in the chunk, one
    // trailing).
    const std::string needle = "060021388200f6f0f447d0fcd9c64743";
    auto it = std::search(bytes.begin(), bytes.end(),
                          needle.begin(), needle.end());
    EXPECT_NE(it, bytes.end()) << "missing sensor_msgs/Image MD5 in bag";

    // And the "mono8" encoding string appears at least once per message.
    const std::string mono = "mono8";
    size_t mono_count = 0;
    for (auto p = bytes.begin();
         (p = std::search(p, bytes.end(), mono.begin(), mono.end())) != bytes.end();
         ++p) {
        ++mono_count;
    }
    EXPECT_GE(mono_count, 3u);
}

TEST_F(RosbagWriterTest, DestructorClosesOpenFile) {
    {
        RosbagWriter w(path_);
        w.open();
        // intentionally don't call close()
    }
    const auto bytes = read_all();
    ASSERT_GE(bytes.size(), 13u);
    EXPECT_EQ(std::string(bytes.begin(), bytes.begin() + 13), "#ROSBAG V2.0\n");
}

TEST_F(RosbagWriterTest, OpenIsIdempotent) {
    RosbagWriter w(path_);
    w.open();
    w.open();  // no-op
    w.add_mono8_image(1'000'000'000ull, 4, 4, std::vector<uint8_t>(16).data());
    w.close();
    w.close();  // no-op

    EXPECT_EQ(w.messages_written(), 1u);
}

// Emit a bag at a deterministic path with NO cleanup, so the writer can be
// validated against an out-of-process ROS toolchain (e.g.
// `docker run … guesswork/kalibr:latest rosbag info /data/gw_validate.bag`).
// The unit suite alone can't catch every format-spec subtlety; this test
// makes it cheap to re-validate when the format-emitting code changes.
TEST(RosbagWriterValidationFixture, EmitsBagAtStablePath) {
    const auto p = std::filesystem::temp_directory_path() / "gw_validate.bag";
    std::filesystem::remove(p);
    RosbagWriter w(p, "/cam0/image_raw", "cam0");
    w.open();
    const uint32_t W = 64, H = 48;
    std::vector<uint8_t> pixels(W * H);
    for (int i = 0; i < 5; ++i) {
        std::fill(pixels.begin(), pixels.end(), static_cast<uint8_t>(i * 40));
        w.add_mono8_image((static_cast<uint64_t>(i) + 1) * 100'000'000ull, W, H,
                          pixels.data());
    }
    w.close();
    std::cerr << "[validation] wrote bag to: " << p
              << " (" << std::filesystem::file_size(p) << " bytes)\n";
}

}  // namespace gw
