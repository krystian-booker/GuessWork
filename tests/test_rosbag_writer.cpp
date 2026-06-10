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

namespace {

size_t count_substr(const std::vector<uint8_t>& hay, const std::string& needle) {
    size_t n  = 0;
    auto   it = hay.begin();
    for (;;) {
        it = std::search(it, hay.end(), needle.begin(), needle.end());
        if (it == hay.end()) break;
        ++n;
        ++it;
    }
    return n;
}

}  // namespace

TEST_F(RosbagWriterTest, MultiConnectionBagCarriesAllTopics) {
    RosbagWriter w(path_, "/cam0/image_raw", "cam0");
    const uint32_t cam1 = w.add_image_connection("/cam1/image_raw", "cam1");
    const uint32_t imu  = w.add_imu_connection("/imu0", "imu0");
    EXPECT_EQ(cam1, 1u);
    EXPECT_EQ(imu, 2u);
    w.open();

    const uint32_t W = 16, H = 8;
    std::vector<uint8_t> pixels(W * H, 0x42);
    w.add_mono8_image(0, 1'000'000'000ull, W, H, pixels.data());
    w.add_mono8_image(cam1, 1'000'000'000ull, W, H, pixels.data());
    const float accel[3] = {0.0f, 0.0f, 9.81f};
    const float gyro[3]  = {0.1f, -0.2f, 0.3f};
    w.add_imu_sample(imu, 1'000'500'000ull, accel, gyro);
    w.add_imu_sample(imu, 1'003'000'000ull, accel, gyro);
    w.close();

    EXPECT_EQ(w.messages_written(), 4u);

    const auto bytes = read_all();
    // Each Connection record carries the topic twice (record header field +
    // data block field), and each connection is written twice (chunk head +
    // trailing index region) → exactly 4 occurrences with a single chunk.
    EXPECT_EQ(count_substr(bytes, "/cam0/image_raw"), 4u);
    EXPECT_EQ(count_substr(bytes, "/cam1/image_raw"), 4u);
    EXPECT_EQ(count_substr(bytes, "/imu0"), 4u);
    EXPECT_EQ(count_substr(bytes, "060021388200f6f0f447d0fcd9c64743"), 4u);  // 2 image conns × 2
    EXPECT_EQ(count_substr(bytes, "6a62c6daae103f4ff57a132d6f95cec2"), 2u);
    // Per-connection IndexData records: "conn=" appears in connection records
    // (2 per conn) and IndexData records (1 per conn per chunk) and message
    // records (1 per message) — just assert the Imu definition's embedded
    // types made it through (genpy needs them to build a deserializer).
    EXPECT_GE(count_substr(bytes, "MSG: geometry_msgs/Quaternion"), 2u);
    EXPECT_GE(count_substr(bytes, "MSG: geometry_msgs/Vector3"), 2u);

    // BagHeader conn_count must be 3. The field is "conn_count=" followed by
    // a u32 LE — locate the first occurrence (the BagHeader is the only
    // record with that field).
    const std::string cc = "conn_count=";
    auto it = std::search(bytes.begin(), bytes.end(), cc.begin(), cc.end());
    ASSERT_NE(it, bytes.end());
    it += static_cast<std::ptrdiff_t>(cc.size());
    uint32_t conn_count = 0;
    std::memcpy(&conn_count, &*it, 4);
    EXPECT_EQ(conn_count, 3u);
}

TEST_F(RosbagWriterTest, ImuMessageEncodesNoOrientationConvention) {
    RosbagWriter w(path_, "/cam0/image_raw", "cam0");
    const uint32_t imu = w.add_imu_connection("/imu0", "imu0");
    w.open();
    const float accel[3] = {1.5f, -2.5f, 9.81f};
    const float gyro[3]  = {0.25f, 0.5f, -0.75f};
    w.add_imu_sample(imu, 7'000'000'123ull, accel, gyro);
    w.close();

    const auto bytes = read_all();

    // orientation_covariance[0] = -1.0 marks "no orientation estimate". The
    // f64 -1.0 (bytes 00..F0 BF LE) directly follows the identity quaternion
    // (0,0,0,1) — search for quaternion-w 1.0 followed by -1.0.
    const uint8_t quat_w_then_cov[] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F,   // 1.0
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xBF};  // -1.0
    auto it = std::search(bytes.begin(), bytes.end(),
                          std::begin(quat_w_then_cov), std::end(quat_w_then_cov));
    EXPECT_NE(it, bytes.end()) << "missing quaternion-w=1.0 followed by cov[0]=-1.0";

    // Gyro x as f64.
    const double gx = 0.25;
    uint8_t gx_bytes[8];
    std::memcpy(gx_bytes, &gx, 8);
    EXPECT_NE(std::search(bytes.begin(), bytes.end(),
                          std::begin(gx_bytes), std::end(gx_bytes)),
              bytes.end());

    // Accel z as f64 (9.81f widened to double).
    const double az = static_cast<double>(9.81f);
    uint8_t az_bytes[8];
    std::memcpy(az_bytes, &az, 8);
    EXPECT_NE(std::search(bytes.begin(), bytes.end(),
                          std::begin(az_bytes), std::end(az_bytes)),
              bytes.end());
}

TEST_F(RosbagWriterTest, ConnectionRegistrationAfterOpenThrows) {
    RosbagWriter w(path_);
    w.open();
    EXPECT_THROW(w.add_image_connection("/cam1/image_raw", "cam1"),
                 std::runtime_error);
    EXPECT_THROW(w.add_imu_connection("/imu0", "imu0"), std::runtime_error);
    w.close();
}

TEST_F(RosbagWriterTest, MessageTypeMustMatchConnection) {
    RosbagWriter w(path_, "/cam0/image_raw", "cam0");
    const uint32_t imu = w.add_imu_connection("/imu0", "imu0");
    w.open();
    const float    zeros[3] = {0, 0, 0};
    const uint32_t W = 4, H = 4;
    std::vector<uint8_t> pixels(W * H);
    EXPECT_THROW(w.add_mono8_image(imu, 1, W, H, pixels.data()), std::runtime_error);
    EXPECT_THROW(w.add_imu_sample(0, 1, zeros, zeros), std::runtime_error);
    EXPECT_THROW(w.add_imu_sample(99, 1, zeros, zeros), std::runtime_error);
    w.close();
    EXPECT_EQ(w.messages_written(), 0u);
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

// Multi-topic sibling of the fixture above: 2 image topics + /imu0, for
// `rosbag info /data/gw_validate_multi.bag` inside the Kalibr container.
TEST(RosbagWriterValidationFixture, EmitsMultiTopicBagAtStablePath) {
    const auto p = std::filesystem::temp_directory_path() / "gw_validate_multi.bag";
    std::filesystem::remove(p);
    RosbagWriter w(p, "/cam0/image_raw", "cam0");
    const uint32_t cam1 = w.add_image_connection("/cam1/image_raw", "cam1");
    const uint32_t imu  = w.add_imu_connection("/imu0", "imu0");
    w.open();
    const uint32_t W = 64, H = 48;
    std::vector<uint8_t> pixels(W * H);
    for (int i = 0; i < 5; ++i) {
        const uint64_t ts = (static_cast<uint64_t>(i) + 1) * 100'000'000ull;
        std::fill(pixels.begin(), pixels.end(), static_cast<uint8_t>(i * 40));
        w.add_mono8_image(0, ts, W, H, pixels.data());
        w.add_mono8_image(cam1, ts, W, H, pixels.data());
        for (int k = 0; k < 40; ++k) {
            const float accel[3] = {0.0f, 0.0f, 9.81f};
            const float gyro[3]  = {0.01f * static_cast<float>(k), 0.0f, 0.0f};
            w.add_imu_sample(imu, ts + static_cast<uint64_t>(k) * 2'500'000ull,
                             accel, gyro);
        }
    }
    w.close();
    std::cerr << "[validation] wrote multi-topic bag to: " << p
              << " (" << std::filesystem::file_size(p) << " bytes)\n";
}

}  // namespace gw
