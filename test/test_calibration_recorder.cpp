#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "posest/calibration/CalibrationRecorder.h"

namespace {

std::filesystem::path tempDir(const std::string& name) {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    return std::filesystem::temp_directory_path() /
           ("posest_" + name + "_" + std::to_string(stamp));
}

std::string readFile(const std::filesystem::path& path) {
    std::ifstream in(path);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

}  // namespace

TEST(CalibrationRecorder, WritesMatchedFramesImuTriggersAndSessionMetadata) {
    const auto dir = tempDir("recorder");
    posest::calibration::CalibrationRecorder recorder({
        dir,
        {"cam0"},
        {{"cam0", true, 6, 20.0, 500, 0}},
        0.1,
        0,
        1.0,  // require every frame to match a trigger
    });

    const auto anchor = std::chrono::steady_clock::time_point{std::chrono::microseconds(1000)};
    recorder.start();
    recorder.publish(posest::CameraTriggerEvent{
        anchor,
        1000,
        6,
        1,
        0,
    });
    {
        posest::ImuSample s;
        s.timestamp = anchor + std::chrono::microseconds(50);
        s.accel_mps2 = {1.0, 2.0, 3.0};
        s.gyro_radps = {4.0, 5.0, 6.0};
        s.temperature_c = 25.0;
        recorder.publish(s);
    }

    auto frame = std::make_shared<posest::Frame>();
    frame->camera_id = "cam0";
    frame->sequence = 7;
    frame->capture_time = anchor + std::chrono::microseconds(100);
    frame->image = cv::Mat(4, 4, CV_8UC3, cv::Scalar(10, 20, 30));
    recorder.deliver(frame);
    recorder.stop();
    recorder.throwIfUnacceptable();

    const auto stats = recorder.stats();
    EXPECT_EQ(stats.frames_seen, 1u);
    EXPECT_EQ(stats.frames_recorded, 1u);
    EXPECT_EQ(stats.imu_samples_recorded, 1u);
    EXPECT_EQ(stats.trigger_events_recorded, 1u);
    EXPECT_TRUE(std::filesystem::exists(dir / "images" / "cam0_7.png"));
    EXPECT_NE(readFile(dir / "frames.csv").find("cam0,7"), std::string::npos);
    EXPECT_NE(readFile(dir / "session.json").find("\"frames_recorded\": 1"),
              std::string::npos);

    std::filesystem::remove_all(dir);
}

// W4: when --require-imu=no, the daemon constructs the recorder with
// min_trigger_match_fraction=0.0 so frames captured without any matching
// trigger event still survive throwIfUnacceptable. The session.json
// imu_samples_recorded counter is 0 in this path; make_rosbag.py uses that
// signal to skip the IMU topic entirely and keep every recorded frame.
TEST(CalibrationRecorder, AcceptsUnmatchedFramesWhenMinFractionIsZero) {
    const auto dir = tempDir("recorder_no_imu");
    posest::calibration::CalibrationRecorder recorder({
        dir,
        {"cam0"},
        {{"cam0", true, 6, 20.0, 500, 0}},
        0.1,
        0,
        0.0,  // intrinsic-only: any match ratio is acceptable
    });

    const auto anchor = std::chrono::steady_clock::time_point{std::chrono::microseconds(1000)};
    recorder.start();
    auto frame = std::make_shared<posest::Frame>();
    frame->camera_id = "cam0";
    frame->sequence = 11;
    frame->capture_time = anchor + std::chrono::microseconds(100);
    frame->image = cv::Mat(4, 4, CV_8UC3, cv::Scalar(10, 20, 30));
    recorder.deliver(frame);
    recorder.stop();
    EXPECT_NO_THROW(recorder.throwIfUnacceptable());

    const auto stats = recorder.stats();
    EXPECT_EQ(stats.frames_seen, 1u);
    EXPECT_EQ(stats.frames_recorded, 0u);
    EXPECT_EQ(stats.frames_without_trigger, 1u);
    EXPECT_EQ(stats.imu_samples_recorded, 0u);
    EXPECT_TRUE(std::filesystem::exists(dir / "images" / "cam0_11.png"));
    EXPECT_NE(readFile(dir / "session.json").find("\"imu_samples_recorded\": 0"),
              std::string::npos);

    std::filesystem::remove_all(dir);
}
