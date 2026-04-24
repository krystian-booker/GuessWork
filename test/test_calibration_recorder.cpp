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
    recorder.publish(posest::ImuSample{
        anchor + std::chrono::microseconds(50),
        {1.0, 2.0, 3.0},
        {4.0, 5.0, 6.0},
        25.0,
        0,
    });

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
