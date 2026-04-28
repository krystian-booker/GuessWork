#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "posest/calibration/CalibrationTargetWriter.h"
#include "posest/config/CalibrationParsers.h"

namespace {

std::filesystem::path tempPath(const std::string& name) {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    return std::filesystem::temp_directory_path() /
           ("posest_target_" + name + "_" + std::to_string(stamp) + ".yaml");
}

std::string readWhole(const std::filesystem::path& path) {
    std::ifstream in(path);
    std::ostringstream out;
    out << in.rdbuf();
    return out.str();
}

}  // namespace

TEST(CalibrationTargetWriter, AprilGridYamlContainsExpectedKeys) {
    posest::runtime::CalibrationTargetConfig target;
    target.id = "apgrid_88";
    target.type = "aprilgrid";
    target.rows = 6;
    target.cols = 6;
    target.tag_size_m = 0.088;
    target.tag_spacing_ratio = 0.3;
    target.tag_family = "tag36h11";

    const auto path = tempPath("aprilgrid");
    posest::calibration::writeKalibrTargetYaml(target, path);
    const auto contents = readWhole(path);
    EXPECT_NE(contents.find("target_type: 'aprilgrid'"), std::string::npos);
    EXPECT_NE(contents.find("tagCols: 6"), std::string::npos);
    EXPECT_NE(contents.find("tagRows: 6"), std::string::npos);
    EXPECT_NE(contents.find("tagSize: 0.088"), std::string::npos);
    EXPECT_NE(contents.find("tagSpacing: 0.3"), std::string::npos);
    EXPECT_NE(contents.find("tagFamily: 'tag36h11'"), std::string::npos);

    std::filesystem::remove(path);
}

TEST(CalibrationTargetWriter, AprilGridRoundTripsThroughParser) {
    posest::runtime::CalibrationTargetConfig target;
    target.id = "apgrid_88";
    target.type = "aprilgrid";
    target.rows = 6;
    target.cols = 6;
    target.tag_size_m = 0.088;
    target.tag_spacing_ratio = 0.3;
    target.tag_family = "tag36h11";

    const auto path = tempPath("aprilgrid_rt");
    posest::calibration::writeKalibrTargetYaml(target, path);
    const auto parsed =
        posest::config::parseKalibrTargetYaml(path, "apgrid_rt");

    EXPECT_EQ(parsed.id, "apgrid_rt");
    EXPECT_EQ(parsed.type, "aprilgrid");
    EXPECT_EQ(parsed.rows, 6);
    EXPECT_EQ(parsed.cols, 6);
    EXPECT_DOUBLE_EQ(parsed.tag_size_m, 0.088);
    EXPECT_DOUBLE_EQ(parsed.tag_spacing_ratio, 0.3);
    EXPECT_EQ(parsed.tag_family, "tag36h11");

    std::filesystem::remove(path);
}

TEST(CalibrationTargetWriter, CheckerboardRoundTripsThroughParser) {
    posest::runtime::CalibrationTargetConfig target;
    target.id = "chk_25";
    target.type = "checkerboard";
    target.rows = 7;
    target.cols = 6;
    target.square_size_m = 0.025;

    const auto path = tempPath("checker");
    posest::calibration::writeKalibrTargetYaml(target, path);
    const auto parsed =
        posest::config::parseKalibrTargetYaml(path, "chk_rt");

    EXPECT_EQ(parsed.type, "checkerboard");
    EXPECT_EQ(parsed.rows, 7);
    EXPECT_EQ(parsed.cols, 6);
    EXPECT_DOUBLE_EQ(parsed.square_size_m, 0.025);

    std::filesystem::remove(path);
}

TEST(CalibrationTargetWriter, CirclegridRoundTripsThroughParser) {
    posest::runtime::CalibrationTargetConfig target;
    target.id = "circles";
    target.type = "circlegrid";
    target.rows = 4;
    target.cols = 11;
    target.square_size_m = 0.04;

    const auto path = tempPath("circles");
    posest::calibration::writeKalibrTargetYaml(target, path);
    const auto parsed =
        posest::config::parseKalibrTargetYaml(path, "circles_rt");

    EXPECT_EQ(parsed.type, "circlegrid");
    EXPECT_EQ(parsed.rows, 4);
    EXPECT_EQ(parsed.cols, 11);
    EXPECT_DOUBLE_EQ(parsed.square_size_m, 0.04);

    std::filesystem::remove(path);
}

TEST(CalibrationTargetWriter, RejectsUnsupportedType) {
    posest::runtime::CalibrationTargetConfig target;
    target.id = "weird";
    target.type = "fiducial_marker";
    target.rows = 1;
    target.cols = 1;

    const auto path = tempPath("weird");
    EXPECT_THROW(
        posest::calibration::writeKalibrTargetYaml(target, path),
        std::invalid_argument);
}
