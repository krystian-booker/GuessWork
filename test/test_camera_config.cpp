#include <gtest/gtest.h>

#include "posest/XmlCameraConfigSource.h"

namespace {

const char* kValidTwoCameras = R"(
<?xml version="1.0" encoding="UTF-8"?>
<cameras>
  <camera id="cam0" type="v4l2"
          device="/dev/v4l/by-id/usb-046d_C920_12345678-video-index0">
    <format width="1920" height="1080" fps="30" pixel_format="mjpeg" />
    <controls>
      <control name="exposure_auto" value="1" />
      <control name="exposure_absolute" value="166" />
      <control name="gain" value="64" />
    </controls>
  </camera>
  <camera id="cam1" type="v4l2"
          device="/dev/v4l/by-path/pci-0000:00:14.0-usb-0:2:1.0-video-index0">
    <format width="640" height="480" fps="60" pixel_format="yuyv" />
    <controls>
      <control name="exposure_auto" value="3" />
    </controls>
  </camera>
</cameras>
)";

}  // namespace

TEST(XmlCameraConfigSource, ParsesValidMultiCameraConfig) {
    auto source = posest::XmlCameraConfigSource::fromString(kValidTwoCameras);
    auto configs = source.loadAll();

    ASSERT_EQ(configs.size(), 2u);

    auto& c0 = configs[0];
    EXPECT_EQ(c0.id, "cam0");
    EXPECT_EQ(c0.type, "v4l2");
    EXPECT_EQ(c0.device, "/dev/v4l/by-id/usb-046d_C920_12345678-video-index0");
    EXPECT_EQ(c0.format.width, 1920);
    EXPECT_EQ(c0.format.height, 1080);
    EXPECT_DOUBLE_EQ(c0.format.fps, 30.0);
    EXPECT_EQ(c0.format.pixel_format, "mjpeg");
    ASSERT_EQ(c0.controls.size(), 3u);
    EXPECT_EQ(c0.controls[0].name, "exposure_auto");
    EXPECT_EQ(c0.controls[0].value, 1);
    EXPECT_EQ(c0.controls[1].name, "exposure_absolute");
    EXPECT_EQ(c0.controls[1].value, 166);
    EXPECT_EQ(c0.controls[2].name, "gain");
    EXPECT_EQ(c0.controls[2].value, 64);

    auto& c1 = configs[1];
    EXPECT_EQ(c1.id, "cam1");
    EXPECT_EQ(c1.type, "v4l2");
    EXPECT_EQ(c1.format.width, 640);
    EXPECT_EQ(c1.format.height, 480);
    EXPECT_DOUBLE_EQ(c1.format.fps, 60.0);
    EXPECT_EQ(c1.format.pixel_format, "yuyv");
    ASSERT_EQ(c1.controls.size(), 1u);
    EXPECT_EQ(c1.controls[0].name, "exposure_auto");
    EXPECT_EQ(c1.controls[0].value, 3);
}

TEST(XmlCameraConfigSource, MissingIdThrows) {
    auto source = posest::XmlCameraConfigSource::fromString(R"(
        <cameras>
            <camera type="v4l2" device="/dev/video0" />
        </cameras>
    )");
    EXPECT_THROW(source.loadAll(), std::runtime_error);
}

TEST(XmlCameraConfigSource, MissingTypeThrows) {
    auto source = posest::XmlCameraConfigSource::fromString(R"(
        <cameras>
            <camera id="cam0" device="/dev/video0" />
        </cameras>
    )");
    EXPECT_THROW(source.loadAll(), std::runtime_error);
}

TEST(XmlCameraConfigSource, MissingDeviceThrows) {
    auto source = posest::XmlCameraConfigSource::fromString(R"(
        <cameras>
            <camera id="cam0" type="v4l2" />
        </cameras>
    )");
    EXPECT_THROW(source.loadAll(), std::runtime_error);
}

TEST(XmlCameraConfigSource, MissingFormatUsesDefaults) {
    auto source = posest::XmlCameraConfigSource::fromString(R"(
        <cameras>
            <camera id="cam0" type="v4l2" device="/dev/video0">
                <controls>
                    <control name="gain" value="50" />
                </controls>
            </camera>
        </cameras>
    )");
    auto configs = source.loadAll();
    ASSERT_EQ(configs.size(), 1u);
    EXPECT_EQ(configs[0].format.width, 1920);
    EXPECT_EQ(configs[0].format.height, 1080);
    EXPECT_DOUBLE_EQ(configs[0].format.fps, 30.0);
    EXPECT_EQ(configs[0].format.pixel_format, "mjpeg");
    ASSERT_EQ(configs[0].controls.size(), 1u);
}

TEST(XmlCameraConfigSource, MissingControlsGivesEmptyVector) {
    auto source = posest::XmlCameraConfigSource::fromString(R"(
        <cameras>
            <camera id="cam0" type="v4l2" device="/dev/video0">
                <format width="640" height="480" fps="60" pixel_format="yuyv" />
            </camera>
        </cameras>
    )");
    auto configs = source.loadAll();
    ASSERT_EQ(configs.size(), 1u);
    EXPECT_TRUE(configs[0].controls.empty());
}

TEST(XmlCameraConfigSource, UnknownControlNamesPassThrough) {
    auto source = posest::XmlCameraConfigSource::fromString(R"(
        <cameras>
            <camera id="cam0" type="v4l2" device="/dev/video0">
                <controls>
                    <control name="some_future_control" value="42" />
                </controls>
            </camera>
        </cameras>
    )");
    auto configs = source.loadAll();
    ASSERT_EQ(configs.size(), 1u);
    ASSERT_EQ(configs[0].controls.size(), 1u);
    EXPECT_EQ(configs[0].controls[0].name, "some_future_control");
    EXPECT_EQ(configs[0].controls[0].value, 42);
}

TEST(XmlCameraConfigSource, MissingCamerasRootThrows) {
    auto source = posest::XmlCameraConfigSource::fromString(R"(
        <something_else>
            <camera id="cam0" type="v4l2" device="/dev/video0" />
        </something_else>
    )");
    EXPECT_THROW(source.loadAll(), std::runtime_error);
}

TEST(XmlCameraConfigSource, EmptyCamerasReturnsEmpty) {
    auto source = posest::XmlCameraConfigSource::fromString(R"(
        <cameras />
    )");
    auto configs = source.loadAll();
    EXPECT_TRUE(configs.empty());
}

TEST(XmlCameraConfigSource, InvalidXmlThrows) {
    auto source = posest::XmlCameraConfigSource::fromString("not xml at all <><>");
    EXPECT_THROW(source.loadAll(), std::runtime_error);
}
