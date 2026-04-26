#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>

#include <linux/videodev2.h>

#include "posest/CameraConfig.h"
#include "posest/CameraProducer.h"
#include "posest/V4L2Producer.h"

using posest::v4l2::V4L2Producer;

// --- controlNameToCid tests ---

TEST(V4L2Producer, ControlNameToCidExposureAuto) {
    EXPECT_EQ(V4L2Producer::controlNameToCid("exposure_auto"),
              V4L2_CID_EXPOSURE_AUTO);
}

TEST(V4L2Producer, ControlNameToCidExposureAbsolute) {
    EXPECT_EQ(V4L2Producer::controlNameToCid("exposure_absolute"),
              V4L2_CID_EXPOSURE_ABSOLUTE);
}

TEST(V4L2Producer, ControlNameToCidGain) {
    EXPECT_EQ(V4L2Producer::controlNameToCid("gain"), V4L2_CID_GAIN);
}

TEST(V4L2Producer, ControlNameToCidBrightness) {
    EXPECT_EQ(V4L2Producer::controlNameToCid("brightness"), V4L2_CID_BRIGHTNESS);
}

TEST(V4L2Producer, ControlNameToCidContrast) {
    EXPECT_EQ(V4L2Producer::controlNameToCid("contrast"), V4L2_CID_CONTRAST);
}

TEST(V4L2Producer, ControlNameToCidSaturation) {
    EXPECT_EQ(V4L2Producer::controlNameToCid("saturation"), V4L2_CID_SATURATION);
}

TEST(V4L2Producer, ControlNameToCidSharpness) {
    EXPECT_EQ(V4L2Producer::controlNameToCid("sharpness"), V4L2_CID_SHARPNESS);
}

TEST(V4L2Producer, ControlNameToCidWhiteBalanceAuto) {
    EXPECT_EQ(V4L2Producer::controlNameToCid("white_balance_auto"),
              V4L2_CID_AUTO_WHITE_BALANCE);
}

TEST(V4L2Producer, ControlNameToCidWhiteBalanceTemperature) {
    EXPECT_EQ(V4L2Producer::controlNameToCid("white_balance_temperature"),
              V4L2_CID_WHITE_BALANCE_TEMPERATURE);
}

TEST(V4L2Producer, ControlNameToCidBacklightCompensation) {
    EXPECT_EQ(V4L2Producer::controlNameToCid("backlight_compensation"),
              V4L2_CID_BACKLIGHT_COMPENSATION);
}

TEST(V4L2Producer, ControlNameToCidFocusAuto) {
    EXPECT_EQ(V4L2Producer::controlNameToCid("focus_auto"), V4L2_CID_FOCUS_AUTO);
}

TEST(V4L2Producer, ControlNameToCidFocusAbsolute) {
    EXPECT_EQ(V4L2Producer::controlNameToCid("focus_absolute"),
              V4L2_CID_FOCUS_ABSOLUTE);
}

TEST(V4L2Producer, ControlNameToCidPowerLineFrequency) {
    EXPECT_EQ(V4L2Producer::controlNameToCid("power_line_frequency"),
              V4L2_CID_POWER_LINE_FREQUENCY);
}

TEST(V4L2Producer, ControlNameToCidUnknownReturnsNegative) {
    EXPECT_EQ(V4L2Producer::controlNameToCid("nonexistent_control"), -1);
    EXPECT_EQ(V4L2Producer::controlNameToCid(""), -1);
}

// --- timevalToSteadyClock tests ---

TEST(V4L2Producer, TimevalToSteadyClockZero) {
    auto tp = V4L2Producer::timevalToSteadyClock(0, 0);
    EXPECT_EQ(tp.time_since_epoch().count(), 0);
}

TEST(V4L2Producer, TimevalToSteadyClockSecondsOnly) {
    auto tp = V4L2Producer::timevalToSteadyClock(5, 0);
    auto dur = tp.time_since_epoch();
    auto secs = std::chrono::duration_cast<std::chrono::seconds>(dur);
    EXPECT_EQ(secs.count(), 5);
}

TEST(V4L2Producer, TimevalToSteadyClockMicrosecondsOnly) {
    auto tp = V4L2Producer::timevalToSteadyClock(0, 500000);
    auto dur = tp.time_since_epoch();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(dur);
    EXPECT_EQ(us.count(), 500000);
}

TEST(V4L2Producer, TimevalToSteadyClockCombined) {
    auto tp = V4L2Producer::timevalToSteadyClock(10, 250000);
    auto dur = tp.time_since_epoch();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(dur);
    EXPECT_EQ(us.count(), 10250000);
}

TEST(V4L2Producer, TimevalToSteadyClockLargeTimestamp) {
    auto tp = V4L2Producer::timevalToSteadyClock(1700000000, 123456);
    auto dur = tp.time_since_epoch();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(dur);
    EXPECT_EQ(us.count(), 1700000000123456LL);
}

namespace {

posest::CameraConfig makeV4l2Config() {
    posest::CameraConfig cfg;
    cfg.id = "v4l2_test";
    cfg.type = "v4l2";
    cfg.device = "/dev/null";   // never opened in these tests
    cfg.format.width = 640;
    cfg.format.height = 480;
    cfg.format.fps = 30.0;
    cfg.format.pixel_format = "yuyv";
    return cfg;
}

}  // namespace

TEST(V4L2Producer, SetTriggerModeFreeRunIsNoop) {
    V4L2Producer producer(makeV4l2Config());
    EXPECT_NO_THROW(producer.setTriggerMode(posest::TriggerMode::FreeRun));
}

TEST(V4L2Producer, SetTriggerModeExternalRejected) {
    V4L2Producer producer(makeV4l2Config());
    EXPECT_THROW(producer.setTriggerMode(posest::TriggerMode::External),
                 posest::NotSupportedError);
    EXPECT_THROW(producer.setTriggerMode(posest::TriggerMode::Software),
                 posest::NotSupportedError);
}

TEST(V4L2Producer, SetControlOnUnopenedDeviceThrows) {
    V4L2Producer producer(makeV4l2Config());
    EXPECT_THROW(producer.setControl("brightness", 50), std::runtime_error);
}

TEST(V4L2Producer, SetControlUnknownNameThrowsInvalidArgument) {
    V4L2Producer producer(makeV4l2Config());
    EXPECT_THROW(producer.setControl("not_a_real_control", 1),
                 std::invalid_argument);
}

TEST(V4L2Producer, GetControlOnUnopenedDeviceReturnsNullopt) {
    V4L2Producer producer(makeV4l2Config());
    EXPECT_FALSE(producer.getControl("brightness").has_value());
    EXPECT_FALSE(producer.getControl("not_a_real_control").has_value());
}

TEST(V4L2Producer, PixelFormatCatalogCoversCanonicalNames) {
    // The expanded catalog from the camera-producer subsystem audit.
    EXPECT_TRUE(V4L2Producer::isPixelFormatSupported("mjpeg"));
    EXPECT_TRUE(V4L2Producer::isPixelFormatSupported("yuyv"));
    EXPECT_TRUE(V4L2Producer::isPixelFormatSupported("grey"));
    EXPECT_TRUE(V4L2Producer::isPixelFormatSupported("nv12"));
    EXPECT_TRUE(V4L2Producer::isPixelFormatSupported("bgr3"));
    EXPECT_TRUE(V4L2Producer::isPixelFormatSupported("bayer_rggb8"));
    EXPECT_TRUE(V4L2Producer::isPixelFormatSupported("bayer_grbg8"));
    EXPECT_TRUE(V4L2Producer::isPixelFormatSupported("bayer_gbrg8"));
    EXPECT_TRUE(V4L2Producer::isPixelFormatSupported("bayer_bggr8"));
    EXPECT_FALSE(V4L2Producer::isPixelFormatSupported("not_a_format"));
    EXPECT_THROW((void)V4L2Producer::pixelFormatFourcc("not_a_format"),
                 std::runtime_error);
}

TEST(V4L2Producer, CapabilitiesBackendIsV4l2) {
    V4L2Producer producer(makeV4l2Config());
    const auto caps = producer.capabilities();
    EXPECT_EQ(caps.backend, "v4l2");
    EXPECT_TRUE(caps.supports_set_control);
    EXPECT_TRUE(caps.supports_get_control);
    EXPECT_FALSE(caps.supports_set_trigger_mode);
    ASSERT_EQ(caps.trigger_modes.size(), 1u);
    EXPECT_EQ(caps.trigger_modes[0], posest::TriggerMode::FreeRun);
}
