#include <gtest/gtest.h>

#include <chrono>
#include <linux/videodev2.h>

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
