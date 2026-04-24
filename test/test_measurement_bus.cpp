#include <chrono>
#include <variant>

#include <gtest/gtest.h>

#include "posest/MeasurementBus.h"

using namespace std::chrono_literals;

TEST(MeasurementBus, PreservesFifoOrderAcrossMeasurementTypes) {
    posest::MeasurementBus bus(4);
    const auto t0 = std::chrono::steady_clock::now();

    posest::ImuSample imu;
    imu.timestamp = t0;
    posest::WheelOdometrySample odom;
    odom.timestamp = t0 + 1ms;

    EXPECT_TRUE(bus.publish(imu));
    EXPECT_TRUE(bus.publish(odom));

    auto first = bus.take();
    auto second = bus.take();

    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    EXPECT_TRUE(std::holds_alternative<posest::ImuSample>(*first));
    EXPECT_TRUE(std::holds_alternative<posest::WheelOdometrySample>(*second));
}

TEST(MeasurementBus, DropsNewestWhenBoundedQueueIsFull) {
    posest::MeasurementBus bus(1);

    EXPECT_TRUE(bus.publish(posest::ImuSample{}));
    EXPECT_FALSE(bus.publish(posest::WheelOdometrySample{}));
    EXPECT_EQ(bus.droppedNewestCount(), 1u);
    EXPECT_EQ(bus.size(), 1u);

    auto measurement = bus.take();
    ASSERT_TRUE(measurement.has_value());
    EXPECT_TRUE(std::holds_alternative<posest::ImuSample>(*measurement));
}

TEST(MeasurementBus, ShutdownUnblocksTake) {
    posest::MeasurementBus bus(1);
    bus.shutdown();

    auto measurement = bus.take();
    EXPECT_FALSE(measurement.has_value());
    EXPECT_FALSE(bus.publish(posest::ImuSample{}));
}
