#include <gtest/gtest.h>

#include "posest/MeasurementBus.h"
#include "posest/TeeSink.h"

TEST(TeeSink, ImuSampleFansOutToBothBuses) {
    posest::MeasurementBus fusion_bus(4);
    posest::MeasurementBus vio_bus(4);
    posest::TeeSink tee;
    tee.addRoute<posest::ImuSample>(&fusion_bus);
    tee.addRoute<posest::ImuSample>(&vio_bus);

    EXPECT_TRUE(tee.publish(posest::ImuSample{}));
    EXPECT_EQ(fusion_bus.size(), 1u);
    EXPECT_EQ(vio_bus.size(), 1u);
}

TEST(TeeSink, NonRoutedTypesAreSilentlyAccepted) {
    posest::MeasurementBus fusion_bus(4);
    posest::TeeSink tee;
    // Only IMU is routed; ToF samples have no destination.
    tee.addRoute<posest::ImuSample>(&fusion_bus);
    EXPECT_TRUE(tee.publish(posest::ToFSample{}));
    EXPECT_EQ(fusion_bus.size(), 0u);
}

TEST(TeeSink, ChassisOnlyGoesToFusionWhenSoConfigured) {
    posest::MeasurementBus fusion_bus(4);
    posest::MeasurementBus vio_bus(4);
    posest::TeeSink tee;
    tee.addRoute<posest::ImuSample>(&fusion_bus);
    tee.addRoute<posest::ImuSample>(&vio_bus);
    tee.addRoute<posest::ChassisSpeedsSample>(&fusion_bus);

    tee.publish(posest::ChassisSpeedsSample{});
    EXPECT_EQ(fusion_bus.size(), 1u);
    EXPECT_EQ(vio_bus.size(), 0u);
}

TEST(TeeSink, ReturnsFalseIfAnyDownstreamDrops) {
    posest::MeasurementBus tiny(1);
    posest::MeasurementBus big(4);
    posest::TeeSink tee;
    tee.addRoute<posest::ImuSample>(&tiny);
    tee.addRoute<posest::ImuSample>(&big);

    EXPECT_TRUE(tee.publish(posest::ImuSample{}));   // both accept
    EXPECT_FALSE(tee.publish(posest::ImuSample{}));  // tiny full → false
    EXPECT_EQ(big.size(), 2u);
    EXPECT_EQ(tiny.size(), 1u);
}
