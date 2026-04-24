#include <chrono>
#include <memory>
#include <mutex>
#include <vector>

#include <gtest/gtest.h>

#include "posest/MeasurementBus.h"
#include "posest/fusion/FusionService.h"

using namespace std::chrono_literals;

namespace {

class RecordingSink final : public posest::fusion::IFusionOutputSink {
public:
    void publish(posest::FusedPoseEstimate estimate) override {
        std::lock_guard<std::mutex> g(mu);
        estimates.push_back(estimate);
    }

    std::mutex mu;
    std::vector<posest::FusedPoseEstimate> estimates;
};

}  // namespace

TEST(FusionService, ProcessesOrderedMeasurementsAndPublishesEstimate) {
    posest::MeasurementBus bus(8);
    posest::fusion::FusionService fusion(bus);
    auto sink = std::make_shared<RecordingSink>();
    fusion.addOutputSink(sink);

    fusion.start();

    posest::WheelOdometrySample odom;
    odom.timestamp = std::chrono::steady_clock::now();
    odom.chassis_delta.x_m = 1.5;
    ASSERT_TRUE(bus.publish(odom));

    std::this_thread::sleep_for(20ms);
    fusion.stop();

    const auto stats = fusion.stats();
    EXPECT_EQ(stats.measurements_processed, 1u);
    EXPECT_EQ(stats.stale_measurements, 0u);

    const auto latest = fusion.latestEstimate();
    ASSERT_TRUE(latest.has_value());
    EXPECT_DOUBLE_EQ(latest->field_to_robot.x_m, 1.5);

    std::lock_guard<std::mutex> g(sink->mu);
    ASSERT_EQ(sink->estimates.size(), 1u);
}

TEST(FusionService, RejectsStaleMeasurements) {
    posest::MeasurementBus bus(8);
    posest::fusion::FusionService fusion(bus);
    fusion.start();

    const auto now = std::chrono::steady_clock::now();
    posest::ImuSample newer;
    newer.timestamp = now;
    posest::ImuSample older;
    older.timestamp = now - 1ms;

    ASSERT_TRUE(bus.publish(newer));
    ASSERT_TRUE(bus.publish(older));

    std::this_thread::sleep_for(20ms);
    fusion.stop();

    const auto stats = fusion.stats();
    EXPECT_EQ(stats.measurements_processed, 1u);
    EXPECT_EQ(stats.stale_measurements, 1u);
}
