#include <gtest/gtest.h>

#include <atomic>
#include <thread>

#include "core/measurement_bus.hpp"

namespace gw {

TEST(MeasurementBusTest, EverySubscriberSeesEveryValue) {
    MeasurementBus<int> bus;
    auto a = bus.subscribe();
    auto b = bus.subscribe();

    for (int i = 0; i < 5; ++i) bus.publish(i);

    int v;
    for (int i = 0; i < 5; ++i) {
        ASSERT_TRUE(bus.try_pop(a, v));
        EXPECT_EQ(v, i);
        ASSERT_TRUE(bus.try_pop(b, v));
        EXPECT_EQ(v, i);
    }
    EXPECT_FALSE(bus.try_pop(a, v));
    EXPECT_FALSE(bus.try_pop(b, v));
}

TEST(MeasurementBusTest, BoundedQueueDropsOldest) {
    MeasurementBus<int> bus;
    auto h = bus.subscribe(/*capacity=*/3);

    for (int i = 0; i < 10; ++i) bus.publish(i);

    int v;
    ASSERT_TRUE(bus.try_pop(h, v));
    EXPECT_EQ(v, 7);  // 0..6 dropped, oldest survivor is 7
    ASSERT_TRUE(bus.try_pop(h, v));
    EXPECT_EQ(v, 8);
    ASSERT_TRUE(bus.try_pop(h, v));
    EXPECT_EQ(v, 9);
    EXPECT_FALSE(bus.try_pop(h, v));
    EXPECT_EQ(bus.dropped(h), 7u);
}

TEST(MeasurementBusTest, LateSubscriberMissesHistory) {
    MeasurementBus<int> bus;
    bus.publish(1);
    auto h = bus.subscribe();
    bus.publish(2);

    int v;
    ASSERT_TRUE(bus.try_pop(h, v));
    EXPECT_EQ(v, 2);
    EXPECT_FALSE(bus.try_pop(h, v));
}

TEST(MeasurementBusTest, WaitPopBlocksUntilPublish) {
    MeasurementBus<int> bus;
    auto h = bus.subscribe();

    std::atomic<int> got{-1};
    std::thread consumer([&] {
        int v;
        if (bus.wait_pop(h, v)) got.store(v);
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    bus.publish(42);
    consumer.join();
    EXPECT_EQ(got.load(), 42);
}

TEST(MeasurementBusTest, UnsubscribeWakesBlockedConsumer) {
    MeasurementBus<int> bus;
    auto h = bus.subscribe();

    std::atomic<bool> returned_false{false};
    std::thread consumer([&] {
        int v;
        if (!bus.wait_pop(h, v)) returned_false.store(true);
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    bus.unsubscribe(h);
    consumer.join();
    EXPECT_TRUE(returned_false.load());
}

TEST(MeasurementBusTest, UnsubscribedHandleStopsReceiving) {
    MeasurementBus<int> bus;
    auto h = bus.subscribe();
    bus.unsubscribe(h);
    bus.publish(7);

    int v;
    EXPECT_FALSE(bus.try_pop(h, v));
}

}  // namespace gw
