#include <chrono>
#include <unordered_map>

#include <gtest/gtest.h>

#include "posest/CameraTriggerCache.h"
#include "posest/teensy/Protocol.h"

using namespace std::chrono_literals;

namespace {

posest::CameraTriggerEvent makeEvent(
    std::int32_t pin,
    std::uint32_t sequence,
    std::uint64_t teensy_time_us,
    posest::Timestamp host_time,
    std::uint32_t status_flags = 0u) {
    posest::CameraTriggerEvent ev;
    ev.pin = pin;
    ev.trigger_sequence = sequence;
    ev.teensy_time_us = teensy_time_us;
    ev.timestamp = host_time;
    ev.status_flags = status_flags;
    return ev;
}

}  // namespace

TEST(CameraTriggerCache, LookupReturnsMostRecentBeforeCaptureWithinWindow) {
    posest::CameraTriggerCache cache(
        std::unordered_map<std::int32_t, std::string>{{4, "cam0"}, {5, "cam1"}},
        50ms);

    const posest::Timestamp t0 = posest::Timestamp{} + 1000ms;
    cache.recordEvent(makeEvent(4, 1, 100, t0));
    cache.recordEvent(makeEvent(4, 2, 200, t0 + 10ms));
    cache.recordEvent(makeEvent(5, 11, 1100, t0 + 5ms));

    const auto cam0 = cache.lookup("cam0", t0 + 12ms);
    ASSERT_TRUE(cam0.has_value());
    EXPECT_EQ(cam0->trigger_sequence, 2u);
    EXPECT_EQ(cam0->teensy_time_us, 200u);

    const auto cam1 = cache.lookup("cam1", t0 + 6ms);
    ASSERT_TRUE(cam1.has_value());
    EXPECT_EQ(cam1->trigger_sequence, 11u);
}

TEST(CameraTriggerCache, LookupReturnsNulloptWhenOutsideWindow) {
    posest::CameraTriggerCache cache(
        std::unordered_map<std::int32_t, std::string>{{4, "cam0"}}, 50ms);
    const posest::Timestamp t0 = posest::Timestamp{} + 1000ms;
    cache.recordEvent(makeEvent(4, 1, 100, t0));

    EXPECT_FALSE(cache.lookup("cam0", t0 + 100ms).has_value());
    // capture earlier than any recorded event also returns nullopt.
    EXPECT_FALSE(cache.lookup("cam0", t0 - 1ms).has_value());
}

TEST(CameraTriggerCache, IgnoresEventsOnUnmappedPins) {
    posest::CameraTriggerCache cache(
        std::unordered_map<std::int32_t, std::string>{{4, "cam0"}}, 50ms);
    const posest::Timestamp t0 = posest::Timestamp{} + 1000ms;
    cache.recordEvent(makeEvent(7, 99, 100, t0));

    EXPECT_FALSE(cache.lookup("cam0", t0).has_value());
}

TEST(CameraTriggerCache, SkipsUnsynchronizedTimeEvents) {
    posest::CameraTriggerCache cache(
        std::unordered_map<std::int32_t, std::string>{{4, "cam0"}}, 50ms);
    const posest::Timestamp t0 = posest::Timestamp{} + 1000ms;
    cache.recordEvent(makeEvent(
        4, 1, 100, t0, posest::teensy::kStatusUnsynchronizedTime));

    EXPECT_FALSE(cache.lookup("cam0", t0).has_value());
}

TEST(CameraTriggerCache, CapacityPrunesOldestFirst) {
    posest::CameraTriggerCache cache(
        std::unordered_map<std::int32_t, std::string>{{4, "cam0"}},
        /*match_window=*/1000ms,
        /*per_camera_capacity=*/3);
    const posest::Timestamp t0 = posest::Timestamp{} + 1000ms;
    for (std::uint32_t i = 0; i < 5; ++i) {
        cache.recordEvent(makeEvent(
            4, i, 100u + i,
            t0 + std::chrono::milliseconds(i * 10)));
    }

    // Capacity is 3, so events 0 and 1 should have been pruned. The most
    // recent event (sequence 4) is the lookup result for a query just past
    // the last insert.
    const auto stamp = cache.lookup("cam0", t0 + 50ms);
    ASSERT_TRUE(stamp.has_value());
    EXPECT_EQ(stamp->trigger_sequence, 4u);
}

TEST(CameraTriggerCache, ClearRemovesAllStamps) {
    posest::CameraTriggerCache cache(
        std::unordered_map<std::int32_t, std::string>{{4, "cam0"}}, 50ms);
    const posest::Timestamp t0 = posest::Timestamp{} + 1000ms;
    cache.recordEvent(makeEvent(4, 1, 100, t0));
    cache.clear();
    EXPECT_FALSE(cache.lookup("cam0", t0).has_value());
}
