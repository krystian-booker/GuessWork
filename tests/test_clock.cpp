#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "core/clock.hpp"

namespace gw {

TEST(ClockTest, NowReturnsNonZero) {
    EXPECT_GT(Clock::now_ns(), 0u);
}

TEST(ClockTest, NowIsMonotonic) {
    const uint64_t a = Clock::now_ns();
    const uint64_t b = Clock::now_ns();
    EXPECT_GE(b, a);
}

TEST(ClockTest, AdvancesWithSleep) {
    const uint64_t a = Clock::now_ns();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    const uint64_t b = Clock::now_ns();
    EXPECT_GE(b - a, 4'000'000u);   // at least 4 ms elapsed
    EXPECT_LT(b - a, 500'000'000u); // sanity: under 500 ms
}

TEST(ClockTest, AgreesWithSteadyClockRoughly) {
    const uint64_t gw_start = Clock::now_ns();
    const auto     sc_start = std::chrono::steady_clock::now();

    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    const uint64_t gw_elapsed = Clock::now_ns() - gw_start;
    const auto     sc_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::steady_clock::now() - sc_start)
                                    .count();

    // Both wrap mach_absolute_time under the hood on macOS; allow 5 ms slack.
    const int64_t delta = static_cast<int64_t>(gw_elapsed) - static_cast<int64_t>(sc_elapsed);
    EXPECT_LT(std::abs(delta), 5'000'000);
}

}  // namespace gw
