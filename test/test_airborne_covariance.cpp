#include <chrono>

#include <gtest/gtest.h>

#include "posest/vio/AirborneCovariance.h"

using namespace std::chrono_literals;
using posest::vio::AirborneState;
using posest::vio::AirborneThresholds;
using posest::vio::AirborneTracker;
using posest::vio::inflate;

namespace {

// Build a 6×6 row-major identity-scaled covariance.
std::array<double, 36> diag(double v) {
    std::array<double, 36> c{};
    for (int i = 0; i < 6; ++i) {
        c[static_cast<std::size_t>(i * 6 + i)] = v;
    }
    return c;
}

}  // namespace

TEST(AirborneTracker, StartsGroundedAndStaysOnLowReading) {
    AirborneTracker t;
    const auto t0 = std::chrono::steady_clock::time_point{};
    EXPECT_EQ(t.update(0.05, t0), AirborneState::kGrounded);
    EXPECT_EQ(t.update(0.10, t0 + 10ms), AirborneState::kGrounded);
}

TEST(AirborneTracker, EntersAirborneAboveThreshold) {
    AirborneTracker t;
    const auto t0 = std::chrono::steady_clock::time_point{};
    EXPECT_EQ(t.update(0.20, t0), AirborneState::kAirborne);
}

TEST(AirborneTracker, HoldsAirborneInHysteresisBand) {
    AirborneTracker t;
    const auto t0 = std::chrono::steady_clock::time_point{};
    // Liftoff.
    EXPECT_EQ(t.update(0.20, t0), AirborneState::kAirborne);
    // 0.14 m is below `above_m` (0.15) but above `below_m` (0.127):
    // hysteresis band — must NOT transition out yet.
    EXPECT_EQ(t.update(0.14, t0 + 5ms), AirborneState::kAirborne);
}

TEST(AirborneTracker, EntersSettlingOnLandingAndExitsAfterDelay) {
    AirborneTracker t;
    const auto t0 = std::chrono::steady_clock::time_point{};
    // Liftoff and clear landing.
    EXPECT_EQ(t.update(0.20, t0), AirborneState::kAirborne);
    EXPECT_EQ(t.update(0.10, t0 + 10ms), AirborneState::kSettling);
    // Still settling at +30 ms (well under the 50 ms default).
    EXPECT_EQ(t.update(0.05, t0 + 30ms), AirborneState::kSettling);
    // Settle window expires.
    EXPECT_EQ(t.update(0.05, t0 + 65ms), AirborneState::kGrounded);
}

TEST(AirborneTracker, MissingReadingPreservesStateAndCounts) {
    AirborneTracker t;
    const auto t0 = std::chrono::steady_clock::time_point{};
    EXPECT_EQ(t.update(0.20, t0), AirborneState::kAirborne);
    EXPECT_EQ(t.update(std::nullopt, t0 + 5ms), AirborneState::kAirborne);
    EXPECT_EQ(t.missingCount(), 1u);
    EXPECT_EQ(t.update(std::nullopt, t0 + 10ms), AirborneState::kAirborne);
    EXPECT_EQ(t.missingCount(), 2u);
}

TEST(AirborneTracker, MissingReadingStillExpiresSettleTimer) {
    // Belt and braces: a ToF dropout right at the tail of a settle
    // must not pin us in kSettling forever.
    AirborneTracker t;
    const auto t0 = std::chrono::steady_clock::time_point{};
    t.update(0.20, t0);                     // airborne
    t.update(0.10, t0 + 5ms);               // settling, deadline +55 ms
    EXPECT_EQ(t.update(std::nullopt, t0 + 100ms), AirborneState::kGrounded);
}

TEST(InflateDiagonal, GroundedReturnsInputUnchanged) {
    const auto in = diag(1.0);
    const auto out = inflate(in, AirborneState::kGrounded, 1e6, 1e9);
    EXPECT_EQ(in, out);
}

TEST(InflateDiagonal, AirborneScalesAndCaps) {
    auto in = diag(0.001);
    in[1] = 0.5;  // off-diagonal — must be left untouched
    const double factor = 1e3;
    const double cap = 1e6;
    const auto out = inflate(in, AirborneState::kAirborne, factor, cap);
    // Diagonals scaled.
    EXPECT_NEAR(out[0], 0.001 * factor, 1e-12);
    EXPECT_NEAR(out[7], 0.001 * factor, 1e-12);
    // Off-diagonal preserved.
    EXPECT_DOUBLE_EQ(out[1], 0.5);
    // Cap applied: with a huge starting variance, output saturates at cap.
    auto big = diag(1e9);
    const auto capped = inflate(big, AirborneState::kAirborne, factor, cap);
    EXPECT_DOUBLE_EQ(capped[0], cap);
}

TEST(InflateDiagonal, SettlingTreatedAsAirborne) {
    const auto in = diag(0.01);
    const auto a = inflate(in, AirborneState::kAirborne, 100.0, 1e6);
    const auto s = inflate(in, AirborneState::kSettling, 100.0, 1e6);
    EXPECT_EQ(a, s);
}
