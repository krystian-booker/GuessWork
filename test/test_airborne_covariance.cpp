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

// ---------------------------------------------------------------------------
// Carpet-mount scenario: camera mounted 4 inches (0.10 m) above the carpet,
// facing down. Default thresholds are above_m=0.15 / below_m=0.127, sized
// for ~2 inches of headroom over carpet pile and chassis-pose jitter. The
// tests below pin behaviour at the actual operating distances rather than
// the broad 0.05 / 0.20 buckets the earlier suite uses, so a future
// threshold change is visible as a concrete carpet-regression failure.

TEST(AirborneTracker, CarpetMountStaysGroundedAcrossPileNoise) {
    // Sustained reading at the nominal 0.10 m mount with realistic
    // sub-centimeter noise in both directions. None of these touches
    // the airborne band; missing-count must remain 0.
    AirborneTracker t;
    auto now = std::chrono::steady_clock::time_point{};
    for (double d : {0.099, 0.101, 0.105, 0.108, 0.097, 0.102, 0.110}) {
        EXPECT_EQ(t.update(d, now), AirborneState::kGrounded) << "d=" << d;
        now += 10ms;
    }
    EXPECT_EQ(t.missingCount(), 0u);
}

TEST(AirborneTracker, CarpetBumpInsideHysteresisStaysGroundedFromCold) {
    // 0.13 m sits between below_m (0.127) and above_m (0.15) — that is
    // the hysteresis band. Hysteresis only matters once we have left
    // the grounded state; from a cold start a bump into this band must
    // remain grounded, otherwise carpet pile or a wire-routing slot
    // would routinely push us into kAirborne and mute VIO.
    AirborneTracker t;
    const auto t0 = std::chrono::steady_clock::time_point{};
    EXPECT_EQ(t.update(0.130, t0), AirborneState::kGrounded);
    EXPECT_EQ(t.update(0.140, t0 + 10ms), AirborneState::kGrounded);
    EXPECT_EQ(t.update(0.149, t0 + 20ms), AirborneState::kGrounded);
}

TEST(AirborneTracker, ExactlyAtUpperThresholdStaysGrounded) {
    // Boundary: the predicate is `>= above_m` for entry. Pin the
    // semantic — a reading exactly at 0.15 m is NOT airborne. Anything
    // tighter is impossible at f64 precision and would be a flap risk.
    AirborneTracker t;
    EXPECT_EQ(t.update(0.150, std::chrono::steady_clock::time_point{}),
              AirborneState::kGrounded);
}

TEST(AirborneTracker, JustAboveCarpetThresholdEntersAirborne) {
    // 0.16 m = 4 inches mount + ~6 mm. A real liftoff during a robot
    // bump must trip airborne; this pins the headroom margin so a future
    // threshold relaxation surfaces here.
    AirborneTracker t;
    EXPECT_EQ(t.update(0.160, std::chrono::steady_clock::time_point{}),
              AirborneState::kAirborne);
}

TEST(AirborneTracker, CarpetLandingFollowsLiftoffSettleGroundedSequence) {
    // Full carpet-cycle: nominal grounded → 4 cm bump (still grounded)
    // → liftoff (airborne) → return to nominal (settling 50 ms) →
    // grounded. This is the common path during a chassis pop over an
    // FRC field carpet seam, and the test pins it end-to-end so the
    // sequence stays observable as a single failure.
    AirborneTracker t;
    auto now = std::chrono::steady_clock::time_point{};
    EXPECT_EQ(t.update(0.100, now), AirborneState::kGrounded);
    now += 10ms;
    EXPECT_EQ(t.update(0.140, now), AirborneState::kGrounded);  // pile bump
    now += 10ms;
    EXPECT_EQ(t.update(0.220, now), AirborneState::kAirborne);  // liftoff
    now += 10ms;
    EXPECT_EQ(t.update(0.105, now), AirborneState::kSettling);  // landing
    now += 30ms;
    EXPECT_EQ(t.update(0.103, now), AirborneState::kSettling);  // mid-settle
    now += 30ms;  // crosses the 50 ms deadline (now t+80 from landing)
    EXPECT_EQ(t.update(0.101, now), AirborneState::kGrounded);
}
