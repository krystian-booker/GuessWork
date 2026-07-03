#include <gtest/gtest.h>

#include <cmath>

#include "core/attitude_filter.hpp"

namespace gw {

namespace {

constexpr double kG = 9.80665;

ImuSample sample(uint64_t t_ns, float ax, float ay, float az, float gx,
                 float gy, float gz) {
    ImuSample s;
    s.t_ns     = t_ns;
    s.accel[0] = ax; s.accel[1] = ay; s.accel[2] = az;
    s.gyro[0]  = gx; s.gyro[1]  = gy; s.gyro[2]  = gz;
    return s;
}

}  // namespace

TEST(AttitudeFilterTest, InitializesLevelFromGravity) {
    AttitudeFilter f;
    EXPECT_FALSE(f.snapshot().initialized);
    // Static, level: accel reads +g on body Z.
    f.feed(sample(1'000'000, 0, 0, kG, 0, 0, 0));
    const auto s = f.snapshot();
    EXPECT_TRUE(s.initialized);
    EXPECT_NEAR(s.roll_deg, 0.0, 0.5);
    EXPECT_NEAR(s.pitch_deg, 0.0, 0.5);
    EXPECT_NEAR(s.yaw_deg, 0.0, 0.5);
}

TEST(AttitudeFilterTest, IgnoresNonGravityMagnitudesForInit) {
    AttitudeFilter f;
    f.feed(sample(1'000'000, 0, 0, 0.1f, 0, 0, 0));   // free-fall-ish
    EXPECT_FALSE(f.snapshot().initialized);
    f.feed(sample(2'000'000, 0, 0, 30.0f, 0, 0, 0));  // impact
    EXPECT_FALSE(f.snapshot().initialized);
}

TEST(AttitudeFilterTest, IntegratesConstantYawRate) {
    AttitudeFilter f;
    uint64_t t = 1'000'000;
    f.feed(sample(t, 0, 0, kG, 0, 0, 0));
    // 90°/s about body Z for exactly 1 s at 400 Hz. Body Z stays aligned
    // with world Z, so the accel correction never fights the yaw.
    const float wz = static_cast<float>(M_PI / 2);
    for (int i = 0; i < 400; ++i) {
        t += 2'500'000;  // 2.5 ms
        f.feed(sample(t, 0, 0, kG, 0, 0, wz));
    }
    EXPECT_NEAR(f.snapshot().yaw_deg, 90.0, 2.0);
    EXPECT_NEAR(f.snapshot().roll_deg, 0.0, 1.0);
    EXPECT_NEAR(f.snapshot().pitch_deg, 0.0, 1.0);
}

TEST(AttitudeFilterTest, AccelCorrectionPullsTiltBackToGravity) {
    AttitudeFilter f;
    uint64_t t = 1'000'000;
    f.feed(sample(t, 0, 0, kG, 0, 0, 0));
    // Inject a gyro glitch: +30° roll the accel never confirms…
    const float wx = static_cast<float>(30.0 * M_PI / 180.0);
    t += 2'500'000;
    f.feed(sample(t, 0, 0, kG, wx * 400, 0, 0));  // one 2.5 ms step = 30°
    EXPECT_GT(std::abs(f.snapshot().roll_deg), 20.0);
    // …then 5 s of static level samples: the complementary term (τ ≈ 1 s)
    // must bleed the error back out.
    for (int i = 0; i < 2000; ++i) {
        t += 2'500'000;
        f.feed(sample(t, 0, 0, kG, 0, 0, 0));
    }
    EXPECT_NEAR(f.snapshot().roll_deg, 0.0, 1.0);
}

TEST(AttitudeFilterTest, TiltedInitMatchesAccelDirection) {
    AttitudeFilter f;
    // 90° roll: gravity now reads on body +Y (accel = R_bw · (0,0,g)).
    f.feed(sample(1'000'000, 0, static_cast<float>(kG), 0, 0, 0, 0));
    const auto s = f.snapshot();
    ASSERT_TRUE(s.initialized);
    EXPECT_NEAR(std::abs(s.roll_deg), 90.0, 1.0);
}

TEST(AttitudeFilterTest, ZeroYawKeepsRollPitch) {
    AttitudeFilter f;
    uint64_t t = 1'000'000;
    f.feed(sample(t, 0, 0, kG, 0, 0, 0));
    const float wz = static_cast<float>(M_PI / 4);
    for (int i = 0; i < 400; ++i) {
        t += 2'500'000;
        f.feed(sample(t, 0, 0, kG, 0, 0, wz));
    }
    ASSERT_GT(f.snapshot().yaw_deg, 30.0);
    f.zero_yaw();
    EXPECT_NEAR(f.snapshot().yaw_deg, 0.0, 0.5);
    EXPECT_NEAR(f.snapshot().roll_deg, 0.0, 1.0);
    EXPECT_NEAR(f.snapshot().pitch_deg, 0.0, 1.0);
}

TEST(AttitudeFilterTest, ResetReturnsToUninitialized) {
    AttitudeFilter f;
    f.feed(sample(1'000'000, 0, 0, kG, 0, 0, 0));
    ASSERT_TRUE(f.snapshot().initialized);
    f.reset();
    EXPECT_FALSE(f.snapshot().initialized);
    EXPECT_EQ(f.snapshot().samples, 0u);
}

}  // namespace gw
