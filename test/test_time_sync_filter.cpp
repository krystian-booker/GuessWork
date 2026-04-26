#include <cstdint>

#include <gtest/gtest.h>

#include "posest/teensy/TimeSyncFilter.h"

namespace {

posest::teensy::TimeSyncFilter::Sample makeSample(
    std::int64_t teensy_midpoint_us,
    std::int64_t offset_us,
    std::uint64_t round_trip_us = 500) {
    return {teensy_midpoint_us, offset_us, round_trip_us};
}

}  // namespace

TEST(TimeSyncFilter, FirstSampleSetsOffsetExactly) {
    posest::teensy::TimeSyncFilter filter;
    EXPECT_FALSE(filter.established());
    EXPECT_TRUE(filter.update(makeSample(1'000'000, 7500, 800)));
    EXPECT_TRUE(filter.established());
    EXPECT_EQ(filter.offsetUs(), 7500);
    EXPECT_EQ(filter.accepted(), 1u);
    EXPECT_EQ(filter.rejected(), 0u);
    EXPECT_DOUBLE_EQ(filter.skewPerUs(), 0.0);
}

TEST(TimeSyncFilter, ConvergesViaEMAAcrossSamples) {
    posest::teensy::TimeSyncFilter filter;
    for (int i = 0; i < 50; ++i) {
        const std::int64_t teensy_midpoint =
            1'000'000 + static_cast<std::int64_t>(i) * 1'000'000;
        EXPECT_TRUE(filter.update(makeSample(teensy_midpoint, 8000, 600)));
    }
    EXPECT_NEAR(static_cast<double>(filter.offsetUs()), 8000.0, 1.0);
    EXPECT_LT(std::abs(filter.skewPpm()), 1.0);
}

TEST(TimeSyncFilter, RejectsRttOutliersAfterBootstrap) {
    posest::teensy::TimeSyncFilter filter;
    // Bootstrap with three quick samples so the gating threshold can engage.
    EXPECT_TRUE(filter.update(makeSample(1'000'000, 5000, 600)));
    EXPECT_TRUE(filter.update(makeSample(2'000'000, 5000, 700)));
    EXPECT_TRUE(filter.update(makeSample(3'000'000, 5000, 650)));
    EXPECT_EQ(filter.accepted(), 3u);

    // A normal sample stays accepted.
    EXPECT_TRUE(filter.update(makeSample(4'000'000, 5000, 720)));

    // A 50 ms RTT outlier blows past min_rtt * 2 + 1 ms and must be rejected.
    EXPECT_FALSE(filter.update(makeSample(5'000'000, 99'000, 50'000)));
    EXPECT_EQ(filter.rejected(), 1u);
    EXPECT_EQ(filter.accepted(), 4u);
    // The outlier offset must not have moved the EMA.
    EXPECT_NEAR(static_cast<double>(filter.offsetUs()), 5000.0, 1.0);
}

TEST(TimeSyncFilter, SkewTracksLinearDrift) {
    posest::teensy::TimeSyncFilter filter;
    // Inject a known +10 ppm drift: the host clock runs 10 ppm faster than the
    // Teensy. Each successive sample increases offset by 10 us per second of
    // teensy time, so over 8 samples spaced 1 s apart the regression slope
    // should be 1e-5 (offset_us per teensy_us), i.e. 10 ppm.
    const std::int64_t kStartTeensyUs = 1'000'000;
    const std::int64_t kBaseOffsetUs = 4000;
    constexpr double kSlopePerUs = 1.0e-5;  // 10 ppm
    for (int i = 0; i < 8; ++i) {
        const std::int64_t teensy_midpoint =
            kStartTeensyUs + static_cast<std::int64_t>(i) * 1'000'000;
        const std::int64_t offset_us = kBaseOffsetUs +
                                       static_cast<std::int64_t>(
                                           kSlopePerUs *
                                           static_cast<double>(teensy_midpoint -
                                                               kStartTeensyUs));
        EXPECT_TRUE(filter.update(makeSample(teensy_midpoint, offset_us, 600)));
    }
    EXPECT_NEAR(filter.skewPpm(), 10.0, 0.5);
}

TEST(TimeSyncFilter, ApplyAdjustsForOffsetAndSkew) {
    posest::teensy::TimeSyncFilter filter;
    EXPECT_TRUE(filter.update(makeSample(1'000'000, 1000, 500)));
    EXPECT_TRUE(filter.update(makeSample(2'000'000, 1010, 500)));
    EXPECT_TRUE(filter.update(makeSample(3'000'000, 1020, 500)));
    // Slope ~ 10 us per 1 s = 10 ppm. anchor = last sample's teensy_midpoint.
    const std::int64_t teensy_query_us = 4'000'000;
    const std::int64_t host_time_us = filter.apply(teensy_query_us);
    // offset EMA after 3 samples is 1015 (initialized 1000, then EMA toward
    // 1010 and 1020); skew * (4e6 - 3e6) ≈ 10 us.
    EXPECT_NEAR(
        static_cast<double>(host_time_us),
        static_cast<double>(teensy_query_us) +
            static_cast<double>(filter.offsetUs()) +
            filter.skewPerUs() *
                static_cast<double>(teensy_query_us - filter.anchorTeensyUs()),
        1.0);
}

TEST(TimeSyncFilter, ResetClearsAllState) {
    posest::teensy::TimeSyncFilter filter;
    EXPECT_TRUE(filter.update(makeSample(1'000'000, 9000, 700)));
    filter.reset();
    EXPECT_FALSE(filter.established());
    EXPECT_EQ(filter.accepted(), 0u);
    EXPECT_EQ(filter.offsetUs(), 0);
    EXPECT_EQ(filter.rejected(), 0u);
    EXPECT_DOUBLE_EQ(filter.skewPerUs(), 0.0);
}
