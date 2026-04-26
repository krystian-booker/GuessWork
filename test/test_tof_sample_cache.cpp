#include <gtest/gtest.h>

#include "posest/MeasurementTypes.h"
#include "posest/ToFSampleCache.h"

namespace {

posest::ToFSample makeSample(std::uint32_t trigger_sequence, double distance_m) {
    posest::ToFSample sample;
    sample.trigger_sequence = trigger_sequence;
    sample.raw_distance_m = distance_m;
    sample.distance_m = distance_m;
    return sample;
}

}  // namespace

TEST(ToFSampleCache, LookupBySequenceReturnsRecordedSample) {
    posest::ToFSampleCache cache("cam0");
    cache.recordSample(makeSample(7, 1.234));

    const auto found = cache.lookupBySequence("cam0", 7);
    ASSERT_TRUE(found.has_value());
    EXPECT_EQ(found->trigger_sequence, 7u);
    EXPECT_DOUBLE_EQ(found->distance_m, 1.234);
}

TEST(ToFSampleCache, LookupForWrongCameraReturnsNullopt) {
    posest::ToFSampleCache cache("cam_vio");
    cache.recordSample(makeSample(3, 0.5));

    EXPECT_FALSE(cache.lookupBySequence("cam_apriltag", 3).has_value());
}

TEST(ToFSampleCache, ReturnsMostRecentSampleForDuplicateSequence) {
    // The exact-match join is keyed on the firmware-side trigger_sequence; if
    // a duplicate ever lands (e.g. sequence wrap), the newest entry wins so
    // the producer pulls fresh depth.
    posest::ToFSampleCache cache("cam0");
    cache.recordSample(makeSample(11, 0.1));
    cache.recordSample(makeSample(11, 0.9));

    const auto found = cache.lookupBySequence("cam0", 11);
    ASSERT_TRUE(found.has_value());
    EXPECT_DOUBLE_EQ(found->distance_m, 0.9);
}

TEST(ToFSampleCache, RingEvictsOldestWhenCapacityExceeded) {
    posest::ToFSampleCache cache("cam0", /*capacity=*/3);
    for (std::uint32_t i = 1; i <= 5; ++i) {
        cache.recordSample(makeSample(i, static_cast<double>(i) * 0.01));
    }

    // Sequences 1 and 2 should have been evicted; 3, 4, 5 remain.
    EXPECT_FALSE(cache.lookupBySequence("cam0", 1).has_value());
    EXPECT_FALSE(cache.lookupBySequence("cam0", 2).has_value());
    ASSERT_TRUE(cache.lookupBySequence("cam0", 3).has_value());
    ASSERT_TRUE(cache.lookupBySequence("cam0", 5).has_value());
    EXPECT_DOUBLE_EQ(cache.lookupBySequence("cam0", 5)->distance_m, 0.05);
}

TEST(ToFSampleCache, ClearRemovesAllSamples) {
    posest::ToFSampleCache cache("cam0");
    cache.recordSample(makeSample(1, 0.5));
    cache.clear();
    EXPECT_FALSE(cache.lookupBySequence("cam0", 1).has_value());
}
