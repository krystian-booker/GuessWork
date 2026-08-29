#include <gtest/gtest.h>

#include <cstdint>
#include <cstdlib>

#include "core/clock_sync.hpp"

namespace gw {

namespace {

// Deterministic uniform jitter in [lo, hi) µs — LCG, no <random> seeding
// variance across platforms.
struct Lcg {
    uint64_t state = 0x9E3779B97F4A7C15ull;
    uint64_t next() {
        state = state * 6364136223846793005ull + 1442695040888963407ull;
        return state >> 33;
    }
    uint64_t uniform(uint64_t lo, uint64_t hi) { return lo + next() % (hi - lo); }
};

// Drives a synthetic stream at `rate_hz` for `seconds` of RIO time:
// arrival = rio + offset_us + drift_ppm·t + jitter[lo, hi). Returns the last
// arrival stamp (the "now" for healthy()).
uint64_t drive(ClockSync& sync, Lcg& rng, uint64_t& rio_us,
               double seconds, double offset_us, double drift_ppm,
               uint64_t jitter_lo_us, uint64_t jitter_hi_us,
               double rate_hz = 100.0) {
    const uint64_t step_us = static_cast<uint64_t>(1e6 / rate_hz);
    const uint64_t n       = static_cast<uint64_t>(seconds * rate_hz);
    const uint64_t rio0    = rio_us;
    uint64_t last_arrival  = 0;
    for (uint64_t i = 0; i < n; ++i) {
        const double drift =
            drift_ppm * 1e-6 * static_cast<double>(rio_us - rio0);
        const uint64_t jitter = rng.uniform(jitter_lo_us, jitter_hi_us);
        last_arrival = rio_us + static_cast<uint64_t>(offset_us + drift) + jitter;
        sync.feed(rio_us, last_arrival);
        rio_us += step_us;
    }
    return last_arrival;
}

}  // namespace

TEST(ClockSyncTest, UnhealthyDuringWarmUp) {
    ClockSync sync;
    Lcg          rng;
    uint64_t     rio = 10'000'000;
    // 0.8 s — fewer than the 4 completed buckets the fit needs.
    const auto now = drive(sync, rng, rio, 0.8, 5e6, 0.0, 100, 1000);
    EXPECT_FALSE(sync.healthy(now));
    EXPECT_FALSE(sync.to_local_ns(rio).has_value());
    EXPECT_FALSE(sync.to_remote_us(now * 1000).has_value());
    EXPECT_GT(sync.samples(), 0u);
}

TEST(ClockSyncTest, RecoversConstantOffsetUnderJitter) {
    ClockSync sync;
    Lcg          rng;
    uint64_t     rio = 50'000'000;
    const double offset = 7'000'000.0;  // 7 s between the two clocks
    const auto now = drive(sync, rng, rio, 10.0, offset, 0.0, 100, 5000);

    ASSERT_TRUE(sync.healthy(now));
    // The windowed minimum sits at offset + jitter floor (100 µs) + the
    // min-of-bucket residue — well under 1 ms above the true offset.
    EXPECT_GE(sync.offset_us(), offset);
    EXPECT_LE(sync.offset_us(), offset + 1000.0);
    EXPECT_NEAR(sync.drift_ppm(), 0.0, 25.0);
    EXPECT_EQ(sync.resets(), 0u);

    const auto mapped = sync.to_local_ns(rio);
    ASSERT_TRUE(mapped.has_value());
    const double err_us =
        static_cast<double>(*mapped) / 1000.0 - (static_cast<double>(rio) + offset);
    EXPECT_GE(err_us, 0.0);
    EXPECT_LE(err_us, 1000.0);
}

TEST(ClockSyncTest, TracksCrystalDrift) {
    ClockSync sync;
    Lcg          rng;
    uint64_t     rio = 1'000'000;
    // +50 ppm relative drift, tight jitter (CAN+ISR latency clusters tightly
    // in practice; the 5 ms tail is exercised by the spike test).
    const auto now = drive(sync, rng, rio, 30.0, 2e6, 50.0, 100, 1000);

    ASSERT_TRUE(sync.healthy(now));
    EXPECT_NEAR(sync.drift_ppm(), 50.0, 15.0);

    // Mapping error at the window edge stays bounded.
    const auto mapped = sync.to_local_ns(rio);
    ASSERT_TRUE(mapped.has_value());
    const double truth_us = static_cast<double>(rio) + 2e6 +
                            50.0 * 1e-6 * static_cast<double>(rio - 1'000'000);
    const double err_us = static_cast<double>(*mapped) / 1000.0 - truth_us;
    EXPECT_NEAR(err_us, 0.0, 300.0);
}

TEST(ClockSyncTest, IgnoresLatencySpikes) {
    ClockSync sync;
    Lcg          rng;
    uint64_t     rio = 5'000'000;
    const double offset = 3e6;
    // Hand-rolled drive with a 20 ms spike every 50th sample.
    uint64_t last_arrival = 0;
    for (int i = 0; i < 1000; ++i) {  // 10 s @ 100 Hz
        uint64_t jitter = rng.uniform(100, 1000);
        if (i % 50 == 49) jitter += 20'000;
        last_arrival = rio + static_cast<uint64_t>(offset) + jitter;
        sync.feed(rio, last_arrival);
        rio += 10'000;
    }
    ASSERT_TRUE(sync.healthy(last_arrival));
    // The bucket minimum rejects the spikes entirely.
    EXPECT_GE(sync.offset_us(), offset);
    EXPECT_LE(sync.offset_us(), offset + 1000.0);
    EXPECT_EQ(sync.resets(), 0u);
}

TEST(ClockSyncTest, BackwardRioJumpResetsAndReWarms) {
    ClockSync sync;
    Lcg          rng;
    uint64_t     rio = 60'000'000;
    auto now = drive(sync, rng, rio, 5.0, 1e6, 0.0, 100, 1000);
    ASSERT_TRUE(sync.healthy(now));

    // RIO reboot: its clock restarts near zero (offset implicitly changes too).
    rio = 100'000;
    now = drive(sync, rng, rio, 0.2, 66e6, 0.0, 100, 1000);
    EXPECT_EQ(sync.resets(), 1u);
    EXPECT_FALSE(sync.healthy(now));  // re-warming

    now = drive(sync, rng, rio, 3.0, 66e6, 0.0, 100, 1000);
    EXPECT_TRUE(sync.healthy(now));
    EXPECT_GE(sync.offset_us(), 66e6);
    EXPECT_LE(sync.offset_us(), 66e6 + 1000.0);
}

TEST(ClockSyncTest, OffsetStepResets) {
    ClockSync sync;
    Lcg          rng;
    uint64_t     rio = 20'000'000;
    auto now = drive(sync, rng, rio, 5.0, 1e6, 0.0, 100, 1000);
    ASSERT_TRUE(sync.healthy(now));

    // The sync controller side of the mapping steps by +100 ms (rio keeps counting
    // forward — e.g. an FPGA time re-sync on the controller).
    now = drive(sync, rng, rio, 0.1, 1e6 + 100'000.0, 0.0, 100, 1000);
    EXPECT_EQ(sync.resets(), 1u);

    now = drive(sync, rng, rio, 3.0, 1e6 + 100'000.0, 0.0, 100, 1000);
    EXPECT_TRUE(sync.healthy(now));
}

TEST(ClockSyncTest, GoesStaleWithoutFeeds) {
    ClockSync sync;
    Lcg          rng;
    uint64_t     rio = 30'000'000;
    const auto now = drive(sync, rng, rio, 5.0, 1e6, 0.0, 100, 1000);
    ASSERT_TRUE(sync.healthy(now));
    EXPECT_FALSE(sync.healthy(now + 1'000'000));  // 1 s with no samples
    // The fit itself is still usable — mappings keep working for late
    // consumers; only the liveness gate trips.
    EXPECT_TRUE(sync.to_local_ns(rio).has_value());
}

TEST(ClockSyncTest, MappingRoundTrips) {
    ClockSync sync;
    Lcg          rng;
    uint64_t     rio = 40'000'000;
    drive(sync, rng, rio, 10.0, 5e6, 30.0, 100, 1000);

    const uint64_t probe_rio = rio - 2'000'000;
    const auto t_ns = sync.to_local_ns(probe_rio);
    ASSERT_TRUE(t_ns.has_value());
    const auto back = sync.to_remote_us(*t_ns);
    ASSERT_TRUE(back.has_value());
    EXPECT_NEAR(static_cast<double>(*back), static_cast<double>(probe_rio), 2.0);
}

TEST(ClockSyncTest, ManualResetClearsState) {
    ClockSync sync;
    Lcg          rng;
    uint64_t     rio = 10'000'000;
    const auto now = drive(sync, rng, rio, 5.0, 1e6, 0.0, 100, 1000);
    ASSERT_TRUE(sync.healthy(now));
    sync.reset();
    EXPECT_FALSE(sync.healthy(now));
    EXPECT_EQ(sync.samples(), 0u);
    EXPECT_DOUBLE_EQ(sync.offset_us(), 0.0);
}

}  // namespace gw
