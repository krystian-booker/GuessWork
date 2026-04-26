#include <unity.h>

#include "CanBridge.h"

using namespace posest::firmware;

namespace {

// Helper to drive the time-sync filter without standing up FlexCAN.
void feed(CanBridge& bridge, std::uint64_t rio_time_us, std::uint64_t now_us) {
    bridge.testHookFeedRioTimeSync(rio_time_us, now_us);
}

}  // namespace

void test_bootstrap_accepts_first_three_unconditionally() {
    CanBridge bridge;
    CanBusConfig config;
    config.enabled = false;  // skip FlexCAN init
    bridge.begin(config);

    // Three samples with offsets that would otherwise be rejected as a
    // step > kRioMaxStepUs once bootstrap completes. All must be accepted.
    feed(bridge, /*rio_time_us=*/1'000, /*now_us=*/1'000'000);  // offset 999000
    feed(bridge, /*rio_time_us=*/2'000, /*now_us=*/2'000'000);  // offset 1998000
    feed(bridge, /*rio_time_us=*/3'000, /*now_us=*/3'000'000);  // offset 2997000

    TEST_ASSERT_TRUE(bridge.rioOffsetValid());
    TEST_ASSERT_EQUAL_UINT32(0, bridge.rioConsecutiveRejections());
}

void test_steady_state_rejects_large_step() {
    CanBridge bridge;
    CanBusConfig config;
    config.enabled = false;
    bridge.begin(config);

    // Bootstrap with small consistent offset of 1000 us.
    feed(bridge, 1'000, 2'000);
    feed(bridge, 2'000, 3'000);
    feed(bridge, 3'000, 4'000);

    const std::int64_t baseline = bridge.rioOffsetUs();

    // Now feed a sample whose offset jumps by 100 ms. Should be rejected.
    feed(bridge, 4'000, 104'000);

    TEST_ASSERT_EQUAL_INT64(baseline, bridge.rioOffsetUs());
    TEST_ASSERT_EQUAL_UINT32(1, bridge.rioConsecutiveRejections());
}

void test_rio_reboot_triggers_resync() {
    CanBridge bridge;
    CanBusConfig config;
    config.enabled = false;
    bridge.begin(config);

    // Bootstrap with a steady offset of 1000 us (RIO ahead by ~1ms relative
    // to its first ping). Three samples to complete bootstrap.
    feed(bridge, 1'000, 2'000);
    feed(bridge, 2'000, 3'000);
    feed(bridge, 3'000, 4'000);

    // A few more in-range samples to confirm steady state.
    for (std::uint64_t i = 0; i < 5; ++i) {
        feed(bridge, 4'000 + i * 1'000, 5'000 + i * 1'000);
    }
    TEST_ASSERT_TRUE(bridge.rioOffsetValid());
    const std::int64_t pre_reboot_offset = bridge.rioOffsetUs();

    // Simulate RIO reboot: rio_time_us drops back to ~0 while teensy's
    // monotonic now_us keeps climbing. Offset jumps by tens of seconds —
    // every ping fails the gate.
    std::uint64_t now = 100'000'000;  // 100 seconds of teensy uptime
    for (std::uint32_t i = 0; i < 31; ++i) {
        feed(bridge, /*rio_time_us=*/1'000 + i * 1'000, /*now_us=*/now);
        now += 10'000;
    }
    // 31 consecutive rejects so far; offset should still match pre-reboot.
    TEST_ASSERT_EQUAL_UINT32(31, bridge.rioConsecutiveRejections());
    TEST_ASSERT_EQUAL_INT64(pre_reboot_offset, bridge.rioOffsetUs());

    // 32nd reject triggers re-bootstrap and re-anchors on this sample.
    feed(bridge, /*rio_time_us=*/32'000, /*now_us=*/now);
    TEST_ASSERT_TRUE(bridge.rioOffsetValid());
    TEST_ASSERT_EQUAL_UINT32(0, bridge.rioConsecutiveRejections());
    // New offset should reflect the post-reboot epoch (now - 32_000 ~= now).
    TEST_ASSERT_INT64_WITHIN(
        100, static_cast<std::int64_t>(now) - 32'000, bridge.rioOffsetUs());
}

void test_stale_clears_validity() {
    CanBridge bridge;
    CanBusConfig config;
    config.enabled = false;
    config.rio_offset_stale_us = 1'000'000;  // 1 second
    bridge.begin(config);

    feed(bridge, 1'000, 2'000);
    feed(bridge, 2'000, 3'000);
    feed(bridge, 3'000, 4'000);
    TEST_ASSERT_TRUE(bridge.rioOffsetValid());

    // Advance now past the stale window with no new pings.
    bridge.testHookCheckStaleness(/*now_us=*/2'000'000);  // ~2s after last ping
    TEST_ASSERT_FALSE(bridge.rioOffsetValid());
}

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;
    UNITY_BEGIN();
    RUN_TEST(test_bootstrap_accepts_first_three_unconditionally);
    RUN_TEST(test_steady_state_rejects_large_step);
    RUN_TEST(test_rio_reboot_triggers_resync);
    RUN_TEST(test_stale_clears_validity);
    return UNITY_END();
}
