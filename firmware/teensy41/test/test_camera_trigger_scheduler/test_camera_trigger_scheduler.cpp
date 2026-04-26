#include <unity.h>

#include "CameraTriggerScheduler.h"
#include "FirmwareConfig.h"
#include "Protocol.h"

using namespace posest::firmware;

namespace {

VioCompanionConfig makeVioCfg(bool led_enabled, bool tof_enabled,
                              std::uint32_t led_pulse_us,
                              std::uint32_t tof_offset_us,
                              std::uint32_t tof_divisor = 1u,
                              std::uint8_t vio_slot_index = 0u) {
    VioCompanionConfig cfg;
    cfg.led_enabled = led_enabled;
    cfg.vio_slot_index = vio_slot_index;
    cfg.led_pulse_width_us = led_pulse_us;
    cfg.tof_enabled = tof_enabled;
    cfg.tof_offset_after_flash_us = tof_offset_us;
    cfg.tof_divisor = tof_divisor;
    return cfg;
}

}  // namespace

// ---------- applyVioCompanion validation ----------

void test_apply_vio_companion_rejects_out_of_range_slot() {
    CameraTriggerScheduler s;
    auto cfg = makeVioCfg(true, true, 400u, 500u, 1u, /*slot=*/kMaxCameraSyncOutputs);
    const std::uint32_t flags = s.applyVioCompanion(cfg);
    TEST_ASSERT_TRUE((flags & kConfigAckInvalidVioConfig) != 0u);
}

void test_apply_vio_companion_rejects_zero_led_pulse() {
    CameraTriggerScheduler s;
    auto cfg = makeVioCfg(/*led=*/true, false, /*pulse=*/0u, /*offset=*/0u);
    const std::uint32_t flags = s.applyVioCompanion(cfg);
    TEST_ASSERT_TRUE((flags & kConfigAckInvalidVioConfig) != 0u);
}

void test_apply_vio_companion_rejects_multiplex_violation() {
    CameraTriggerScheduler s;
    // offset (300us) shorter than LED pulse (400us) — would let ToF ranging
    // start while the IR LED is still on.
    auto cfg = makeVioCfg(true, true, /*pulse=*/400u, /*offset=*/300u);
    const std::uint32_t flags = s.applyVioCompanion(cfg);
    TEST_ASSERT_TRUE((flags & kConfigAckVioMultiplexViolation) != 0u);
}

void test_apply_vio_companion_rejects_zero_divisor_when_tof_enabled() {
    CameraTriggerScheduler s;
    auto cfg = makeVioCfg(false, /*tof=*/true, 0u, 500u, /*divisor=*/0u);
    const std::uint32_t flags = s.applyVioCompanion(cfg);
    TEST_ASSERT_TRUE((flags & kConfigAckInvalidVioConfig) != 0u);
}

void test_apply_vio_companion_accepts_valid_config() {
    CameraTriggerScheduler s;
    auto cfg = makeVioCfg(/*led=*/true, /*tof=*/true,
                          /*pulse=*/400u, /*offset=*/500u, /*divisor=*/1u);
    TEST_ASSERT_EQUAL_UINT32(0u, s.applyVioCompanion(cfg));
}

// ---------- ToF deadline scheduling ----------
// led_enabled=false to avoid the LED-fall IntervalTimer racing the test.

void test_simulate_vio_rise_schedules_tof_deadline() {
    CameraTriggerScheduler s;
    auto cfg = makeVioCfg(/*led=*/false, /*tof=*/true,
                          /*pulse=*/0u, /*offset=*/500u, /*divisor=*/1u);
    TEST_ASSERT_EQUAL_UINT32(0u, s.applyVioCompanion(cfg));

    s.testHookSimulateVioRise(/*event_time_us=*/10'000u);

    TEST_ASSERT_TRUE(s.testHookTofStartPending());
    TEST_ASSERT_EQUAL_UINT32(10'500u, s.testHookTofStartDeadline());
    TEST_ASSERT_EQUAL_UINT32(1u, s.testHookTofPendingTriggerSequence());
}

void test_consume_tof_start_due_returns_false_before_deadline() {
    CameraTriggerScheduler s;
    auto cfg = makeVioCfg(false, true, 0u, 500u, 1u);
    s.applyVioCompanion(cfg);
    s.testHookSimulateVioRise(10'000u);

    std::uint32_t out_seq = 0u;
    TEST_ASSERT_FALSE(s.consumeToFStartDue(/*now=*/10'499u, out_seq));
    TEST_ASSERT_TRUE(s.testHookTofStartPending());
}

void test_consume_tof_start_due_returns_true_at_or_after_deadline() {
    CameraTriggerScheduler s;
    auto cfg = makeVioCfg(false, true, 0u, 500u, 1u);
    s.applyVioCompanion(cfg);
    s.testHookSimulateVioRise(10'000u);

    std::uint32_t out_seq = 0u;
    TEST_ASSERT_TRUE(s.consumeToFStartDue(/*now=*/10'500u, out_seq));
    TEST_ASSERT_EQUAL_UINT32(1u, out_seq);
    TEST_ASSERT_FALSE(s.testHookTofStartPending());
}

void test_tof_divisor_subsamples_deadlines() {
    CameraTriggerScheduler s;
    // divisor=2: deadline scheduled on rises 1, 3, 5, ... (counter is
    // incremented post-decision so the first rise schedules a deadline).
    auto cfg = makeVioCfg(false, true, 0u, 500u, /*divisor=*/2u);
    s.applyVioCompanion(cfg);

    s.testHookSimulateVioRise(1'000u);
    TEST_ASSERT_TRUE(s.testHookTofStartPending());
    // Drain the pending deadline so we can observe the next rise's effect.
    std::uint32_t out_seq = 0u;
    TEST_ASSERT_TRUE(s.consumeToFStartDue(1'500u, out_seq));

    s.testHookSimulateVioRise(2'000u);
    // Second rise should be skipped by divisor=2.
    TEST_ASSERT_FALSE(s.testHookTofStartPending());

    s.testHookSimulateVioRise(3'000u);
    TEST_ASSERT_TRUE(s.testHookTofStartPending());
    TEST_ASSERT_EQUAL_UINT32(3'500u, s.testHookTofStartDeadline());
}

void test_tof_disabled_does_not_schedule_deadline() {
    CameraTriggerScheduler s;
    auto cfg = makeVioCfg(false, /*tof=*/false, 0u, 0u, 1u);
    s.applyVioCompanion(cfg);
    s.testHookSimulateVioRise(1'000u);
    TEST_ASSERT_FALSE(s.testHookTofStartPending());
}

// ---------- GCD-based master period math ----------

void test_gcd_u32_basic_cases() {
    TEST_ASSERT_EQUAL_UINT32(12u, CameraTriggerScheduler::testHookGcdU32(36u, 24u));
    TEST_ASSERT_EQUAL_UINT32(7u, CameraTriggerScheduler::testHookGcdU32(7u, 0u));
    TEST_ASSERT_EQUAL_UINT32(1u, CameraTriggerScheduler::testHookGcdU32(13u, 7u));
}

void test_period_from_rate_rejects_invalid() {
    TEST_ASSERT_EQUAL_UINT32(0u, CameraTriggerScheduler::testHookPeriodFromRate(0.0));
    TEST_ASSERT_EQUAL_UINT32(0u, CameraTriggerScheduler::testHookPeriodFromRate(-30.0));
    // 1 MHz → period would be 1 us, which the validator rejects (< 2 us).
    TEST_ASSERT_EQUAL_UINT32(0u, CameraTriggerScheduler::testHookPeriodFromRate(1'000'000.0));
}

void test_period_from_rate_round_trips_30_and_120_hz() {
    TEST_ASSERT_EQUAL_UINT32(33'333u, CameraTriggerScheduler::testHookPeriodFromRate(30.0));
    TEST_ASSERT_EQUAL_UINT32(8'333u, CameraTriggerScheduler::testHookPeriodFromRate(120.0));
}

void test_master_period_for_30_and_120_hz_mix() {
    // 4× 30 Hz + 1× 120 Hz: master period is gcd(33333, 8333) = 1 us increment.
    // The relevant sanity check is that 33333 % gcd == 0 and 8333 % gcd == 0.
    const std::uint32_t a = CameraTriggerScheduler::testHookPeriodFromRate(30.0);
    const std::uint32_t b = CameraTriggerScheduler::testHookPeriodFromRate(120.0);
    const std::uint32_t g = CameraTriggerScheduler::testHookGcdU32(a, b);
    TEST_ASSERT_TRUE(g > 0u);
    TEST_ASSERT_EQUAL_UINT32(0u, a % g);
    TEST_ASSERT_EQUAL_UINT32(0u, b % g);
    // 33333 / gcd is a multiple of 4× 8333 / gcd within rounding (rate ratio 4:1).
    TEST_ASSERT_EQUAL_UINT32(a / g, 4u * (b / g) + 1u);  // 33333 = 4*8333 + 1 us rounding error
}

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;
    UNITY_BEGIN();
    RUN_TEST(test_apply_vio_companion_rejects_out_of_range_slot);
    RUN_TEST(test_apply_vio_companion_rejects_zero_led_pulse);
    RUN_TEST(test_apply_vio_companion_rejects_multiplex_violation);
    RUN_TEST(test_apply_vio_companion_rejects_zero_divisor_when_tof_enabled);
    RUN_TEST(test_apply_vio_companion_accepts_valid_config);
    RUN_TEST(test_simulate_vio_rise_schedules_tof_deadline);
    RUN_TEST(test_consume_tof_start_due_returns_false_before_deadline);
    RUN_TEST(test_consume_tof_start_due_returns_true_at_or_after_deadline);
    RUN_TEST(test_tof_divisor_subsamples_deadlines);
    RUN_TEST(test_tof_disabled_does_not_schedule_deadline);
    RUN_TEST(test_gcd_u32_basic_cases);
    RUN_TEST(test_period_from_rate_rejects_invalid);
    RUN_TEST(test_period_from_rate_round_trips_30_and_120_hz);
    RUN_TEST(test_master_period_for_30_and_120_hz_mix);
    return UNITY_END();
}
