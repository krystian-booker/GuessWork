#include <unity.h>

#include "Vl53l4cdToF.h"

using namespace posest::firmware;

namespace {

Vl53l4cdToF::Config makeEnabledConfig() {
    Vl53l4cdToF::Config cfg;
    cfg.enabled = true;
    cfg.i2c_address = 0x29;
    cfg.timing_budget_ms = 10;
    cfg.intermeasurement_period_ms = 20;
    return cfg;
}

}  // namespace

void test_force_idle_clears_state() {
    Vl53l4cdToF tof;
    tof.testHookForceIdle(makeEnabledConfig());
    TEST_ASSERT_FALSE(tof.isRanging());
    TEST_ASSERT_EQUAL_UINT32(0, tof.errorFlags());
    TEST_ASSERT_EQUAL_UINT32(0, tof.samplesEmitted());
    TEST_ASSERT_EQUAL_UINT32(0, tof.i2cFailures());
    TEST_ASSERT_EQUAL_UINT32(0, tof.overruns());
}

void test_start_ranging_flips_state_to_ranging() {
    Vl53l4cdToF tof;
    tof.testHookForceIdle(makeEnabledConfig());
    TEST_ASSERT_TRUE(tof.startRanging(42u, 1000u));
    TEST_ASSERT_TRUE(tof.isRanging());
    TEST_ASSERT_EQUAL_UINT32(0, tof.overruns());
}

void test_start_ranging_while_ranging_bumps_overrun() {
    Vl53l4cdToF tof;
    tof.testHookForceIdle(makeEnabledConfig());
    TEST_ASSERT_TRUE(tof.startRanging(1u, 1000u));
    TEST_ASSERT_FALSE(tof.startRanging(2u, 1100u));
    TEST_ASSERT_EQUAL_UINT32(1, tof.overruns());
    TEST_ASSERT_TRUE(tof.isRanging());
}

void test_poll_returns_false_until_result_injected() {
    Vl53l4cdToF tof;
    tof.testHookForceIdle(makeEnabledConfig());
    TEST_ASSERT_TRUE(tof.startRanging(7u, 1000u));
    ToFSamplePayload sample;
    TEST_ASSERT_FALSE(tof.poll(sample, 1100u));
    TEST_ASSERT_TRUE(tof.isRanging());
    TEST_ASSERT_EQUAL_UINT32(0, tof.samplesEmitted());
}

void test_poll_returns_injected_result_with_trigger_sequence() {
    Vl53l4cdToF tof;
    tof.testHookForceIdle(makeEnabledConfig());
    TEST_ASSERT_TRUE(tof.startRanging(99u, 1000u));
    // Inject distance=1234 mm, range_status=0 (valid), signal=8 MCPS, ambient=2 MCPS.
    // 9.7 fixed-point: 8 MCPS = 8 << 7 = 1024; 2 MCPS = 2 << 7 = 256.
    tof.testHookInjectResult(1234u, 0u, 1024u, 256u);

    ToFSamplePayload sample;
    TEST_ASSERT_TRUE(tof.poll(sample, 1500u));
    TEST_ASSERT_FALSE(tof.isRanging());
    TEST_ASSERT_EQUAL_UINT32(99u, sample.trigger_sequence);
    TEST_ASSERT_EQUAL_UINT32(1234u, sample.distance_mm);
    TEST_ASSERT_EQUAL_UINT32(0u, sample.range_status);
    TEST_ASSERT_EQUAL_UINT32(500u, sample.ranging_duration_us);
    TEST_ASSERT_EQUAL_UINT32(1500u, sample.teensy_time_us);
    // 1024 * 1000 / 128 = 8000 kcps; 256 * 1000 / 128 = 2000 kcps.
    TEST_ASSERT_EQUAL_UINT32(8000u, sample.signal_rate_kcps);
    TEST_ASSERT_EQUAL_UINT32(2000u, sample.ambient_rate_kcps);
    TEST_ASSERT_EQUAL_UINT32(1u, tof.samplesEmitted());
}

void test_poll_when_idle_returns_false() {
    Vl53l4cdToF tof;
    tof.testHookForceIdle(makeEnabledConfig());
    ToFSamplePayload sample;
    TEST_ASSERT_FALSE(tof.poll(sample, 100u));
}

void test_note_overrun_raises_counter_and_error_flags() {
    Vl53l4cdToF tof;
    tof.testHookForceIdle(makeEnabledConfig());
    TEST_ASSERT_EQUAL_UINT32(0, tof.errorFlags());
    tof.noteOverrun();
    tof.noteOverrun();
    TEST_ASSERT_EQUAL_UINT32(2, tof.overruns());
    TEST_ASSERT_TRUE(tof.errorFlags() != 0u);
}

void test_disabled_config_makes_start_ranging_a_noop() {
    Vl53l4cdToF tof;
    Vl53l4cdToF::Config cfg = makeEnabledConfig();
    cfg.enabled = false;
    tof.testHookForceIdle(cfg);
    TEST_ASSERT_FALSE(tof.startRanging(1u, 0u));
    TEST_ASSERT_FALSE(tof.isRanging());
}

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;
    UNITY_BEGIN();
    RUN_TEST(test_force_idle_clears_state);
    RUN_TEST(test_start_ranging_flips_state_to_ranging);
    RUN_TEST(test_start_ranging_while_ranging_bumps_overrun);
    RUN_TEST(test_poll_returns_false_until_result_injected);
    RUN_TEST(test_poll_returns_injected_result_with_trigger_sequence);
    RUN_TEST(test_poll_when_idle_returns_false);
    RUN_TEST(test_note_overrun_raises_counter_and_error_flags);
    RUN_TEST(test_disabled_config_makes_start_ranging_a_noop);
    return UNITY_END();
}
