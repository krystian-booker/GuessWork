#include "timebase.hpp"

#include <HardwareTimer.h>

#include "trigger_engine.hpp"

namespace {

volatile uint32_t g_time_high = 0;
HardwareTimer g_timer(TIM2);

uint32_t timer_clock_hz() {
    uint32_t hz = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) hz *= 2;
    return hz;
}

void on_rollover() { ++g_time_high; }
void on_compare() { gw_fw::trigger_timer_isr(); }

}  // namespace

namespace gw_fw::timebase {

void begin() {
    g_timer.pause();
    g_timer.setPrescaleFactor(timer_clock_hz() / 1'000'000u);
    g_timer.setOverflow(0xFFFFFFFFu, TICK_FORMAT);
    g_timer.setMode(1, TIMER_OUTPUT_COMPARE);
    g_timer.setCaptureCompare(1, 0xFFFFFFFFu, TICK_COMPARE_FORMAT);
    // Apply PSC/ARR while interrupts are still detached: refresh() creates an
    // update event that must not be mistaken for a real 32-bit rollover.
    g_timer.refresh();
    g_time_high = 0;
    g_timer.setCount(0, TICK_FORMAT);
    g_timer.attachInterrupt(on_rollover);
    g_timer.attachInterrupt(1, on_compare);
    g_timer.setInterruptPriority(5, 0);
    g_timer.resume();
    g_timer.pauseChannel(1);
}

uint32_t now_us32() { return TIM2->CNT; }

uint64_t now_us() {
    uint32_t high_a;
    uint32_t low;
    uint32_t high_b;
    do {
        high_a = g_time_high;
        low = TIM2->CNT;
        if ((TIM2->SR & TIM_SR_UIF) && low < 0x80000000u) ++high_a;
        high_b = g_time_high;
    } while (high_a != high_b && !(TIM2->SR & TIM_SR_UIF));
    return (static_cast<uint64_t>(high_a) << 32) | low;
}

void set_compare(uint32_t deadline_us) {
    const uint32_t now = TIM2->CNT;
    if (static_cast<int32_t>(deadline_us - now) < 5) deadline_us = now + 5;
    g_timer.setCaptureCompare(1, deadline_us, TICK_COMPARE_FORMAT);
    g_timer.resumeChannel(1);
}

void disable_compare() { g_timer.pauseChannel(1); }

}  // namespace gw_fw::timebase
