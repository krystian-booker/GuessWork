#pragma once

#include <Arduino.h>

namespace gw_fw {

// Wrap-extending 64-bit microsecond clock built on micros(), which wraps at
// 2^32 µs ≈ 71.6 minutes — longer than a practice session, so the 32-bit
// value cannot be used as a timestamp ground truth on its own.
//
// Safe to call from ISRs and the main loop: the read-modify-write of the
// wrap state runs with IRQs masked (a handful of cycles). The rollover is
// only observed when somebody calls this, so loop() must poll it at least
// once per wrap period even when no triggers are armed.
inline uint64_t now_us64() {
    static uint32_t hi      = 0;
    static uint32_t last_lo = 0;
    uint32_t primask;
    __asm__ volatile("mrs %0, primask" : "=r"(primask));
    __disable_irq();
    const uint32_t lo = micros();
    if (lo < last_lo) ++hi;
    last_lo = lo;
    const uint64_t t = (static_cast<uint64_t>(hi) << 32) | lo;
    if ((primask & 1u) == 0) __enable_irq();
    return t;
}

}  // namespace gw_fw
