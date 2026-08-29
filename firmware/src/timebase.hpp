#pragma once

#include <Arduino.h>
#include <stdint.h>

namespace gw_fw::timebase {

void begin();
uint64_t now_us();
uint32_t now_us32();
void set_compare(uint32_t deadline_us);
void disable_compare();

}  // namespace gw_fw::timebase

