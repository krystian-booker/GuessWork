#pragma once

#include <stdint.h>

#include "sync_controller_protocol.h"

namespace gw_fw {

struct TriggerQueueEvent {
    uint8_t slot;
    uint32_t index;
    uint64_t t_us;
};

void trigger_begin();
gw_sync::AckStatus trigger_set_config(const gw_sync::GroupConfig* groups,
                                      uint8_t count);
gw_sync::AckStatus trigger_arm();
void trigger_stop();
gw_sync::AckStatus trigger_test_output(uint8_t logical_output);
bool trigger_pop(TriggerQueueEvent& event);
bool trigger_is_armed();
uint32_t trigger_drop_count();

// Called only by the TIM2 ISR.
void trigger_timer_isr();

}  // namespace gw_fw

