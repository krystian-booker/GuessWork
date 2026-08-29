#pragma once

#include <Arduino.h>

namespace gw_fw::board {

// MicoAir F405 V2 motor pads.  These are exposed to the host as stable
// logical outputs 1..6; the GPIO names never cross the USB protocol.
struct OutputPin {
    GPIO_TypeDef* port;
    uint16_t mask;
};

constexpr OutputPin kOutputs[] = {
    {GPIOB, GPIO_PIN_0},   // M1
    {GPIOB, GPIO_PIN_1},   // M2
    {GPIOA, GPIO_PIN_15},  // M3
    {GPIOB, GPIO_PIN_3},   // M4
    {GPIOB, GPIO_PIN_4},   // M5
    {GPIOB, GPIO_PIN_5},   // M6
};

constexpr PinName kImuSck     = PB_13;
constexpr PinName kImuMiso    = PC_2;
constexpr PinName kImuMosi    = PC_3;
constexpr PinName kAccelCs    = PC_13;
constexpr PinName kGyroCs     = PC_14;

// The camera optocoupler is wired OPTO_IN -> regulated 5 V and OPTO_GND ->
// M1..M6.  LOW is therefore the safe idle state; HIGH creates the Line0
// falling edge used as FrameStart.  Never change this polarity without also
// changing the documented camera wiring.
inline void initialise_outputs_idle() {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef init{};
    init.Mode = GPIO_MODE_OUTPUT_PP;
    init.Pull = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    init.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &init);
    GPIOA->BSRR = static_cast<uint32_t>(GPIO_PIN_15) << 16;

    init.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOB, &init);
    GPIOB->BSRR = static_cast<uint32_t>(init.Pin) << 16;
}

inline void set_output_mask_high(uint8_t output_mask) {
    uint32_t a = 0;
    uint32_t b = 0;
    for (size_t i = 0; i < 6; ++i) {
        if (!(output_mask & (1u << i))) continue;
        if (kOutputs[i].port == GPIOA) a |= kOutputs[i].mask;
        else b |= kOutputs[i].mask;
    }
    if (a) GPIOA->BSRR = a;
    if (b) GPIOB->BSRR = b;
}

inline void set_output_mask_low(uint8_t output_mask) {
    uint32_t a = 0;
    uint32_t b = 0;
    for (size_t i = 0; i < 6; ++i) {
        if (!(output_mask & (1u << i))) continue;
        if (kOutputs[i].port == GPIOA) a |= kOutputs[i].mask;
        else b |= kOutputs[i].mask;
    }
    if (a) GPIOA->BSRR = a << 16;
    if (b) GPIOB->BSRR = b << 16;
}

}  // namespace gw_fw::board

