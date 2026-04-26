#include "Vl53l4cdToF.h"

#include <Arduino.h>
#include <Wire.h>

#include "FirmwareConfig.h"

namespace posest::firmware {

namespace {

// VL53L4CD register addresses (subset). 16-bit register pointer, big-endian
// over the wire. Source: UM2931 (VL53L4CD application note) and the public
// register map distributed with STSW-IMG026.
constexpr std::uint16_t kRegSoftReset = 0x0000;
constexpr std::uint16_t kRegGpioHvMuxCtrl = 0x0030;
constexpr std::uint16_t kRegGpioTioHvStatus = 0x0031;
constexpr std::uint16_t kRegSystemInterruptClear = 0x0086;
constexpr std::uint16_t kRegSystemStart = 0x0087;
constexpr std::uint16_t kRegFirmwareSystemStatus = 0x00E5;
constexpr std::uint16_t kRegIdentificationModelId = 0x010F;

// Result block. Each field is at a fixed offset from the chip's reporting
// origin per UM2931 §3.4.7.
constexpr std::uint16_t kRegResultRangeStatus = 0x0089;
constexpr std::uint16_t kRegResultSpadCount = 0x008C;       // 16-bit, 8.8 fixed-point spad count
constexpr std::uint16_t kRegResultSignalRate = 0x008E;       // 16-bit, 9.7 fixed-point MCPS
constexpr std::uint16_t kRegResultAmbientRate = 0x0090;      // 16-bit, 9.7 fixed-point MCPS
constexpr std::uint16_t kRegResultSigma = 0x0092;            // 16-bit, 14.2 fixed-point mm
constexpr std::uint16_t kRegResultDistanceMm = 0x0096;       // 16-bit mm

constexpr std::uint16_t kExpectedModelId = 0xEBAA;  // chip ID + model type for VL53L4CD

// Single-shot start command per UM2931 §3.4.6: writes 0x10 to SystemStart.
constexpr std::uint8_t kSystemStartSingleShot = 0x10;
constexpr std::uint8_t kSystemStartContinuous = 0x40;
constexpr std::uint8_t kSystemStartStop = 0x00;

// VL53L4CD I2C clock — datasheet supports up to 1 MHz fast-mode-plus, but
// UM2931 example code defaults to 400 kHz. Stick with fast mode for
// compatibility with hand-soldered breakouts and shared bus loads.
constexpr std::uint32_t kI2cClockHz = 400000u;

// MCPS-to-kcps conversion: result registers carry the rate in 9.7 fixed-point
// MCPS. Convert to kcps by multiplying by 1000 then shifting right 7. The
// product fits in 32 bits for any signal rate well below the chip's saturation.
constexpr std::uint16_t mcps9_7_to_kcps(std::uint16_t raw) {
    return static_cast<std::uint16_t>(
        (static_cast<std::uint32_t>(raw) * 1000u) >> 7u);
}

// VL53L4CD default config block per UM2931 §3.3 (Table 3) and the public
// register map distributed with STSW-IMG026. These 91 bytes program registers
// 0x002D through 0x0087. Most positions are flagged "not user-modifiable" in
// the application note; the values below match the chip's documented default
// boot configuration. Sigma threshold defaults to 90 mm (regs 0x0064/0x0065),
// the GPIO interrupt is configured for new-sample-ready (reg 0x0046 = 0x20),
// and the trailing system_start byte (reg 0x0087) is left at 0x00 so the
// chip stays idle until startSingleShot() is called.
constexpr std::uint8_t kDefaultConfig[] = {
    0x12, 0x00, 0x00, 0x11, 0x02, 0x00, 0x02, 0x08,
    0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x14,
    0x21, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00,
    0xC8, 0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08,
    0x00, 0x00, 0x01, 0xCC, 0x07, 0x01, 0xF1, 0x05,
    0x00, 0xA0, 0x00, 0x80, 0x08, 0x38, 0x00, 0x00,
    0x00, 0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x07, 0x05, 0x06, 0x06,
    0x00, 0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00,
    0x00, 0x01, 0x00,
};
static_assert(sizeof(kDefaultConfig) == (0x0087 - 0x002D + 1),
              "VL53L4CD default config block must cover registers 0x002D..0x0087");

constexpr std::uint16_t kDefaultConfigStart = 0x002D;

// Boot watchdog. Datasheet says the chip's firmware boots in <= ~1.2 ms after
// a soft reset; 100 ms is generous. Warm-up wait is 1 s — a chip on the bus
// and configured properly fires in well under that.
constexpr std::uint32_t kBootTimeoutMs = 100;
constexpr std::uint32_t kWarmupTimeoutMs = 1000;

}  // namespace

bool Vl53l4cdToF::begin(const Config& cfg, std::uint64_t now_us) {
    (void)now_us;
    cfg_ = cfg;
    state_ = State::kIdle;
    error_flags_ = 0;
    samples_emitted_ = 0;
    i2c_failures_ = 0;
    overruns_ = 0;
    test_mode_ = false;
    test_pending_result_ = false;

    if (!cfg_.enabled) {
        // Disabled by config — leave the bus untouched and report idle.
        return true;
    }

    Wire.begin();
    Wire.setClock(kI2cClockHz);

    if (!sensorInit()) {
        state_ = State::kInitFailed;
        error_flags_ |= 1u;  // mapped to kErrorTofInitFailed at main.cpp
        return false;
    }
    return true;
}

bool Vl53l4cdToF::reconfigure(const Config& cfg, std::uint64_t now_us) {
    // The chip's timing-budget math requires values cached during sensorInit
    // (when we eventually vendor the proper ULD), so a full re-init is the
    // safest path on reconfigure. Matches the prior driver's behavior.
    return begin(cfg, now_us);
}

bool Vl53l4cdToF::startRanging(std::uint32_t trigger_sequence, std::uint64_t now_us) {
    if (state_ == State::kInitFailed || !cfg_.enabled) {
        return false;
    }
    if (state_ == State::kRanging) {
        ++overruns_;
        return false;
    }
    if (!test_mode_ && !startSingleShot()) {
        ++i2c_failures_;
        error_flags_ |= 1u;  // I2C failure — main maps to kErrorTofI2cFailure
        return false;
    }
    state_ = State::kRanging;
    ranging_start_us_ = now_us;
    pending_trigger_sequence_ = trigger_sequence;
    return true;
}

bool Vl53l4cdToF::poll(ToFSamplePayload& out_payload, std::uint64_t now_us) {
    if (state_ != State::kRanging) {
        return false;
    }

    Result result{};
    if (test_mode_) {
        if (!test_pending_result_) {
            return false;
        }
        result = test_result_;
        test_pending_result_ = false;
    } else {
        bool ready = false;
        if (!checkDataReady(ready)) {
            ++i2c_failures_;
            error_flags_ |= 1u;
            return false;
        }
        if (!ready) {
            return false;
        }
        if (!readResult(result)) {
            ++i2c_failures_;
            error_flags_ |= 1u;
            return false;
        }
        // Clear the chip-side interrupt so the next single-shot can fire. A
        // failure here doesn't invalidate the sample we just read.
        if (!clearInterrupt()) {
            ++i2c_failures_;
            error_flags_ |= 1u;
        }
    }

    out_payload = ToFSamplePayload{};
    out_payload.teensy_time_us = now_us;
    out_payload.trigger_sequence = pending_trigger_sequence_;
    out_payload.distance_mm = result.distance_mm;
    out_payload.range_status = result.range_status;
    out_payload.ranging_duration_us = static_cast<std::uint32_t>(now_us - ranging_start_us_);
    out_payload.signal_rate_kcps = mcps9_7_to_kcps(result.signal_rate_kcps);
    out_payload.ambient_rate_kcps = mcps9_7_to_kcps(result.ambient_rate_kcps);
    state_ = State::kIdle;
    ++samples_emitted_;
    return true;
}

bool Vl53l4cdToF::sensorInit() {
    // Confirm the chip is on the bus before we touch it. Model id is a fixed
    // 16-bit value at register 0x010F-0x0110 for VL53L4CD.
    std::uint16_t model_id = 0;
    if (!readU16(kRegIdentificationModelId, model_id) ||
        model_id != kExpectedModelId) {
        return false;
    }

    if (!waitForBoot()) {
        return false;
    }

    // Program the documented default register block (0x002D..0x0087). The
    // chip is left in idle (system_start = 0x00 at the trailing byte).
    if (!writeBytes(kDefaultConfigStart, kDefaultConfig, sizeof(kDefaultConfig))) {
        return false;
    }

    // UM2931 §3.3 prescribes a one-shot warm-up ranging immediately after
    // init: start ranging, wait for the first interrupt, clear it, then stop.
    // Without this the first user-driven single-shot can return invalid data.
    if (!writeU8(kRegSystemStart, kSystemStartContinuous)) {
        return false;
    }
    const std::uint32_t deadline = millis() + kWarmupTimeoutMs;
    bool ready = false;
    while (!ready) {
        if (static_cast<std::int32_t>(millis() - deadline) >= 0) {
            return false;
        }
        if (!checkDataReady(ready)) {
            return false;
        }
        delay(1);
    }
    if (!clearInterrupt() || !writeU8(kRegSystemStart, kSystemStartStop)) {
        return false;
    }
    return true;
}

bool Vl53l4cdToF::waitForBoot() {
    const std::uint32_t deadline = millis() + kBootTimeoutMs;
    while (true) {
        std::uint8_t status = 0;
        if (readU8(kRegFirmwareSystemStatus, status) && status == 0x03) {
            return true;
        }
        if (static_cast<std::int32_t>(millis() - deadline) >= 0) {
            return false;
        }
        delay(1);
    }
}

bool Vl53l4cdToF::startSingleShot() {
    return writeU8(kRegSystemStart, kSystemStartSingleShot);
}

bool Vl53l4cdToF::checkDataReady(bool& ready) {
    // Per UM2931 §3.4.4: read the GPIO HV mux polarity once, then poll the
    // GPIO TIO HV status register; data is ready when bit 0 of TIO status
    // matches the polarity bit (bit 4 of HV mux ctrl).
    std::uint8_t hv_mux = 0;
    std::uint8_t tio_status = 0;
    if (!readU8(kRegGpioHvMuxCtrl, hv_mux) ||
        !readU8(kRegGpioTioHvStatus, tio_status)) {
        return false;
    }
    const std::uint8_t int_polarity = (hv_mux & 0x10u) ? 0u : 1u;
    ready = ((tio_status & 0x01u) == int_polarity);
    return true;
}

bool Vl53l4cdToF::readResult(Result& out) {
    std::uint8_t status = 0;
    std::uint16_t spad = 0;
    std::uint16_t signal = 0;
    std::uint16_t ambient = 0;
    std::uint16_t sigma = 0;
    std::uint16_t distance = 0;
    if (!readU8(kRegResultRangeStatus, status) ||
        !readU16(kRegResultSpadCount, spad) ||
        !readU16(kRegResultSignalRate, signal) ||
        !readU16(kRegResultAmbientRate, ambient) ||
        !readU16(kRegResultSigma, sigma) ||
        !readU16(kRegResultDistanceMm, distance)) {
        return false;
    }
    out.range_status = status;
    // SPAD count is in 8.8 fixed-point; the integer count lives in the high byte.
    out.spad_count = static_cast<std::uint16_t>(spad >> 8u);
    out.signal_rate_kcps = signal;       // 9.7 fixed-point MCPS, host converts
    out.ambient_rate_kcps = ambient;     // 9.7 fixed-point MCPS
    out.sigma_mm = static_cast<std::uint16_t>(sigma >> 2u);  // 14.2 fixed-point
    out.distance_mm = distance;
    return true;
}

bool Vl53l4cdToF::clearInterrupt() {
    return writeU8(kRegSystemInterruptClear, 0x01);
}

bool Vl53l4cdToF::writeBytes(std::uint16_t reg, const std::uint8_t* data, std::size_t len) {
    Wire.beginTransmission(cfg_.i2c_address);
    Wire.write(static_cast<std::uint8_t>((reg >> 8u) & 0xFFu));
    Wire.write(static_cast<std::uint8_t>(reg & 0xFFu));
    for (std::size_t i = 0; i < len; ++i) {
        // Wire's transmit buffer is bounded (32 B on AVR, 256 B on Teensy 4.x).
        // The 91-byte default config fits; guard explicitly so a future
        // larger write doesn't silently truncate.
        if (Wire.write(data[i]) == 0u) {
            return false;
        }
    }
    return Wire.endTransmission() == 0;
}

bool Vl53l4cdToF::readBytes(std::uint16_t reg, std::uint8_t* out, std::size_t len) {
    Wire.beginTransmission(cfg_.i2c_address);
    Wire.write(static_cast<std::uint8_t>((reg >> 8u) & 0xFFu));
    Wire.write(static_cast<std::uint8_t>(reg & 0xFFu));
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    const std::uint8_t got = Wire.requestFrom(
        static_cast<int>(cfg_.i2c_address), static_cast<int>(len));
    if (got != len) {
        return false;
    }
    for (std::size_t i = 0; i < len; ++i) {
        out[i] = static_cast<std::uint8_t>(Wire.read());
    }
    return true;
}

bool Vl53l4cdToF::writeU8(std::uint16_t reg, std::uint8_t value) {
    return writeBytes(reg, &value, 1);
}

bool Vl53l4cdToF::readU8(std::uint16_t reg, std::uint8_t& out) {
    return readBytes(reg, &out, 1);
}

bool Vl53l4cdToF::readU16(std::uint16_t reg, std::uint16_t& out) {
    std::uint8_t buf[2] = {0, 0};
    if (!readBytes(reg, buf, sizeof(buf))) {
        return false;
    }
    out = static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(buf[0]) << 8u) | buf[1]);
    return true;
}

}  // namespace posest::firmware
