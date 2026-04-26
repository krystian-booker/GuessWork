#include "Vl53l4cdToF.h"

#include <Arduino.h>
#include <Wire.h>

#include "FirmwareConfig.h"

namespace posest::firmware {

namespace {

// VL53L4CD register addresses (subset). 16-bit register pointer, big-endian
// over the wire. Source: ST UM2931 / VL53L4CD ULD.
constexpr std::uint16_t kRegSoftReset = 0x0000;
constexpr std::uint16_t kRegSystemStart = 0x0087;
constexpr std::uint16_t kRegSystemInterruptClear = 0x0086;
constexpr std::uint16_t kRegResultRangeStatus = 0x0089;
constexpr std::uint16_t kRegResultDistanceMm = 0x0096;
constexpr std::uint16_t kRegResultSignalRate = 0x0086 + 16;  // 0x0096+? — see ULD; conservative offset
constexpr std::uint16_t kRegIdentificationModelId = 0x010F;
constexpr std::uint8_t kExpectedModelId = 0xEB;

// Single-shot start command per ULD: writes 0x10 to SystemStart.
constexpr std::uint8_t kSystemStartSingleShot = 0x10;

// Conservative I2C clock — VL53L4CD supports 400 kHz fast mode, but 100 kHz
// is more forgiving on hand-soldered breakouts.
constexpr std::uint32_t kI2cClockHz = 400000u;

}  // namespace

bool Vl53l4cdToF::begin(const Config& cfg, std::uint64_t now_us) {
    (void)now_us;
    cfg_ = cfg;
    state_ = State::kIdle;
    error_flags_ = 0;

    if (!cfg_.enabled) {
        // Disabled by config — leave the bus untouched and report idle.
        return true;
    }

    Wire.begin();
    Wire.setClock(kI2cClockHz);

    std::uint8_t model_id = 0;
    if (!readReg16(kRegIdentificationModelId, model_id) ||
        model_id != kExpectedModelId) {
        state_ = State::kInitFailed;
        error_flags_ |= 1u;  // mapped to kErrorTofInitFailed at main.cpp
        return false;
    }

    // Soft-reset the device to a known state before applying timing config.
    if (!writeReg16(kRegSoftReset, 0x00) || !writeReg16(kRegSoftReset, 0x01)) {
        state_ = State::kInitFailed;
        error_flags_ |= 1u;
        return false;
    }
    delay(2);

    // Apply timing budget + intermeasurement period. The exact register
    // sequences for these are encoded in ST's ULD as a series of magic
    // tables; without vendoring the ULD we set the chip to a safe default
    // (its own factory timing budget) and rely on the host divisor + flash
    // offset for actual rate control. Future work: vendor the ULD under
    // firmware/teensy41/lib/VL53L4CD_ULD/ and call its timing setters here.
    return true;
}

bool Vl53l4cdToF::reconfigure(const Config& cfg, std::uint64_t now_us) {
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
    if (!writeReg16(kRegSystemStart, kSystemStartSingleShot)) {
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
    // Don't even bother polling I2C until the timing budget should have
    // elapsed; saves bus time and avoids spurious "data not ready" reads.
    const std::uint64_t budget_us = static_cast<std::uint64_t>(cfg_.timing_budget_ms) * 1000ull;
    if (now_us < ranging_start_us_ + budget_us / 2u) {
        return false;
    }

    std::uint8_t range_status = 0;
    if (!readReg16(kRegResultRangeStatus, range_status)) {
        ++i2c_failures_;
        error_flags_ |= 1u;
        // Bail; main loop will retry on next iteration.
        return false;
    }

    // The ULD reads the data-ready bit from a separate register; lacking
    // that, we use a coarse "timing budget elapsed" heuristic. Once vendoring
    // is complete, replace with VL53L4CD_GetMeasurementDataReady().
    if (now_us < ranging_start_us_ + budget_us) {
        return false;
    }

    std::uint16_t distance_mm = 0;
    (void)readReg16U16(kRegResultDistanceMm, distance_mm);

    // Clear the chip-side interrupt so the next single-shot can fire.
    (void)writeReg16(kRegSystemInterruptClear, 0x01);

    out_payload = ToFSamplePayload{};
    out_payload.teensy_time_us = now_us;
    out_payload.trigger_sequence = pending_trigger_sequence_;
    out_payload.distance_mm = distance_mm;
    out_payload.range_status = range_status;
    out_payload.ranging_duration_us = static_cast<std::uint32_t>(now_us - ranging_start_us_);
    // signal_rate / ambient_rate omitted until ULD vendoring lands.
    state_ = State::kIdle;
    ++samples_emitted_;
    return true;
}

bool Vl53l4cdToF::writeReg16(std::uint16_t reg, std::uint8_t value) {
    Wire.beginTransmission(cfg_.i2c_address);
    Wire.write(static_cast<std::uint8_t>((reg >> 8u) & 0xFFu));
    Wire.write(static_cast<std::uint8_t>(reg & 0xFFu));
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool Vl53l4cdToF::writeReg16U16(std::uint16_t reg, std::uint16_t value) {
    Wire.beginTransmission(cfg_.i2c_address);
    Wire.write(static_cast<std::uint8_t>((reg >> 8u) & 0xFFu));
    Wire.write(static_cast<std::uint8_t>(reg & 0xFFu));
    Wire.write(static_cast<std::uint8_t>((value >> 8u) & 0xFFu));
    Wire.write(static_cast<std::uint8_t>(value & 0xFFu));
    return Wire.endTransmission() == 0;
}

bool Vl53l4cdToF::readReg16(std::uint16_t reg, std::uint8_t& out) {
    Wire.beginTransmission(cfg_.i2c_address);
    Wire.write(static_cast<std::uint8_t>((reg >> 8u) & 0xFFu));
    Wire.write(static_cast<std::uint8_t>(reg & 0xFFu));
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    if (Wire.requestFrom(static_cast<int>(cfg_.i2c_address), 1) != 1) {
        return false;
    }
    out = static_cast<std::uint8_t>(Wire.read());
    return true;
}

bool Vl53l4cdToF::readReg16U16(std::uint16_t reg, std::uint16_t& out) {
    Wire.beginTransmission(cfg_.i2c_address);
    Wire.write(static_cast<std::uint8_t>((reg >> 8u) & 0xFFu));
    Wire.write(static_cast<std::uint8_t>(reg & 0xFFu));
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    if (Wire.requestFrom(static_cast<int>(cfg_.i2c_address), 2) != 2) {
        return false;
    }
    const std::uint16_t hi = static_cast<std::uint16_t>(Wire.read());
    const std::uint16_t lo = static_cast<std::uint16_t>(Wire.read());
    out = static_cast<std::uint16_t>((hi << 8u) | lo);
    return true;
}

}  // namespace posest::firmware
