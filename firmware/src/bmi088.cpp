#include "bmi088.hpp"

#include <Arduino.h>
#include <SPI.h>
#include <math.h>

#include "board.hpp"
#include "timebase.hpp"

namespace gw_fw {
namespace {

SPIClass g_spi(board::kImuMosi, board::kImuMiso, board::kImuSck);
SPISettings g_settings(10'000'000, MSBFIRST, SPI_MODE0);

void select(PinName pin) { digitalWrite(pin, LOW); }
void deselect(PinName pin) { digitalWrite(pin, HIGH); }

uint8_t accel_read(uint8_t reg) {
    g_spi.beginTransaction(g_settings);
    select(board::kAccelCs);
    g_spi.transfer(reg | 0x80u);
    g_spi.transfer(0x00);
    const uint8_t value = g_spi.transfer(0x00);
    deselect(board::kAccelCs);
    g_spi.endTransaction();
    return value;
}

void accel_read_many(uint8_t reg, uint8_t* out, size_t count) {
    g_spi.beginTransaction(g_settings);
    select(board::kAccelCs);
    g_spi.transfer(reg | 0x80u);
    g_spi.transfer(0x00);
    for (size_t i = 0; i < count; ++i) out[i] = g_spi.transfer(0x00);
    deselect(board::kAccelCs);
    g_spi.endTransaction();
}

void accel_write(uint8_t reg, uint8_t value) {
    g_spi.beginTransaction(g_settings);
    select(board::kAccelCs);
    g_spi.transfer(reg & 0x7Fu);
    g_spi.transfer(value);
    deselect(board::kAccelCs);
    g_spi.endTransaction();
    delayMicroseconds(5);
}

uint8_t gyro_read(uint8_t reg) {
    g_spi.beginTransaction(g_settings);
    select(board::kGyroCs);
    g_spi.transfer(reg | 0x80u);
    const uint8_t value = g_spi.transfer(0x00);
    deselect(board::kGyroCs);
    g_spi.endTransaction();
    return value;
}

void gyro_read_many(uint8_t reg, uint8_t* out, size_t count) {
    g_spi.beginTransaction(g_settings);
    select(board::kGyroCs);
    g_spi.transfer(reg | 0x80u);
    for (size_t i = 0; i < count; ++i) out[i] = g_spi.transfer(0x00);
    deselect(board::kGyroCs);
    g_spi.endTransaction();
}

void gyro_write(uint8_t reg, uint8_t value) {
    g_spi.beginTransaction(g_settings);
    select(board::kGyroCs);
    g_spi.transfer(reg & 0x7Fu);
    g_spi.transfer(value);
    deselect(board::kGyroCs);
    g_spi.endTransaction();
    delayMicroseconds(5);
}

int16_t i16(const uint8_t* bytes) {
    return static_cast<int16_t>(static_cast<uint16_t>(bytes[0]) |
                                (static_cast<uint16_t>(bytes[1]) << 8));
}

void rotate_to_board(float sensor_x, float sensor_y, float sensor_z,
                     float* board_xyz) {
    // Official MicoAir ArduPilot hwdef: ROTATION_ROLL_180_YAW_270.
    board_xyz[0] = -sensor_y;
    board_xyz[1] = -sensor_x;
    board_xyz[2] = -sensor_z;
}

}  // namespace

bool Bmi088::begin() {
    pinMode(board::kAccelCs, OUTPUT);
    pinMode(board::kGyroCs, OUTPUT);
    deselect(board::kAccelCs);
    deselect(board::kGyroCs);
    g_spi.begin();

    // The accelerometer always starts in I2C mode. A CS rising edge from a
    // dummy read selects SPI until the next POR or soft reset.
    (void)accel_read(0x00);
    accel_write(0x7E, 0xB6);
    gyro_write(0x14, 0xB6);
    delay(50);
    // Soft reset returns the accelerometer interface to I2C mode, so select
    // SPI again before performing the real chip-ID read.
    (void)accel_read(0x00);
    if (accel_read(0x00) != 0x1E || gyro_read(0x00) != 0x0F) {
        ok_ = false;
        return false;
    }

    // Match Bosch's reference power-mode sequence: active configuration,
    // delay, sensor enable, then allow the analog path to settle.
    accel_write(0x7C, 0x00);  // active power mode
    delay(5);
    accel_write(0x7D, 0x04);  // accelerometer enabled
    delay(50);
    accel_write(0x40, 0x8A);  // 400 Hz, OSR4 (40 Hz cutoff)
    accel_write(0x41, 0x03);  // +/-24 g
    gyro_write(0x11, 0x00);   // normal mode
    delay(30);
    gyro_write(0x0F, 0x00);   // +/-2000 dps
    gyro_write(0x10, 0x03);   // 400 Hz, 47 Hz bandwidth

    ok_ = accel_read(0x40) == 0x8A && accel_read(0x41) == 0x03 &&
          accel_read(0x7C) == 0x00 && accel_read(0x7D) == 0x04 &&
          gyro_read(0x0F) == 0x00 && gyro_read(0x10) == 0x03 &&
          gyro_read(0x11) == 0x00;
    if (!ok_) return false;
    next_sample_us_ = timebase::now_us32() + 2500;
    return true;
}

bool Bmi088::poll(gw_sync::ImuRecord& sample) {
    if (!ok_) return false;
    const uint32_t now = timebase::now_us32();
    if (static_cast<int32_t>(now - next_sample_us_) < 0) return false;
    if (static_cast<int32_t>(now - next_sample_us_) >= 2500) ++drop_count_;
    do {
        next_sample_us_ += 2500;
    } while (static_cast<int32_t>(now - next_sample_us_) >= 0);

    uint8_t accel_raw[6];
    uint8_t gyro_raw[6];
    const uint64_t before = timebase::now_us();
    accel_read_many(0x12, accel_raw, sizeof(accel_raw));
    gyro_read_many(0x02, gyro_raw, sizeof(gyro_raw));
    const uint64_t after = timebase::now_us();
    sample.t_us = before + (after - before) / 2;

    constexpr float kAccelScale = 24.0f * 9.80665f / 32768.0f;
    constexpr float kGyroScale = 2000.0f * static_cast<float>(M_PI) /
                                  (180.0f * 32768.0f);
    rotate_to_board(i16(accel_raw) * kAccelScale,
                    i16(accel_raw + 2) * kAccelScale,
                    i16(accel_raw + 4) * kAccelScale, sample.accel);
    rotate_to_board(i16(gyro_raw) * kGyroScale,
                    i16(gyro_raw + 2) * kGyroScale,
                    i16(gyro_raw + 4) * kGyroScale, sample.gyro);
    ++sample_count_;
    return true;
}

}  // namespace gw_fw
