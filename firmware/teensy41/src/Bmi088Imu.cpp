#include "Bmi088Imu.h"

#include <Arduino.h>
#include <SPI.h>

#include <cmath>
#include <cstring>

namespace posest::firmware {

namespace {

double accelRangeFactor(std::uint32_t range_g) {
    switch (range_g) {
        case 3:
            return 3.0;
        case 6:
            return 6.0;
        case 24:
            return 24.0;
        case 12:
        default:
            return 12.0;
    }
}

double gyroRangeFactor(std::uint32_t range_dps) {
    switch (range_dps) {
        case 125:
            return 125.0;
        case 250:
            return 250.0;
        case 500:
            return 500.0;
        case 1000:
            return 1000.0;
        case 2000:
        default:
            return 2000.0;
    }
}

bool mapAccelRange(std::uint32_t range_g, std::uint8_t& out) {
    switch (range_g) {
        case 3:
            out = BMI088_ACCEL_RANGE_3G;
            return true;
        case 6:
            out = BMI088_ACCEL_RANGE_6G;
            return true;
        case 12:
            out = BMI088_ACCEL_RANGE_12G;
            return true;
        case 24:
            out = BMI088_ACCEL_RANGE_24G;
            return true;
        default:
            return false;
    }
}

bool mapAccelOdr(std::uint32_t hz, std::uint8_t& out) {
    switch (hz) {
        case 12:  // 12.5 Hz rounded
            out = BMI08_ACCEL_ODR_12_5_HZ;
            return true;
        case 25:
            out = BMI08_ACCEL_ODR_25_HZ;
            return true;
        case 50:
            out = BMI08_ACCEL_ODR_50_HZ;
            return true;
        case 100:
            out = BMI08_ACCEL_ODR_100_HZ;
            return true;
        case 200:
            out = BMI08_ACCEL_ODR_200_HZ;
            return true;
        case 400:
            out = BMI08_ACCEL_ODR_400_HZ;
            return true;
        case 800:
            out = BMI08_ACCEL_ODR_800_HZ;
            return true;
        case 1600:
            out = BMI08_ACCEL_ODR_1600_HZ;
            return true;
        default:
            return false;
    }
}

bool mapAccelBandwidth(std::uint32_t code, std::uint8_t& out) {
    switch (code) {
        case 0:
            out = BMI08_ACCEL_BW_OSR4;
            return true;
        case 1:
            out = BMI08_ACCEL_BW_OSR2;
            return true;
        case 2:
            out = BMI08_ACCEL_BW_NORMAL;
            return true;
        default:
            return false;
    }
}

bool mapGyroRange(std::uint32_t dps, std::uint8_t& out) {
    switch (dps) {
        case 125:
            out = BMI08_GYRO_RANGE_125_DPS;
            return true;
        case 250:
            out = BMI08_GYRO_RANGE_250_DPS;
            return true;
        case 500:
            out = BMI08_GYRO_RANGE_500_DPS;
            return true;
        case 1000:
            out = BMI08_GYRO_RANGE_1000_DPS;
            return true;
        case 2000:
            out = BMI08_GYRO_RANGE_2000_DPS;
            return true;
        default:
            return false;
    }
}

bool mapGyroBandwidth(std::uint32_t code, std::uint8_t& out) {
    switch (code) {
        case 0:
            out = BMI08_GYRO_BW_532_ODR_2000_HZ;
            return true;
        case 1:
            out = BMI08_GYRO_BW_230_ODR_2000_HZ;
            return true;
        case 2:
            out = BMI08_GYRO_BW_116_ODR_1000_HZ;
            return true;
        case 3:
            out = BMI08_GYRO_BW_47_ODR_400_HZ;
            return true;
        default:
            return false;
    }
}

bool mapDataSyncMode(std::uint32_t hz, std::uint8_t& out) {
    switch (hz) {
        case 400:
            out = BMI08_ACCEL_DATA_SYNC_MODE_400HZ;
            return true;
        case 1000:
            out = BMI08_ACCEL_DATA_SYNC_MODE_1000HZ;
            return true;
        case 2000:
            out = BMI08_ACCEL_DATA_SYNC_MODE_2000HZ;
            return true;
        default:
            return false;
    }
}

}  // namespace

Bmi088Imu* Bmi088Imu::active_instance_ = nullptr;

bool Bmi088Imu::begin(std::uint64_t now_us) {
    stats_ = Bmi088Stats{};
    initialized_ = false;
    samples_since_temperature_ = kTemperatureSampleInterval;
    last_data_ready_time_us_ = now_us;
    accel_range_g_ = 12;
    gyro_range_dps_ = 2000;
    accel_scale_ = (kGravityMps2 * accelRangeFactor(accel_range_g_)) / 32768.0;
    gyro_scale_ = (gyroRangeFactor(gyro_range_dps_) * kDegToRad) / 32768.0;
    time64_high_ = static_cast<std::uint32_t>(now_us >> 32u);
    time64_low_prev_ = static_cast<std::uint32_t>(now_us & 0xFFFFFFFFu);

    pinMode(kBmi088ProtocolSelectPin, OUTPUT);
    digitalWrite(kBmi088ProtocolSelectPin, LOW);
    pinMode(kBmi088AccelCsPin, OUTPUT);
    pinMode(kBmi088GyroCsPin, OUTPUT);
    digitalWrite(kBmi088AccelCsPin, HIGH);
    digitalWrite(kBmi088GyroCsPin, HIGH);
    pinMode(kBmi088Int1Pin, INPUT);
    pinMode(kBmi088Int2Pin, INPUT);

    SPI1.setMISO(kBmi088SpiMisoPin);
    SPI1.setMOSI(kBmi088SpiMosiPin);
    SPI1.setSCK(kBmi088SpiSckPin);
    SPI1.begin();
    delay(10);

    if (!initializeBoschApi()) {
        stats_.error_flags |= kErrorImuInitFailure;
        return false;
    }

    if (!runSelfTest()) {
        stats_.error_flags |= kErrorImuSelfTestFailure;
        return false;
    }

    if (!configureDataSync(1000)) {
        stats_.error_flags |= kErrorImuInitFailure;
        return false;
    }

    active_instance_ = this;
    attachInterrupt(digitalPinToInterrupt(kBmi088Int1Pin), dataReadyThunk, RISING);
    initialized_ = true;
    stats_.initialized = true;
    return true;
}

bool Bmi088Imu::reconfigure(
    const ImuConfigCommand& command,
    std::uint64_t now_us,
    ImuConfigAckEntry& effective) {
    (void)now_us;
    if (!command.valid) {
        return false;
    }
    if (!applyAccelMeasConf(command.accel_range_g,
                            command.accel_odr_hz,
                            command.accel_bandwidth_code)) {
        return false;
    }
    if (!applyGyroMeasConf(command.gyro_range_dps,
                           command.gyro_bandwidth_code)) {
        return false;
    }
    if (command.run_selftest_on_boot && !runSelfTest()) {
        stats_.error_flags |= kErrorImuSelfTestFailure;
        return false;
    }
    if (!configureDataSync(command.data_sync_rate_hz)) {
        return false;
    }
    accel_range_g_ = command.accel_range_g;
    gyro_range_dps_ = command.gyro_range_dps;
    accel_scale_ = (kGravityMps2 * accelRangeFactor(accel_range_g_)) / 32768.0;
    gyro_scale_ = (gyroRangeFactor(gyro_range_dps_) * kDegToRad) / 32768.0;

    effective.accel_range_g = accel_range_g_;
    effective.accel_odr_hz = command.accel_odr_hz;
    effective.accel_bandwidth_code = command.accel_bandwidth_code;
    effective.gyro_range_dps = gyro_range_dps_;
    effective.gyro_bandwidth_code = command.gyro_bandwidth_code;
    effective.data_sync_rate_hz = command.data_sync_rate_hz;
    effective.reserved = 0;
    return true;
}

bool Bmi088Imu::poll(ImuPayload& payload) {
    if (!initialized_) {
        return false;
    }

    std::uint32_t timestamp32 = 0;
    std::uint32_t overrun_count = 0;
    if (!popDrdyTimestamp(timestamp32, overrun_count)) {
        return false;
    }

    if (overrun_count > 0u) {
        stats_.sample_overruns += overrun_count;
        stats_.error_flags |= kErrorImuSampleOverrun;
    }

    return readSynchronizedSample(timestamp32, payload);
}

void Bmi088Imu::checkForMissedDataReady(std::uint64_t now_us) {
    if (!initialized_) {
        return;
    }
    if (now_us >= last_data_ready_time_us_ &&
        now_us - last_data_ready_time_us_ > kMissedDrdyTimeoutUs) {
        stats_.error_flags |= kErrorImuMissedDrdy;
    }
}

void Bmi088Imu::onDataReadyIsr(std::uint32_t timestamp_us) {
    const std::uint8_t count = drdy_count_;
    if (count >= kDrdyQueueCapacity) {
        ++pending_overruns_;
        return;
    }

    drdy_timestamps_[drdy_head_] = timestamp_us;
    drdy_head_ = static_cast<std::uint8_t>((drdy_head_ + 1u) % kDrdyQueueCapacity);
    drdy_count_ = static_cast<std::uint8_t>(count + 1u);
    ++stats_.drdy_events;
}

Bmi088Stats Bmi088Imu::stats() const {
    Bmi088Stats copy = stats_;
    copy.initialized = initialized_;
    return copy;
}

std::uint32_t Bmi088Imu::errorFlags() const {
    return stats_.error_flags;
}

double Bmi088Imu::accelRawToMps2(std::int16_t raw) {
    return (static_cast<double>(raw) * kGravityMps2 * kAccelRangeG) / 32768.0;
}

double Bmi088Imu::gyroRawToRadps(std::int16_t raw) {
    const double dps = (static_cast<double>(raw) * kGyroRangeDps) / 32768.0;
    return dps * kDegToRad;
}

double Bmi088Imu::accelRawToMps2Scaled(std::int16_t raw) const {
    const double scale = accel_scale_ > 0.0 ? accel_scale_
                                            : (kGravityMps2 * kAccelRangeG) / 32768.0;
    return static_cast<double>(raw) * scale;
}

double Bmi088Imu::gyroRawToRadpsScaled(std::int16_t raw) const {
    const double scale = gyro_scale_ > 0.0 ? gyro_scale_
                                           : (kGyroRangeDps * kDegToRad) / 32768.0;
    return static_cast<double>(raw) * scale;
}

bool Bmi088Imu::popDrdyTimestamp(
    std::uint32_t& timestamp_us,
    std::uint32_t& overrun_count) {
    noInterrupts();
    if (drdy_count_ == 0u) {
        overrun_count = pending_overruns_;
        pending_overruns_ = 0;
        interrupts();
        return false;
    }

    timestamp_us = drdy_timestamps_[drdy_tail_];
    drdy_tail_ = static_cast<std::uint8_t>((drdy_tail_ + 1u) % kDrdyQueueCapacity);
    drdy_count_ = static_cast<std::uint8_t>(drdy_count_ - 1u);
    overrun_count = pending_overruns_;
    pending_overruns_ = 0;
    interrupts();
    return true;
}

bool Bmi088Imu::initializeBoschApi() {
    std::memset(&dev_, 0, sizeof(dev_));
    accel_target_ = SpiTarget{this, kBmi088AccelCsPin};
    gyro_target_ = SpiTarget{this, kBmi088GyroCsPin};

    dev_.intf = BMI08_SPI_INTF;
    dev_.variant = BMI088_VARIANT;
    dev_.read = spiRead;
    dev_.write = spiWrite;
    dev_.delay_us = delayUs;
    dev_.intf_ptr_accel = &accel_target_;
    dev_.intf_ptr_gyro = &gyro_target_;
    dev_.read_write_len = 32;

    std::int8_t result = bmi08xa_init(&dev_);
    recordResult(result);
    if (result != BMI08_OK) {
        return false;
    }

    result = bmi08g_init(&dev_);
    recordResult(result);
    if (result != BMI08_OK) {
        return false;
    }

    result = bmi08a_soft_reset(&dev_);
    recordResult(result);
    if (result != BMI08_OK) {
        return false;
    }

    result = bmi08a_load_config_file(&dev_);
    recordResult(result);
    if (result != BMI08_OK) {
        return false;
    }

    dev_.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
    result = bmi08a_set_power_mode(&dev_);
    recordResult(result);
    if (result != BMI08_OK) {
        return false;
    }

    dev_.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
    result = bmi08g_set_power_mode(&dev_);
    recordResult(result);
    if (result != BMI08_OK) {
        return false;
    }

    return true;
}

bool Bmi088Imu::runSelfTest() {
    std::int8_t result = bmi08xa_perform_selftest(&dev_);
    recordResult(result);
    if (result != BMI08_OK) {
        return false;
    }
    result = bmi08g_perform_selftest(&dev_);
    recordResult(result);
    if (result != BMI08_OK) {
        return false;
    }
    // Self-test leaves both sensors in suspend/idle; bring power back up.
    dev_.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
    result = bmi08a_set_power_mode(&dev_);
    recordResult(result);
    if (result != BMI08_OK) {
        return false;
    }
    dev_.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
    result = bmi08g_set_power_mode(&dev_);
    recordResult(result);
    return result == BMI08_OK;
}

bool Bmi088Imu::applyAccelMeasConf(std::uint32_t range_g,
                                   std::uint32_t odr_hz,
                                   std::uint32_t bandwidth_code) {
    std::uint8_t range = 0;
    std::uint8_t odr = 0;
    std::uint8_t bw = 0;
    if (!mapAccelRange(range_g, range) || !mapAccelOdr(odr_hz, odr) ||
        !mapAccelBandwidth(bandwidth_code, bw)) {
        return false;
    }
    dev_.accel_cfg.range = range;
    dev_.accel_cfg.odr = odr;
    dev_.accel_cfg.bw = bw;
    const std::int8_t result = bmi08a_set_meas_conf(&dev_);
    recordResult(result);
    return result == BMI08_OK;
}

bool Bmi088Imu::applyGyroMeasConf(std::uint32_t range_dps,
                                  std::uint32_t bandwidth_code) {
    std::uint8_t range = 0;
    std::uint8_t bw = 0;
    if (!mapGyroRange(range_dps, range) || !mapGyroBandwidth(bandwidth_code, bw)) {
        return false;
    }
    dev_.gyro_cfg.range = range;
    dev_.gyro_cfg.bw = bw;
    const std::int8_t result = bmi08g_set_meas_conf(&dev_);
    recordResult(result);
    return result == BMI08_OK;
}

bool Bmi088Imu::configureDataSync(std::uint32_t data_sync_rate_hz) {
    std::uint8_t mode = 0;
    if (!mapDataSyncMode(data_sync_rate_hz, mode)) {
        return false;
    }
    dev_.accel_cfg.range = BMI088_ACCEL_RANGE_12G;
    dev_.gyro_cfg.range = BMI08_GYRO_RANGE_2000_DPS;
    if (mapAccelRange(accel_range_g_, dev_.accel_cfg.range) == false) {
        // Fall back to 12g.
        accel_range_g_ = 12;
        dev_.accel_cfg.range = BMI088_ACCEL_RANGE_12G;
    }
    if (mapGyroRange(gyro_range_dps_, dev_.gyro_cfg.range) == false) {
        gyro_range_dps_ = 2000;
        dev_.gyro_cfg.range = BMI08_GYRO_RANGE_2000_DPS;
    }

    struct bmi08_data_sync_cfg sync_cfg{};
    sync_cfg.mode = mode;
    std::int8_t result = bmi08xa_configure_data_synchronization(sync_cfg, &dev_);
    recordResult(result);
    if (result != BMI08_OK) {
        return false;
    }

    struct bmi08_int_cfg int_config{};
    int_config.accel_int_config_1.int_channel = BMI08_INT_CHANNEL_2;
    int_config.accel_int_config_1.int_type = BMI08_ACCEL_SYNC_INPUT;
    int_config.accel_int_config_1.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    int_config.accel_int_config_1.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

    int_config.accel_int_config_2.int_channel = BMI08_INT_CHANNEL_1;
    int_config.accel_int_config_2.int_type = BMI08_ACCEL_INT_SYNC_DATA_RDY;
    int_config.accel_int_config_2.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    int_config.accel_int_config_2.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

    int_config.gyro_int_config_1.int_channel = BMI08_INT_CHANNEL_4;
    int_config.gyro_int_config_1.int_type = BMI08_GYRO_INT_DATA_RDY;
    int_config.gyro_int_config_1.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    int_config.gyro_int_config_1.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

    int_config.gyro_int_config_2.int_channel = BMI08_INT_CHANNEL_3;
    int_config.gyro_int_config_2.int_type = BMI08_GYRO_INT_DATA_RDY;
    int_config.gyro_int_config_2.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    int_config.gyro_int_config_2.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = BMI08_DISABLE;

    result = bmi08a_set_data_sync_int_config(&int_config, &dev_);
    recordResult(result);
    return result == BMI08_OK;
}

bool Bmi088Imu::readSynchronizedSample(
    std::uint32_t timestamp32,
    ImuPayload& payload) {
    struct bmi08_sensor_data accel{};
    struct bmi08_sensor_data gyro{};
    const std::int8_t result = bmi08a_get_synchronized_data(&accel, &gyro, &dev_);
    recordResult(result);
    if (result != BMI08_OK) {
        recordReadFailure(result);
        return false;
    }

    payload = ImuPayload{};
    payload.teensy_time_us = expandTimestamp(timestamp32);
    last_data_ready_time_us_ = payload.teensy_time_us;
    payload.accel_mps2 = {
        accelRawToMps2Scaled(accel.x),
        accelRawToMps2Scaled(accel.y),
        accelRawToMps2Scaled(accel.z)};
    payload.gyro_radps = {
        gyroRawToRadpsScaled(gyro.x),
        gyroRawToRadpsScaled(gyro.y),
        gyroRawToRadpsScaled(gyro.z)};
    payload.temperature_c = latest_temperature_c_;
    payload.status_flags = 0;

    if (stats_.sample_overruns > 0u) {
        payload.status_flags |= kImuStatusSampleOverrun;
    }
    if (isSaturated(accel.x) || isSaturated(accel.y) || isSaturated(accel.z)) {
        payload.status_flags |= kImuStatusAccelSaturated;
        ++stats_.accel_saturations;
        stats_.error_flags |= kErrorImuAccelSaturated;
    }
    if (isSaturated(gyro.x) || isSaturated(gyro.y) || isSaturated(gyro.z)) {
        payload.status_flags |= kImuStatusGyroSaturated;
        ++stats_.gyro_saturations;
        stats_.error_flags |= kErrorImuGyroSaturated;
    }

    double temperature_c = latest_temperature_c_;
    if (readTemperatureIfDue(temperature_c)) {
        latest_temperature_c_ = temperature_c;
        payload.temperature_c = temperature_c;
    }

    ++stats_.samples_read;
    return true;
}

bool Bmi088Imu::readTemperatureIfDue(double& temperature_c) {
    if (++samples_since_temperature_ < kTemperatureSampleInterval) {
        return false;
    }
    samples_since_temperature_ = 0;

    std::int32_t temperature_milli_c = 0;
    const std::int8_t result = bmi08a_get_sensor_temperature(&dev_, &temperature_milli_c);
    recordResult(result);
    if (result != BMI08_OK) {
        recordReadFailure(result);
        return false;
    }

    temperature_c = static_cast<double>(temperature_milli_c) / 1000.0;
    return true;
}

void Bmi088Imu::recordResult(std::int8_t result) {
    stats_.last_result = result;
    if (result != BMI08_OK) {
        stats_.error_flags |= kErrorImuSpiFailure;
    }
}

void Bmi088Imu::recordReadFailure(std::int8_t result) {
    stats_.last_result = result;
    ++stats_.read_failures;
    stats_.error_flags |= kErrorImuSpiFailure;
}

std::uint64_t Bmi088Imu::expandTimestamp(std::uint32_t timestamp32) {
    // Sample the 32-bit clock fresh inside this function so the high-half
    // anchor never reflects a stale reference. We track the most recent low
    // value we've seen and bump high every time it wraps.
    const std::uint32_t now_low = micros();
    if (now_low < time64_low_prev_) {
        ++time64_high_;
    }
    time64_low_prev_ = now_low;

    // Reconstruct the queued ISR timestamp: it is at or before now_low, but
    // may have been recorded just before a wrap. If timestamp32 > now_low it
    // must belong to the previous high half.
    std::uint32_t high = time64_high_;
    if (timestamp32 > now_low) {
        if (high == 0u) {
            return static_cast<std::uint64_t>(timestamp32);
        }
        --high;
    }
    return (static_cast<std::uint64_t>(high) << 32u) | timestamp32;
}

bool Bmi088Imu::isSaturated(std::int16_t raw) {
    return raw <= -32760 || raw >= 32760;
}

BMI08_INTF_RET_TYPE Bmi088Imu::spiRead(
    std::uint8_t reg_addr,
    std::uint8_t* reg_data,
    std::uint32_t len,
    void* intf_ptr) {
    if (!reg_data || !intf_ptr) {
        return -1;
    }

    auto* target = static_cast<SpiTarget*>(intf_ptr);
    SPI1.beginTransaction(SPISettings(kSpiFrequencyHz, MSBFIRST, SPI_MODE3));
    digitalWrite(target->chip_select_pin, LOW);
    SPI1.transfer(reg_addr);
    for (std::uint32_t i = 0; i < len; ++i) {
        reg_data[i] = SPI1.transfer(0x00);
    }
    digitalWrite(target->chip_select_pin, HIGH);
    SPI1.endTransaction();
    return 0;
}

BMI08_INTF_RET_TYPE Bmi088Imu::spiWrite(
    std::uint8_t reg_addr,
    const std::uint8_t* reg_data,
    std::uint32_t len,
    void* intf_ptr) {
    if (!intf_ptr || (len > 0u && !reg_data)) {
        return -1;
    }

    auto* target = static_cast<SpiTarget*>(intf_ptr);
    SPI1.beginTransaction(SPISettings(kSpiFrequencyHz, MSBFIRST, SPI_MODE3));
    digitalWrite(target->chip_select_pin, LOW);
    SPI1.transfer(reg_addr);
    for (std::uint32_t i = 0; i < len; ++i) {
        SPI1.transfer(reg_data[i]);
    }
    digitalWrite(target->chip_select_pin, HIGH);
    SPI1.endTransaction();
    return 0;
}

void Bmi088Imu::delayUs(std::uint32_t period_us, void* intf_ptr) {
    (void)intf_ptr;
    delayMicroseconds(period_us);
}

void Bmi088Imu::dataReadyThunk() {
    if (active_instance_) {
        active_instance_->onDataReadyIsr(micros());
    }
}

}  // namespace posest::firmware
