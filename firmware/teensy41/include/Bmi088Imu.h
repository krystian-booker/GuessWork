#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

#include "FirmwareConfig.h"
#include "Protocol.h"

extern "C" {
#include "bmi08.h"
#include "bmi08x.h"
}

namespace posest::firmware {

constexpr std::uint32_t kImuStatusReadFailure = 1u << 8u;
constexpr std::uint32_t kImuStatusSampleOverrun = 1u << 9u;
constexpr std::uint32_t kImuStatusAccelSaturated = 1u << 10u;
constexpr std::uint32_t kImuStatusGyroSaturated = 1u << 11u;

struct Bmi088Stats {
    bool initialized{false};
    std::uint32_t error_flags{0};
    std::uint32_t drdy_events{0};
    std::uint32_t samples_read{0};
    std::uint32_t read_failures{0};
    std::uint32_t sample_overruns{0};
    std::uint32_t accel_saturations{0};
    std::uint32_t gyro_saturations{0};
    std::int8_t last_result{0};
};

class Bmi088Imu final {
public:
    bool begin(std::uint64_t now_us);
    bool reconfigure(
        const ImuConfigCommand& command,
        std::uint64_t now_us,
        ImuConfigAckEntry& effective);
    bool poll(ImuPayload& payload);
    void checkForMissedDataReady(std::uint64_t now_us);
    void onDataReadyIsr(std::uint32_t timestamp_us);

    Bmi088Stats stats() const;
    std::uint32_t errorFlags() const;
    bool initialized() const { return initialized_; }
    std::uint8_t pendingDrdyCount() const { return drdy_count_; }
    std::uint32_t pendingDrdyOverruns() const { return pending_overruns_; }

    static double accelRawToMps2(std::int16_t raw);
    static double gyroRawToRadps(std::int16_t raw);
    double accelRawToMps2Scaled(std::int16_t raw) const;
    double gyroRawToRadpsScaled(std::int16_t raw) const;

private:
    struct SpiTarget {
        Bmi088Imu* owner{nullptr};
        std::uint8_t chip_select_pin{0};
    };

    bool popDrdyTimestamp(std::uint32_t& timestamp_us, std::uint32_t& overrun_count);
    bool initializeBoschApi();
    bool configureDataSync(std::uint32_t data_sync_rate_hz);
    bool runSelfTest();
    bool applyAccelMeasConf(std::uint32_t range_g,
                            std::uint32_t odr_hz,
                            std::uint32_t bandwidth_code);
    bool applyGyroMeasConf(std::uint32_t range_dps,
                           std::uint32_t bandwidth_code);
    bool readSynchronizedSample(std::uint32_t timestamp32,
                                ImuPayload& payload);
    bool readTemperatureIfDue(double& temperature_c);
    void recordResult(std::int8_t result);
    void recordReadFailure(std::int8_t result);
    std::uint64_t expandTimestamp(std::uint32_t timestamp32);

    static bool isSaturated(std::int16_t raw);
    static BMI08_INTF_RET_TYPE spiRead(
        std::uint8_t reg_addr,
        std::uint8_t* reg_data,
        std::uint32_t len,
        void* intf_ptr);
    static BMI08_INTF_RET_TYPE spiWrite(
        std::uint8_t reg_addr,
        const std::uint8_t* reg_data,
        std::uint32_t len,
        void* intf_ptr);
    static void delayUs(std::uint32_t period_us, void* intf_ptr);
    static void dataReadyThunk();

    static constexpr std::size_t kDrdyQueueCapacity = 8;
    static constexpr double kAccelRangeG = 12.0;
    static constexpr double kGyroRangeDps = 2000.0;
    static constexpr double kGravityMps2 = 9.80665;
    static constexpr double kDegToRad = 0.017453292519943295;
    static constexpr std::uint32_t kSpiFrequencyHz = 5000000;
    static constexpr std::uint32_t kTemperatureSampleInterval = 100;
    static constexpr std::uint64_t kMissedDrdyTimeoutUs = 10000;

    std::uint32_t accel_range_g_{12};
    std::uint32_t gyro_range_dps_{2000};
    double accel_scale_{0.0};
    double gyro_scale_{0.0};
    std::uint32_t time64_high_{0};
    std::uint32_t time64_low_prev_{0};

    struct bmi08_dev dev_{};
    SpiTarget accel_target_{this, kBmi088AccelCsPin};
    SpiTarget gyro_target_{this, kBmi088GyroCsPin};
    bool initialized_{false};
    std::uint64_t last_data_ready_time_us_{0};
    double latest_temperature_c_{std::numeric_limits<double>::quiet_NaN()};
    std::uint32_t samples_since_temperature_{kTemperatureSampleInterval};
    Bmi088Stats stats_{};

    volatile std::uint8_t drdy_head_{0};
    volatile std::uint8_t drdy_tail_{0};
    volatile std::uint8_t drdy_count_{0};
    volatile std::uint32_t drdy_timestamps_[kDrdyQueueCapacity]{};
    volatile std::uint32_t pending_overruns_{0};

    static Bmi088Imu* active_instance_;
};

}  // namespace posest::firmware
