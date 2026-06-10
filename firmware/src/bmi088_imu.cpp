#include "bmi088_imu.h"

#include <BMI088.h>
#include <SPI.h>

#include "binary_proto.h"
#include "time64.h"

namespace gw_fw {

namespace {

Bmi088Accel accel(SPI, kImuAccelCsPin);
Bmi088Gyro  gyro(SPI, kImuGyroCsPin);

// A batch sitting unflushed longer than this goes out partial. At the
// nominal 400 Hz a 4-sample batch fills in 10 ms, so this only matters if
// the sensor slows down or stalls.
constexpr uint32_t kBatchMaxAgeUs = 15'000;

}  // namespace

Bmi088Streamer* Bmi088Streamer::s_self_ = nullptr;

void Bmi088Streamer::isr_trampoline() {
    if (s_self_) s_self_->isr_drdy();
}

void Bmi088Streamer::isr_drdy() {
    if (drdy_pending_) isr_drops_ = isr_drops_ + 1;
    drdy_t_us_    = now_us64();
    drdy_pending_ = true;
}

bool Bmi088Streamer::begin() {
    s_self_ = this;

    if (accel.begin() < 0) return false;
    if (gyro.begin() < 0) return false;

    // FRC robots see hard impacts — run the widest ranges. 400 Hz ODR on
    // both dies; the gyro's DRDY paces the combined stream.
    bool cfg_ok = true;
    cfg_ok &= accel.setOdr(Bmi088Accel::ODR_400HZ_BW_40HZ);
    cfg_ok &= accel.setRange(Bmi088Accel::RANGE_24G);
    cfg_ok &= gyro.setOdr(Bmi088Gyro::ODR_400HZ_BW_47HZ);
    cfg_ok &= gyro.setRange(Bmi088Gyro::RANGE_2000DPS);
    cfg_ok &= gyro.pinModeInt3(Bmi088Gyro::PUSH_PULL, Bmi088Gyro::ACTIVE_HIGH);
    cfg_ok &= gyro.mapDrdyInt3(true);
    if (!cfg_ok) return false;

    pinMode(kImuGyroDrdyPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(kImuGyroDrdyPin), isr_trampoline, RISING);

    ok_ = true;
    return true;
}

void Bmi088Streamer::poll() {
    if (!ok_) return;

    // Snapshot the DRDY latch with IRQs masked — the 64-bit timestamp is not
    // an atomic read on a 32-bit core.
    bool     pending;
    uint64_t t_us;
    noInterrupts();
    pending       = drdy_pending_;
    t_us          = drdy_t_us_;
    drdy_pending_ = false;
    interrupts();

    if (pending) {
        gyro.readSensor();
        accel.readSensor();

        if (batch_count_ == 0) batch_oldest_us_ = t_us;
        size_t off = 1 + static_cast<size_t>(batch_count_) * kImuRecordBytes;
        off = put_u64(batch_buf_, off, t_us);
        off = put_f32(batch_buf_, off, accel.getAccelX_mss());
        off = put_f32(batch_buf_, off, accel.getAccelY_mss());
        off = put_f32(batch_buf_, off, accel.getAccelZ_mss());
        off = put_f32(batch_buf_, off, gyro.getGyroX_rads());
        off = put_f32(batch_buf_, off, gyro.getGyroY_rads());
        off = put_f32(batch_buf_, off, gyro.getGyroZ_rads());
        ++batch_count_;
        ++sample_count_;
    }

    if (batch_count_ == 0) return;
    const bool full  = batch_count_ >= kImuBatchMax;
    const bool stale = (now_us64() - batch_oldest_us_) > kBatchMaxAgeUs;
    if (full || stale) flush_batch();
}

void Bmi088Streamer::flush_batch() {
    batch_buf_[0] = static_cast<uint8_t>(batch_count_);
    const uint8_t payload_len =
        static_cast<uint8_t>(1 + batch_count_ * kImuRecordBytes);

    uint8_t frame[6 + sizeof(batch_buf_)];
    const size_t n = build_frame(BinType::ImuBatch, batch_buf_, payload_len, frame);

    // Never block the loop on a stalled host: drop the batch if the USB
    // buffer can't take the whole frame. HEARTBEAT's drop counter makes the
    // loss visible host-side.
    if (SerialUSB1.availableForWrite() >= static_cast<int>(n)) {
        SerialUSB1.write(frame, n);
    } else {
        tx_drops_ += static_cast<uint32_t>(batch_count_);
    }
    batch_count_ = 0;
}

}  // namespace gw_fw
