#include "CanBridge.h"

#include <Arduino.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include "ByteCodec.h"
#include "CanSchema.h"

namespace posest::firmware {

namespace {

#if POSEST_HAS_FLEXCAN_T4

// Singleton FlexCAN_T4 FD instance. Teensy 4.1 routes the FD-capable controller
// to CAN3 (RX = pin 30, TX = pin 31).
FlexCAN_T4FD<CAN3, RX_SIZE_64, TX_SIZE_32> g_can_bus;

#endif  // POSEST_HAS_FLEXCAN_T4

constexpr std::size_t kChassisSpeedsFrameSize =
    can_schema::kRioChassisSpeedsPayloadBytes;
constexpr std::size_t kTimeSyncFrameSize = can_schema::kRioTimeSyncPayloadBytes;

// Encode three diagonal-only sigmas from the 6×6 host covariance. The covariance
// is gtsam::Pose3 tangent ordering [rx, ry, rz, tx, ty, tz]; we extract:
//   sigma_x = sqrt(cov(tx, tx)) at index 3*6 + 3
//   sigma_y = sqrt(cov(ty, ty)) at index 4*6 + 4
//   sigma_yaw = sqrt(cov(rz, rz)) at index 2*6 + 2
// Each diagonal entry is variance, hence the sqrt. Negative or non-finite
// entries map to NaN/sentinel so the RIO can detect them via the status word.
float sigmaFromVariance(double variance) {
    if (!(variance >= 0.0)) {  // catches NaN and negatives
        return std::numeric_limits<float>::quiet_NaN();
    }
    return static_cast<float>(std::sqrt(variance));
}

std::uint16_t encodeSigmaYawMrad(double variance) {
    if (!(variance >= 0.0)) {
        return can_schema::kTeensyPoseSigmaYawMradNaN;
    }
    const double mrad = std::sqrt(variance) * 1000.0;
    if (!(mrad < 65535.0)) {
        return can_schema::kTeensyPoseSigmaYawMradNaN;
    }
    return static_cast<std::uint16_t>(std::lround(mrad));
}

void writeFusedPoseFrame(
    const FusedPosePayload& pose,
    std::uint64_t teensy_time_us,
    std::uint8_t* out) {
    std::memset(out, 0, can_schema::kTeensyPosePayloadBytes);

    std::size_t offset = can_schema::kTeensyPoseTeensyTimeOffset;
    appendU64(out, offset, teensy_time_us);

    offset = can_schema::kTeensyPoseXOffset;
    appendDouble(out, offset, pose.x_m);
    offset = can_schema::kTeensyPoseYOffset;
    appendDouble(out, offset, pose.y_m);

    offset = can_schema::kTeensyPoseZOffset;
    appendFloat(out, offset, static_cast<float>(pose.z_m));
    offset = can_schema::kTeensyPoseRollOffset;
    appendFloat(out, offset, static_cast<float>(pose.roll_rad));
    offset = can_schema::kTeensyPosePitchOffset;
    appendFloat(out, offset, static_cast<float>(pose.pitch_rad));
    offset = can_schema::kTeensyPoseYawOffset;
    appendFloat(out, offset, static_cast<float>(pose.yaw_rad));

    const float vx = pose.has_velocity ? static_cast<float>(pose.vx_mps)
                                       : std::numeric_limits<float>::quiet_NaN();
    const float vy = pose.has_velocity ? static_cast<float>(pose.vy_mps)
                                       : std::numeric_limits<float>::quiet_NaN();
    const float vz = pose.has_velocity ? static_cast<float>(pose.vz_mps)
                                       : std::numeric_limits<float>::quiet_NaN();
    offset = can_schema::kTeensyPoseVxOffset;
    appendFloat(out, offset, vx);
    offset = can_schema::kTeensyPoseVyOffset;
    appendFloat(out, offset, vy);
    offset = can_schema::kTeensyPoseVzOffset;
    appendFloat(out, offset, vz);

    // Diagonals for x (tx), y (ty), yaw (rz) — see comment on sigmaFromVariance.
    const double var_tx = pose.covariance[3 * 6 + 3];
    const double var_ty = pose.covariance[4 * 6 + 4];
    const double var_rz = pose.covariance[2 * 6 + 2];
    offset = can_schema::kTeensyPoseSigmaXOffset;
    appendFloat(out, offset, sigmaFromVariance(var_tx));
    offset = can_schema::kTeensyPoseSigmaYOffset;
    appendFloat(out, offset, sigmaFromVariance(var_ty));
    offset = can_schema::kTeensyPoseSigmaYawMradOffset;
    appendU16(out, offset, encodeSigmaYawMrad(var_rz));

    offset = can_schema::kTeensyPoseStatusOffset;
    appendU16(out, offset, static_cast<std::uint16_t>(pose.status_flags & 0xFFFFu));
}

}  // namespace

void CanBridge::begin(const CanBusConfig& config) {
    config_ = config;
    pose_tx_period_us_ =
        config.pose_publish_hz > 0u ? (1'000'000u / config.pose_publish_hz) : 10'000u;
    next_pose_tx_us_ = micros();
    rio_offset_valid_ = false;
    rio_offset_us_ema_ = 0;
    rio_bootstrap_remaining_ = kRioBootstrapSamples;
    rio_consecutive_rejections_ = 0;
    pending_head_ = 0;
    pending_count_ = 0;
    error_flags_ = 0;
    status_flags_ = 0;
    initialized_ = false;

    if (!config.enabled) {
        return;
    }

#if POSEST_HAS_FLEXCAN_T4
    g_can_bus.begin();
    CANFD_timings_t timings;
    timings.clock = CLK_24MHz;
    timings.baudrate = config.nominal_bitrate_bps;
    timings.baudrateFD = config.data_bitrate_bps;
    timings.propdelay = 190;
    timings.bus_length = 1;
    timings.sample = 70;
    g_can_bus.setBaudRate(timings);
    g_can_bus.setRegions(64);
    g_can_bus.enableMBInterrupts();
    initialized_ = true;
#else
    error_flags_ |= kCanInitFailure;
#endif
}

void CanBridge::poll(std::uint64_t now_us) {
    if (!initialized_) {
        return;
    }

#if POSEST_HAS_FLEXCAN_T4
    g_can_bus.events();

    CANFD_message_t msg;
    while (g_can_bus.read(msg)) {
        ++rx_frames_;
        handleRxFrame(msg.id, msg.buf, msg.len, now_us);
    }
#else
    (void)now_us;
#endif

    maybeSendFusedPose(now_us);
    checkRioStaleness(now_us);
}

void CanBridge::checkRioStaleness(std::uint64_t now_us) {
    if (rio_offset_valid_ && config_.rio_offset_stale_us > 0u &&
        now_us > last_rio_offset_time_us_ &&
        now_us - last_rio_offset_time_us_ > config_.rio_offset_stale_us) {
        rio_offset_valid_ = false;
    }
}

void CanBridge::setLatestFusedPose(const FusedPosePayload& pose) {
    latest_pose_ = pose;
    has_latest_pose_ = true;
    ++received_fused_poses_;
}

void CanBridge::handleRxFrame(
    std::uint32_t can_id,
    const std::uint8_t* data,
    std::size_t length,
    std::uint64_t now_us) {
    if (can_id == config_.rio_chassis_speeds_can_id) {
        if (length < can_schema::kRioChassisSpeedsStatusOffset + 4u) {
            ++rx_decode_failures_;
            status_flags_ |= kCanFrameTruncated;
            return;
        }
        handleRioChassisSpeeds(data, length, now_us);
        return;
    }
    if (can_id == config_.rio_time_sync_can_id) {
        if (length < kTimeSyncFrameSize) {
            ++rx_decode_failures_;
            status_flags_ |= kCanFrameTruncated;
            return;
        }
        handleRioTimeSync(data, length, now_us);
        return;
    }
    // Unknown ID — ignore quietly. This is normal on a shared bus.
}

void CanBridge::handleRioChassisSpeeds(
    const std::uint8_t* data,
    std::size_t /*length*/,
    std::uint64_t now_us) {
    ChassisSpeedsPayload payload;
    payload.teensy_time_us = now_us;
    payload.rio_time_us = static_cast<std::uint64_t>(
        readI64(data, can_schema::kRioChassisSpeedsRioTimeOffset));
    payload.vx_mps = readDouble(data, can_schema::kRioChassisSpeedsVxOffset);
    payload.vy_mps = readDouble(data, can_schema::kRioChassisSpeedsVyOffset);
    payload.omega_radps =
        readDouble(data, can_schema::kRioChassisSpeedsOmegaOffset);
    payload.status_flags = readU32(data, can_schema::kRioChassisSpeedsStatusOffset);
    if (!rio_offset_valid_) {
        payload.status_flags |= kStatusUnsynchronizedRioTime;
    }
    enqueuePendingChassisSpeeds(payload);
}

void CanBridge::handleRioTimeSync(
    const std::uint8_t* data,
    std::size_t /*length*/,
    std::uint64_t now_us) {
    const std::uint64_t rio_time_us =
        readU64(data, can_schema::kRioTimeSyncRioTimeOffset);
    applyRioTimeSync(rio_time_us, now_us);
}

// rio_time_us is the RoboRIO FPGA microsecond counter (monotonic since RIO
// power-on). offset_new = teensy_recv - rio_send approximates the constant
// clock skew between the two domains, modulo CAN bus latency. The EMA filters
// out latency jitter; the abs_delta gate rejects single-sample outliers.
//
// Bootstrap: accept the first kRioBootstrapSamples unconditionally so the
// EMA anchors before the gate engages.
//
// RIO reboot: rio_time_us drops to ~0, every subsequent ping fails the gate.
// After kRioRejectionResetThreshold consecutive rejections we force a
// re-bootstrap so the filter re-anchors on the new RIO epoch instead of
// staying stuck on the pre-reboot offset forever.
void CanBridge::applyRioTimeSync(std::uint64_t rio_time_us, std::uint64_t now_us) {
    const std::int64_t offset_new =
        static_cast<std::int64_t>(now_us) - static_cast<std::int64_t>(rio_time_us);

    ++rio_pings_received_;

    if (rio_offset_valid_ && rio_bootstrap_remaining_ == 0u) {
        const std::int64_t delta = offset_new - rio_offset_us_ema_;
        const std::int64_t abs_delta = delta < 0 ? -delta : delta;
        if (abs_delta > kRioMaxStepUs) {
            ++rio_pings_rejected_;
            ++rio_consecutive_rejections_;
            status_flags_ |= kRioStatusPingRejected;
            if (rio_consecutive_rejections_ < kRioRejectionResetThreshold) {
                return;
            }
            // Sustained rejection: assume the RoboRIO rebooted and start
            // over. Fall through and let this sample re-anchor as the new
            // bootstrap baseline.
            rio_offset_valid_ = false;
            rio_bootstrap_remaining_ = kRioBootstrapSamples;
            rio_consecutive_rejections_ = 0;
        }
    }

    if (!rio_offset_valid_) {
        rio_offset_us_ema_ = offset_new;
    } else {
        rio_offset_us_ema_ =
            rio_offset_us_ema_ + (offset_new - rio_offset_us_ema_) / 8;
    }
    rio_offset_valid_ = true;
    last_rio_offset_time_us_ = now_us;
    rio_consecutive_rejections_ = 0;
    if (rio_bootstrap_remaining_ > 0u) {
        --rio_bootstrap_remaining_;
    }
}

void CanBridge::enqueuePendingChassisSpeeds(const ChassisSpeedsPayload& payload) {
    if (pending_count_ >= kPendingChassisSpeedsCapacity) {
        ++rx_decode_failures_;
        status_flags_ |= kCanRxOverrun;
        // Drop oldest to keep the freshest measurement.
        pending_head_ = (pending_head_ + 1u) % kPendingChassisSpeedsCapacity;
        --pending_count_;
    }
    const std::size_t tail =
        (pending_head_ + pending_count_) % kPendingChassisSpeedsCapacity;
    pending_chassis_speeds_[tail] = payload;
    ++pending_count_;
}

bool CanBridge::popPendingChassisSpeeds(ChassisSpeedsPayload& out) {
    if (pending_count_ == 0u) {
        return false;
    }
    out = pending_chassis_speeds_[pending_head_];
    pending_head_ = (pending_head_ + 1u) % kPendingChassisSpeedsCapacity;
    --pending_count_;
    return true;
}

void CanBridge::maybeSendFusedPose(std::uint64_t now_us) {
    if (!has_latest_pose_ || !config_.enabled) {
        return;
    }
    if (now_us < next_pose_tx_us_) {
        return;
    }
    next_pose_tx_us_ = now_us + pose_tx_period_us_;

    if (!transmitTeensyPose(latest_pose_, now_us)) {
        ++tx_overruns_;
        status_flags_ |= kCanTxOverrun;
    }
}

bool CanBridge::transmitTeensyPose(
    const FusedPosePayload& pose,
    std::uint64_t now_us) {
#if POSEST_HAS_FLEXCAN_T4
    CANFD_message_t msg{};
    msg.id = config_.teensy_pose_can_id;
    msg.brs = 1;
    msg.edl = 1;
    msg.len = static_cast<std::uint8_t>(can_schema::kTeensyPosePayloadBytes);
    writeFusedPoseFrame(pose, now_us, msg.buf);
    const int rc = g_can_bus.write(msg);
    if (rc < 0) {
        return false;
    }
    ++tx_frames_;
    return true;
#else
    (void)pose;
    (void)now_us;
    return false;
#endif
}

std::uint32_t CanBridge::rioStatusFlags() const {
    std::uint32_t flags = 0;
    if (!rio_offset_valid_) {
        flags |= kHealthRioUnsynchronized;
    }
    if ((status_flags_ & kRioStatusPingRejected) != 0u) {
        flags |= kHealthRioPingRejected;
    }
    return flags;
}

CanBridgeStats CanBridge::stats() const {
    CanBridgeStats out;
    out.initialized = initialized_;
    out.rx_frames = rx_frames_;
    out.tx_frames = tx_frames_;
    out.rx_decode_failures = rx_decode_failures_;
    out.tx_overruns = tx_overruns_;
    out.rio_pings_received = rio_pings_received_;
    out.rio_pings_rejected = rio_pings_rejected_;
    out.rio_offset_valid = rio_offset_valid_;
    out.rio_offset_us = rio_offset_us_ema_;
    out.last_rio_offset_time_us = last_rio_offset_time_us_;
    out.pending_chassis_speeds = static_cast<std::uint32_t>(pending_count_);
    out.status_flags = status_flags_;
    return out;
}

}  // namespace posest::firmware
