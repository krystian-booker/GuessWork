#pragma once

#include <cstddef>
#include <cstdint>

#include "FirmwareConfig.h"
#include "Protocol.h"

#if __has_include(<FlexCAN_T4.h>)
#define POSEST_HAS_FLEXCAN_T4 1
#include <FlexCAN_T4.h>
#else
#define POSEST_HAS_FLEXCAN_T4 0
#endif

namespace posest::firmware {

struct CanBusConfig {
    bool enabled{false};
    std::uint32_t nominal_bitrate_bps{1'000'000};
    std::uint32_t data_bitrate_bps{2'000'000};
    std::uint32_t pose_publish_hz{100};
    std::uint64_t rio_offset_stale_us{1'000'000};
    std::uint32_t rio_chassis_speeds_can_id{0x100};
    std::uint32_t rio_time_sync_can_id{0x101};
    std::uint32_t teensy_pose_can_id{0x180};
};

struct CanBridgeStats {
    bool initialized{false};
    std::uint32_t rx_frames{0};
    std::uint32_t tx_frames{0};
    std::uint32_t rx_decode_failures{0};
    std::uint32_t tx_overruns{0};
    std::uint32_t rio_pings_received{0};
    std::uint32_t rio_pings_rejected{0};
    bool rio_offset_valid{false};
    std::int64_t rio_offset_us{0};
    std::uint64_t last_rio_offset_time_us{0};
    std::uint32_t pending_chassis_speeds{0};
    std::uint32_t status_flags{0};
};

// Buffered ChassisSpeeds frames waiting to be written to USB by the main loop.
class CanBridge final {
public:
    static constexpr std::size_t kPendingChassisSpeedsCapacity = 8;

    void begin(const CanBusConfig& config = CanBusConfig{});
    // Drives RX decode + periodic FusedPose TX. Call once per loop iteration.
    void poll(std::uint64_t now_us);

    // Called by the host-protocol dispatch when a FusedPose USB frame arrives.
    void setLatestFusedPose(const FusedPosePayload& pose);

    // Set whenever the host protocol receives a CanTx command we cannot
    // currently honor. Kept as a stat counter only.
    void handleUnsupportedCanTx() {
        ++unsupported_can_tx_frames_;
        status_flags_ |= kCanFrameTruncated;
    }

    // True iff a ChassisSpeeds payload is queued for USB transmit.
    bool popPendingChassisSpeeds(ChassisSpeedsPayload& out);

    bool hasLatestPose() const { return has_latest_pose_; }
    const FusedPosePayload& latestPose() const { return latest_pose_; }
    std::uint32_t errorFlags() const { return error_flags_; }
    std::uint32_t txQueueDepth() const { return tx_queue_depth_; }
    std::uint32_t receivedFusedPoses() const { return received_fused_poses_; }
    std::uint32_t unsupportedCanTxFrames() const { return unsupported_can_tx_frames_; }
    bool rioOffsetValid() const { return rio_offset_valid_; }
    std::int64_t rioOffsetUs() const { return rio_offset_us_ema_; }
    std::uint32_t rioConsecutiveRejections() const { return rio_consecutive_rejections_; }
    std::uint32_t rioStatusFlags() const;
    CanBridgeStats stats() const;

    // Test hooks. Production paths use the private byte-buffer overloads
    // driven by handleRxFrame / poll; these expose the same logic to unit
    // tests that cannot stand up FlexCAN_T4.
    void testHookFeedRioTimeSync(std::uint64_t rio_time_us, std::uint64_t now_us) {
        applyRioTimeSync(rio_time_us, now_us);
    }
    void testHookCheckStaleness(std::uint64_t now_us) { checkRioStaleness(now_us); }

private:
    void handleRxFrame(std::uint32_t can_id,
                       const std::uint8_t* data,
                       std::size_t length,
                       std::uint64_t now_us);
    void handleRioChassisSpeeds(const std::uint8_t* data, std::size_t length,
                                std::uint64_t now_us);
    void handleRioTimeSync(const std::uint8_t* data, std::size_t length,
                            std::uint64_t now_us);
    void applyRioTimeSync(std::uint64_t rio_time_us, std::uint64_t now_us);
    void checkRioStaleness(std::uint64_t now_us);
    void enqueuePendingChassisSpeeds(const ChassisSpeedsPayload& payload);
    void maybeSendFusedPose(std::uint64_t now_us);
    bool transmitTeensyPose(const FusedPosePayload& pose, std::uint64_t now_us);

    CanBusConfig config_{};
    bool initialized_{false};
    FusedPosePayload latest_pose_{};
    bool has_latest_pose_{false};
    std::uint32_t error_flags_{0};
    std::uint32_t status_flags_{0};
    std::uint32_t received_fused_poses_{0};
    std::uint32_t unsupported_can_tx_frames_{0};

    ChassisSpeedsPayload pending_chassis_speeds_[kPendingChassisSpeedsCapacity]{};
    std::size_t pending_head_{0};
    std::size_t pending_count_{0};

    bool rio_offset_valid_{false};
    std::int64_t rio_offset_us_ema_{0};
    std::uint64_t last_rio_offset_time_us_{0};
    std::uint32_t rio_pings_received_{0};
    std::uint32_t rio_pings_rejected_{0};
    std::uint32_t rio_consecutive_rejections_{0};
    static constexpr std::uint32_t kRioBootstrapSamples = 3;
    static constexpr std::uint32_t kRioRejectionResetThreshold = 32;
    std::uint32_t rio_bootstrap_remaining_{kRioBootstrapSamples};

    std::uint32_t rx_frames_{0};
    std::uint32_t tx_frames_{0};
    std::uint32_t rx_decode_failures_{0};
    std::uint32_t tx_overruns_{0};
    std::uint32_t tx_queue_depth_{0};

    std::uint64_t next_pose_tx_us_{0};
    std::uint64_t pose_tx_period_us_{10'000};

    static constexpr std::int64_t kRioMaxStepUs = 5'000;
};

}  // namespace posest::firmware
