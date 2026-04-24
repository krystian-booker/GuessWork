#pragma once

#include <cstdint>

#include "FirmwareConfig.h"
#include "Protocol.h"

namespace posest::firmware {

class CanBridge final {
public:
    void begin() {}

    void setLatestFusedPose(const FusedPosePayload& pose) {
        latest_pose_ = pose;
        has_latest_pose_ = true;
        ++received_fused_poses_;
    }

    void handleUnsupportedCanTx() {
        ++unsupported_can_tx_frames_;
        error_flags_ |= kErrorCanUnsupported;
    }

    bool hasLatestPose() const { return has_latest_pose_; }
    const FusedPosePayload& latestPose() const { return latest_pose_; }
    std::uint32_t errorFlags() const { return error_flags_; }
    std::uint32_t txQueueDepth() const { return 0; }
    std::uint32_t receivedFusedPoses() const { return received_fused_poses_; }
    std::uint32_t unsupportedCanTxFrames() const { return unsupported_can_tx_frames_; }

private:
    FusedPosePayload latest_pose_{};
    bool has_latest_pose_{false};
    std::uint32_t error_flags_{0};
    std::uint32_t received_fused_poses_{0};
    std::uint32_t unsupported_can_tx_frames_{0};
};

}  // namespace posest::firmware
