#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <vector>

#include "posest/fusion/IFusionOutputSink.h"
#include "posest/runtime/RuntimeConfig.h"
#include "posest/teensy/Protocol.h"

namespace posest::teensy {

class TeensyService final : public fusion::IFusionOutputSink {
public:
    explicit TeensyService(runtime::TeensyConfig config);

    void publish(FusedPoseEstimate estimate) override;

    std::optional<Frame> takeLastOutboundFrame() const;

private:
    static std::vector<std::uint8_t> encodeFusedPosePayload(const FusedPoseEstimate& estimate);

    runtime::TeensyConfig config_;
    mutable std::mutex mu_;
    std::optional<Frame> last_outbound_frame_;
    std::uint32_t next_sequence_{0};
};

}  // namespace posest::teensy
