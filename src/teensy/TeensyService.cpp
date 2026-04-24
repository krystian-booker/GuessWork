#include "posest/teensy/TeensyService.h"

#include <array>
#include <cstring>
#include <utility>

namespace posest::teensy {

namespace {

void appendDouble(std::vector<std::uint8_t>& out, double value) {
    std::array<std::uint8_t, sizeof(double)> bytes{};
    std::memcpy(bytes.data(), &value, sizeof(double));
    out.insert(out.end(), bytes.begin(), bytes.end());
}

void appendU32(std::vector<std::uint8_t>& out, std::uint32_t value) {
    out.push_back(static_cast<std::uint8_t>(value & 0xFFu));
    out.push_back(static_cast<std::uint8_t>((value >> 8u) & 0xFFu));
    out.push_back(static_cast<std::uint8_t>((value >> 16u) & 0xFFu));
    out.push_back(static_cast<std::uint8_t>((value >> 24u) & 0xFFu));
}

}  // namespace

TeensyService::TeensyService(runtime::TeensyConfig config)
    : config_(std::move(config)) {}

void TeensyService::publish(FusedPoseEstimate estimate) {
    Frame frame;
    frame.type = MessageType::FusedPose;
    frame.sequence = next_sequence_++;
    frame.payload = encodeFusedPosePayload(estimate);

    std::lock_guard<std::mutex> g(mu_);
    last_outbound_frame_ = std::move(frame);
}

std::optional<Frame> TeensyService::takeLastOutboundFrame() const {
    std::lock_guard<std::mutex> g(mu_);
    return last_outbound_frame_;
}

std::vector<std::uint8_t> TeensyService::encodeFusedPosePayload(
    const FusedPoseEstimate& estimate) {
    std::vector<std::uint8_t> payload;
    payload.reserve(4 + 3 * sizeof(double));
    appendDouble(payload, estimate.field_to_robot.x_m);
    appendDouble(payload, estimate.field_to_robot.y_m);
    appendDouble(payload, estimate.field_to_robot.theta_rad);
    appendU32(payload, estimate.status_flags);
    return payload;
}

}  // namespace posest::teensy
