#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <shared_mutex>
#include <string_view>
#include <thread>
#include <vector>

#include "consumer/consumer.hpp"
#include "core/frame_channel.hpp"

namespace gw::encoder {
class H264Encoder;
class Mono8ToNv12;
}

namespace gw::server {

class WebRtcPeer;

// Subscribes to a FrameChannel, downsamples Mono8 -> NV12, encodes H.264 via
// VideoToolbox, and pushes the encoded Annex-B byte stream to every registered
// WebRTC peer. Implements the existing IConsumer pattern so it slots into the
// same producer pipeline as PreviewConsumer.
//
// Performance:
//   - Encoder + converter only run when there is at least one registered peer.
//     With zero peers, the worker thread pulls frames from the channel and
//     immediately releases them — no encoding work at all.
//   - Encoding is rate-limited to the configured target FPS regardless of the
//     producer's higher native frame rate.
class StreamConsumer final : public gw::IConsumer {
public:
    StreamConsumer(uint32_t out_w,
                   uint32_t out_h,
                   uint32_t target_fps,
                   uint32_t bitrate_bps);
    ~StreamConsumer() override;

    StreamConsumer(const StreamConsumer&)            = delete;
    StreamConsumer& operator=(const StreamConsumer&) = delete;

    std::string_view name()   const override { return name_; }
    void             attach(gw::FrameChannel& ch) override;
    void             detach() override;

    // Add a peer to the broadcast set. The consumer takes ownership of the
    // shared_ptr and wires up its close handler so the peer is destroyed
    // safely on the encode thread (not from inside libdatachannel callbacks).
    // Adding a peer triggers a keyframe request so the new peer can decode
    // immediately.
    void add_peer(std::shared_ptr<WebRtcPeer> peer);

    // Diagnostic counters.
    uint64_t encoded_frame_count() const { return encoded_frames_.load(std::memory_order_relaxed); }
    size_t   peer_count()          const;

private:
    void run();
    void on_encoded(std::vector<uint8_t> annex_b, uint64_t pts_us);
    void purge_closed_peers();

    std::string name_ = "stream";

    uint32_t fps_;

    std::unique_ptr<gw::encoder::Mono8ToNv12> conv_;
    std::unique_ptr<gw::encoder::H264Encoder> enc_;

    gw::FrameChannel*                  channel_ = nullptr;
    gw::FrameChannel::SubscriberHandle sub_;

    std::atomic<bool> running_{false};
    std::thread       worker_;

    mutable std::shared_mutex                peers_mu_;
    std::vector<std::shared_ptr<WebRtcPeer>> peers_;

    std::mutex                               closed_mu_;
    std::vector<WebRtcPeer*>                 closed_pending_;

    std::atomic<uint64_t> encoded_frames_{0};
};

}  // namespace gw::server
