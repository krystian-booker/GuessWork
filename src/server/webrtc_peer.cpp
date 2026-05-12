#include "server/webrtc_peer.hpp"

#include <rtc/rtc.hpp>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <future>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

namespace gw::server {

namespace {
constexpr uint32_t kVideoSsrc       = 0xdeadbeef;
// Payload type 109 in Chrome's offer is mapped to:
//   H264/90000 profile-level-id=42e01f packetization-mode=1
// which matches libdatachannel's DEFAULT_H264_VIDEO_PROFILE exactly. The
// browser's answer reuses the offer's codec→pt mapping, so we MUST pick a pt
// it already labels as H.264 — picking an arbitrary pt like 96 (which Chrome
// reserves for VP8) means our RTP packets get routed to the VP8 decoder.
constexpr uint8_t  kVideoPayload    = 109;
constexpr auto     kIceGatherTimeout = std::chrono::seconds(5);
}  // namespace

struct WebRtcPeer::Impl {
    std::shared_ptr<rtc::PeerConnection>          pc;
    std::shared_ptr<rtc::Track>                   track;
    std::shared_ptr<rtc::RtpPacketizationConfig>  rtp_config;

    std::mutex            cb_mu;
    std::function<void()> close_cb;
    bool                  fired_close = false;

    void notify_closed() {
        std::function<void()> local;
        {
            std::lock_guard lk(cb_mu);
            if (fired_close || !close_cb) return;
            fired_close = true;
            local       = close_cb;  // copy so we don't hold the lock during user callback
        }
        try { local(); } catch (...) {}
    }
};

WebRtcPeer::WebRtcPeer() : impl_(std::make_unique<Impl>()) {
    rtc::Configuration cfg;
    // Empty ICE servers — localhost works on host candidates alone.
    cfg.disableAutoNegotiation = false;

    impl_->pc = std::make_shared<rtc::PeerConnection>(cfg);

    auto* impl_raw = impl_.get();
    impl_->pc->onStateChange([impl_raw](rtc::PeerConnection::State state) {
        using S = rtc::PeerConnection::State;
        if (state == S::Closed || state == S::Failed || state == S::Disconnected) {
            impl_raw->notify_closed();
        }
    });

    // The mid here MUST match the mid of the m-line in the remote offer.
    // libdatachannel's processLocalDescription() matches local tracks to remote
    // m-lines by mid (peerconnection.cpp:908); if no match, it reciprocates
    // the offer's media block WITHOUT our SSRC/codec config — RTP flows but
    // the browser can't bind it to an inbound stream. Browsers use "0" for
    // the first m-line in a single-media offer, which is our only case.
    rtc::Description::Video media("0", rtc::Description::Direction::SendOnly);
    media.addH264Codec(static_cast<int>(kVideoPayload));
    media.addSSRC(kVideoSsrc, "video-send", "guesswork-stream", "video-track");

    impl_->track = impl_->pc->addTrack(media);

    impl_->rtp_config = std::make_shared<rtc::RtpPacketizationConfig>(
        kVideoSsrc,
        "video-send",
        kVideoPayload,
        rtc::H264RtpPacketizer::defaultClockRate);

    auto packetizer = std::make_shared<rtc::H264RtpPacketizer>(
        rtc::NalUnit::Separator::StartSequence,
        impl_->rtp_config);

    impl_->track->setMediaHandler(packetizer);
}

WebRtcPeer::~WebRtcPeer() {
    try {
        if (impl_ && impl_->pc) {
            impl_->pc->close();
        }
    } catch (...) {}
}

std::string WebRtcPeer::create_answer(const std::string& offer_sdp) {
    auto gathering_done = std::make_shared<std::promise<void>>();
    auto future         = gathering_done->get_future();

    impl_->pc->onGatheringStateChange(
        [gathering_done](rtc::PeerConnection::GatheringState state) {
            if (state == rtc::PeerConnection::GatheringState::Complete) {
                // Guard against re-entry: set_value throws on second call.
                try { gathering_done->set_value(); } catch (...) {}
            }
        });

    // setRemoteDescription with tracks added triggers answer generation and
    // starts ICE gathering.
    impl_->pc->setRemoteDescription(rtc::Description(offer_sdp, "offer"));

    if (future.wait_for(kIceGatherTimeout) != std::future_status::ready) {
        throw std::runtime_error("WebRtcPeer: ICE gathering timed out");
    }

    auto local = impl_->pc->localDescription();
    if (!local) {
        throw std::runtime_error("WebRtcPeer: no local description after gathering");
    }
    return std::string(*local);
}

void WebRtcPeer::send_h264(const std::vector<uint8_t>& annex_b, uint64_t pts_us) {
    if (!impl_->track || !impl_->track->isOpen() || annex_b.empty()) return;

    // Sets the RTP timestamp that the packetizer will use for this packet.
    impl_->rtp_config->timestamp =
        impl_->rtp_config->secondsToTimestamp(static_cast<double>(pts_us) / 1'000'000.0);

    rtc::binary msg(annex_b.size());
    std::memcpy(msg.data(), annex_b.data(), annex_b.size());

    try {
        impl_->track->send(std::move(msg));
    } catch (const std::exception& e) {
        std::cerr << "WebRtcPeer::send_h264 failed: " << e.what() << "\n";
    }
}

void WebRtcPeer::on_closed(std::function<void()> cb) {
    std::lock_guard lk(impl_->cb_mu);
    impl_->close_cb = std::move(cb);
}

}  // namespace gw::server
