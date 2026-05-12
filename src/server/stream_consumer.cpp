#include "server/stream_consumer.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>

#include "core/frame.hpp"
#include "encoder/h264_encoder.hpp"
#include "encoder/mono8_to_nv12.hpp"
#include "server/webrtc_peer.hpp"

namespace gw::server {

StreamConsumer::StreamConsumer(uint32_t out_w,
                               uint32_t out_h,
                               uint32_t target_fps,
                               uint32_t bitrate_bps)
    : fps_(target_fps) {
    conv_ = std::make_unique<gw::encoder::Mono8ToNv12>(out_w, out_h);

    auto cb = [this](std::vector<uint8_t> annex_b, bool /*keyframe*/, uint64_t pts_us) {
        on_encoded(std::move(annex_b), pts_us);
    };
    enc_ = std::make_unique<gw::encoder::H264Encoder>(
        out_w, out_h, target_fps, bitrate_bps, std::move(cb));
}

StreamConsumer::~StreamConsumer() {
    try { detach(); } catch (...) {}
}

void StreamConsumer::attach(gw::FrameChannel& ch) {
    if (running_.load()) return;
    channel_ = &ch;
    sub_     = ch.subscribe();
    running_.store(true);
    worker_  = std::thread([this] { run(); });
}

void StreamConsumer::detach() {
    if (!running_.exchange(false)) return;
    if (channel_ && sub_) channel_->unsubscribe(sub_);
    if (worker_.joinable()) worker_.join();
    sub_.reset();
    channel_ = nullptr;
}

void StreamConsumer::add_peer(std::shared_ptr<WebRtcPeer> peer) {
    WebRtcPeer* raw = peer.get();
    // Defer destruction to the encode thread: libdatachannel forbids
    // destroying a PeerConnection from inside its own callbacks.
    peer->on_closed([this, raw] {
        std::lock_guard lk(closed_mu_);
        closed_pending_.push_back(raw);
    });
    size_t count;
    {
        std::unique_lock lk(peers_mu_);
        peers_.push_back(std::move(peer));
        count = peers_.size();
    }
    if (enc_) enc_->request_keyframe();
    std::cerr << "StreamConsumer: peer added, count=" << count << "\n";
}

void StreamConsumer::purge_closed_peers() {
    std::vector<WebRtcPeer*> to_purge;
    {
        std::lock_guard lk(closed_mu_);
        to_purge.swap(closed_pending_);
    }
    if (to_purge.empty()) return;

    std::unique_lock lk(peers_mu_);
    peers_.erase(
        std::remove_if(peers_.begin(), peers_.end(),
                       [&](const std::shared_ptr<WebRtcPeer>& p) {
                           return std::find(to_purge.begin(), to_purge.end(),
                                            p.get()) != to_purge.end();
                       }),
        peers_.end());
}

size_t StreamConsumer::peer_count() const {
    std::shared_lock lk(peers_mu_);
    return peers_.size();
}

void StreamConsumer::run() {
    using clock_t = std::chrono::steady_clock;
    const auto period = std::chrono::microseconds(1'000'000 / fps_);
    auto       last_encode = clock_t::now() - period;  // allow first frame immediately

    while (running_.load(std::memory_order_acquire)) {
        gw::Frame* f = channel_->next_frame(sub_);
        if (!f) break;  // detached

        purge_closed_peers();

        const auto now = clock_t::now();

        bool has_peers;
        {
            std::shared_lock lk(peers_mu_);
            has_peers = !peers_.empty();
        }

        if (has_peers && (now - last_encode) >= period) {
            if (CVPixelBufferRef nv12 = conv_->convert(f->pixel_buffer())) {
                enc_->encode(nv12, f->host_capture_ns() / 1000);
                CFRelease(nv12);
                last_encode = now;
            }
        }

        f->release();
    }
}

void StreamConsumer::on_encoded(std::vector<uint8_t> annex_b, uint64_t pts_us) {
    encoded_frames_.fetch_add(1, std::memory_order_relaxed);

    std::shared_lock lk(peers_mu_);
    for (auto& p : peers_) {
        p->send_h264(annex_b, pts_us);
    }
}

}  // namespace gw::server
