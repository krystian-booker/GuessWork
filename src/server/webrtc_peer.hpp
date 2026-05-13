#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace gw::server {

// One WebRTC peer (one browser tab). Owns an rtc::PeerConnection set up for
// send-only H.264 video via a single track. Constructed empty; the caller
// passes a remote SDP offer to create_answer() and gets back an SDP answer
// once ICE gathering finishes (non-trickle for simple HTTP signaling).
class WebRtcPeer {
public:
    WebRtcPeer();
    ~WebRtcPeer();

    WebRtcPeer(const WebRtcPeer&)            = delete;
    WebRtcPeer& operator=(const WebRtcPeer&) = delete;

    // Sets the remote description from a browser SDP offer and creates a
    // matching answer. Blocks until ICE gathering completes (typically <100 ms
    // on localhost). Returns the SDP answer string. Throws on timeout / error.
    std::string create_answer(const std::string& offer_sdp);

    // Send one Annex-B-formatted H.264 frame. Non-blocking; libdatachannel
    // packetizes and queues internally. Safe to call from any thread.
    void send_h264(const std::vector<uint8_t>& annex_b, uint64_t pts_us);

    // Invoked once when the peer transitions to Closed or Failed state.
    void on_closed(std::function<void()> cb);

private:
    struct Impl;
    // shared_ptr so libdatachannel callbacks can capture a weak_ptr<Impl>
    // and lock it safely (close() is async; raw-pointer capture UAFs after
    // ~WebRtcPeer).
    std::shared_ptr<Impl> impl_;
};

}  // namespace gw::server
