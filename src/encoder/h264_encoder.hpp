#pragma once

#include <CoreVideo/CoreVideo.h>
#include <VideoToolbox/VideoToolbox.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace gw::encoder {

// Hardware H.264 encoder around VTCompressionSession. Configured for low
// latency, Baseline profile, constant-ish bitrate, no frame reordering.
//
// Output is Annex-B-formatted: each NAL unit prefixed by 0x00 00 00 01. On
// keyframes the SPS and PPS NALs are prepended so a receiver joining at any
// keyframe boundary has everything it needs to decode forward.
class H264Encoder {
public:
    using FrameCallback = std::function<void(std::vector<uint8_t> annex_b,
                                             bool                 keyframe,
                                             uint64_t             pts_us)>;

    H264Encoder(uint32_t width,
                uint32_t height,
                uint32_t target_fps,
                uint32_t bitrate_bps,
                FrameCallback cb);
    ~H264Encoder();

    H264Encoder(const H264Encoder&)            = delete;
    H264Encoder& operator=(const H264Encoder&) = delete;

    // Submit a frame. The frame callback may be invoked synchronously or from
    // a VideoToolbox internal thread, possibly after this call returns.
    void encode(CVPixelBufferRef nv12, uint64_t pts_us);

    // Hint VT that the next encoded frame should be an IDR.
    void request_keyframe();

    uint32_t width()  const { return w_; }
    uint32_t height() const { return h_; }

private:
    static void vt_output_callback(void* ref_con,
                                   void* src_frame_ref_con,
                                   OSStatus status,
                                   VTEncodeInfoFlags info_flags,
                                   CMSampleBufferRef sample);
    void on_encoded(OSStatus status, CMSampleBufferRef sample);

    VTCompressionSessionRef session_ = nullptr;
    FrameCallback           cb_;
    uint32_t                w_;
    uint32_t                h_;
    std::atomic<bool>       keyframe_requested_{false};
};

}  // namespace gw::encoder
