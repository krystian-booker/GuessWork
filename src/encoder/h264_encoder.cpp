#include "encoder/h264_encoder.hpp"

#include <CoreFoundation/CoreFoundation.h>
#include <CoreMedia/CoreMedia.h>

#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

namespace gw::encoder {

namespace {

constexpr uint8_t kAnnexBStart[4] = {0x00, 0x00, 0x00, 0x01};

void vt_set_int(VTCompressionSessionRef s, CFStringRef key, int32_t value) {
    CFNumberRef n = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &value);
    VTSessionSetProperty(s, key, n);
    CFRelease(n);
}

}  // namespace

H264Encoder::H264Encoder(uint32_t width,
                         uint32_t height,
                         uint32_t target_fps,
                         uint32_t bitrate_bps,
                         FrameCallback cb)
    : cb_(std::move(cb)), w_(width), h_(height) {

    CFMutableDictionaryRef encoder_spec = CFDictionaryCreateMutable(
        kCFAllocatorDefault, 0,
        &kCFTypeDictionaryKeyCallBacks,
        &kCFTypeDictionaryValueCallBacks);
    CFDictionarySetValue(encoder_spec,
        kVTVideoEncoderSpecification_EnableHardwareAcceleratedVideoEncoder,
        kCFBooleanTrue);

    const OSStatus s = VTCompressionSessionCreate(
        kCFAllocatorDefault,
        static_cast<int32_t>(width),
        static_cast<int32_t>(height),
        kCMVideoCodecType_H264,
        encoder_spec,
        /*sourceImageBufferAttributes=*/nullptr,
        /*compressedDataAllocator=*/nullptr,
        &vt_output_callback,
        this,
        &session_);
    CFRelease(encoder_spec);

    if (s != noErr || !session_) {
        throw std::runtime_error("VTCompressionSessionCreate failed: " +
                                 std::to_string(s));
    }

    VTSessionSetProperty(session_, kVTCompressionPropertyKey_RealTime,            kCFBooleanTrue);
    VTSessionSetProperty(session_, kVTCompressionPropertyKey_AllowFrameReordering, kCFBooleanFalse);
    VTSessionSetProperty(session_, kVTCompressionPropertyKey_ProfileLevel,
                         kVTProfileLevel_H264_Baseline_AutoLevel);

    vt_set_int(session_, kVTCompressionPropertyKey_AverageBitRate,
               static_cast<int32_t>(bitrate_bps));
    vt_set_int(session_, kVTCompressionPropertyKey_ExpectedFrameRate,
               static_cast<int32_t>(target_fps));
    vt_set_int(session_, kVTCompressionPropertyKey_MaxKeyFrameInterval,
               static_cast<int32_t>(target_fps) * 2);  // ~2s keyframe interval

    VTCompressionSessionPrepareToEncodeFrames(session_);
}

H264Encoder::~H264Encoder() {
    if (session_) {
        VTCompressionSessionCompleteFrames(session_, kCMTimeInvalid);
        VTCompressionSessionInvalidate(session_);
        CFRelease(session_);
        session_ = nullptr;
    }
}

void H264Encoder::request_keyframe() {
    keyframe_requested_.store(true, std::memory_order_relaxed);
}

void H264Encoder::encode(CVPixelBufferRef nv12, uint64_t pts_us) {
    if (!session_ || !nv12) return;

    const CMTime pts      = CMTimeMake(static_cast<int64_t>(pts_us), 1'000'000);
    const CMTime duration = kCMTimeInvalid;

    CFDictionaryRef frame_props = nullptr;
    if (keyframe_requested_.exchange(false, std::memory_order_relaxed)) {
        CFMutableDictionaryRef d = CFDictionaryCreateMutable(
            kCFAllocatorDefault, 0,
            &kCFTypeDictionaryKeyCallBacks,
            &kCFTypeDictionaryValueCallBacks);
        CFDictionarySetValue(d, kVTEncodeFrameOptionKey_ForceKeyFrame, kCFBooleanTrue);
        frame_props = d;
    }

    VTCompressionSessionEncodeFrame(
        session_, nv12, pts, duration,
        frame_props,
        /*sourceFrameRefcon=*/nullptr,
        /*infoFlagsOut=*/nullptr);

    if (frame_props) CFRelease(frame_props);
}

// Runs on a VideoToolbox internal thread; serialized per session.
void H264Encoder::vt_output_callback(void* ref_con,
                                     void* /*src_frame_ref_con*/,
                                     OSStatus status,
                                     VTEncodeInfoFlags /*info_flags*/,
                                     CMSampleBufferRef sample) {
    auto* self = static_cast<H264Encoder*>(ref_con);
    self->on_encoded(status, sample);
}

void H264Encoder::on_encoded(OSStatus status, CMSampleBufferRef sample) {
    if (status != noErr || !sample || !CMSampleBufferDataIsReady(sample)) return;

    // Keyframe? Check the NotSync attachment on sample 0.
    bool keyframe = true;
    if (CFArrayRef attachments = CMSampleBufferGetSampleAttachmentsArray(sample, false)) {
        if (CFArrayGetCount(attachments) > 0) {
            auto a = static_cast<CFDictionaryRef>(CFArrayGetValueAtIndex(attachments, 0));
            auto not_sync = static_cast<CFBooleanRef>(
                CFDictionaryGetValue(a, kCMSampleAttachmentKey_NotSync));
            if (not_sync && CFBooleanGetValue(not_sync)) keyframe = false;
        }
    }

    std::vector<uint8_t> out;
    out.reserve(64 * 1024);

    // On keyframes, prepend SPS + PPS (and any subsequent parameter sets) so a
    // receiver joining at this IDR has the parameter sets in-band.
    if (keyframe) {
        CMVideoFormatDescriptionRef fmt = CMSampleBufferGetFormatDescription(sample);
        if (fmt) {
            size_t param_count = 0;
            int    nal_hdr_size = 0;
            CMVideoFormatDescriptionGetH264ParameterSetAtIndex(
                fmt, 0, nullptr, nullptr, &param_count, &nal_hdr_size);
            for (size_t i = 0; i < param_count; ++i) {
                const uint8_t* nal_data = nullptr;
                size_t         nal_size = 0;
                if (CMVideoFormatDescriptionGetH264ParameterSetAtIndex(
                        fmt, i, &nal_data, &nal_size, nullptr, nullptr) == noErr) {
                    out.insert(out.end(), std::begin(kAnnexBStart), std::end(kAnnexBStart));
                    out.insert(out.end(), nal_data, nal_data + nal_size);
                }
            }
        }
    }

    // AVCC payload format: 4-byte big-endian length, then NAL bytes, repeating.
    CMBlockBufferRef block = CMSampleBufferGetDataBuffer(sample);
    size_t total = 0;
    char*  base  = nullptr;
    if (CMBlockBufferGetDataPointer(block, 0, nullptr, &total, &base) == noErr && base) {
        size_t off = 0;
        while (off + 4 <= total) {
            uint32_t nal_size_be = 0;
            std::memcpy(&nal_size_be, base + off, 4);
            const uint32_t nal_size = CFSwapInt32BigToHost(nal_size_be);
            off += 4;
            if (nal_size == 0 || off + nal_size > total) break;
            out.insert(out.end(), std::begin(kAnnexBStart), std::end(kAnnexBStart));
            out.insert(out.end(),
                       reinterpret_cast<uint8_t*>(base) + off,
                       reinterpret_cast<uint8_t*>(base) + off + nal_size);
            off += nal_size;
        }
    }

    if (out.empty()) return;

    const CMTime pts = CMSampleBufferGetPresentationTimeStamp(sample);
    const uint64_t pts_us = (CMTIME_IS_VALID(pts))
        ? static_cast<uint64_t>(CMTimeGetSeconds(pts) * 1'000'000.0)
        : 0;

    if (cb_) cb_(std::move(out), keyframe, pts_us);
}

}  // namespace gw::encoder
