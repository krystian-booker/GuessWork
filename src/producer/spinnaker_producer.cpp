#include "producer/spinnaker_producer.hpp"
#include "producer/spinnaker_producer_internal.hpp"

#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>

#include <CoreVideo/CoreVideo.h>

#include <SpinGenApi/SpinnakerGenApi.h>

#include "core/clock.hpp"
#include "core/frame.hpp"
#include "producer/spinnaker_user_buffer_pool.hpp"
#include "producer/spinnaker_video_modes.hpp"

namespace gw {

namespace {

// Per-camera user-buffer pool size. Spinnaker NewestOnly requires ≥ 3;
// we add headroom for the publisher's working slot, channel-latest, and one
// or two consumer in-flight frames.
constexpr uint32_t kPoolCapacity = 6;
constexpr uint64_t kGetNextImageTimeoutMs = 1000;

void set_enum_node(Spinnaker::GenApi::INodeMap& nm,
                   const char*                 node_name,
                   const char*                 entry_name) {
    Spinnaker::GenApi::CEnumerationPtr ptr = nm.GetNode(node_name);
    if (!Spinnaker::GenApi::IsReadable(ptr) || !Spinnaker::GenApi::IsWritable(ptr)) {
        throw std::runtime_error(std::string("Spinnaker node not writable: ") + node_name);
    }
    Spinnaker::GenApi::CEnumEntryPtr entry = ptr->GetEntryByName(entry_name);
    if (!Spinnaker::GenApi::IsReadable(entry)) {
        throw std::runtime_error(std::string("Spinnaker entry not available: ") + node_name +
                                 "::" + entry_name);
    }
    ptr->SetIntValue(entry->GetValue());
}

int64_t read_int_node(Spinnaker::GenApi::INodeMap& nm, const char* node_name) {
    Spinnaker::GenApi::CIntegerPtr ptr = nm.GetNode(node_name);
    if (!Spinnaker::GenApi::IsReadable(ptr)) {
        throw std::runtime_error(std::string("Spinnaker node not readable: ") + node_name);
    }
    return ptr->GetValue();
}

void set_int_node(Spinnaker::GenApi::INodeMap& nm,
                  const char*                  node_name,
                  int64_t                      value) {
    Spinnaker::GenApi::CIntegerPtr ptr = nm.GetNode(node_name);
    if (!Spinnaker::GenApi::IsWritable(ptr)) {
        throw std::runtime_error(std::string("Spinnaker node not writable: ") + node_name);
    }
    ptr->SetValue(value);
}

void set_bool_node(Spinnaker::GenApi::INodeMap& nm,
                   const char*                  node_name,
                   bool                         value) {
    Spinnaker::GenApi::CBooleanPtr ptr = nm.GetNode(node_name);
    if (!Spinnaker::GenApi::IsWritable(ptr)) {
        throw std::runtime_error(std::string("Spinnaker node not writable: ") + node_name);
    }
    ptr->SetValue(value);
}

// Configure the camera as a hardware-trigger slave on Line0. The Spinnaker
// nodemap requires TriggerMode=Off before changing the selector / source, so
// we toggle Off → write → On regardless of the previous state.
void configure_hardware_trigger(Spinnaker::GenApi::INodeMap& nm) {
    set_enum_node(nm, "TriggerMode",       "Off");
    set_enum_node(nm, "TriggerSelector",   "FrameStart");
    set_enum_node(nm, "TriggerSource",     "Line0");
    set_enum_node(nm, "TriggerActivation", "RisingEdge");
    // TriggerOverlap=ReadOut lets the camera arm its next trigger during the
    // current frame readout, removing a one-frame deadtime. Some models
    // (e.g. Chameleon3) only expose "Off"; tolerate the absence.
    try { set_enum_node(nm, "TriggerOverlap", "ReadOut"); } catch (...) {}
    set_enum_node(nm, "TriggerMode",       "On");
}

// Per-node helpers for the live-settings path. Each tolerates the node being
// absent or non-readable so partial-support cameras drop fields silently
// rather than fail the whole apply.

std::optional<double> try_read_float(Spinnaker::GenApi::INodeMap& nm, const char* name) {
    try {
        Spinnaker::GenApi::CFloatPtr p = nm.GetNode(name);
        if (Spinnaker::GenApi::IsReadable(p)) return p->GetValue();
    } catch (...) {}
    return std::nullopt;
}

// Anything other than "Off" (e.g. "Continuous", "Once") reads back as true.
std::optional<bool> try_read_auto_enum(Spinnaker::GenApi::INodeMap& nm, const char* name) {
    try {
        Spinnaker::GenApi::CEnumerationPtr p = nm.GetNode(name);
        if (!Spinnaker::GenApi::IsReadable(p)) return std::nullopt;
        Spinnaker::GenApi::CEnumEntryPtr cur = p->GetCurrentEntry();
        if (!Spinnaker::GenApi::IsReadable(cur)) return std::nullopt;
        const std::string sym(cur->GetSymbolic().c_str());
        return sym != "Off";
    } catch (...) { return std::nullopt; }
}

// Returns the post-clamp, post-quantization readback value.
std::optional<double> try_write_float(Spinnaker::GenApi::INodeMap& nm,
                                      const char*                  name,
                                      double                       value) {
    try {
        Spinnaker::GenApi::CFloatPtr p = nm.GetNode(name);
        if (!Spinnaker::GenApi::IsWritable(p)) return std::nullopt;
        const double lo = p->GetMin();
        const double hi = p->GetMax();
        if (value < lo) value = lo;
        if (value > hi) value = hi;
        p->SetValue(value);
        return p->GetValue();
    } catch (...) { return std::nullopt; }
}

std::optional<bool> try_write_auto_enum(Spinnaker::GenApi::INodeMap& nm,
                                        const char*                  name,
                                        bool                         on) {
    try {
        Spinnaker::GenApi::CEnumerationPtr p = nm.GetNode(name);
        if (!Spinnaker::GenApi::IsWritable(p)) return std::nullopt;
        Spinnaker::GenApi::CEnumEntryPtr entry =
            p->GetEntryByName(on ? "Continuous" : "Off");
        if (!Spinnaker::GenApi::IsReadable(entry)) return std::nullopt;
        p->SetIntValue(entry->GetValue());
        return try_read_auto_enum(nm, name);
    } catch (...) { return std::nullopt; }
}

std::optional<CameraSettingRange> read_range(Spinnaker::GenApi::INodeMap& nm,
                                             const char*                  name) {
    try {
        Spinnaker::GenApi::CFloatPtr p = nm.GetNode(name);
        if (!Spinnaker::GenApi::IsReadable(p)) return std::nullopt;
        CameraSettingRange r;
        r.min  = p->GetMin();
        r.max  = p->GetMax();
        try { r.unit = std::string(p->GetUnit().c_str()); } catch (...) {}
        return r;
    } catch (...) { return std::nullopt; }
}

CameraSettingsValues read_all_settings(Spinnaker::GenApi::INodeMap& nm) {
    CameraSettingsValues v;
    v.gain_auto     = try_read_auto_enum(nm, "GainAuto");
    v.gain          = try_read_float    (nm, "Gain");
    v.exposure_auto = try_read_auto_enum(nm, "ExposureAuto");
    v.exposure      = try_read_float    (nm, "ExposureTime");
    return v;
}

CameraSettingsLimits read_all_limits(Spinnaker::GenApi::INodeMap& nm) {
    CameraSettingsLimits l;
    l.gain     = read_range(nm, "Gain");
    l.exposure = read_range(nm, "ExposureTime");
    return l;
}

// BlackLevelEnabled adds a pedestal that hurts AprilTag detection; we never
// expose it as a setting, so force it off where the camera supports it.
void try_disable_black_level(Spinnaker::GenApi::INodeMap& nm) {
    try {
        Spinnaker::GenApi::CBooleanPtr p = nm.GetNode("BlackLevelEnabled");
        if (!Spinnaker::GenApi::IsWritable(p)) return;
        p->SetValue(false);
    } catch (...) {}
}

// Caller holds the nodemap mutex.
CameraSettingsValues apply_patch_locked(Spinnaker::GenApi::INodeMap&   nm,
                                        const CameraSettingsPatch&     patch) {
    CameraSettingsValues out;

    auto apply_auto_block = [&](const char*               auto_node,
                                const char*               value_node,
                                std::optional<bool>       auto_in,
                                std::optional<double>     value_in,
                                std::optional<bool>&      out_auto,
                                std::optional<double>&    out_value) {
        // Going auto→manual without an explicit value: seed the manual value
        // from the current converged reading so we don't snap to a stale one.
        std::optional<double> seeded_value = value_in;
        if (auto_in && !*auto_in && !value_in) {
            seeded_value = try_read_float(nm, value_node);
        }
        if (auto_in) {
            out_auto = try_write_auto_enum(nm, auto_node, *auto_in);
        }
        if (seeded_value) {
            out_value = try_write_float(nm, value_node, *seeded_value);
            if (!out_value) {
                out_value = try_read_float(nm, value_node);
            }
        } else {
            out_value = try_read_float(nm, value_node);
        }
    };

    apply_auto_block("GainAuto",     "Gain",
                     patch.gain_auto,     patch.gain,
                     out.gain_auto,       out.gain);
    apply_auto_block("ExposureAuto", "ExposureTime",
                     patch.exposure_auto, patch.exposure,
                     out.exposure_auto,   out.exposure);

    return out;
}

bool patch_is_empty(const CameraSettingsPatch& p) {
    return !p.gain_auto && !p.gain && !p.exposure_auto && !p.exposure;
}

}  // namespace

// -------------------------------------------------------------------------
// Impl
// -------------------------------------------------------------------------

struct SpinnakerProducer::Impl {
    // Declaration order is load-bearing: `channel` must be destroyed BEFORE
    // `user_pool` because the channel may hold a Frame whose recycle callback
    // points back into the pool. C++ destroys members in reverse declaration
    // order, so user_pool is listed first.
    std::string                              name;
    std::string                              serial;
    std::optional<std::string>               mode;
    CameraSettingsValues                     initial_settings;
    HardwareSyncConfig                       hw_sync;
    FrameFormat                              format{};
    VideoModeList                            cached_modes;
    CameraSettingsLimits                     cached_limits;
    std::optional<SpinnakerUserBufferPool>   user_pool;
    FrameChannel                             channel;
    std::unique_ptr<SpinnakerCameraBinding>  binding;       // system + cam smart ptrs
    std::atomic<bool>                        stop_requested{false};
    std::atomic<bool>                        streaming{false};
    std::atomic<bool>                        owner_user{false};
    std::thread                              worker;
    uint64_t                                 sequence_counter = 0;
    // Serializes nodemap writes between apply_settings_live() and the start()
    // initial-apply path. The capture loop does NOT take this mutex —
    // GetNextImage is independent of nodemap state and we must not stall a
    // slider write behind a frame timeout.
    std::mutex                               nodemap_mu;

    std::atomic<uint64_t>                    total_published{0};
    std::atomic<uint64_t>                    total_dropped{0};
    std::atomic<uint64_t>                    total_incomplete{0};

    Impl(std::string n, std::string s,
         std::optional<std::string> m, CameraSettingsValues init,
         HardwareSyncConfig hs)
        : name(std::move(n)), serial(std::move(s)), mode(std::move(m)),
          initial_settings(std::move(init)), hw_sync(hs) {}

    void capture_loop();
};

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------

SpinnakerProducer::SpinnakerProducer(std::string                name,
                                     std::string                serial,
                                     std::optional<std::string> mode,
                                     CameraSettingsValues       initial_settings,
                                     HardwareSyncConfig         hw_sync)
    : impl_(std::make_unique<Impl>(std::move(name), std::move(serial),
                                   std::move(mode), std::move(initial_settings),
                                   hw_sync)) {}

SpinnakerProducer::~SpinnakerProducer() {
    try {
        stop();
    } catch (...) {
        // Destructors must not throw.
    }
}

std::string_view     SpinnakerProducer::name()   const { return impl_->name; }
std::string_view     SpinnakerProducer::serial() const { return impl_->serial; }
FrameFormat          SpinnakerProducer::format() const { return impl_->format; }
FrameChannel&        SpinnakerProducer::channel()      { return impl_->channel; }
const VideoModeList& SpinnakerProducer::cached_video_modes() const {
    return impl_->cached_modes;
}

const CameraSettingsLimits& SpinnakerProducer::cached_settings_limits() const {
    return impl_->cached_limits;
}

CameraSettingsValues SpinnakerProducer::apply_settings_live(const CameraSettingsPatch& patch) {
    if (!impl_->streaming.load() || !impl_->binding || !impl_->binding->cam) {
        throw std::runtime_error("apply_settings_live: producer not streaming");
    }
    std::lock_guard<std::mutex> lk(impl_->nodemap_mu);
    Spinnaker::GenApi::INodeMap& nm = impl_->binding->cam->GetNodeMap();
    return apply_patch_locked(nm, patch);
}

CameraSettingsValues SpinnakerProducer::current_settings() {
    if (!impl_->streaming.load() || !impl_->binding || !impl_->binding->cam) {
        throw std::runtime_error("current_settings: producer not streaming");
    }
    std::lock_guard<std::mutex> lk(impl_->nodemap_mu);
    Spinnaker::GenApi::INodeMap& nm = impl_->binding->cam->GetNodeMap();
    return read_all_settings(nm);
}

void SpinnakerProducer::bind_camera(std::unique_ptr<SpinnakerCameraBinding> binding) {
    if (impl_->streaming.load()) {
        throw std::runtime_error("SpinnakerProducer::bind_camera called while streaming");
    }
    impl_->binding = std::move(binding);
}

void SpinnakerProducer::start() {
    if (impl_->streaming.load()) return;
    if (!impl_->binding || !impl_->binding->cam) {
        throw std::runtime_error("SpinnakerProducer::start without bound camera");
    }

    Spinnaker::CameraPtr& cam = impl_->binding->cam;

    try {
        cam->Init();

        // Force NewestOnly buffer mode on the TL stream. The buffer count must
        // also be pinned to the size of the user pool we're about to register:
        // - In Auto count mode Spinnaker sizes its internal queue independently
        //   of SetUserBuffers — typically 3 — which causes the acquisition
        //   engine to starve on DMA after a few hundred frames whenever the
        //   application holds even one buffer briefly.
        // - In Manual count mode the count defaults to the camera's prior value
        //   (often still 3), so we must explicitly set StreamBufferCountManual
        //   to match kPoolCapacity, otherwise Spinnaker silently uses fewer
        //   slots than we provided in SetUserBuffers and the acquisition engine
        //   wedges once the steady-state in-flight count exceeds its quota.
        Spinnaker::GenApi::INodeMap& tl_stream_nm = cam->GetTLStreamNodeMap();
        set_enum_node(tl_stream_nm, "StreamBufferHandlingMode", "NewestOnly");
        set_enum_node(tl_stream_nm, "StreamBufferCountMode",    "Manual");
        set_int_node (tl_stream_nm, "StreamBufferCountManual",  kPoolCapacity);

        // Cache the available modes for this camera while we hold Init.
        impl_->cached_modes = enumerate_video_modes_initialized(cam);

        // Force Mono8 on the device. The VideoMode (sensor mode) must be set
        // FIRST since it determines which pixel formats are advertised.
        Spinnaker::GenApi::INodeMap& dev_nm = cam->GetNodeMap();
        if (impl_->mode && !impl_->mode->empty()) {
            set_enum_node(dev_nm, "VideoMode", impl_->mode->c_str());
        }
        set_enum_node(dev_nm, "PixelFormat",     "Mono8");
        set_enum_node(dev_nm, "AcquisitionMode", "Continuous");  // belt-and-suspenders, default on Chameleon3
        // Chunk mode must be off in BOTH modes: it survives until power-cycle
        // and inflates PayloadSize past width*height, which fails the
        // buffer-pool size checks below. (Frame-drop tracking uses the U3V
        // transport-layer frame id instead — see capture_loop.)
        try { set_bool_node(dev_nm, "ChunkModeActive", false); } catch (...) {}
        if (impl_->hw_sync.enabled) {
            configure_hardware_trigger(dev_nm);
        } else {
            // Leave a previously-configured camera in a known-freerun state:
            // toggling hw-sync off MUST be visible at the camera level, not
            // just at the supervisor / DB level.
            try { set_enum_node(dev_nm, "TriggerMode", "Off"); } catch (...) {}
        }
        // Reset the pulse stamper's per-pin state so the first frame after
        // (re)start seeds a fresh FrameID baseline. Safe even when hw-sync is
        // off: the stamper just clears nonexistent state.
        if (impl_->hw_sync.stamper) {
            impl_->hw_sync.stamper->reset_pin_state(impl_->hw_sync.trigger_output_pin);
        }

        const int64_t w = read_int_node(dev_nm, "Width");
        const int64_t h = read_int_node(dev_nm, "Height");
        impl_->format = FrameFormat{
            .width        = static_cast<uint32_t>(w),
            .height       = static_cast<uint32_t>(h),
            .pixel_format = kCVPixelFormatType_OneComponent8,
        };
        const int64_t payload = read_int_node(dev_nm, "PayloadSize");
        std::cout << "[" << impl_->name << "/" << impl_->serial << "] "
                  << w << "x" << h << " Mono8 (payload " << payload << " B)\n";

        impl_->user_pool.emplace(impl_->format, kPoolCapacity, static_cast<uint64_t>(payload));

        // Stride sanity: for Mono8 with no row padding, Spinnaker writes
        // `width` bytes/row and our IOSurface must match.
        const size_t bpr = impl_->user_pool->bytes_per_row();
        if (bpr != static_cast<size_t>(w)) {
            throw std::runtime_error(
                "IOSurface row stride (" + std::to_string(bpr) +
                ") does not match camera width (" + std::to_string(w) + ")");
        }
        if (payload != w * h) {
            throw std::runtime_error(
                "Camera PayloadSize (" + std::to_string(payload) +
                ") does not match width*height (" + std::to_string(w * h) + ")");
        }

        auto addrs = impl_->user_pool->base_addresses();
        cam->SetBufferOwnership(Spinnaker::SPINNAKER_BUFFER_OWNERSHIP_USER);
        impl_->owner_user.store(true);
        cam->SetUserBuffers(addrs.data(),
                            static_cast<uint64_t>(addrs.size()),
                            impl_->user_pool->buffer_size());

        cam->BeginAcquisition();
        impl_->streaming.store(true);

        // Best-effort initial setup against a now-streaming camera. A
        // partial-support camera should still stream if any of this fails.
        {
            std::lock_guard<std::mutex> lk(impl_->nodemap_mu);
            impl_->cached_limits = read_all_limits(dev_nm);
            try_disable_black_level(dev_nm);
            if (!patch_is_empty(impl_->initial_settings)) {
                try {
                    (void)apply_patch_locked(dev_nm, impl_->initial_settings);
                } catch (const Spinnaker::Exception& e) {
                    std::cerr << "[" << impl_->name << "] initial settings apply failed: "
                              << e.what() << "\n";
                }
            }
        }
    } catch (const Spinnaker::Exception& e) {
        // Best-effort cleanup so a retry of start() (or destructor) is safe.
        try { cam->DeInit(); } catch (...) {}
        impl_->user_pool.reset();
        throw std::runtime_error(std::string("Spinnaker error during start(): ") + e.what());
    }

    impl_->stop_requested.store(false);
    impl_->worker = std::thread([this] { impl_->capture_loop(); });
}

void SpinnakerProducer::stop() {
    if (!impl_) return;
    if (impl_->worker.joinable()) {
        impl_->stop_requested.store(true);
        impl_->worker.join();
    }
    if (impl_->binding && impl_->binding->cam) {
        Spinnaker::CameraPtr& cam = impl_->binding->cam;
        if (impl_->streaming.exchange(false)) {
            try { cam->EndAcquisition(); } catch (...) {}
        }
        if (impl_->owner_user.exchange(false)) {
            try {
                cam->SetBufferOwnership(Spinnaker::SPINNAKER_BUFFER_OWNERSHIP_SYSTEM);
            } catch (...) {}
        }
        try { cam->DeInit(); } catch (...) {}
    }
    // Do NOT reset user_pool here: the FrameChannel may still hold a Frame
    // whose recycle callback points back into the pool. Pool destruction is
    // deferred to ~Impl, where the channel is destroyed first (member
    // declaration order is load-bearing).
    impl_->binding.reset();   // releases CameraPtr/SystemPtr refcounts
}

// -------------------------------------------------------------------------
// Capture loop
// -------------------------------------------------------------------------

void SpinnakerProducer::Impl::capture_loop() {
    Spinnaker::CameraPtr& cam = binding->cam;
    while (!stop_requested.load(std::memory_order_acquire)) {
        Spinnaker::ImagePtr img;
        try {
            img = cam->GetNextImage(kGetNextImageTimeoutMs);
        } catch (const Spinnaker::Exception& e) {
            // Timeouts are expected in hw-sync mode whenever no trigger pulses
            // arrive (Teensy disarmed/unplugged) — GetNextImage already blocked
            // for the full timeout, so just poll again quietly.
            if (e.GetError() == Spinnaker::SPINNAKER_ERR_TIMEOUT) continue;
            std::cerr << "[" << name << "] Spinnaker GetNextImage failed: " << e.what() << "\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        if (img->IsIncomplete()) {
            total_incomplete.fetch_add(1, std::memory_order_relaxed);
            img->Release();
            continue;
        }

        const uint64_t camera_ts = static_cast<uint64_t>(img->GetTimeStamp());

        // In hardware-sync mode we replace the camera's own timestamp with
        // the Teensy-side rising-edge timestamp for THIS frame. Two cameras
        // wired to the same trigger group resolve to the same pulse event,
        // which is the mechanism behind the "identical timestamps" guarantee.
        uint64_t stamp_ts = camera_ts;
        if (hw_sync.enabled && hw_sync.stamper) {
            // The U3V leader's block id is a strictly-monotonic per-exposure
            // counter (gaps = dropped frames), which is all the pulse matcher
            // needs. +1 keeps the first frame (TL id 0) out of the "no frame
            // id available" sentinel below — only deltas matter to the
            // matcher, so a constant offset is harmless. If the camera ever
            // reports no id (UINT64_MAX), the +1 wraps to 0 and we fall back
            // to the camera timestamp.
            const uint64_t frame_id = static_cast<uint64_t>(img->GetFrameID()) + 1;
            if (frame_id != 0) {
                const uint64_t pulse_ns =
                    hw_sync.stamper->pop_pulse_ns(hw_sync.trigger_output_pin, frame_id);
                if (pulse_ns != 0) stamp_ts = pulse_ns;
            }
        }

        Frame* f = user_pool->checkout(std::move(img));
        if (!f) {
            total_dropped.fetch_add(1, std::memory_order_relaxed);
            continue;
        }

        f->set_sequence(++sequence_counter);
        f->set_host_capture_ns(Clock::now_ns());
        f->set_camera_ts_ns(stamp_ts);
        f->set_producer_id(name);

        channel.publish(f);
        total_published.fetch_add(1, std::memory_order_relaxed);
    }
}

SpinnakerProducerStats SpinnakerProducer::stats() const {
    return SpinnakerProducerStats{
        .total_published  = impl_->total_published.load(std::memory_order_relaxed),
        .total_dropped    = impl_->total_dropped.load(std::memory_order_relaxed),
        .total_incomplete = impl_->total_incomplete.load(std::memory_order_relaxed),
    };
}

}  // namespace gw
