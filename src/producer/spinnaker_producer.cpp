#include "producer/spinnaker_producer.hpp"

#include <atomic>
#include <chrono>
#include <iostream>
#include <optional>
#include <thread>
#include <utility>

#include <CoreVideo/CoreVideo.h>

// Spinnaker's SPINNAKER_DEPRECATED_CLASS macro emits
//   [[deprecated(...)]] __attribute__((visibility("default"))) class
// which recent Apple Clang rejects as "misplaced attributes". We don't use
// any deprecated APIs, so we pre-include the SDK's platform header (which
// defines the macro), then redefine it to strip the deprecation attribute
// before the rest of the SDK headers are pulled in.
#include <SpinnakerPlatform.h>
#undef  SPINNAKER_DEPRECATED_CLASS
#define SPINNAKER_DEPRECATED_CLASS(msg) class SPINNAKER_API

#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

#include "core/clock.hpp"
#include "core/frame.hpp"
#include "producer/spinnaker_user_buffer_pool.hpp"

namespace gw {

namespace {

// Per-camera user-buffer pool size. Spinnaker NewestOnly requires ≥ 3;
// we add headroom for the publisher's working slot, channel-latest, and one
// or two consumer in-flight frames.
constexpr uint32_t kPoolCapacity = 6;
constexpr uint64_t kGetNextImageTimeoutMs = 1000;

// Helpers ------------------------------------------------------------------

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

}  // namespace

// -------------------------------------------------------------------------
// Impl
// -------------------------------------------------------------------------

struct SpinnakerProducer::Impl {
    std::string                              name;
    FrameFormat                              format{};
    FrameChannel                             channel;
    std::optional<SpinnakerUserBufferPool>   user_pool;
    Spinnaker::SystemPtr                     system;
    Spinnaker::CameraList                    cam_list;
    Spinnaker::CameraPtr                     cam;
    std::atomic<bool>                        stop_requested{false};
    std::atomic<bool>                        streaming{false};
    std::atomic<bool>                        owner_user{false};   // true once SetBufferOwnership(USER) succeeds
    std::thread                              worker;
    uint64_t                                 sequence_counter = 0;

    std::atomic<uint64_t>                    total_published{0};
    std::atomic<uint64_t>                    total_dropped{0};
    std::atomic<uint64_t>                    total_incomplete{0};

    explicit Impl(std::string n) : name(std::move(n)) {}

    void capture_loop();
};

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------

SpinnakerProducer::SpinnakerProducer(std::string name)
    : impl_(std::make_unique<Impl>(std::move(name))) {}

SpinnakerProducer::~SpinnakerProducer() {
    try {
        stop();
    } catch (...) {
        // Destructors must not throw.
    }
}

std::string_view SpinnakerProducer::name() const   { return impl_->name; }
FrameFormat      SpinnakerProducer::format() const { return impl_->format; }
FrameChannel&    SpinnakerProducer::channel()      { return impl_->channel; }

void SpinnakerProducer::start() {
    if (impl_->streaming.load()) return;

    impl_->system = Spinnaker::System::GetInstance();
    const Spinnaker::LibraryVersion v = impl_->system->GetLibraryVersion();
    std::cout << "Spinnaker library version: " << v.major << "." << v.minor << "." << v.type
              << "." << v.build << "\n";

    impl_->cam_list = impl_->system->GetCameras();
    const unsigned int n = impl_->cam_list.GetSize();
    std::cout << "Cameras detected: " << n << "\n";
    if (n == 0) {
        impl_->cam_list.Clear();
        impl_->system->ReleaseInstance();
        impl_->system = nullptr;
        throw std::runtime_error("No Spinnaker cameras detected.");
    }

    impl_->cam = impl_->cam_list.GetByIndex(0);

    try {
        // Read serial from the TL-device nodemap (available before Init()).
        Spinnaker::GenApi::INodeMap& tl_dev_nm = impl_->cam->GetTLDeviceNodeMap();
        Spinnaker::GenApi::CStringPtr serial   = tl_dev_nm.GetNode("DeviceSerialNumber");
        if (Spinnaker::GenApi::IsReadable(serial)) {
            std::cout << "Camera serial: " << serial->GetValue() << "\n";
        }

        impl_->cam->Init();

        // Force NewestOnly buffer mode on the TL stream.
        Spinnaker::GenApi::INodeMap& tl_stream_nm = impl_->cam->GetTLStreamNodeMap();
        set_enum_node(tl_stream_nm, "StreamBufferHandlingMode", "NewestOnly");
        std::cout << "Stream buffer handling mode: NewestOnly\n";

        // Force Mono8 on the device.
        Spinnaker::GenApi::INodeMap& dev_nm = impl_->cam->GetNodeMap();
        set_enum_node(dev_nm, "PixelFormat", "Mono8");

        const int64_t w = read_int_node(dev_nm, "Width");
        const int64_t h = read_int_node(dev_nm, "Height");
        impl_->format = FrameFormat{
            .width        = static_cast<uint32_t>(w),
            .height       = static_cast<uint32_t>(h),
            .pixel_format = kCVPixelFormatType_OneComponent8,
        };
        const int64_t payload = read_int_node(dev_nm, "PayloadSize");
        std::cout << "Resolution: " << w << "x" << h << " Mono8 (payload " << payload << " B)\n";

        // Allocate the user-buffer pool. Buffers are 1024-byte rounded; their
        // IOSurface base addresses go straight to Spinnaker.
        impl_->user_pool.emplace(impl_->format, kPoolCapacity, static_cast<uint64_t>(payload));

        // Stride sanity: for Mono8 with no row padding, Spinnaker writes
        // `width` bytes/row and our IOSurface must match. PayloadSize == w*h
        // also confirms no per-row Spinnaker padding.
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

        // Hand our IOSurface base addresses to Spinnaker as DMA targets.
        auto addrs = impl_->user_pool->base_addresses();
        impl_->cam->SetBufferOwnership(Spinnaker::SPINNAKER_BUFFER_OWNERSHIP_USER);
        impl_->owner_user.store(true);
        impl_->cam->SetUserBuffers(addrs.data(),
                                   static_cast<uint64_t>(addrs.size()),
                                   impl_->user_pool->buffer_size());
        std::cout << "User-buffer DMA enabled (" << addrs.size() << " buffers x "
                  << impl_->user_pool->buffer_size() << " B)\n";

        impl_->cam->BeginAcquisition();
        impl_->streaming.store(true);
        std::cout << "Streaming...\n";
    } catch (const Spinnaker::Exception& e) {
        // Best-effort cleanup so a retry of start() (or main's destructor) is safe.
        try { impl_->cam->DeInit(); } catch (...) {}
        impl_->cam = nullptr;
        impl_->cam_list.Clear();
        impl_->system->ReleaseInstance();
        impl_->system = nullptr;
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
    if (impl_->streaming.exchange(false)) {
        try { impl_->cam->EndAcquisition(); } catch (...) {}
    }
    // Restore default buffer ownership BEFORE DeInit so the camera comes back
    // clean on the next launch (otherwise Spinnaker may complain about stale
    // user buffers from a previous session).
    if (impl_->owner_user.exchange(false)) {
        try {
            impl_->cam->SetBufferOwnership(Spinnaker::SPINNAKER_BUFFER_OWNERSHIP_SYSTEM);
        } catch (...) {}
    }
    if (impl_->cam) {
        try { impl_->cam->DeInit(); } catch (...) {}
    }
    impl_->user_pool.reset();
    impl_->cam = nullptr;
    impl_->cam_list.Clear();
    if (impl_->system) {
        impl_->system->ReleaseInstance();
        impl_->system = nullptr;
    }
}

// -------------------------------------------------------------------------
// Capture loop
// -------------------------------------------------------------------------

void SpinnakerProducer::Impl::capture_loop() {
    while (!stop_requested.load(std::memory_order_acquire)) {
        Spinnaker::ImagePtr img;
        try {
            img = cam->GetNextImage(kGetNextImageTimeoutMs);
        } catch (const Spinnaker::Exception& e) {
            std::cerr << "Spinnaker GetNextImage failed: " << e.what() << "\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        if (img->IsIncomplete()) {
            total_incomplete.fetch_add(1, std::memory_order_relaxed);
            img->Release();
            continue;
        }

        // Read camera-side metadata before transferring the ImagePtr to the
        // pool — once moved the local `img` is empty.
        const uint64_t camera_ts = static_cast<uint64_t>(img->GetTimeStamp());

        Frame* f = user_pool->checkout(std::move(img));
        if (!f) {
            // Pointer returned by GetNextImage didn't match any of our slots —
            // would indicate Spinnaker handed us a buffer we don't own.
            total_dropped.fetch_add(1, std::memory_order_relaxed);
            continue;
        }

        f->set_sequence(++sequence_counter);
        f->set_host_capture_ns(Clock::now_ns());
        f->set_camera_ts_ns(camera_ts);
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
