#include "producer/spinnaker_producer.hpp"
#include "producer/spinnaker_producer_internal.hpp"

#include <atomic>
#include <chrono>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>

#include <CoreVideo/CoreVideo.h>

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
    // Declaration order is load-bearing: `channel` must be destroyed BEFORE
    // `user_pool` because the channel may hold a Frame whose recycle callback
    // points back into the pool. C++ destroys members in reverse declaration
    // order, so user_pool is listed first.
    std::string                              name;
    std::string                              serial;
    FrameFormat                              format{};
    std::optional<SpinnakerUserBufferPool>   user_pool;
    FrameChannel                             channel;
    std::unique_ptr<SpinnakerCameraBinding>  binding;       // system + cam smart ptrs
    std::atomic<bool>                        stop_requested{false};
    std::atomic<bool>                        streaming{false};
    std::atomic<bool>                        owner_user{false};
    std::thread                              worker;
    uint64_t                                 sequence_counter = 0;

    std::atomic<uint64_t>                    total_published{0};
    std::atomic<uint64_t>                    total_dropped{0};
    std::atomic<uint64_t>                    total_incomplete{0};

    Impl(std::string n, std::string s) : name(std::move(n)), serial(std::move(s)) {}

    void capture_loop();
};

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------

SpinnakerProducer::SpinnakerProducer(std::string name, std::string serial)
    : impl_(std::make_unique<Impl>(std::move(name), std::move(serial))) {}

SpinnakerProducer::~SpinnakerProducer() {
    try {
        stop();
    } catch (...) {
        // Destructors must not throw.
    }
}

std::string_view SpinnakerProducer::name()   const { return impl_->name; }
std::string_view SpinnakerProducer::serial() const { return impl_->serial; }
FrameFormat      SpinnakerProducer::format() const { return impl_->format; }
FrameChannel&    SpinnakerProducer::channel()      { return impl_->channel; }

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

        // Force NewestOnly buffer mode on the TL stream.
        Spinnaker::GenApi::INodeMap& tl_stream_nm = cam->GetTLStreamNodeMap();
        set_enum_node(tl_stream_nm, "StreamBufferHandlingMode", "NewestOnly");

        // Force Mono8 on the device.
        Spinnaker::GenApi::INodeMap& dev_nm = cam->GetNodeMap();
        set_enum_node(dev_nm, "PixelFormat", "Mono8");

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

        Frame* f = user_pool->checkout(std::move(img));
        if (!f) {
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
