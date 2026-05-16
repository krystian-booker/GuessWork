#include "consumer/rosbag_recording_consumer.hpp"

#include <CoreVideo/CoreVideo.h>

#include <cstring>
#include <iostream>
#include <stdexcept>
#include <utility>

#include "core/frame.hpp"

namespace gw {

RosbagRecordingConsumer::RosbagRecordingConsumer(
    std::filesystem::path session_root,
    std::filesystem::path target_yaml_source,
    std::string           topic,
    std::string           frame_id,
    std::string           name)
    : name_(std::move(name)),
      session_root_(std::move(session_root)),
      bag_path_(session_root_ / "calibration.bag"),
      target_yaml_source_(std::move(target_yaml_source)),
      topic_(std::move(topic)),
      frame_id_(std::move(frame_id)) {}

RosbagRecordingConsumer::~RosbagRecordingConsumer() {
    try { detach(); } catch (...) {}
}

void RosbagRecordingConsumer::attach(FrameChannel& ch) {
    if (running_.load()) return;

    std::error_code ec;
    std::filesystem::create_directories(session_root_, ec);
    if (ec) {
        throw std::runtime_error("RosbagRecordingConsumer: create_directories failed: " +
                                 ec.message());
    }

    // Copy the AprilGrid target alongside the bag so a single -v mount in
    // the suggested docker command gives Kalibr both inputs. Overwrite any
    // existing target.yaml from a prior attach.
    if (!target_yaml_source_.empty() &&
        std::filesystem::exists(target_yaml_source_)) {
        std::error_code cp_ec;
        std::filesystem::copy_file(
            target_yaml_source_,
            session_root_ / "target.yaml",
            std::filesystem::copy_options::overwrite_existing,
            cp_ec);
        if (cp_ec) {
            std::cerr << "RosbagRecordingConsumer: failed to copy target.yaml: "
                      << cp_ec.message() << "\n";
            // Not fatal — the user can rerun with --target pointing at a
            // local copy if this step fails.
        }
    }

    writer_ = std::make_unique<RosbagWriter>(bag_path_, topic_, frame_id_);
    writer_->open();  // may throw on filesystem error — that's the desired surface

    channel_ = &ch;
    sub_     = ch.subscribe();

    stopping_.store(false, std::memory_order_release);
    running_.store(true,  std::memory_order_release);

    writer_thread_ = std::thread([this] { run_writer(); });
    pull_thread_   = std::thread([this] { run_pull(); });
}

void RosbagRecordingConsumer::detach() {
    if (!running_.exchange(false)) return;

    if (channel_ && sub_) channel_->unsubscribe(sub_);
    if (pull_thread_.joinable()) pull_thread_.join();
    if (writer_thread_.joinable()) writer_thread_.join();

    {
        std::lock_guard lk(queue_mu_);
        queue_.clear();
    }

    sub_.reset();
    channel_ = nullptr;
    if (writer_) {
        try { writer_->close(); } catch (...) {}
        writer_.reset();
    }
}

void RosbagRecordingConsumer::run_pull() {
    while (running_.load(std::memory_order_acquire)) {
        Frame* f = channel_->next_frame(sub_);
        if (!f) break;  // detached

        // Account for FrameChannel "latest-only" gaps as dropped frames so
        // the diagnostic reflects what the producer published vs. what we
        // saw.
        const uint64_t seq = f->sequence();
        if (last_seq_ != 0 && seq > last_seq_ + 1) {
            frames_dropped_.fetch_add(seq - last_seq_ - 1, std::memory_order_relaxed);
        }
        last_seq_ = seq;

        // Copy the Mono8 plane out of the IOSurface-backed CVPixelBuffer so
        // we can release the Frame immediately. The producer pool is tiny;
        // retaining frames for the write duration would starve it.
        ImageTask task;
        task.timestamp_ns = f->host_capture_ns();
        task.width        = f->width();
        task.height       = f->height();

        bool copied = false;
        CVPixelBufferRef pb = f->pixel_buffer();
        if (pb &&
            CVPixelBufferLockBaseAddress(pb, kCVPixelBufferLock_ReadOnly) ==
                kCVReturnSuccess) {
            const auto* base = static_cast<const uint8_t*>(
                CVPixelBufferGetBaseAddress(pb));
            const size_t stride = CVPixelBufferGetBytesPerRow(pb);
            if (base) {
                const size_t bytes =
                    static_cast<size_t>(task.width) * static_cast<size_t>(task.height);
                task.pixels = std::make_unique<uint8_t[]>(bytes);
                if (stride == task.width) {
                    std::memcpy(task.pixels.get(), base, bytes);
                } else {
                    for (uint32_t y = 0; y < task.height; ++y) {
                        std::memcpy(task.pixels.get() + y * task.width,
                                    base + y * stride,
                                    task.width);
                    }
                }
                copied = true;
            }
            CVPixelBufferUnlockBaseAddress(pb, kCVPixelBufferLock_ReadOnly);
        }
        f->release();

        if (!copied) {
            frames_dropped_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }

        // Enqueue or drop - never block: backpressure must not propagate
        // back into the FrameChannel subscriber.
        {
            std::lock_guard lk(queue_mu_);
            if (queue_.size() >= kQueueCapacity) {
                frames_dropped_.fetch_add(1, std::memory_order_relaxed);
                continue;
            }
            queue_.push_back(std::move(task));
        }
        queue_cv_.notify_one();
    }

    // Signal the writer thread to drain and exit.
    {
        std::lock_guard lk(queue_mu_);
        stopping_.store(true, std::memory_order_release);
    }
    queue_cv_.notify_all();
}

void RosbagRecordingConsumer::run_writer() {
    for (;;) {
        ImageTask task;
        {
            std::unique_lock lk(queue_mu_);
            queue_cv_.wait(lk, [&] {
                return !queue_.empty() ||
                       stopping_.load(std::memory_order_acquire);
            });
            if (queue_.empty()) return;  // stopping and drained
            task = std::move(queue_.front());
            queue_.pop_front();
        }

        try {
            writer_->add_mono8_image(task.timestamp_ns,
                                     task.width, task.height,
                                     task.pixels.get());
            frames_written_.fetch_add(1, std::memory_order_relaxed);
        } catch (const std::exception& e) {
            std::cerr << "RosbagRecordingConsumer: write failed: " << e.what() << "\n";
            frames_dropped_.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

}  // namespace gw
