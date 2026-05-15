#include "consumer/recording_consumer.hpp"

#include <CoreVideo/CoreVideo.h>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#include "consumer/recording_png.hpp"
#include "core/frame.hpp"

namespace gw {

namespace {

// Writes a EuRoC-format sensor.yaml. Only `resolution` is strictly required
// by Basalt's EuRoC loader (without it `initCamIntrinsics()` aborts on the
// w > 0 && h > 0 assertion). Remaining fields are placeholders that the
// calibrator overwrites during optimization; we include them so the file is
// recognizable to any other EuRoC consumer that may grow into this dataset.
void write_sensor_yaml(const std::filesystem::path& path,
                       uint32_t                     width,
                       uint32_t                     height) {
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        std::cerr << "RecordingConsumer: cannot write sensor.yaml at " << path << "\n";
        return;
    }
    const double cx = static_cast<double>(width)  / 2.0;
    const double cy = static_cast<double>(height) / 2.0;
    out << "%YAML:1.0\n"
        << "sensor_type: camera\n"
        << "comment: GuessWork camera (mono)\n"
        << "T_BS:\n"
        << "  cols: 4\n"
        << "  rows: 4\n"
        << "  data: [1.0, 0.0, 0.0, 0.0,\n"
        << "         0.0, 1.0, 0.0, 0.0,\n"
        << "         0.0, 0.0, 1.0, 0.0,\n"
        << "         0.0, 0.0, 0.0, 1.0]\n"
        << "rate_hz: 20\n"
        << "resolution: [" << width << ", " << height << "]\n"
        << "camera_model: pinhole\n"
        << "intrinsics: [600.0, 600.0, " << cx << ", " << cy << "]\n"
        << "distortion_model: radial-tangential\n"
        << "distortion_coefficients: [0.0, 0.0, 0.0, 0.0]\n";
}

}  // namespace

RecordingConsumer::RecordingConsumer(std::filesystem::path root_dir,
                                     std::string           name)
    : name_(std::move(name)),
      root_dir_(std::move(root_dir)),
      cam0_dir_(root_dir_ / "mav0" / "cam0"),
      data_dir_(cam0_dir_ / "data") {}

RecordingConsumer::~RecordingConsumer() {
    try { detach(); } catch (...) {}
}

void RecordingConsumer::attach(FrameChannel& ch) {
    if (running_.load()) return;

    std::error_code ec;
    std::filesystem::create_directories(data_dir_, ec);
    if (ec) {
        throw std::runtime_error("RecordingConsumer: create_directories failed: " +
                                 ec.message());
    }

    // Basalt's EuRoC loader hardcodes num_cams=2 and tries to read cam1/data/.
    // Pointing cam1 at cam0 with a relative symlink keeps mono recordings
    // self-contained (the session directory remains movable).
    std::error_code lnk_ec;
    std::filesystem::create_directory_symlink(
        "cam0", root_dir_ / "mav0" / "cam1", lnk_ec);
    // Ignore lnk_ec: pre-existing symlink from a prior attach() is fine.

    {
        std::lock_guard lk(csv_mu_);
        csv_.open(cam0_dir_ / "data.csv", std::ios::out | std::ios::trunc);
        if (!csv_.is_open()) {
            throw std::runtime_error("RecordingConsumer: cannot open data.csv");
        }
        csv_ << "#timestamp [ns],filename\n";
        csv_.flush();
    }
    wrote_sensor_yaml_.store(false, std::memory_order_release);

    channel_ = &ch;
    sub_     = ch.subscribe();

    stopping_.store(false, std::memory_order_release);
    running_.store(true,  std::memory_order_release);

    workers_.reserve(kWorkerCount);
    for (size_t i = 0; i < kWorkerCount; ++i) {
        workers_.emplace_back([this] { run_worker(); });
    }
    pull_thread_ = std::thread([this] { run_pull(); });
}

void RecordingConsumer::detach() {
    if (!running_.exchange(false)) return;

    // Wake the pull thread out of next_frame() and join it. run_pull() sets
    // stopping_ and notifies the workers as it exits, so the joins below
    // unblock once the pull thread completes.
    if (channel_ && sub_) channel_->unsubscribe(sub_);
    if (pull_thread_.joinable()) pull_thread_.join();

    for (auto& w : workers_) {
        if (w.joinable()) w.join();
    }
    workers_.clear();
    {
        std::lock_guard lk(queue_mu_);
        queue_.clear();
    }

    sub_.reset();
    channel_ = nullptr;
    close_csv();
}

void RecordingConsumer::close_csv() {
    std::lock_guard lk(csv_mu_);
    if (csv_.is_open()) csv_.close();
}

void RecordingConsumer::run_pull() {
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
        // we can release the Frame immediately. The producer pool is tiny
        // (6 slots); retaining frames for the encode duration would starve
        // the producer.
        EncodeTask task;
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
                for (uint32_t y = 0; y < task.height; ++y) {
                    std::memcpy(task.pixels.get() + y * task.width,
                                base + y * stride,
                                task.width);
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

        // Enqueue or drop. We deliberately don't block: backpressure from a
        // slow disk should manifest as drops, not stall the FrameChannel
        // subscriber (which would starve the producer of recycled slots if
        // we ever held more than one frame retained).
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

    // Signal workers to drain and exit.
    {
        std::lock_guard lk(queue_mu_);
        stopping_.store(true, std::memory_order_release);
    }
    queue_cv_.notify_all();
}

void RecordingConsumer::run_worker() {
    for (;;) {
        EncodeTask task;
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

        const std::string filename = std::to_string(task.timestamp_ns) + ".png";
        const auto        path     = data_dir_ / filename;
        const bool ok = write_mono8_png(path, task.width, task.height,
                                        task.width, task.pixels.get());
        if (!ok) {
            std::cerr << "RecordingConsumer: write failed for " << path << "\n";
            frames_dropped_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }

        // First successful write: stamp sensor.yaml with the now-known
        // dimensions. compare_exchange ensures exactly one worker wins.
        bool expected = false;
        if (wrote_sensor_yaml_.compare_exchange_strong(
                expected, true,
                std::memory_order_acq_rel,
                std::memory_order_acquire)) {
            write_sensor_yaml(cam0_dir_ / "sensor.yaml", task.width, task.height);
        }

        {
            std::lock_guard lk(csv_mu_);
            if (csv_.is_open()) {
                csv_ << task.timestamp_ns << "," << filename << "\n";
            }
        }
        frames_written_.fetch_add(1, std::memory_order_relaxed);
    }
}

}  // namespace gw
