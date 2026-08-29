#include "consumer/multi_topic_bag_recorder.hpp"

#include <CoreVideo/CoreVideo.h>

#include <cstring>
#include <iostream>
#include <stdexcept>
#include <utility>

#include "core/frame.hpp"
#include "core/mono8_copy.hpp"

namespace gw {

MultiTopicBagRecorder::MultiTopicBagRecorder(
    std::filesystem::path        session_root,
    std::filesystem::path        target_yaml_source,
    std::vector<CameraInputSpec> cameras,
    MeasurementBus<ImuSample>*   imu_bus,
    std::string                  imu_topic,
    std::string                  imu_frame_id)
    : session_root_(std::move(session_root)),
      bag_path_(session_root_ / "calibration.bag"),
      target_yaml_source_(std::move(target_yaml_source)),
      imu_bus_(imu_bus),
      imu_topic_(std::move(imu_topic)),
      imu_frame_id_(std::move(imu_frame_id)) {
    if (cameras.empty()) {
        throw std::invalid_argument("MultiTopicBagRecorder: no cameras");
    }
    cams_.reserve(cameras.size());
    for (auto& spec : cameras) {
        if (!spec.channel) {
            throw std::invalid_argument("MultiTopicBagRecorder: null channel");
        }
        auto state  = std::make_unique<CamState>();
        state->spec = std::move(spec);
        cams_.push_back(std::move(state));
    }
}

MultiTopicBagRecorder::~MultiTopicBagRecorder() {
    try { stop(); } catch (...) {}
}

void MultiTopicBagRecorder::start() {
    if (running_.load()) return;

    std::error_code ec;
    std::filesystem::create_directories(session_root_, ec);
    if (ec) {
        throw std::runtime_error("MultiTopicBagRecorder: create_directories failed: " +
                                 ec.message());
    }

    // Copy the AprilGrid target alongside the bag so a single -v mount in
    // the suggested docker command gives Kalibr all inputs.
    if (!target_yaml_source_.empty() &&
        std::filesystem::exists(target_yaml_source_)) {
        std::error_code cp_ec;
        std::filesystem::copy_file(target_yaml_source_,
                                   session_root_ / "target.yaml",
                                   std::filesystem::copy_options::overwrite_existing,
                                   cp_ec);
        if (cp_ec) {
            std::cerr << "MultiTopicBagRecorder: failed to copy target.yaml: "
                      << cp_ec.message() << "\n";
        }
    }

    // Connection ids: cameras in vector order (cam 0 via the constructor),
    // then the IMU.
    writer_ = std::make_unique<RosbagWriter>(bag_path_,
                                             cams_[0]->spec.topic,
                                             cams_[0]->spec.frame_id);
    cams_[0]->conn_id = 0;
    for (size_t i = 1; i < cams_.size(); ++i) {
        cams_[i]->conn_id = writer_->add_image_connection(cams_[i]->spec.topic,
                                                          cams_[i]->spec.frame_id);
    }
    if (imu_bus_) {
        imu_conn_id_ = writer_->add_imu_connection(imu_topic_, imu_frame_id_);
    }
    writer_->open();

    for (auto& cam : cams_) {
        cam->sub      = cam->spec.channel->subscribe();
        cam->last_seq = 0;
        cam->written.store(0, std::memory_order_relaxed);
        cam->dropped.store(0, std::memory_order_relaxed);
    }
    if (imu_bus_) imu_sub_ = imu_bus_->subscribe(kImuQueueCapacity);
    imu_written_.store(0, std::memory_order_relaxed);
    imu_dropped_.store(0, std::memory_order_relaxed);

    stopping_.store(false, std::memory_order_release);
    running_.store(true,  std::memory_order_release);

    writer_thread_ = std::thread([this] { run_writer(); });
    for (size_t i = 0; i < cams_.size(); ++i) {
        cams_[i]->thread = std::thread([this, i] { run_pull(i); });
    }
    if (imu_bus_) imu_thread_ = std::thread([this] { run_imu(); });
}

void MultiTopicBagRecorder::stop() {
    if (!running_.exchange(false)) return;

    for (auto& cam : cams_) {
        if (cam->sub) cam->spec.channel->unsubscribe(cam->sub);
    }
    if (imu_bus_ && imu_sub_) {
        // Fold bus-side drops (slow IMU drain) into the drop counter before
        // the handle goes away.
        imu_dropped_.fetch_add(imu_bus_->dropped(imu_sub_), std::memory_order_relaxed);
        imu_bus_->unsubscribe(imu_sub_);
    }
    for (auto& cam : cams_) {
        if (cam->thread.joinable()) cam->thread.join();
        cam->sub.reset();
    }
    if (imu_thread_.joinable()) imu_thread_.join();
    imu_sub_.reset();

    // All producers are done; tell the writer to drain and exit.
    {
        std::lock_guard lk(queue_mu_);
        stopping_.store(true, std::memory_order_release);
    }
    queue_cv_.notify_all();
    if (writer_thread_.joinable()) writer_thread_.join();

    {
        std::lock_guard lk(queue_mu_);
        image_queue_.clear();
        imu_queue_.clear();
    }
    if (writer_) {
        try { writer_->close(); } catch (...) {}
        writer_.reset();
    }
}

MultiTopicBagRecorder::CameraStats
MultiTopicBagRecorder::camera_stats(size_t camera_idx) const {
    CameraStats s;
    if (camera_idx < cams_.size()) {
        s.written = cams_[camera_idx]->written.load(std::memory_order_relaxed);
        s.dropped = cams_[camera_idx]->dropped.load(std::memory_order_relaxed);
    }
    return s;
}

void MultiTopicBagRecorder::run_pull(size_t camera_idx) {
    CamState& cam = *cams_[camera_idx];
    while (running_.load(std::memory_order_acquire)) {
        Frame* f = cam.spec.channel->next_frame(cam.sub);
        if (!f) break;  // detached

        // Account for FrameChannel "latest-only" gaps as dropped frames.
        const uint64_t seq = f->sequence();
        if (cam.last_seq != 0 && seq > cam.last_seq + 1) {
            cam.dropped.fetch_add(seq - cam.last_seq - 1, std::memory_order_relaxed);
        }
        cam.last_seq = seq;

        ImageTask task;
        task.conn_id      = cam.conn_id;
        task.camera_idx   = camera_idx;
        task.timestamp_ns = f->camera_ts_ns();
        task.width        = f->width();
        task.height       = f->height();

        // No sync controller pulse matched this frame (fallback-stamped). Unusable
        // for a shared-clock bag — see the class comment.
        if (task.timestamp_ns == 0) {
            f->release();
            cam.dropped.fetch_add(1, std::memory_order_relaxed);
            continue;
        }

        // Copy the Mono8 plane out so the Frame can be released immediately
        // (the producer pool is tiny).
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
                copy_mono8(task.pixels.get(), base, stride,
                           task.width, task.height, cam.spec.rotate_180);
                copied = true;
            }
            CVPixelBufferUnlockBaseAddress(pb, kCVPixelBufferLock_ReadOnly);
        }
        f->release();

        if (!copied) {
            cam.dropped.fetch_add(1, std::memory_order_relaxed);
            continue;
        }

        // Enqueue or drop — never block: backpressure must not propagate
        // back into the FrameChannel subscriber.
        {
            std::lock_guard lk(queue_mu_);
            if (image_queue_.size() >= kImageQueueCapacity) {
                cam.dropped.fetch_add(1, std::memory_order_relaxed);
                continue;
            }
            image_queue_.push_back(std::move(task));
        }
        queue_cv_.notify_one();
    }
}

void MultiTopicBagRecorder::run_imu() {
    ImuSample s;
    while (imu_bus_->wait_pop(imu_sub_, s)) {
        std::lock_guard lk(queue_mu_);
        if (imu_queue_.size() >= kImuQueueCapacity) {
            imu_queue_.pop_front();
            imu_dropped_.fetch_add(1, std::memory_order_relaxed);
        }
        imu_queue_.push_back(s);
        queue_cv_.notify_one();
    }
}

void MultiTopicBagRecorder::run_writer() {
    for (;;) {
        ImuSample imu_task;
        ImageTask img_task;
        bool have_imu = false, have_img = false;
        {
            std::unique_lock lk(queue_mu_);
            queue_cv_.wait(lk, [&] {
                return !imu_queue_.empty() || !image_queue_.empty() ||
                       stopping_.load(std::memory_order_acquire);
            });
            // Drain IMU first — cheap writes, keeps the /imu0 connection's
            // latency low while a large image is pending.
            if (!imu_queue_.empty()) {
                imu_task = imu_queue_.front();
                imu_queue_.pop_front();
                have_imu = true;
            } else if (!image_queue_.empty()) {
                img_task = std::move(image_queue_.front());
                image_queue_.pop_front();
                have_img = true;
            } else {
                return;  // stopping and fully drained
            }
        }

        try {
            if (have_imu) {
                writer_->add_imu_sample(imu_conn_id_, imu_task.t_ns,
                                        imu_task.accel, imu_task.gyro);
                imu_written_.fetch_add(1, std::memory_order_relaxed);
            } else if (have_img) {
                writer_->add_mono8_image(img_task.conn_id, img_task.timestamp_ns,
                                         img_task.width, img_task.height,
                                         img_task.pixels.get());
                cams_[img_task.camera_idx]->written.fetch_add(
                    1, std::memory_order_relaxed);
            }
        } catch (const std::exception& e) {
            std::cerr << "MultiTopicBagRecorder: write failed: " << e.what() << "\n";
            if (have_imu) imu_dropped_.fetch_add(1, std::memory_order_relaxed);
            else          cams_[img_task.camera_idx]->dropped.fetch_add(
                              1, std::memory_order_relaxed);
        }
    }
}

}  // namespace gw
