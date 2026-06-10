#include "apriltag/apriltag_consumer.hpp"

#include <CoreVideo/CoreVideo.h>

#include <apriltag.h>
#include <tag36h11.h>

#include <chrono>
#include <utility>

#include "core/clock.hpp"
#include "core/frame.hpp"

namespace gw::apriltag {

namespace {

// Detections with a Hamming correction or a weak decision margin are noise
// at FRC distances — filter early so the solver never sees them.
constexpr double kMinDecisionMargin = 35.0;

constexpr double kLatencyEwmaAlpha = 0.1;

}  // namespace

AprilTagConsumer::AprilTagConsumer(int64_t             camera_id,
                                   std::string         camera_name,
                                   PinholeCamera       intrinsics,
                                   std::optional<Mat4> T_cam_imu,
                                   std::shared_ptr<TagPoseBus> bus,
                                   std::shared_ptr<const SharedTagConfig> shared)
    : camera_id_(camera_id),
      camera_name_(std::move(camera_name)),
      intrinsics_(intrinsics),
      T_cam_imu_(T_cam_imu),
      bus_(std::move(bus)),
      shared_(std::move(shared)) {
    family_   = tag36h11_create();
    detector_ = apriltag_detector_create();
    apriltag_detector_add_family(detector_, family_);
    detector_->quad_decimate = 2.0f;
    detector_->quad_sigma    = 0.0f;
    detector_->refine_edges  = true;
    // 4 worker threads ≈ 12 ms/frame at 2048×1536 on an M4 (2 threads ≈
    // 20 ms). With 4 AprilTag cameras at 30 Hz that's ~1.4 cores total —
    // affordable, and it keeps single-frame latency inside the budget.
    detector_->nthreads      = 4;
}

AprilTagConsumer::~AprilTagConsumer() {
    detach();
    if (detector_) apriltag_detector_destroy(detector_);
    if (family_) tag36h11_destroy(family_);
}

void AprilTagConsumer::attach(gw::FrameChannel& ch) {
    if (running_.load()) return;
    channel_ = &ch;
    sub_     = ch.subscribe();
    rate_last_t_ = std::chrono::steady_clock::now();
    running_.store(true, std::memory_order_release);
    worker_ = std::thread([this] { run(); });
}

void AprilTagConsumer::detach() {
    if (!running_.exchange(false)) return;
    if (channel_ && sub_) channel_->unsubscribe(sub_);
    if (worker_.joinable()) worker_.join();
    sub_.reset();
    channel_ = nullptr;
}

void AprilTagConsumer::update_shared(std::shared_ptr<const SharedTagConfig> shared) {
    std::lock_guard lk(shared_mu_);
    shared_ = std::move(shared);
}

void AprilTagConsumer::run() {
    while (running_.load(std::memory_order_acquire)) {
        gw::Frame* f = channel_->next_frame(sub_);
        if (!f) break;  // detached
        process_frame(f);
    }
}

void AprilTagConsumer::process_frame(gw::Frame* f) {
    frames_seen_.fetch_add(1, std::memory_order_relaxed);

    // Stored intrinsics must match the live geometry — solving with a
    // mismatched K silently produces garbage poses.
    if (intrinsics_.width != f->width() || intrinsics_.height != f->height()) {
        resolution_mismatch_.store(true, std::memory_order_relaxed);
        f->release();
        return;
    }
    resolution_mismatch_.store(false, std::memory_order_relaxed);

    const uint64_t t_capture_host = f->host_capture_ns();
    const uint64_t t_camera       = f->camera_ts_ns();

    // Detect zero-copy on the locked IOSurface plane. The detector only
    // reads the buffer during apriltag_detector_detect; the returned
    // detections hold corner coordinates by value, so the lock is released
    // before any geometry work.
    std::vector<TagObservation> observations;
    {
        CVPixelBufferRef pb = f->pixel_buffer();
        if (!pb || CVPixelBufferLockBaseAddress(pb, kCVPixelBufferLock_ReadOnly) !=
                       kCVReturnSuccess) {
            f->release();
            return;
        }
        auto* base = static_cast<uint8_t*>(
            const_cast<void*>(CVPixelBufferGetBaseAddress(pb)));
        if (base) {
            image_u8_t img{
                .width  = static_cast<int32_t>(f->width()),
                .height = static_cast<int32_t>(f->height()),
                .stride = static_cast<int32_t>(CVPixelBufferGetBytesPerRow(pb)),
                .buf    = base,
            };
            zarray_t* dets = apriltag_detector_detect(detector_, &img);
            for (int i = 0; i < zarray_size(dets); ++i) {
                apriltag_detection_t* det = nullptr;
                zarray_get(dets, i, &det);
                if (det->hamming != 0) continue;
                if (det->decision_margin < kMinDecisionMargin) continue;
                TagObservation obs;
                obs.id              = det->id;
                obs.decision_margin = det->decision_margin;
                for (int c = 0; c < 4; ++c) {
                    obs.corners_px[c] = {det->p[c][0], det->p[c][1]};
                }
                observations.push_back(obs);
            }
            apriltag_detections_destroy(dets);
        }
        CVPixelBufferUnlockBaseAddress(pb, kCVPixelBufferLock_ReadOnly);
    }
    f->release();

    detections_total_.fetch_add(observations.size(), std::memory_order_relaxed);

    // Snapshot the shared config once per frame.
    std::shared_ptr<const SharedTagConfig> shared;
    {
        std::lock_guard lk(shared_mu_);
        shared = shared_;
    }

    // Per-tag ranges for the status page — intrinsics-only, so bench
    // validation works before any extrinsics exist.
    std::vector<TagStatusEntry> tag_entries;
    tag_entries.reserve(observations.size());
    const double tag_size =
        (shared && shared->layout) ? shared->layout->tag_size_m : kFrcTagSizeM;
    for (const auto& obs : observations) {
        tag_entries.push_back(
            {obs.id, obs.decision_margin,
             single_tag_range_m(obs, intrinsics_, tag_size)});
    }

    // Robot-pose solve needs the full chain.
    const bool chain_ok = T_cam_imu_.has_value() && shared && shared->layout &&
                          shared->T_robot_imu;
    std::optional<Mat4> published_pose;
    int64_t             published_t_ns = 0;
    double              reproj         = 0.0;
    if (!observations.empty()) {
        if (!chain_ok) {
            skipped_no_extrinsics_.fetch_add(1, std::memory_order_relaxed);
        } else {
            const Mat4 T_cam_robot =
                mat4_mul(*T_cam_imu_, mat4_inverse_se3(*shared->T_robot_imu));
            const auto result = estimate_robot_pose(
                observations, intrinsics_, *shared->layout, T_cam_robot,
                shared->est);
            if (result.pose) {
                TagPoseMeasurement m;
                if (t_camera != 0) {
                    m.t_ns         = static_cast<int64_t>(t_camera);
                    m.clock_source = TagPoseMeasurement::Clock::kTeensy;
                } else {
                    m.t_ns         = static_cast<int64_t>(t_capture_host);
                    m.clock_source = TagPoseMeasurement::Clock::kHost;
                }
                m.camera_id          = camera_id_;
                m.T_field_robot      = result.pose->T_field_robot;
                m.cov                = result.pose->cov;
                m.n_tags             = result.pose->n_tags;
                m.mean_reproj_err_px = result.pose->mean_reproj_err_px;
                m.tag_ids            = result.pose->tag_ids;
                bus_->publish(m);
                published_.fetch_add(1, std::memory_order_relaxed);
                published_pose = result.pose->T_field_robot;
                published_t_ns = m.t_ns;
                reproj         = result.pose->mean_reproj_err_px;
            } else {
                switch (result.skip) {
                    case SkipReason::kNoKnownTags:
                        skipped_no_tags_.fetch_add(1, std::memory_order_relaxed);
                        break;
                    case SkipReason::kAmbiguous:
                        skipped_ambiguous_.fetch_add(1, std::memory_order_relaxed);
                        break;
                    case SkipReason::kHighReprojErr:
                        skipped_high_reproj_.fetch_add(1, std::memory_order_relaxed);
                        break;
                    default:
                        skipped_solve_failed_.fetch_add(1, std::memory_order_relaxed);
                        break;
                }
            }
        }
    }

    // Latency: trigger-to-published, measured on the host clock.
    const double latency_ms =
        static_cast<double>(gw::Clock::now_ns() - t_capture_host) / 1e6;

    {
        std::lock_guard lk(stats_mu_);
        last_tags_       = std::move(tag_entries);
        last_latency_ms_ = latency_ms;
        latency_ewma_ms_ = (latency_ewma_ms_ == 0.0)
                               ? latency_ms
                               : (1.0 - kLatencyEwmaAlpha) * latency_ewma_ms_ +
                                     kLatencyEwmaAlpha * latency_ms;
        if (published_pose) {
            last_pose_          = published_pose;
            last_pose_t_ns_     = published_t_ns;
            mean_reproj_err_px_ = reproj;
        }
        // Rolling 1 s detection-rate window (FpsSampler pattern).
        const auto now = std::chrono::steady_clock::now();
        const auto dt  = now - rate_last_t_;
        if (dt >= std::chrono::milliseconds(1000)) {
            const double secs = std::chrono::duration<double>(dt).count();
            const auto   total = frames_seen_.load(std::memory_order_relaxed);
            det_per_s_ = secs > 0
                             ? static_cast<double>(total - rate_last_count_) / secs
                             : 0.0;
            rate_last_t_     = now;
            rate_last_count_ = total;
        }
    }
}

ConsumerStatusSnapshot AprilTagConsumer::snapshot() const {
    ConsumerStatusSnapshot s;
    s.camera_id   = camera_id_;
    s.camera_name = camera_name_;

    s.frames_seen           = frames_seen_.load(std::memory_order_relaxed);
    s.detections_total      = detections_total_.load(std::memory_order_relaxed);
    s.published             = published_.load(std::memory_order_relaxed);
    s.skipped_no_tags       = skipped_no_tags_.load(std::memory_order_relaxed);
    s.skipped_ambiguous     = skipped_ambiguous_.load(std::memory_order_relaxed);
    s.skipped_high_reproj   = skipped_high_reproj_.load(std::memory_order_relaxed);
    s.skipped_no_extrinsics = skipped_no_extrinsics_.load(std::memory_order_relaxed);
    s.skipped_solve_failed  = skipped_solve_failed_.load(std::memory_order_relaxed);

    bool chain_ok;
    {
        std::lock_guard lk(shared_mu_);
        chain_ok = T_cam_imu_.has_value() && shared_ && shared_->layout &&
                   shared_->T_robot_imu;
    }
    if (resolution_mismatch_.load(std::memory_order_relaxed)) {
        s.reason = "resolution_mismatch";
    } else if (!chain_ok) {
        s.reason = "no_extrinsics_chain";
    } else {
        s.reason = "ok";
    }

    {
        std::lock_guard lk(stats_mu_);
        s.det_per_s          = det_per_s_;
        s.last_latency_ms    = last_latency_ms_;
        s.latency_ewma_ms    = latency_ewma_ms_;
        s.mean_reproj_err_px = mean_reproj_err_px_;
        s.last_tags          = last_tags_;
        s.last_pose          = last_pose_;
        s.last_pose_t_ns     = last_pose_t_ns_;
    }
    return s;
}

}  // namespace gw::apriltag
