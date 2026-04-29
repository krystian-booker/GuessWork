// KimeraBackend.cpp — IVioBackend adapter over MIT-SPARK/Kimera-VIO.
//
// COMPILED ONLY WHEN POSEST_BUILD_VIO IS ON (Mac Mini target by default;
// opt-in on Linux for developers with Kimera installed locally).
//
// This file is a thin shim: the real lifting happens inside Kimera's
// own VIO::Pipeline. Our job is to (a) load the YAML param folder, (b)
// hand frames + IMU to Kimera's input queues without blocking, and (c)
// translate Kimera's BackendOutput into our IVioBackend::VioBackendOutput.
//
// API drift warning: Kimera's class names, queue method signatures, and
// callback registration entry points have changed between releases.
// The names below are accurate against the upstream master branch as of
// the time this file was written; verify against the installed version
// before flight. The TODO(KIMERA-API) markers flag the surfaces most
// prone to drift.

#include "posest/vio/IVioBackend.h"

#include <atomic>
#include <cassert>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <opencv2/imgproc.hpp>

// Header names track the installed Kimera-VIO master at the time of
// writing (see scripts/install_kimera_deps.sh for the pinned commit).
// Kimera renamed `ImuFrontEnd*` → `ImuFrontend*` and
// `VioBackEnd-*` → `VioBackend-*` between the early integration spike
// and the current install; if a future bump renames them again,
// update these and the API call sites below.
#include <kimera-vio/pipeline/MonoImuPipeline.h>
#include <kimera-vio/pipeline/Pipeline.h>
#include <kimera-vio/pipeline/Pipeline-definitions.h>
#include <kimera-vio/frontend/Frame.h>
#include <kimera-vio/frontend/CameraParams.h>
#include <kimera-vio/imu-frontend/ImuFrontend.h>
#include <kimera-vio/imu-frontend/ImuFrontend-definitions.h>
#include <kimera-vio/backend/VioBackend-definitions.h>

namespace posest::vio {

namespace {

// Minimum number of tracked landmarks at which we trust the backend's
// pose enough to forward to the consumer. Tunable; see the
// `tracking_ok` heuristic note in emitOutput.
constexpr int kMinLandmarksForValid = 5;

// `Pipeline::registerBackendOutputCallback` is `protected` in the
// installed Kimera headers (Pipeline.h:215). Expose it through a
// trivial subclass — no behavior change, just an access promotion.
class MonoImuPipelinePublic final : public VIO::MonoImuPipeline {
public:
    using VIO::MonoImuPipeline::MonoImuPipeline;
    using VIO::Pipeline::registerBackendOutputCallback;
    using VIO::Pipeline::registerFrontendOutputCallback;
};

}  // namespace

class KimeraBackend final : public IVioBackend {
public:
    explicit KimeraBackend(std::string param_dir)
        : param_dir_(std::move(param_dir)) {}

    ~KimeraBackend() override { stop(); }

    void setOutputCallback(OutputCallback cb) override {
        std::lock_guard<std::mutex> g(mu_);
        callback_ = std::move(cb);
    }

    bool tryPushFrame(std::uint64_t teensy_time_us,
                      const cv::Mat& image) override {
        if (!running_.load() || !pipeline_) {
            return false;
        }
        // Kimera's monocular frontend operates on CV_8UC1; convert if the
        // upstream producer hands us a colour image. V4L2 currently
        // delivers 8-bit BGR/MJPEG-decoded BGR or YUYV→BGR, so this
        // covers every supported camera path. Anything 16-bit / Bayer
        // would be a producer-side bug; assert rather than silently
        // truncating.
        assert(image.depth() == CV_8U);

        cv::Mat gray;
        if (image.channels() == 1) {
            gray = image;
        } else {
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        }

        // Frame ctor: (id, timestamp_ns, cam_param, img). Kimera's
        // Frame stores `img_` as a shallow copy by default
        // (frontend/Frame.h:51-56), so we clone() to give it
        // independent ownership of the pixel data — the upstream
        // caller is allowed to recycle `image`'s buffer the moment
        // tryPushFrame returns.
        auto frame = std::make_unique<VIO::Frame>(
            /*id=*/static_cast<VIO::FrameId>(teensy_time_us),
            /*timestamp_ns=*/static_cast<VIO::Timestamp>(teensy_time_us) *
                1000LL,
            /*cam_param=*/left_cam_params_,
            /*image=*/gray.clone());
        // fillLeftFrameQueue returns void; Kimera's queue is unbounded
        // by default, so we report success and rely on the consumer's
        // upstream LatestFrameSlot drop-oldest semantics for
        // backpressure.
        pipeline_->fillLeftFrameQueue(std::move(frame));
        return true;
    }

    bool tryPushImu(std::uint64_t teensy_time_us,
                    const Eigen::Vector3d& accel_mps2,
                    const Eigen::Vector3d& gyro_radps) override {
        if (!running_.load() || !pipeline_) {
            return false;
        }
        // ImuMeasurement is (ImuStamp, ImuAccGyr) where ImuStamp is
        // alias for VIO::Timestamp (int64 ns) and ImuAccGyr is
        // Eigen::Matrix<double,6,1> ordered [acc; gyro] —
        // matches our ImuSample directly.
        VIO::ImuAccGyr acc_gyr;
        acc_gyr << accel_mps2, gyro_radps;
        const VIO::Timestamp ts_ns =
            static_cast<VIO::Timestamp>(teensy_time_us) * 1000LL;
        // fillSingleImuQueue returns void; same backpressure caveat
        // as tryPushFrame above.
        pipeline_->fillSingleImuQueue(VIO::ImuMeasurement(ts_ns, acc_gyr));
        return true;
    }

    void start() override {
        if (running_.exchange(true)) {
            return;
        }
        // VioParams loads PipelineParams.yaml + ImuParams.yaml +
        // LeftCameraParams.yaml + (optional) RightCameraParams.yaml from
        // `param_dir_`. The single-arg ctor is the upstream-master
        // signature; matches what scripts/install_kimera_deps.sh builds.
        params_ = std::make_unique<VIO::VioParams>(param_dir_);

        // KimeraParamWriter emits exactly one camera (LeftCameraParams.yaml).
        // A mismatch means the writer is out of sync with this backend —
        // hard fail rather than silently grabbing index 0.
        if (params_->camera_params_.size() != 1) {
            // Roll back the running flag so a retry after fixing the
            // param dir is possible.
            running_.store(false);
            params_.reset();
            // No exceptions: surface the error via a no-op callback. The
            // consumer's first-frame skip will keep the system alive
            // until the operator notices the absence of pose updates.
            return;
        }
        left_cam_params_ = params_->camera_params_.at(0);

        pipeline_ = std::make_unique<MonoImuPipelinePublic>(*params_);

        // The callback runs on Kimera's internal backend thread; we
        // copy the smart pointer's referent and route through
        // emitOutput which handles the lock dance and translation.
        // VioBackendModule::OutputCallback is
        // std::function<void(const std::shared_ptr<BackendOutput>&)>
        // (PipelineModule.h:297). The output is non-const so we accept
        // it as `BackendOutput::Ptr`.
        pipeline_->registerBackendOutputCallback(
            [this](const VIO::BackendOutput::Ptr& out) {
                if (out) {
                    emitOutput(*out);
                }
            });

        spin_thread_ = std::thread([this] {
            // Pipeline::spin() blocks until shutdown() is called and
            // returns false on shutdown. The bool is unused.
            (void)pipeline_->spin();
        });
    }

    void stop() override {
        if (!running_.exchange(false)) {
            return;
        }
        // Order matters:
        //   1. running_=false (above) — in-flight tryPush* now bail.
        //   2. shutdown() — signals spin() to return and drains queues.
        //   3. join — wait for spin to actually exit so no callback can
        //      fire against a destroyed pipeline_.
        //   4. pipeline_.reset() / params_.reset() — now safe.
        //   5. callback_=nullptr — last; the spin thread is gone, so no
        //      one will read it again.
        if (pipeline_) {
            pipeline_->shutdown();
        }
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
        pipeline_.reset();
        params_.reset();
        {
            std::lock_guard<std::mutex> g(mu_);
            callback_ = nullptr;
        }
    }

private:
    // Translate Kimera's BackendOutput into our wire-level
    // VioBackendOutput. Runs on Kimera's backend thread.
    //
    // Convention notes (load-bearing — verify against installed
    // Kimera before flight):
    //   - W_State_Blkf_.pose_ is a gtsam::Pose3 in Kimera's W frame;
    //     copy directly into out.world_T_body.
    //   - state_covariance_lkf_ is gtsam::Matrix (Eigen dynamic) sized
    //     15×15 over [rot, pos, vel, bg, ba] in that order. Both
    //     Kimera's leading-6 block and gtsam's Pose3 tangent use
    //     [rx, ry, rz, tx, ty, tz], so the upper-left 6×6 block maps
    //     directly with no reordering.
    //   - PipelinePayload::timestamp_ is in nanoseconds; we recover
    //     teensy_time_us by dividing by 1000 (exact, since we
    //     multiplied on push). BackendOutput inherits this field.
    void emitOutput(const VIO::BackendOutput& kout) {
        VioBackendOutput out;
        out.teensy_time_us =
            static_cast<std::uint64_t>(kout.timestamp_) / 1000ULL;
        out.world_T_body = kout.W_State_Blkf_.pose_;

        out.landmark_count = static_cast<std::int32_t>(kout.landmark_count_);

        const auto& C = kout.state_covariance_lkf_;
        // Defensive: state_covariance_lkf_ is dynamically sized, and
        // very early Kimera outputs (pre-initialization) can be empty.
        // Skip with a degraded marker rather than indexing OOB.
        if (C.rows() < 6 || C.cols() < 6) {
            out.tracking_ok = false;
            out.backend_status = "degraded";
        } else {
            out.pose_covariance = C.block<6, 6>(0, 0);

            // tracking_ok heuristic. BackendOutput has no canonical
            // "tracking is good" boolean upstream; landmark count plus
            // a finite-covariance check is the conservative signal.
            // The consumer applies its own first-frame skip and
            // airborne covariance inflation downstream, so we err
            // toward permissive here.
            const bool finite_cov = out.pose_covariance.diagonal().allFinite();
            out.tracking_ok =
                (kout.landmark_count_ >= kMinLandmarksForValid) && finite_cov;
            out.backend_status = out.tracking_ok ? "valid" : "degraded";
        }

        OutputCallback cb;
        {
            std::lock_guard<std::mutex> g(mu_);
            cb = callback_;
        }
        if (cb) {
            cb(out);
        }
    }

    std::string param_dir_;
    std::atomic<bool> running_{false};
    std::mutex mu_;
    OutputCallback callback_;  // guarded by mu_

    std::unique_ptr<VIO::VioParams> params_;
    VIO::CameraParams left_cam_params_{};
    std::unique_ptr<MonoImuPipelinePublic> pipeline_;
    std::thread spin_thread_;
};

// Factory used by the daemon. Defined here (not in the header) so the
// rest of posest_daemon_lib does not need to see Kimera's headers.
std::unique_ptr<IVioBackend> makeKimeraBackend(const std::string& param_dir) {
    return std::make_unique<KimeraBackend>(param_dir);
}

}  // namespace posest::vio
