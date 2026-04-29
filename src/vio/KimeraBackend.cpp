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

// TODO(KIMERA-API): The exact include paths depend on Kimera's install
// layout. Either of these tends to work; pick the one matching the
// installed version.
#include <kimera-vio/pipeline/MonoImuPipeline.h>
#include <kimera-vio/pipeline/Pipeline.h>
#include <kimera-vio/pipeline/PipelineParams.h>
#include <kimera-vio/frontend/Frame.h>
#include <kimera-vio/frontend/CameraParams.h>
#include <kimera-vio/imu-frontend/ImuFrontEnd.h>
#include <kimera-vio/imu-frontend/ImuFrontEnd-definitions.h>
#include <kimera-vio/backend/VioBackEnd-definitions.h>

namespace posest::vio {

namespace {

// Minimum number of tracked landmarks at which we trust the backend's
// pose enough to forward to the consumer. Tunable; see the
// `tracking_ok` heuristic note in emitOutput.
constexpr int kMinLandmarksForValid = 5;

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

        // TODO(KIMERA-API): VIO::Frame constructor signature varies
        // between releases. The four-arg form below matches recent
        // master. Some versions accept (id, ts, cam, img, copy=true) —
        // in that case drop the explicit clone() and pass `gray`.
        auto frame = std::make_unique<VIO::Frame>(
            /*id=*/teensy_time_us,
            /*timestamp_ns=*/static_cast<VIO::Timestamp>(teensy_time_us) *
                1000ULL,
            /*cam_param=*/left_cam_params_,
            /*image=*/gray.clone());
        return pipeline_->fillSingleCameraQueue(std::move(frame));
    }

    bool tryPushImu(std::uint64_t teensy_time_us,
                    const Eigen::Vector3d& accel_mps2,
                    const Eigen::Vector3d& gyro_radps) override {
        if (!running_.load() || !pipeline_) {
            return false;
        }
        // TODO(KIMERA-API): VIO::ImuMeasurement is (timestamp_ns,
        // Vector6) in recent master; the Vector6 ordering is
        // [acc; gyro] which matches our ImuSample. Verify on the
        // installed version.
        VIO::ImuAccGyr acc_gyr;
        acc_gyr << accel_mps2, gyro_radps;
        const VIO::Timestamp ts_ns =
            static_cast<VIO::Timestamp>(teensy_time_us) * 1000ULL;
        return pipeline_->fillSingleImuQueue(
            VIO::ImuMeasurement(ts_ns, acc_gyr));
    }

    void start() override {
        if (running_.exchange(true)) {
            return;
        }
        // TODO(KIMERA-API): VioParams' constructor signature has
        // historically been (param_folder_path); some forks use
        // (param_folder_path, lcd_param_folder_path). The single-arg
        // form is correct for upstream master.
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

        pipeline_ = std::make_unique<VIO::MonoImuPipeline>(*params_);

        // The callback runs on Kimera's internal backend thread; we
        // copy the smart pointer's referent and route through
        // emitOutput which handles the lock dance and translation.
        pipeline_->registerBackendOutputCallback(
            [this](const VIO::BackendOutput::ConstPtr& out) {
                if (out) {
                    emitOutput(*out);
                }
            });

        spin_thread_ = std::thread([this] {
            // pipeline_->spin() blocks until shutdown() is called.
            pipeline_->spin();
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
    //   - state_covariance_lkf_ is 15×15 over [rot, pos, vel, bg, ba]
    //     in that order. Both Kimera's leading-6 block and gtsam's
    //     Pose3 tangent use [rx, ry, rz, tx, ty, tz], so the
    //     upper-left 6×6 block maps directly with no reordering.
    //   - timestamp_kf_ is in nanoseconds; we recover teensy_time_us
    //     by dividing by 1000 (exact, since we multiplied on push).
    void emitOutput(const VIO::BackendOutput& kout) {
        VioBackendOutput out;
        out.teensy_time_us =
            static_cast<std::uint64_t>(kout.timestamp_kf_) / 1000ULL;
        out.world_T_body = kout.W_State_Blkf_.pose_;

        const auto& C = kout.state_covariance_lkf_;
        out.pose_covariance = C.block<6, 6>(0, 0);

        // tracking_ok heuristic. BackendOutput has no canonical
        // "tracking is good" boolean upstream; landmark count plus a
        // finite-covariance check is the conservative signal. The
        // consumer applies its own first-frame skip and airborne
        // covariance inflation downstream, so we err toward
        // permissive here.
        const bool finite_cov = C.diagonal().allFinite();
        out.tracking_ok =
            (kout.landmarks_kf_ >= kMinLandmarksForValid) && finite_cov;
        out.backend_status = out.tracking_ok ? "valid" : "degraded";

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
    std::unique_ptr<VIO::MonoImuPipeline> pipeline_;
    std::thread spin_thread_;
};

// Factory used by the daemon. Defined here (not in the header) so the
// rest of posest_daemon_lib does not need to see Kimera's headers.
std::unique_ptr<IVioBackend> makeKimeraBackend(const std::string& param_dir) {
    return std::make_unique<KimeraBackend>(param_dir);
}

}  // namespace posest::vio
