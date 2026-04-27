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
#include <memory>
#include <mutex>
#include <string>
#include <utility>

// TODO(KIMERA-API): The exact include paths depend on Kimera's install
// layout. Either of these tends to work; pick the one matching the
// installed version.
//
// #include <kimera-vio/pipeline/Pipeline.h>
// #include <kimera-vio/pipeline/MonoImuPipeline.h>
// #include <kimera-vio/frontend/Frame.h>
// #include <kimera-vio/imu-frontend/ImuFrontEnd.h>
// #include <kimera-vio/backend/VioBackEnd-definitions.h>

namespace posest::vio {

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
        if (!running_.load()) {
            return false;
        }
        // TODO(KIMERA-API): construct VIO::Frame and call
        // pipeline_->fillSingleCameraQueue(VIO::Frame{
        //     /*id*/ teensy_time_us,
        //     /*timestamp_ns*/ teensy_time_us * 1000,
        //     /*cam_param*/ camera_params_,
        //     /*image*/ image,
        // });
        // The Kimera queue is a thread-safe ThreadsafeQueue with
        // try_emplace semantics — return its bool result so the
        // consumer can count drops.
        (void)teensy_time_us;
        (void)image;
        return true;
    }

    bool tryPushImu(std::uint64_t teensy_time_us,
                    const Eigen::Vector3d& accel_mps2,
                    const Eigen::Vector3d& gyro_radps) override {
        if (!running_.load()) {
            return false;
        }
        // TODO(KIMERA-API): construct VIO::ImuMeasurement and push to
        // pipeline_->fillSingleImuQueue(...). Kimera expects ns
        // timestamps; teensy_time_us * 1000 is correct.
        (void)teensy_time_us;
        (void)accel_mps2;
        (void)gyro_radps;
        return true;
    }

    void start() override {
        if (running_.exchange(true)) {
            return;
        }
        // TODO(KIMERA-API): load YAML params from param_dir_:
        //   VIO::VioParams params(param_dir_);
        //   pipeline_ = std::make_unique<VIO::MonoImuPipeline>(params);
        //   pipeline_->registerBackendOutputCallback(
        //     [this](const VIO::BackendOutput::ConstPtr& out) {
        //         emitOutput(*out);
        //     });
        //   pipeline_->spin();
    }

    void stop() override {
        if (!running_.exchange(false)) {
            return;
        }
        // TODO(KIMERA-API):
        //   if (pipeline_) {
        //       pipeline_->shutdown();
        //       pipeline_.reset();
        //   }
        std::lock_guard<std::mutex> g(mu_);
        callback_ = nullptr;
    }

private:
    // TODO(KIMERA-API): translate VIO::BackendOutput → VioBackendOutput.
    // Notes:
    //   - W_State_Blkf_.pose_ is a gtsam::Pose3 in Kimera's W frame;
    //     copy directly into out.world_T_body.
    //   - state_covariance_lkf_ is 15×15 over [rot,pos,vel,bg,ba] in
    //     that order; copy the upper-left 6×6 block. Confirm Kimera's
    //     internal convention is [rot,pos] (matching gtsam Pose3
    //     tangent [rx,ry,rz,tx,ty,tz]) — there is no sign flip but
    //     the *order* must be verified against the installed Kimera.
    //   - tracking_ok = (frontend_status == TrackingStatus::VALID).

    std::string param_dir_;
    std::atomic<bool> running_{false};
    std::mutex mu_;
    OutputCallback callback_;
    // std::unique_ptr<VIO::Pipeline> pipeline_;
};

// Factory used by the daemon. Defined here (not in the header) so the
// rest of posest_daemon_lib does not need to see Kimera's headers.
std::unique_ptr<IVioBackend> makeKimeraBackend(const std::string& param_dir) {
    return std::make_unique<KimeraBackend>(param_dir);
}

}  // namespace posest::vio
