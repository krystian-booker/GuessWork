#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "server/kalibr_job.hpp"  // CalibrationJobStatus, kalibr_augmented_path
#include "server/subprocess_job.hpp"

namespace gw::server {

class CameraRepository;

// Splits a camchain-imucam YAML document into one cam0-keyed YAML payload
// per camera, ordered cam0..camN. Throws gw::calib::CalibrationParseError
// when the document's camera count differs from expected_cams or the shape
// is malformed. Pure — exposed for unit tests; KalibrImuJob::handle_exit is
// the production caller.
std::vector<std::string> build_imu_extrinsics_payloads(
    const std::string& camchain_imucam_yaml, size_t expected_cams);

// Wraps a SubprocessJob running docker/kalibr/calibrate_imu.sh: the
// camera-IMU extrinsics flow (kalibr_calibrate_imu_camera, preceded by
// kalibr_calibrate_cameras for the multi-camera pair flow). On a successful
// exit, splits `<session>/camchain-imucam-calibration.yaml` per camera and
// stores each camera's block (with a guesswork_meta footer) via
// CameraRepository::set_imu_extrinsics.
//
// Same status/log surface as KalibrJob so the SSE route handler shape is
// reused. One job per supervisor; the supervisor enforces the global
// one-Kalibr-job-at-a-time constraint across both job kinds.
class KalibrImuJob {
public:
    // camera_ids are in topic order (camera_ids[i] recorded /cam<i>/image_raw).
    // focal_hints_px (pair flow) maps 1:1 onto camera_ids; pass empty for the
    // single-camera flow (camchain.yaml provides the intrinsics there).
    KalibrImuJob(CameraRepository&     repo,
                 std::vector<int64_t>  camera_ids,
                 std::filesystem::path session_root,
                 std::filesystem::path calibrate_imu_script_path,
                 std::string           model,
                 std::vector<uint32_t> focal_hints_px,
                 std::string           session_id);
    ~KalibrImuJob();

    KalibrImuJob(const KalibrImuJob&)            = delete;
    KalibrImuJob& operator=(const KalibrImuJob&) = delete;

    void start();
    void cancel() { sub_.cancel(); }

    CalibrationJobStatus status() const;

    size_t      log_bytes() const                     { return sub_.log_bytes(); }
    std::string log_snapshot() const                  { return sub_.log_snapshot(); }
    std::string log_slice(size_t off, size_t n) const { return sub_.log_slice(off, n); }
    size_t      wait_for_log(size_t off, std::chrono::milliseconds t) const {
        return sub_.wait_for_log(off, t);
    }

    const std::vector<int64_t>&  camera_ids()   const { return camera_ids_; }
    const std::filesystem::path& session_root() const { return session_root_; }
    const std::string&           model()        const { return model_; }

private:
    static std::string join_hints(const std::vector<uint32_t>& hints);

    void handle_exit(SubprocessState s, int exit_code);

    CameraRepository&          repo_;
    const std::vector<int64_t> camera_ids_;
    std::filesystem::path      session_root_;
    const std::string          model_;
    const std::string          session_id_;
    std::atomic<bool>          calibration_stored_{false};

    mutable std::mutex         err_mu_;
    std::optional<std::string> upload_error_;  // guarded by err_mu_

    SubprocessJob              sub_;
};

}  // namespace gw::server
