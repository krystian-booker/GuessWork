#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>

#include "server/subprocess_job.hpp"

namespace gw::server {

class CameraRepository;

// Snapshot of an active or just-finished Kalibr job. Mirrored as JSON by the
// /api/cameras/<id>/calibration/job route.
struct CalibrationJobStatus {
    SubprocessState state              = SubprocessState::Pending;
    std::string     model;             // "pinhole-radtan" or "pinhole-equi"
    int             exit_code          = 0;
    uint64_t        started_at_ms      = 0;
    uint64_t        ended_at_ms        = 0;
    size_t          log_bytes          = 0;
    bool            calibration_stored = false;
    std::string     upload_error;      // set when post-exit camchain upload failed
};

// Derived Kalibr inputs for a given lens. focal_hint_px is the manual-init
// focal-length hint kalibr_calibrate_cameras consumes via GW_KALIBR_FOCAL_HINT;
// model is "pinhole-equi" (Kannala-Brandt fisheye) for wide HFOV, else
// "pinhole-radtan". Shared between KalibrJob and the supervisor's suggested-
// command builder so both stay in lock-step.
struct KalibrLensConfig {
    uint32_t    focal_hint_px;
    std::string model;
};
KalibrLensConfig derive_kalibr_lens(double   focal_length_mm,
                                    double   pixel_pitch_mm,
                                    uint32_t sensor_width_px);

// Wraps a SubprocessJob with Kalibr-specific orchestration: derives lens
// inputs, runs calibrate.sh, and on a successful exit reads
// `<session>/calibration-camchain.yaml` back through CameraRepository so the
// UI refreshes without a separate upload step.
//
// One job per supervisor instance; the supervisor enforces global concurrency.
class KalibrJob {
public:
    KalibrJob(CameraRepository&          repo,
              int64_t                    camera_id,
              std::filesystem::path      session_root,
              double                     focal_length_mm,
              std::filesystem::path      calibrate_script_path,
              double                     pixel_pitch_mm,
              uint32_t                   sensor_width_px);
    ~KalibrJob();

    KalibrJob(const KalibrJob&)            = delete;
    KalibrJob& operator=(const KalibrJob&) = delete;

    void start();
    void cancel() { sub_.cancel(); }

    CalibrationJobStatus status() const;

    size_t      log_bytes() const                          { return sub_.log_bytes(); }
    std::string log_snapshot() const                       { return sub_.log_snapshot(); }
    std::string log_slice(size_t off, size_t n) const      { return sub_.log_slice(off, n); }
    size_t      wait_for_log(size_t off,
                             std::chrono::milliseconds t) const {
        return sub_.wait_for_log(off, t);
    }

    int64_t                       camera_id()    const { return camera_id_; }
    const std::filesystem::path&  session_root() const { return session_root_; }
    const std::string&            model()        const { return model_; }

private:
    // Private delegating ctor: lets us compute KalibrLensConfig once and feed
    // it both to the public ctor's argv/env build and to model_ initialization.
    KalibrJob(CameraRepository&            repo,
              int64_t                      camera_id,
              std::filesystem::path        session_root,
              const std::filesystem::path& calibrate_script_path,
              const KalibrLensConfig&      lens);

    void handle_exit(SubprocessState s, int exit_code);

    CameraRepository&             repo_;
    int64_t                       camera_id_;
    std::filesystem::path         session_root_;
    const std::string             model_;
    std::atomic<bool>             calibration_stored_{false};

    mutable std::mutex            err_mu_;
    std::optional<std::string>    upload_error_;  // guarded by err_mu_

    SubprocessJob                 sub_;
};

}  // namespace gw::server
