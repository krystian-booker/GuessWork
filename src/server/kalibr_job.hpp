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

// Public snapshot exposed by the supervisor and the JSON routes. Matches the
// shape promised in the implementation plan: state, exit info, log size, and
// whether we managed to auto-store the resulting camchain.
struct CalibrationJobStatus {
    SubprocessState state            = SubprocessState::Pending;
    std::string     model;             // "pinhole-radtan" or "pinhole-equi"
    int             exit_code        = 0;
    uint64_t        started_at_ms    = 0;
    uint64_t        ended_at_ms      = 0;
    size_t          log_bytes        = 0;
    bool            calibration_stored = false;
    std::string     upload_error;     // populated when state==Failed *because*
                                      // the post-exit upload step failed
};

// Wraps a SubprocessJob with Kalibr-specific orchestration:
//   - builds the calibrate.sh argv from a session dir and a focal length (mm),
//     deriving the focal-px hint and the pinhole-radtan / pinhole-equi model
//     using the same math the supervisor publishes
//   - on a Succeeded exit, looks for `<session>/calibration-camchain.yaml`
//     and writes it back through CameraRepository::set_calibration so the
//     UI's Current Calibration card refreshes without an upload step
//   - surfaces a stable status + log API for SSE / status routes to consume
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

    // Forwarded to SubprocessJob for the SSE pump.
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
    void handle_exit(SubprocessState s, int exit_code);

    CameraRepository&             repo_;
    int64_t                       camera_id_;
    std::filesystem::path         session_root_;
    std::string                   model_;
    std::atomic<bool>             calibration_stored_{false};

    mutable std::mutex            err_mu_;
    std::optional<std::string>    upload_error_;  // guarded by err_mu_

    SubprocessJob                 sub_;
};

}  // namespace gw::server
