#pragma once

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

namespace gw {
class RosbagRecordingConsumer;
}

namespace gw::server {

class CameraSupervisor;
class CameraRepository;
class KalibrJob;
struct CalibrationJobStatus;

// Status snapshot of an in-progress recording session.
struct CalibrationSessionStatus {
    std::string             session_id;
    std::filesystem::path   path;            // root dir of the session
    uint64_t                frames_written = 0;
    uint64_t                frames_dropped = 0;
    uint64_t                elapsed_ms     = 0;
};

// Result returned to the client after a session is stopped. Adds a copy-
// pasteable Kalibr docker command line; the path/counters mirror the
// snapshot fields.
struct CalibrationSessionResult {
    std::string             session_id;
    std::filesystem::path   path;
    uint64_t                frames_written = 0;
    uint64_t                frames_dropped = 0;
    uint64_t                elapsed_ms     = 0;
    std::string             suggested_command;
};

class CalibrationError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// Owns per-camera recording sessions for camera calibration. Each session
// writes a ROS1 bag + target.yaml under <root>/<session_id>/, which the user
// hands off to Kalibr (running in a Docker container) manually. Single session
// per camera at a time.
//
// Thread-safety: a mutex serialises mutations to the session map; the
// RosbagRecordingConsumer attached inside each session does its own work on
// its own worker thread, independent of this lock.
class CalibrationSupervisor {
public:
    // calibrations_root is the directory under which session subdirectories
    // are created (typically ~/.guesswork/calibrations). The repository is
    // consulted at session start to read the camera's focal_length_mm, which
    // drives the Kalibr focal-length hint and the camera model selection in
    // the suggested command.
    CalibrationSupervisor(CameraSupervisor&     cameras,
                          CameraRepository&     repository,
                          std::filesystem::path calibrations_root);
    ~CalibrationSupervisor();

    CalibrationSupervisor(const CalibrationSupervisor&)            = delete;
    CalibrationSupervisor& operator=(const CalibrationSupervisor&) = delete;

    // Starts a recording session for the given camera. Throws CalibrationError
    // if the camera is offline (no live FrameChannel) or a session is already
    // active for this camera.
    CalibrationSessionStatus start(int64_t camera_id);

    // Stops the active session. Throws CalibrationError if no session exists.
    // The on-disk recording is left in place; if the caller chains into
    // start_kalibr_job() the job picks it up from <session>/calibration.bag.
    CalibrationSessionResult stop(int64_t camera_id);

    // Snapshot of an active session, or nullopt if none.
    std::optional<CalibrationSessionStatus> status(int64_t camera_id);

    // --- Kalibr job lifecycle (post-recording) ---
    //
    // Concurrency is global: at most one job system-wide because each Kalibr
    // run consumes ~all of the Colima VM's CPU/RAM. Start throws if another
    // job is still running.

    // Spawn the calibrate.sh subprocess for <session_root> using the camera's
    // recorded focal_length_mm. Returns the initial status snapshot (state
    // will be Running on success). Throws CalibrationError if a job is
    // already active (for any camera).
    CalibrationJobStatus start_kalibr_job(int64_t                      camera_id,
                                          const std::filesystem::path& session_root,
                                          double                       focal_length_mm);

    // Status of the active job, if any. Filtered by camera so the route layer
    // can distinguish "no job for this camera" (404) from "a job for a
    // different camera is running" (returns nullopt either way; the route
    // layer maps both to 404 — see routes_calibration.cpp).
    std::optional<CalibrationJobStatus> kalibr_job_status(int64_t camera_id);

    // Pointer to the underlying job for SSE handlers that need to call
    // wait_for_log / log_slice. Returns nullptr if no active job for this
    // camera. The pointer is valid only while the supervisor's mu_ is held —
    // but SSE handlers retain it implicitly via the shared_ptr we expose
    // (see implementation). Use kalibr_job_handle() in callers.
    std::shared_ptr<KalibrJob> kalibr_job_handle(int64_t camera_id);

    // SIGTERM the job's process group. No-op if no job, or job not for this
    // camera, or job already ended. Returns true if a TERM was issued.
    bool kalibr_job_cancel(int64_t camera_id);

private:
    struct Session {
        std::string                                  session_id;
        std::filesystem::path                        root;
        double                                       focal_length_mm = 0.0;  // captured at start
        std::unique_ptr<RosbagRecordingConsumer>     consumer;
        std::chrono::steady_clock::time_point        started_at;
    };

    CalibrationSessionStatus status_locked(const Session& s) const;
    std::string              build_suggested_command(
                                 const std::filesystem::path& dataset_root,
                                 double                       focal_length_mm) const;

    CameraSupervisor&                 cameras_;
    CameraRepository&                 repository_;
    std::filesystem::path             root_;
    std::mutex                        mu_;
    std::map<int64_t, Session>        sessions_;

    // shared_ptr because SSE handlers hold a copy across an HTTP response's
    // lifetime, which can outlive the supervisor's lock. The supervisor
    // resets its own slot when a new job starts; existing handlers continue
    // to observe the old job until they unwind.
    std::shared_ptr<KalibrJob>        kalibr_job_;
    int64_t                           kalibr_job_camera_ = 0;  // 0 = none
};

}  // namespace gw::server
