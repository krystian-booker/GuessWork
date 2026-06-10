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
#include <vector>

namespace gw {
class RosbagRecordingConsumer;
class MultiTopicBagRecorder;
}

namespace gw::server {

class CameraSupervisor;
class CameraRepository;
class TeensyManager;
class ImuConfigRepository;
class KalibrJob;
class KalibrImuJob;
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
// snapshot fields. sensor_width_px is the live mode width captured at
// session start — start_kalibr_job consumes it for the focal hint.
struct CalibrationSessionResult {
    std::string             session_id;
    std::filesystem::path   path;
    uint64_t                frames_written = 0;
    uint64_t                frames_dropped = 0;
    uint64_t                elapsed_ms     = 0;
    uint32_t                sensor_width_px = 0;
    std::string             suggested_command;
};

// --- Extrinsics (camera-IMU) sessions ---

struct ExtrinsicsCameraStatus {
    int64_t     camera_id = 0;
    std::string topic;            // "/camN/image_raw"
    uint64_t    frames_written = 0;
    uint64_t    frames_dropped = 0;
};

struct ExtrinsicsSessionStatus {
    std::string                         session_id;
    std::filesystem::path               path;
    std::vector<ExtrinsicsCameraStatus> cameras;   // ordered cam0..camN
    uint64_t                            imu_written = 0;
    uint64_t                            imu_dropped = 0;
    uint64_t                            elapsed_ms  = 0;
};

// Stop result — carries everything start_imu_job() needs so the route layer
// can chain DELETE-recording → job start without re-deriving state.
struct ExtrinsicsSessionResult {
    std::string                         session_id;
    std::filesystem::path               path;
    std::vector<ExtrinsicsCameraStatus> cameras;   // ordered cam0..camN
    uint64_t                            imu_written = 0;
    uint64_t                            imu_dropped = 0;
    uint64_t                            elapsed_ms  = 0;
    std::string                         model;            // pair-flow camera model
    std::vector<uint32_t>               focal_hints_px;   // per camera, topic order
    std::string                         suggested_command;
};

class CalibrationError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// Owns calibration recording sessions and Kalibr job lifecycles:
//   - per-camera intrinsics sessions (single image topic bag), and
//   - at most ONE extrinsics session system-wide (N cameras + /imu0 into a
//     single bag, Teensy-clock timestamps) — it owns the IMU stream and the
//     operator is physically exciting the rig, so concurrency is meaningless.
// Each session writes under <root>/<session_id>/.
//
// Thread-safety: a mutex serialises mutations to the session state; the
// recording consumers do their own work on their own worker threads,
// independent of this lock.
class CalibrationSupervisor {
public:
    // calibrations_root is the directory under which session subdirectories
    // are created (typically ~/.guesswork/calibrations). The repository is
    // consulted at session start to read the camera's focal_length_mm, which
    // drives the Kalibr focal-length hint and the camera model selection.
    // teensy provides the IMU sample bus + health gating for extrinsics
    // sessions; imu_config provides the noise model for imu.yaml.
    CalibrationSupervisor(CameraSupervisor&     cameras,
                          CameraRepository&     repository,
                          TeensyManager&        teensy,
                          ImuConfigRepository&  imu_config,
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

    // --- Extrinsics (camera-IMU) recording session ---

    // Starts the system-wide extrinsics session recording camera_ids (in
    // topic order: camera_ids[i] → /cam<i>/image_raw) plus /imu0. Validates,
    // before any side effect:
    //   - 1..2 unique camera ids, no intrinsics session on any of them,
    //   - each camera online with hardware_sync_enabled,
    //   - Teensy connected + armed + telemetry + IMU healthy,
    //   - single-camera flow: stored intrinsics exist, parse, and match the
    //     camera's current mode resolution,
    //   - pair flow: both cameras derive the same Kalibr model.
    // Throws CalibrationError on any violation.
    ExtrinsicsSessionStatus start_extrinsics(const std::vector<int64_t>& camera_ids);

    // Stops the extrinsics session. Throws CalibrationError if none active.
    ExtrinsicsSessionResult stop_extrinsics();

    std::optional<ExtrinsicsSessionStatus> extrinsics_status();

    // --- Kalibr job lifecycle (post-recording) ---
    //
    // Concurrency is global: at most one job system-wide — of EITHER kind
    // (intrinsics or imu-camera) — because each Kalibr run consumes ~all of
    // the Colima VM's CPU/RAM. Start throws if another job is still running.

    // Spawn the calibrate.sh subprocess for <session_root> using the camera's
    // recorded focal_length_mm and the session's live sensor width. Returns
    // the initial status snapshot (state will be Running on success). Throws
    // CalibrationError if a job is already active (for any camera).
    CalibrationJobStatus start_kalibr_job(int64_t                      camera_id,
                                          const std::filesystem::path& session_root,
                                          double                       focal_length_mm,
                                          uint32_t                     sensor_width_px);

    // Spawn the calibrate_imu.sh subprocess for a stopped extrinsics session.
    // Throws CalibrationError if any Kalibr job is already active.
    CalibrationJobStatus start_imu_job(const ExtrinsicsSessionResult& result);

    std::optional<CalibrationJobStatus> imu_job_status();
    std::shared_ptr<KalibrImuJob>       imu_job_handle();
    bool                                imu_job_cancel();

    // Status of the active job, if any. Filtered by camera so the route layer
    // can distinguish "no job for this camera" (404) from "a job for a
    // different camera is running" (returns nullopt either way; the route
    // layer maps both to 404 — see routes_calibration.cpp).
    std::optional<CalibrationJobStatus> kalibr_job_status(int64_t camera_id);

    // Shared handle to the active job for this camera (or nullptr). The
    // shared_ptr keeps the job alive past supervisor lock release, so SSE
    // handlers can hold it for the lifetime of their HTTP response without
    // any further synchronisation.
    std::shared_ptr<KalibrJob> kalibr_job_handle(int64_t camera_id);

    // SIGTERM the job's process group. No-op if no job, or job not for this
    // camera, or job already ended. Returns true if a TERM was issued.
    bool kalibr_job_cancel(int64_t camera_id);

private:
    struct Session {
        std::string                                  session_id;
        std::filesystem::path                        root;
        double                                       focal_length_mm = 0.0;  // captured at start
        uint32_t                                     sensor_width_px = 0;    // live mode width at start
        std::unique_ptr<RosbagRecordingConsumer>     consumer;
        std::chrono::steady_clock::time_point        started_at;
    };

    struct ExtrinsicsParticipant {
        int64_t     camera_id = 0;
        std::string topic;
        std::string frame_id;
        double      focal_length_mm = 0.0;
        uint32_t    sensor_width_px = 0;
    };

    struct ExtrinsicsSession {
        std::string                                session_id;
        std::filesystem::path                      root;
        std::vector<ExtrinsicsParticipant>         cams;
        std::string                                model;
        std::vector<uint32_t>                      focal_hints_px;
        std::unique_ptr<gw::MultiTopicBagRecorder> recorder;
        std::chrono::steady_clock::time_point      started_at;
    };

    CalibrationSessionStatus status_locked(const Session& s) const;
    ExtrinsicsSessionStatus  extrinsics_status_locked(const ExtrinsicsSession& s) const;
    std::string              build_suggested_command(
                                 const std::filesystem::path& dataset_root,
                                 double                       focal_length_mm,
                                 uint32_t                     sensor_width_px) const;
    // Throws CalibrationError if a Kalibr job of either kind is active;
    // reaps finished jobs from both slots. Caller holds mu_.
    void                     reap_or_throw_if_job_running_locked();
    uint32_t                 live_sensor_width(int64_t camera_id);

    CameraSupervisor&                 cameras_;
    CameraRepository&                 repository_;
    TeensyManager&                    teensy_;
    ImuConfigRepository&              imu_config_;
    std::filesystem::path             root_;
    std::mutex                        mu_;
    std::map<int64_t, Session>        sessions_;
    std::optional<ExtrinsicsSession>  ext_session_;

    // shared_ptr because SSE handlers hold a copy across an HTTP response's
    // lifetime, which can outlive the supervisor's lock. The supervisor
    // resets its own slot when a new job starts; existing handlers continue
    // to observe the old job until they unwind.
    std::shared_ptr<KalibrJob>        kalibr_job_;
    int64_t                           kalibr_job_camera_ = 0;  // 0 = none
    std::shared_ptr<KalibrImuJob>     imu_job_;
};

}  // namespace gw::server
