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
    // The on-disk recording is left in place for the user to feed into the
    // Kalibr container.
    CalibrationSessionResult stop(int64_t camera_id);

    // Snapshot of an active session, or nullopt if none.
    std::optional<CalibrationSessionStatus> status(int64_t camera_id);

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
};

}  // namespace gw::server
