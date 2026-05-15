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
class RecordingConsumer;
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
// pasteable basalt_calibrate command line; the path/counters mirror the
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
// writes a EuRoC-format dataset under <root>/<session_id>/, which the user
// hands off to basalt_calibrate manually. Single session per camera at a time.
//
// Thread-safety: a mutex serialises mutations to the session map; the
// RecordingConsumer attached inside each session does its own work on its own
// worker thread, independent of this lock.
class CalibrationSupervisor {
public:
    // calibrations_root is the directory under which session subdirectories
    // are created (typically ~/.guesswork/calibrations). The repository is
    // consulted at session start to read the camera's lens_type, which drives
    // the basalt cam-types in the suggested command.
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
    // The on-disk recording is left in place for the user to feed into
    // basalt_calibrate.
    CalibrationSessionResult stop(int64_t camera_id);

    // Snapshot of an active session, or nullopt if none.
    std::optional<CalibrationSessionStatus> status(int64_t camera_id);

private:
    struct Session {
        std::string                                  session_id;
        std::filesystem::path                        root;
        std::string                                  lens_type;  // captured at start
        std::unique_ptr<RecordingConsumer>           consumer;
        std::chrono::steady_clock::time_point        started_at;
    };

    CalibrationSessionStatus status_locked(const Session& s) const;
    std::string              build_suggested_command(
                                 const std::filesystem::path& dataset_root,
                                 std::string_view             lens_type) const;

    CameraSupervisor&                 cameras_;
    CameraRepository&                 repository_;
    std::filesystem::path             root_;
    std::mutex                        mu_;
    std::map<int64_t, Session>        sessions_;
};

}  // namespace gw::server
