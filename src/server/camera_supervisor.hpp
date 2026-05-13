#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "producer/spinnaker_video_modes.hpp"

namespace gw::server {

class CameraRepository;
class StreamConsumer;

// Default streaming params applied to every camera's StreamConsumer. Per-camera
// overrides are out of scope.
struct StreamParams {
    uint32_t width;
    uint32_t height;
    uint32_t fps;
    uint32_t bitrate_bps;
};

// One entry in /api/cameras/available — a connected Spinnaker camera that is
// not currently mapped in the DB.
struct AvailableCamera {
    std::string serial;
    std::string model;
    std::string vendor;
};

// Per-camera snapshot used by /api/status and /api/cameras list responses.
struct CameraStatus {
    int64_t     id              = 0;
    std::string name;
    std::string serial;
    bool        online          = false;
    uint64_t    frames_produced = 0;
    uint64_t    frames_dropped  = 0;
    uint64_t    frames_incomplete = 0;
    double      fps_1s          = 0.0;
};

// Multi-camera orchestrator.
//
// Owns the single Spinnaker::SystemPtr for the process. Subscribes to
// device-arrival/removal events so producers come online when hardware appears
// and stop cleanly when it disappears. The DB is the source of truth for which
// serials we care about; this class is the source of truth for what is
// currently running.
class CameraSupervisor {
public:
    CameraSupervisor(CameraRepository& repo, StreamParams default_params);
    ~CameraSupervisor();

    CameraSupervisor(const CameraSupervisor&)            = delete;
    CameraSupervisor& operator=(const CameraSupervisor&) = delete;

    // Acquires the Spinnaker system, registers the arrival/removal handler,
    // enumerates current cameras, and starts producers for the ones already
    // present in the DB. Idempotent. Throws on Spinnaker init failure.
    void start();

    // Connected Spinnaker cameras whose serials are NOT in the DB.
    std::vector<AvailableCamera> list_unmapped_connected();

    // Returns the StreamConsumer for the given DB camera id, or nullptr if the
    // camera is not currently online. Holding the shared_ptr keeps the
    // consumer alive even if the slot is torn down concurrently.
    std::shared_ptr<StreamConsumer> stream_consumer_for(int64_t camera_id);

    // O(1) online check without allocating a full snapshot.
    bool is_online(int64_t camera_id);

    // One entry per DB row (online or offline).
    std::vector<CameraStatus> snapshot_all();

    // Called by the routes layer after a successful CRUD operation on the
    // cameras table. on_camera_added starts the producer immediately if the
    // hardware is already connected; otherwise it'll start on the next arrival
    // event. on_camera_removed stops the producer if running.
    // on_camera_updated re-reads the row so cached fields (e.g. name) stay in
    // sync with the DB.
    void on_camera_added(int64_t camera_id);
    void on_camera_updated(int64_t camera_id);
    void on_camera_removed(int64_t camera_id);

    // Returns the producer's cached VideoModeList for an online camera.
    // nullopt if the camera id has no running producer (offline or unknown).
    // Callers distinguish 404 vs 409 by checking repo.get(id) separately.
    std::optional<gw::VideoModeList> list_video_modes_for_id(int64_t camera_id);

    // Walks connected Spinnaker cameras, finds the one with the given serial,
    // and enumerates its VideoMode entries. nullopt if no camera with that
    // serial is currently connected. Spinnaker exceptions during Init are
    // propagated for the route layer to map to 503.
    std::optional<gw::VideoModeList> list_video_modes_for_serial(const std::string& serial);

    // Currently-active mode entry (geometry + max FPS) for an online camera.
    // nullopt if the camera is offline, unknown, or its producer didn't cache
    // a matching entry.
    std::optional<gw::VideoModeOption> current_mode_for(int64_t camera_id);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace gw::server
