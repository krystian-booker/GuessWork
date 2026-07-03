// GW_STUB_SPINNAKER build of CameraSupervisor: no SDK, no hardware — every
// camera is permanently offline. This exists so dev machines and CI without
// the license-gated FLIR Spinnaker SDK can still build and run guesswork and
// the full gw_tests suite; the rest of the system already handles offline
// cameras, so the stub simply reports that state everywhere.
//
// Selected by -DGW_STUB_SPINNAKER=ON in place of camera_supervisor.cpp — the
// two TUs implement the same public header and must be kept in sync with it.

#include "server/camera_supervisor.hpp"

#include <iostream>
#include <stdexcept>
#include <vector>

#include "server/camera_repository.hpp"

namespace gw::server {

struct CameraSupervisor::Impl {
    CameraRepository& repo;
    std::vector<ConsumerFactory> factories;  // accepted, never invoked
};

CameraSupervisor::CameraSupervisor(CameraRepository&  repo,
                                   StreamParams       /*default_params*/,
                                   gw::IPulseStamper* /*stamper*/)
    : impl_(std::make_unique<Impl>(Impl{repo, {}})) {}

CameraSupervisor::~CameraSupervisor() = default;

void CameraSupervisor::register_consumer_factory(ConsumerFactory factory) {
    impl_->factories.push_back(std::move(factory));
}

void CameraSupervisor::start() {
    std::cerr << "==========================================================\n"
                 "CameraSupervisor: GW_STUB_SPINNAKER build — the Spinnaker\n"
                 "SDK is not linked. No camera will ever come online. This\n"
                 "build is for development/CI only, never the robot.\n"
                 "==========================================================\n";
}

std::vector<AvailableCamera> CameraSupervisor::list_unmapped_connected() {
    return {};
}

std::shared_ptr<StreamConsumer> CameraSupervisor::stream_consumer_for(int64_t) {
    return nullptr;
}

FrameChannel* CameraSupervisor::frame_channel_for(int64_t) { return nullptr; }

bool CameraSupervisor::is_online(int64_t) { return false; }

std::vector<CameraStatus> CameraSupervisor::snapshot_all() {
    std::vector<CameraStatus> out;
    for (const auto& row : impl_->repo.list_all()) {
        CameraStatus st;
        st.id               = row.id;
        st.name             = row.name;
        st.serial           = row.serial;
        st.online           = false;
        st.last_start_error = "built with GW_STUB_SPINNAKER (no camera support)";
        out.push_back(std::move(st));
    }
    return out;
}

void CameraSupervisor::on_camera_added(int64_t) {}
void CameraSupervisor::on_camera_updated(int64_t) {}
void CameraSupervisor::on_camera_removed(int64_t) {}

std::optional<gw::VideoModeList> CameraSupervisor::list_video_modes_for_id(int64_t) {
    return std::nullopt;
}

std::optional<gw::VideoModeList>
CameraSupervisor::list_video_modes_for_serial(const std::string&) {
    return std::nullopt;
}

std::optional<gw::VideoModeOption> CameraSupervisor::current_mode_for(int64_t) {
    return std::nullopt;
}

gw::CameraSettingsValues
CameraSupervisor::apply_settings_live(int64_t, const gw::CameraSettingsPatch&) {
    throw std::runtime_error("camera offline (GW_STUB_SPINNAKER build)");
}

std::optional<gw::CameraSettingsLimits>
CameraSupervisor::settings_limits_for_id(int64_t) {
    return std::nullopt;
}

}  // namespace gw::server
