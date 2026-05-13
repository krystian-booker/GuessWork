#include "server/camera_supervisor.hpp"

#include <chrono>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "producer/spinnaker_producer.hpp"
#include "producer/spinnaker_producer_internal.hpp"
#include "server/camera_repository.hpp"
#include "server/stream_consumer.hpp"

#include <SpinGenApi/SpinnakerGenApi.h>

namespace gw::server {

namespace {

// Read a string node from a CameraPtr's TL-device nodemap. Returns empty
// string if the node isn't readable. Safe to call before Init().
std::string read_tl_string(Spinnaker::CameraPtr cam, const char* node_name) {
    try {
        Spinnaker::GenApi::INodeMap& nm = cam->GetTLDeviceNodeMap();
        Spinnaker::GenApi::CStringPtr ptr = nm.GetNode(node_name);
        if (Spinnaker::GenApi::IsReadable(ptr)) {
            return std::string(ptr->GetValue().c_str());
        }
    } catch (...) {
        // Fall through — node not available.
    }
    return "";
}

}  // namespace

// -------------------------------------------------------------------------
// Slot + FPS sampling
// -------------------------------------------------------------------------

struct FpsSampler {
    using Clock     = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;
    static constexpr auto kWindow = std::chrono::milliseconds(1000);

    TimePoint last_t        = Clock::now();
    uint64_t  last_count    = 0;
    double    cached        = 0.0;

    double sample(uint64_t total_count) {
        const auto now = Clock::now();
        const auto dt  = now - last_t;
        if (dt >= kWindow) {
            const auto seconds = std::chrono::duration<double>(dt).count();
            cached     = (seconds > 0.0)
                             ? static_cast<double>(total_count - last_count) / seconds
                             : 0.0;
            last_t     = now;
            last_count = total_count;
        }
        return cached;
    }
};

struct CameraSlot {
    int64_t                            id     = 0;
    std::string                        name;
    std::string                        serial;
    std::optional<std::string>         mode;         // GenICam VideoMode symbolic
    gw::CameraSettingsValues           settings;     // persisted live-tunables
    std::unique_ptr<SpinnakerProducer> producer;     // null when offline
    std::shared_ptr<StreamConsumer>    stream;       // null when offline
    FpsSampler                         fps;
};

namespace {

gw::CameraSettingsValues settings_from_row(const Camera& row) {
    gw::CameraSettingsValues v;
    v.gain_auto     = row.gain_auto;
    v.gain          = row.gain;
    v.exposure_auto = row.exposure_auto;
    v.exposure      = row.exposure;
    return v;
}

}  // namespace

class SystemEventListener : public Spinnaker::InterfaceEventHandler {
public:
    using Cb = std::function<void(Spinnaker::CameraPtr)>;
    SystemEventListener(Cb on_arrival, Cb on_removal)
        : on_arrival_(std::move(on_arrival)), on_removal_(std::move(on_removal)) {}

    void OnDeviceArrival(Spinnaker::CameraPtr cam) override {
        try { on_arrival_(cam); }
        catch (const std::exception& e) {
            std::cerr << "CameraSupervisor: arrival handler threw: " << e.what() << "\n";
        }
    }

    void OnDeviceRemoval(Spinnaker::CameraPtr cam) override {
        try { on_removal_(cam); }
        catch (const std::exception& e) {
            std::cerr << "CameraSupervisor: removal handler threw: " << e.what() << "\n";
        }
    }

private:
    Cb on_arrival_;
    Cb on_removal_;
};

// -------------------------------------------------------------------------
// Impl
// -------------------------------------------------------------------------

struct CameraSupervisor::Impl {
    CameraRepository&                        repo;
    StreamParams                             params;
    std::mutex                               mu;
    Spinnaker::SystemPtr                     system;
    std::unique_ptr<SystemEventListener>     listener;
    std::map<int64_t, CameraSlot>            slots_by_id;
    std::unordered_map<std::string, int64_t> id_by_serial;
    bool                                     started = false;

    Impl(CameraRepository& r, StreamParams p) : repo(r), params(p) {}

    // All of these expect impl mutex to be held by the caller.
    void load_db_into_slots_locked();
    void enumerate_and_match_locked();
    void try_start_slot_locked(CameraSlot& slot, Spinnaker::CameraPtr cam);
    void stop_slot_locked(CameraSlot& slot);
    void on_device_arrival(Spinnaker::CameraPtr cam);   // called from Spinnaker thread
    void on_device_removal(Spinnaker::CameraPtr cam);   // called from Spinnaker thread
};

// -------------------------------------------------------------------------
// Impl methods
// -------------------------------------------------------------------------

void CameraSupervisor::Impl::load_db_into_slots_locked() {
    for (const auto& row : repo.list_all()) {
        if (row.serial.empty()) continue;   // legacy POC rows
        CameraSlot s;
        s.id       = row.id;
        s.name     = row.name;
        s.serial   = row.serial;
        s.mode     = row.mode;
        s.settings = settings_from_row(row);
        slots_by_id.emplace(row.id, std::move(s));
        id_by_serial[row.serial] = row.id;
    }
}

void CameraSupervisor::Impl::try_start_slot_locked(CameraSlot& slot, Spinnaker::CameraPtr cam) {
    if (slot.producer) return;  // already running

    auto binding    = std::make_unique<SpinnakerCameraBinding>();
    binding->system = system;
    binding->cam    = cam;

    auto producer = std::make_unique<SpinnakerProducer>(
        slot.name, slot.serial, slot.mode, slot.settings);
    producer->bind_camera(std::move(binding));
    try {
        producer->start();
    } catch (const std::exception& e) {
        std::cerr << "CameraSupervisor: failed to start camera '" << slot.name
                  << "' (serial " << slot.serial << "): " << e.what() << "\n";
        return;
    }

    auto stream = std::make_shared<StreamConsumer>(
        params.width, params.height, params.fps, params.bitrate_bps);
    stream->attach(producer->channel());

    slot.producer = std::move(producer);
    slot.stream   = std::move(stream);
    slot.fps      = FpsSampler{};
    // Cache what the camera actually accepted (post-clamp, post-quantization).
    try { slot.settings = slot.producer->current_settings(); } catch (...) {}
    std::cerr << "CameraSupervisor: camera '" << slot.name
              << "' (serial " << slot.serial << ") online\n";
}

void CameraSupervisor::Impl::stop_slot_locked(CameraSlot& slot) {
    if (slot.stream) {
        try { slot.stream->detach(); } catch (...) {}
        slot.stream.reset();
    }
    if (slot.producer) {
        try { slot.producer->stop(); } catch (...) {}
        slot.producer.reset();
        std::cerr << "CameraSupervisor: camera '" << slot.name
                  << "' (serial " << slot.serial << ") offline\n";
    }
    slot.fps = FpsSampler{};
}

void CameraSupervisor::Impl::enumerate_and_match_locked() {
    if (!system) return;
    Spinnaker::CameraList cams = system->GetCameras();
    const unsigned int n = cams.GetSize();
    for (unsigned int i = 0; i < n; ++i) {
        Spinnaker::CameraPtr cam = cams.GetByIndex(i);
        const std::string serial = read_tl_string(cam, "DeviceSerialNumber");
        if (serial.empty()) continue;
        auto it = id_by_serial.find(serial);
        if (it == id_by_serial.end()) continue;     // not mapped — ignore
        auto slot_it = slots_by_id.find(it->second);
        if (slot_it == slots_by_id.end()) continue; // stale
        try_start_slot_locked(slot_it->second, cam);
    }
    cams.Clear();
}

void CameraSupervisor::Impl::on_device_arrival(Spinnaker::CameraPtr cam) {
    const std::string serial = read_tl_string(cam, "DeviceSerialNumber");
    if (serial.empty()) return;

    std::lock_guard lk(mu);
    auto it = id_by_serial.find(serial);
    if (it == id_by_serial.end()) return;
    auto slot_it = slots_by_id.find(it->second);
    if (slot_it == slots_by_id.end()) return;
    try_start_slot_locked(slot_it->second, cam);
}

void CameraSupervisor::Impl::on_device_removal(Spinnaker::CameraPtr cam) {
    const std::string serial = read_tl_string(cam, "DeviceSerialNumber");
    if (serial.empty()) return;

    std::lock_guard lk(mu);
    auto it = id_by_serial.find(serial);
    if (it == id_by_serial.end()) return;
    auto slot_it = slots_by_id.find(it->second);
    if (slot_it == slots_by_id.end()) return;
    stop_slot_locked(slot_it->second);
}

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------

CameraSupervisor::CameraSupervisor(CameraRepository& repo, StreamParams params)
    : impl_(std::make_unique<Impl>(repo, params)) {}

CameraSupervisor::~CameraSupervisor() {
    std::lock_guard lk(impl_->mu);
    for (auto& [id, slot] : impl_->slots_by_id) {
        impl_->stop_slot_locked(slot);
    }
    if (impl_->listener && impl_->system) {
        try { impl_->system->UnregisterEventHandler(*impl_->listener); } catch (...) {}
    }
    impl_->listener.reset();
    if (impl_->system) {
        impl_->system->ReleaseInstance();
        impl_->system = nullptr;
    }
}

void CameraSupervisor::start() {
    std::lock_guard lk(impl_->mu);
    if (impl_->started) return;

    impl_->system = Spinnaker::System::GetInstance();
    const Spinnaker::LibraryVersion v = impl_->system->GetLibraryVersion();
    std::cerr << "CameraSupervisor: Spinnaker library "
              << v.major << "." << v.minor << "." << v.type << "." << v.build << "\n";

    impl_->load_db_into_slots_locked();

    auto* impl_ptr = impl_.get();
    impl_->listener = std::make_unique<SystemEventListener>(
        [impl_ptr](Spinnaker::CameraPtr cam) { impl_ptr->on_device_arrival(cam); },
        [impl_ptr](Spinnaker::CameraPtr cam) { impl_ptr->on_device_removal(cam); });
    impl_->system->RegisterEventHandler(*impl_->listener, /*updateInterface=*/true);

    impl_->enumerate_and_match_locked();
    impl_->started = true;
}

std::vector<AvailableCamera> CameraSupervisor::list_unmapped_connected() {
    std::lock_guard lk(impl_->mu);
    std::vector<AvailableCamera> out;
    if (!impl_->system) return out;

    Spinnaker::CameraList cams = impl_->system->GetCameras();
    const unsigned int n = cams.GetSize();
    for (unsigned int i = 0; i < n; ++i) {
        Spinnaker::CameraPtr cam = cams.GetByIndex(i);
        const std::string serial = read_tl_string(cam, "DeviceSerialNumber");
        if (serial.empty()) continue;
        if (impl_->id_by_serial.count(serial)) continue;
        AvailableCamera ac;
        ac.serial = serial;
        ac.model  = read_tl_string(cam, "DeviceModelName");
        ac.vendor = read_tl_string(cam, "DeviceVendorName");
        out.push_back(std::move(ac));
    }
    cams.Clear();
    return out;
}

std::shared_ptr<StreamConsumer> CameraSupervisor::stream_consumer_for(int64_t camera_id) {
    std::lock_guard lk(impl_->mu);
    auto it = impl_->slots_by_id.find(camera_id);
    if (it == impl_->slots_by_id.end()) return nullptr;
    return it->second.stream;
}

bool CameraSupervisor::is_online(int64_t camera_id) {
    std::lock_guard lk(impl_->mu);
    auto it = impl_->slots_by_id.find(camera_id);
    return it != impl_->slots_by_id.end() && it->second.producer != nullptr;
}

std::vector<CameraStatus> CameraSupervisor::snapshot_all() {
    std::lock_guard lk(impl_->mu);
    std::vector<CameraStatus> out;
    out.reserve(impl_->slots_by_id.size());
    for (auto& [id, slot] : impl_->slots_by_id) {
        CameraStatus s;
        s.id     = slot.id;
        s.name   = slot.name;
        s.serial = slot.serial;
        s.online = (slot.producer != nullptr);
        if (slot.producer) {
            const auto ps = slot.producer->stats();
            s.frames_produced   = ps.total_published;
            s.frames_dropped    = ps.total_dropped;
            s.frames_incomplete = ps.total_incomplete;
            s.fps_1s            = slot.fps.sample(ps.total_published);
        }
        out.push_back(std::move(s));
    }
    return out;
}

void CameraSupervisor::on_camera_added(int64_t camera_id) {
    std::lock_guard lk(impl_->mu);
    const auto row = impl_->repo.get(camera_id);
    if (!row) return;
    if (row->serial.empty()) return;

    if (impl_->slots_by_id.count(camera_id)) return;  // already tracking

    CameraSlot s;
    s.id       = row->id;
    s.name     = row->name;
    s.serial   = row->serial;
    s.mode     = row->mode;
    s.settings = settings_from_row(*row);
    auto [it, _] = impl_->slots_by_id.emplace(row->id, std::move(s));
    impl_->id_by_serial[it->second.serial] = it->second.id;

    if (!impl_->system) return;

    // If the camera is already connected, start it now.
    Spinnaker::CameraList cams = impl_->system->GetCameras();
    const unsigned int n = cams.GetSize();
    for (unsigned int i = 0; i < n; ++i) {
        Spinnaker::CameraPtr cam = cams.GetByIndex(i);
        if (read_tl_string(cam, "DeviceSerialNumber") == it->second.serial) {
            impl_->try_start_slot_locked(it->second, cam);
            break;
        }
    }
    cams.Clear();
}

void CameraSupervisor::on_camera_updated(int64_t camera_id) {
    std::lock_guard lk(impl_->mu);
    auto it = impl_->slots_by_id.find(camera_id);
    if (it == impl_->slots_by_id.end()) return;
    const auto row = impl_->repo.get(camera_id);
    if (!row) return;

    CameraSlot& slot = it->second;
    slot.name     = row->name;
    slot.settings = settings_from_row(*row);

    if (row->mode != slot.mode) {
        slot.mode = row->mode;
        // If currently running, restart the producer so the new mode applies.
        if (slot.producer) {
            const std::string serial = slot.serial;
            impl_->stop_slot_locked(slot);

            if (!impl_->system) return;
            Spinnaker::CameraList cams = impl_->system->GetCameras();
            const unsigned int n = cams.GetSize();
            for (unsigned int i = 0; i < n; ++i) {
                Spinnaker::CameraPtr cam = cams.GetByIndex(i);
                if (read_tl_string(cam, "DeviceSerialNumber") == serial) {
                    impl_->try_start_slot_locked(slot, cam);
                    break;
                }
            }
            cams.Clear();
        }
    }
}

std::optional<gw::VideoModeList>
CameraSupervisor::list_video_modes_for_id(int64_t camera_id) {
    std::lock_guard lk(impl_->mu);
    auto it = impl_->slots_by_id.find(camera_id);
    if (it == impl_->slots_by_id.end()) return std::nullopt;
    if (!it->second.producer) return std::nullopt;
    return it->second.producer->cached_video_modes();
}

std::optional<gw::VideoModeOption>
CameraSupervisor::current_mode_for(int64_t camera_id) {
    std::lock_guard lk(impl_->mu);
    auto it = impl_->slots_by_id.find(camera_id);
    if (it == impl_->slots_by_id.end()) return std::nullopt;
    const auto& slot = it->second;
    if (!slot.producer) return std::nullopt;
    const auto& list = slot.producer->cached_video_modes();
    if (!list.supported) return std::nullopt;
    // Prefer the slot's configured mode; fall back to the producer's reported
    // current (when slot.mode is unset, the producer is running the camera's
    // default mode).
    const std::optional<std::string>& want = slot.mode ? slot.mode : list.current;
    if (!want) return std::nullopt;
    for (const auto& opt : list.options) {
        if (opt.name == *want) return opt;
    }
    return std::nullopt;
}

gw::CameraSettingsValues
CameraSupervisor::apply_settings_live(int64_t                          camera_id,
                                      const gw::CameraSettingsPatch&   patch) {
    std::lock_guard lk(impl_->mu);
    auto it = impl_->slots_by_id.find(camera_id);
    if (it == impl_->slots_by_id.end()) {
        throw std::runtime_error("apply_settings_live: unknown camera id");
    }
    CameraSlot& slot = it->second;
    if (!slot.producer) {
        throw std::runtime_error("apply_settings_live: camera is offline");
    }
    const auto applied = slot.producer->apply_settings_live(patch);
    slot.settings = applied;
    return applied;
}

std::optional<gw::CameraSettingsLimits>
CameraSupervisor::settings_limits_for_id(int64_t camera_id) {
    std::lock_guard lk(impl_->mu);
    auto it = impl_->slots_by_id.find(camera_id);
    if (it == impl_->slots_by_id.end()) return std::nullopt;
    if (!it->second.producer) return std::nullopt;
    return it->second.producer->cached_settings_limits();
}

std::optional<gw::VideoModeList>
CameraSupervisor::list_video_modes_for_serial(const std::string& serial) {
    std::lock_guard lk(impl_->mu);
    if (!impl_->system) return std::nullopt;

    Spinnaker::CameraList cams = impl_->system->GetCameras();
    const unsigned int n = cams.GetSize();
    std::optional<gw::VideoModeList> out;
    for (unsigned int i = 0; i < n; ++i) {
        Spinnaker::CameraPtr cam = cams.GetByIndex(i);
        if (read_tl_string(cam, "DeviceSerialNumber") == serial) {
            // Spinnaker exceptions propagate to the route layer (mapped to 503).
            out = gw::enumerate_video_modes_standalone(cam);
            break;
        }
    }
    cams.Clear();
    return out;
}

void CameraSupervisor::on_camera_removed(int64_t camera_id) {
    std::lock_guard lk(impl_->mu);
    auto it = impl_->slots_by_id.find(camera_id);
    if (it == impl_->slots_by_id.end()) return;
    impl_->stop_slot_locked(it->second);
    impl_->id_by_serial.erase(it->second.serial);
    impl_->slots_by_id.erase(it);
}

}  // namespace gw::server
