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
    bool                               hardware_sync_enabled = false;
    std::optional<int64_t>             trigger_output_pin;  // 1..6 when hw-sync is on
    // Mounting rotation (0/90/180/270). Never restarts the producer; a change
    // rebuilds the extra consumers so the VIO feeder's 180° flip tracks it.
    int64_t                            orientation = 0;
    std::unique_ptr<SpinnakerProducer> producer;     // null when offline
    std::shared_ptr<StreamConsumer>    stream;       // null when offline
    std::string                        last_start_error;  // why offline; empty when online
    // Factory-made role consumers (e.g. AprilTag detection), attached after
    // the stream and detached before the producer dies. The cached role /
    // calibration stamps detect when on_camera_updated must rebuild them.
    std::vector<std::shared_ptr<gw::IConsumer>> extra;
    std::optional<std::string>         role;
    std::optional<int64_t>             calibrated_at;
    std::optional<int64_t>             extrinsics_calibrated_at;
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
    gw::IPulseStamper*                       stamper = nullptr;
    std::mutex                               mu;
    Spinnaker::SystemPtr                     system;
    std::unique_ptr<SystemEventListener>     listener;
    std::map<int64_t, CameraSlot>            slots_by_id;
    std::unordered_map<std::string, int64_t> id_by_serial;
    std::vector<ConsumerFactory>             factories;
    bool                                     started = false;
    // Mode capabilities are static per camera, but enumerating them on an
    // unregistered camera costs a full Spinnaker Init()/DeInit() cycle on the
    // device — and rapid cycles can abort inside the SDK's U3V event teardown
    // (pthread_mutex_destroy assert). Cache per serial for the process
    // lifetime so the add-camera dialog touches the hardware at most once.
    std::unordered_map<std::string, gw::VideoModeList> standalone_modes_by_serial;

    Impl(CameraRepository& r, StreamParams p, gw::IPulseStamper* s)
        : repo(r), params(p), stamper(s) {}

    // All of these expect impl mutex to be held by the caller.
    void load_db_into_slots_locked();
    void enumerate_and_match_locked();
    void try_start_slot_locked(CameraSlot& slot, Spinnaker::CameraPtr cam);
    void stop_slot_locked(CameraSlot& slot);
    void start_extra_locked(CameraSlot& slot);
    void stop_extra_locked(CameraSlot& slot);
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
        s.hardware_sync_enabled = row.hardware_sync_enabled;
        s.trigger_output_pin    = row.trigger_output_pin;
        s.orientation           = row.orientation;
        slots_by_id.emplace(row.id, std::move(s));
        id_by_serial[row.serial] = row.id;
    }
}

void CameraSupervisor::Impl::try_start_slot_locked(CameraSlot& slot, Spinnaker::CameraPtr cam) {
    if (slot.producer) return;  // already running

    auto binding    = std::make_unique<SpinnakerCameraBinding>();
    binding->system = system;
    binding->cam    = cam;

    gw::HardwareSyncConfig hw_sync;
    hw_sync.enabled            = slot.hardware_sync_enabled;
    hw_sync.trigger_output_pin = static_cast<uint8_t>(slot.trigger_output_pin.value_or(0));
    hw_sync.stamper            = stamper;
    auto producer = std::make_unique<SpinnakerProducer>(
        slot.name, slot.serial, slot.mode, slot.settings, hw_sync);
    producer->bind_camera(std::move(binding));
    try {
        producer->start();
    } catch (const std::exception& e) {
        std::cerr << "CameraSupervisor: failed to start camera '" << slot.name
                  << "' (serial " << slot.serial << "): " << e.what() << "\n";
        slot.last_start_error = e.what();
        return;
    }
    slot.last_start_error.clear();

    auto stream = std::make_shared<StreamConsumer>(
        params.width, params.height, params.fps, params.bitrate_bps);
    stream->attach(producer->channel());

    slot.producer = std::move(producer);
    slot.stream   = std::move(stream);
    slot.fps      = FpsSampler{};
    // Cache what the camera actually accepted (post-clamp, post-quantization).
    try { slot.settings = slot.producer->current_settings(); } catch (...) {}

    start_extra_locked(slot);

    std::cerr << "CameraSupervisor: camera '" << slot.name
              << "' (serial " << slot.serial << ") online\n";
}

void CameraSupervisor::Impl::stop_slot_locked(CameraSlot& slot) {
    // Extras (and the stream) MUST detach before the producer is destroyed —
    // the FrameChannel lives inside the producer; a consumer still
    // subscribed at destruction time would dangle.
    stop_extra_locked(slot);
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

void CameraSupervisor::Impl::start_extra_locked(CameraSlot& slot) {
    if (!slot.producer || factories.empty()) return;
    const auto row = repo.get(slot.id);
    if (!row) return;
    slot.role                     = row->role;
    slot.calibrated_at            = row->calibrated_at;
    slot.extrinsics_calibrated_at = row->extrinsics_calibrated_at;
    for (const auto& factory : factories) {
        try {
            auto consumer = factory(*row);
            if (!consumer) continue;
            consumer->attach(slot.producer->channel());
            slot.extra.push_back(std::move(consumer));
        } catch (const std::exception& e) {
            // A factory failure (e.g. unparseable calibration) must not take
            // the camera offline — streaming and recording still work.
            std::cerr << "CameraSupervisor: consumer factory failed for '"
                      << slot.name << "': " << e.what() << "\n";
        }
    }
}

void CameraSupervisor::Impl::stop_extra_locked(CameraSlot& slot) {
    for (auto& consumer : slot.extra) {
        try { consumer->detach(); } catch (...) {}
    }
    slot.extra.clear();
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

CameraSupervisor::CameraSupervisor(CameraRepository&  repo,
                                   StreamParams       params,
                                   gw::IPulseStamper* stamper)
    : impl_(std::make_unique<Impl>(repo, params, stamper)) {}

void CameraSupervisor::register_consumer_factory(ConsumerFactory factory) {
    std::lock_guard lk(impl_->mu);
    impl_->factories.push_back(std::move(factory));
}

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

gw::FrameChannel* CameraSupervisor::frame_channel_for(int64_t camera_id) {
    std::lock_guard lk(impl_->mu);
    auto it = impl_->slots_by_id.find(camera_id);
    if (it == impl_->slots_by_id.end() || !it->second.producer) return nullptr;
    return &it->second.producer->channel();
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
        s.last_start_error = slot.last_start_error;
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
    s.hardware_sync_enabled = row->hardware_sync_enabled;
    s.trigger_output_pin    = row->trigger_output_pin;
    s.orientation           = row->orientation;
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

    // Mode, hardware_sync_enabled, and trigger_output_pin all require the
    // producer to re-Init the camera so the relevant GenICam nodes (VideoMode,
    // TriggerMode/Source/Selector) get written. Gain/exposure changes have
    // already been applied live by the route layer.
    const bool mode_changed     = row->mode != slot.mode;
    const bool hw_sync_changed  = row->hardware_sync_enabled != slot.hardware_sync_enabled;
    const bool pin_changed      = row->trigger_output_pin    != slot.trigger_output_pin;

    // Role, calibration, or orientation changes only affect the factory-made
    // extra consumers (orientation drives the VIO feeder's 180° flip) —
    // rebuild them in place against the live channel, no producer restart.
    const bool extra_inputs_changed =
        row->role != slot.role || row->calibrated_at != slot.calibrated_at ||
        row->extrinsics_calibrated_at != slot.extrinsics_calibrated_at ||
        row->orientation != slot.orientation;

    slot.mode                  = row->mode;
    slot.hardware_sync_enabled = row->hardware_sync_enabled;
    slot.trigger_output_pin    = row->trigger_output_pin;
    slot.orientation           = row->orientation;

    if (!(mode_changed || hw_sync_changed || pin_changed)) {
        if (extra_inputs_changed && slot.producer) {
            impl_->stop_extra_locked(slot);
            impl_->start_extra_locked(slot);
        }
        return;
    }
    if (!slot.producer) return;

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

    if (auto cached = impl_->standalone_modes_by_serial.find(serial);
        cached != impl_->standalone_modes_by_serial.end()) {
        return cached->second;
    }

    Spinnaker::CameraList cams = impl_->system->GetCameras();
    const unsigned int n = cams.GetSize();
    std::optional<gw::VideoModeList> out;
    for (unsigned int i = 0; i < n; ++i) {
        Spinnaker::CameraPtr cam = cams.GetByIndex(i);
        if (read_tl_string(cam, "DeviceSerialNumber") == serial) {
            // Spinnaker exceptions propagate to the route layer (mapped to 503).
            out = gw::enumerate_video_modes_standalone(cam);
            impl_->standalone_modes_by_serial[serial] = *out;
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
