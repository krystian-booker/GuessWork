#include "server/calibration_supervisor.hpp"

#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <utility>

#include "consumer/rosbag_recording_consumer.hpp"
#include "core/frame_channel.hpp"
#include "server/camera_repository.hpp"
#include "server/camera_supervisor.hpp"

#ifndef GW_KALIBR_DOCKER_IMAGE
#define GW_KALIBR_DOCKER_IMAGE "guesswork/kalibr:latest"
#endif
#ifndef GW_KALIBR_TARGET_DEFAULT
#define GW_KALIBR_TARGET_DEFAULT ""
#endif
#ifndef GW_KALIBR_CALIBRATE_SCRIPT
#define GW_KALIBR_CALIBRATE_SCRIPT "docker/kalibr/calibrate.sh"
#endif
#ifndef GW_SENSOR_PIXEL_PITCH_MM
// Sony IMX264 / 1/1.8" sensor pixel pitch in millimeters. CMake injects this
// at build time via target_compile_definitions; the fallback keeps the file
// editable in isolation. Override at configure with -DGW_SENSOR_PIXEL_PITCH_MM=…
#define GW_SENSOR_PIXEL_PITCH_MM 0.00345
#endif

namespace gw::server {

namespace {

// "20260513-153021-871" — sortable, human-readable, unique per millisecond.
std::string make_session_id() {
    using namespace std::chrono;
    const auto now    = system_clock::now();
    const auto t      = system_clock::to_time_t(now);
    const auto ms_part = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream os;
    os << std::put_time(&tm, "%Y%m%d-%H%M%S")
       << "-" << std::setw(3) << std::setfill('0') << ms_part.count();
    return os.str();
}

// Shell-quotes a path so the suggested command can be copy-pasted into a
// shell with paths containing spaces. Wraps in single quotes and escapes any
// embedded single quotes.
std::string sh_quote(const std::filesystem::path& p) {
    const std::string s = p.string();
    std::string out;
    out.reserve(s.size() + 2);
    out.push_back('\'');
    for (char c : s) {
        if (c == '\'') out += "'\\''";
        else           out.push_back(c);
    }
    out.push_back('\'');
    return out;
}

}  // namespace

CalibrationSupervisor::CalibrationSupervisor(CameraSupervisor&     cameras,
                                             CameraRepository&     repository,
                                             std::filesystem::path root)
    : cameras_(cameras), repository_(repository), root_(std::move(root)) {
    std::error_code ec;
    std::filesystem::create_directories(root_, ec);
    // Don't throw on construction — start() will surface a clearer error if
    // the path is unwritable.
}

CalibrationSupervisor::~CalibrationSupervisor() {
    std::lock_guard lk(mu_);
    for (auto& [id, s] : sessions_) {
        if (s.consumer) s.consumer->detach();
    }
}

CalibrationSessionStatus CalibrationSupervisor::start(int64_t camera_id) {
    std::lock_guard lk(mu_);
    if (sessions_.count(camera_id)) {
        throw CalibrationError("session already active for this camera");
    }
    const auto row = repository_.get(camera_id);
    if (!row) {
        throw CalibrationError("camera not found");
    }
    gw::FrameChannel* ch = cameras_.frame_channel_for(camera_id);
    if (!ch) {
        throw CalibrationError("camera is offline");
    }

    Session s;
    s.session_id      = make_session_id();
    s.root            = root_ / s.session_id;
    s.focal_length_mm = row->focal_length_mm;
    s.consumer        = std::make_unique<gw::RosbagRecordingConsumer>(
                       s.root,
                       std::filesystem::path(GW_KALIBR_TARGET_DEFAULT));
    s.started_at = std::chrono::steady_clock::now();
    s.consumer->attach(*ch);  // may throw on filesystem error — that's the desired surface

    auto [it, _] = sessions_.emplace(camera_id, std::move(s));
    return status_locked(it->second);
}

CalibrationSessionResult CalibrationSupervisor::stop(int64_t camera_id) {
    std::unique_ptr<RosbagRecordingConsumer> consumer;
    Session s;
    {
        std::lock_guard lk(mu_);
        auto it = sessions_.find(camera_id);
        if (it == sessions_.end()) {
            throw CalibrationError("no active session for this camera");
        }
        s        = std::move(it->second);
        consumer = std::move(s.consumer);
        sessions_.erase(it);
    }

    // detach() outside the supervisor mutex — it joins the worker thread,
    // which can take a few frame-periods; holding the lock would needlessly
    // block status() calls.
    if (consumer) consumer->detach();

    const auto now = std::chrono::steady_clock::now();
    CalibrationSessionResult r;
    r.session_id        = s.session_id;
    r.path              = s.root;
    r.frames_written    = consumer ? consumer->frames_written() : 0;
    r.frames_dropped    = consumer ? consumer->frames_dropped() : 0;
    r.elapsed_ms        = std::chrono::duration_cast<std::chrono::milliseconds>(
                              now - s.started_at).count();
    r.suggested_command = build_suggested_command(s.root, s.focal_length_mm);
    return r;
}

std::optional<CalibrationSessionStatus>
CalibrationSupervisor::status(int64_t camera_id) {
    std::lock_guard lk(mu_);
    auto it = sessions_.find(camera_id);
    if (it == sessions_.end()) return std::nullopt;
    return status_locked(it->second);
}

CalibrationSessionStatus
CalibrationSupervisor::status_locked(const Session& s) const {
    CalibrationSessionStatus st;
    st.session_id     = s.session_id;
    st.path           = s.root;
    st.frames_written = s.consumer ? s.consumer->frames_written() : 0;
    st.frames_dropped = s.consumer ? s.consumer->frames_dropped() : 0;
    st.elapsed_ms     = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - s.started_at).count();
    return st;
}

std::string CalibrationSupervisor::build_suggested_command(
    const std::filesystem::path& dataset_root,
    double                       focal_length_mm) const {
    // We wrap the docker invocation in calibrate.sh so the user gets a single
    // command that brings Colima up, runs Kalibr, and brings Colima back down
    // afterward (only if calibrate.sh started it). The script is bash, so the
    // path needs shell quoting only when it contains spaces.
    const std::filesystem::path script = GW_KALIBR_CALIBRATE_SCRIPT;

    // Convert the user-supplied lens focal length (mm) into the inputs Kalibr
    // actually needs:
    //   focal_px := focal_mm / pixel_pitch_mm   — fed to Kalibr's manual-init
    //                                             prompt via GW_KALIBR_FOCAL_HINT
    //                                             (the script propagates it).
    //   HFOV     := 2 · atan((sensor_w_mm / 2) / focal_mm)
    //   model    := HFOV ≥ 95° ? pinhole-equi (Kannala-Brandt fisheye)
    //                          : pinhole-radtan (standard radial+tangential)
    //
    // TODO(multi-camera): sensor_width_px is currently hard-coded to 2048 to
    // match GW_SENSOR_PIXEL_PITCH_MM's implied IMX264. Once we support cameras
    // with different resolutions, plumb the live producer's reported sensor
    // width through Session::sensor_width_px instead.
    const double pitch_mm        = static_cast<double>(GW_SENSOR_PIXEL_PITCH_MM);
    const double focal_px        = focal_length_mm / pitch_mm;
    constexpr double sensor_w_px = 2048.0;
    const double sensor_w_mm     = pitch_mm * sensor_w_px;
    const double hfov_rad        = 2.0 * std::atan((sensor_w_mm / 2.0) / focal_length_mm);
    const double hfov_deg        = hfov_rad * 180.0 / M_PI;
    const std::string model      = (hfov_deg >= 95.0) ? "pinhole-equi"
                                                      : "pinhole-radtan";

    std::ostringstream os;
    os << "GW_KALIBR_FOCAL_HINT=" << std::fixed << std::setprecision(0) << focal_px
       << " " << sh_quote(script)
       << " " << sh_quote(dataset_root)
       << " " << model;
    return os.str();
}

}  // namespace gw::server
