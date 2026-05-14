#include "server/calibration_supervisor.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <utility>

#include "consumer/recording_consumer.hpp"
#include "core/frame_channel.hpp"
#include "server/camera_supervisor.hpp"

#ifndef GW_BASALT_CALIBRATE_BIN
#define GW_BASALT_CALIBRATE_BIN ""
#endif
#ifndef GW_BASALT_APRILGRID_DEFAULT
#define GW_BASALT_APRILGRID_DEFAULT ""
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
                                             std::filesystem::path root)
    : cameras_(cameras), root_(std::move(root)) {
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
    gw::FrameChannel* ch = cameras_.frame_channel_for(camera_id);
    if (!ch) {
        throw CalibrationError("camera is offline");
    }

    Session s;
    s.session_id = make_session_id();
    s.root       = root_ / s.session_id;
    s.consumer   = std::make_unique<gw::RecordingConsumer>(s.root, "calibration");
    s.started_at = std::chrono::steady_clock::now();
    s.consumer->attach(*ch);  // may throw on filesystem error — that's the desired surface

    auto [it, _] = sessions_.emplace(camera_id, std::move(s));
    return status_locked(it->second);
}

CalibrationSessionResult CalibrationSupervisor::stop(int64_t camera_id) {
    std::unique_ptr<RecordingConsumer> consumer;
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
    r.suggested_command = build_suggested_command(s.root);
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
    const std::filesystem::path& dataset_root) const {
    const std::string bin       = GW_BASALT_CALIBRATE_BIN;
    const std::string aprilgrid = GW_BASALT_APRILGRID_DEFAULT;

    const std::string bin_part       = bin.empty()       ? "basalt_calibrate"
                                                         : sh_quote(bin);
    const std::string aprilgrid_part = aprilgrid.empty() ? "<path-to-aprilgrid.json>"
                                                         : sh_quote(aprilgrid);
    const std::string dataset_part   = sh_quote(dataset_root);
    const std::string result_part    = sh_quote(dataset_root / "result");

    std::ostringstream os;
    os << bin_part
       << " --dataset-path "  << dataset_part
       << " --dataset-type euroc"
       << " --aprilgrid "     << aprilgrid_part
       << " --result-path "   << result_part
       << " --cam-types ds";
    return os.str();
}

}  // namespace gw::server
