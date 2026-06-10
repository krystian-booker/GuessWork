#include "server/calibration_supervisor.hpp"

#include <chrono>
#include <ctime>
#include <fstream>
#include <set>
#include <sstream>
#include <utility>

#include "calibration/calibration_store.hpp"
#include "consumer/multi_topic_bag_recorder.hpp"
#include "consumer/rosbag_recording_consumer.hpp"
#include "core/frame_channel.hpp"
#include "server/camera_repository.hpp"
#include "server/camera_supervisor.hpp"
#include "server/imu_config_repository.hpp"
#include "server/kalibr_imu_job.hpp"
#include "server/kalibr_job.hpp"
#include "server/teensy_manager.hpp"

#ifndef GW_KALIBR_DOCKER_IMAGE
#define GW_KALIBR_DOCKER_IMAGE "guesswork/kalibr:latest"
#endif
#ifndef GW_KALIBR_TARGET_DEFAULT
#define GW_KALIBR_TARGET_DEFAULT ""
#endif
#ifndef GW_KALIBR_CALIBRATE_SCRIPT
#define GW_KALIBR_CALIBRATE_SCRIPT "docker/kalibr/calibrate.sh"
#endif
#ifndef GW_KALIBR_CALIBRATE_IMU_SCRIPT
#define GW_KALIBR_CALIBRATE_IMU_SCRIPT "docker/kalibr/calibrate_imu.sh"
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

// Kalibr's IMU-noise-model guidance: datasheet densities are optimistic for
// a MEMS IMU bolted to a robot — inflate by 10× (or more) so the optimizer
// doesn't over-trust the IMU.
constexpr double kImuNoiseInflation = 10.0;

// Width to assume when the live mode didn't report one (shouldn't happen for
// an online camera; matches GW_SENSOR_PIXEL_PITCH_MM's implied IMX264).
constexpr uint32_t kFallbackSensorWidthPx = 2048;

}  // namespace

CalibrationSupervisor::CalibrationSupervisor(CameraSupervisor&     cameras,
                                             CameraRepository&     repository,
                                             TeensyManager&        teensy,
                                             ImuConfigRepository&  imu_config,
                                             std::filesystem::path root)
    : cameras_(cameras),
      repository_(repository),
      teensy_(teensy),
      imu_config_(imu_config),
      root_(std::move(root)) {
    std::error_code ec;
    std::filesystem::create_directories(root_, ec);
    // Don't throw on construction — start() will surface a clearer error if
    // the path is unwritable.
}

CalibrationSupervisor::~CalibrationSupervisor() {
    // Cancel any active Kalibr job and let the SubprocessJob reader thread
    // unwind before we release the supervisor. The shared_ptrs keep the jobs
    // alive for any in-flight SSE handler too — they'll see Cancelled state
    // when their wait_for_log returns.
    std::shared_ptr<KalibrJob>    job_to_cancel;
    std::shared_ptr<KalibrImuJob> imu_job_to_cancel;
    std::unique_ptr<gw::MultiTopicBagRecorder> recorder_to_stop;
    {
        std::lock_guard lk(mu_);
        job_to_cancel     = kalibr_job_;
        imu_job_to_cancel = imu_job_;
        for (auto& [id, s] : sessions_) {
            if (s.consumer) s.consumer->detach();
        }
        if (ext_session_ && ext_session_->recorder) {
            recorder_to_stop = std::move(ext_session_->recorder);
        }
    }
    if (recorder_to_stop) recorder_to_stop->stop();
    if (job_to_cancel) job_to_cancel->cancel();
    if (imu_job_to_cancel) imu_job_to_cancel->cancel();
}

uint32_t CalibrationSupervisor::live_sensor_width(int64_t camera_id) {
    const auto mode = cameras_.current_mode_for(camera_id);
    if (mode && mode->width && *mode->width > 0) {
        return static_cast<uint32_t>(*mode->width);
    }
    return kFallbackSensorWidthPx;
}

CalibrationSessionStatus CalibrationSupervisor::start(int64_t camera_id) {
    std::lock_guard lk(mu_);
    if (sessions_.count(camera_id)) {
        throw CalibrationError("session already active for this camera");
    }
    if (ext_session_) {
        for (const auto& p : ext_session_->cams) {
            if (p.camera_id == camera_id) {
                throw CalibrationError(
                    "camera is part of the active extrinsics session");
            }
        }
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
    s.sensor_width_px = live_sensor_width(camera_id);
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
    r.sensor_width_px   = s.sensor_width_px;
    r.suggested_command =
        build_suggested_command(s.root, s.focal_length_mm, s.sensor_width_px);
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

void CalibrationSupervisor::reap_or_throw_if_job_running_locked() {
    // Reap any prior job that's already finished — keeps the supervisor's
    // slots free for the new run without forcing the SSE-or-status caller to
    // GC explicitly. The shared_ptrs released here are cheap to destroy
    // (their reader threads already exited); in-flight SSE handlers keep
    // their own copies alive.
    if (kalibr_job_) {
        const auto s = kalibr_job_->status().state;
        if (s == SubprocessState::Running || s == SubprocessState::Pending) {
            throw CalibrationError("another Kalibr job is already running");
        }
        kalibr_job_.reset();
        kalibr_job_camera_ = 0;
    }
    if (imu_job_) {
        const auto s = imu_job_->status().state;
        if (s == SubprocessState::Running || s == SubprocessState::Pending) {
            throw CalibrationError("a Kalibr extrinsics job is already running");
        }
        imu_job_.reset();
    }
}

CalibrationJobStatus CalibrationSupervisor::start_kalibr_job(
    int64_t                      camera_id,
    const std::filesystem::path& session_root,
    double                       focal_length_mm,
    uint32_t                     sensor_width_px) {
    std::shared_ptr<KalibrJob> job;
    {
        std::lock_guard lk(mu_);
        reap_or_throw_if_job_running_locked();

        const std::filesystem::path script = GW_KALIBR_CALIBRATE_SCRIPT;
        const double                pitch  = static_cast<double>(GW_SENSOR_PIXEL_PITCH_MM);
        job = std::make_shared<KalibrJob>(
            repository_, camera_id, session_root, focal_length_mm,
            script, pitch, sensor_width_px);  // ctor calls derive_kalibr_lens internally
        // Rebuild calibration-dependent consumers (e.g. AprilTag) once the
        // result lands. Fires on the job's reader thread with no locks held.
        job->set_on_stored([this](int64_t id) { cameras_.on_camera_updated(id); });
        job->start();  // throws on fork/pipe failure; lock released on unwind

        kalibr_job_        = job;
        kalibr_job_camera_ = camera_id;
    }
    return job->status();
}

std::optional<CalibrationJobStatus>
CalibrationSupervisor::kalibr_job_status(int64_t camera_id) {
    std::shared_ptr<KalibrJob> job;
    {
        std::lock_guard lk(mu_);
        if (!kalibr_job_ || kalibr_job_camera_ != camera_id) return std::nullopt;
        job = kalibr_job_;
    }
    return job->status();
}

std::shared_ptr<KalibrJob>
CalibrationSupervisor::kalibr_job_handle(int64_t camera_id) {
    std::lock_guard lk(mu_);
    if (!kalibr_job_ || kalibr_job_camera_ != camera_id) return nullptr;
    return kalibr_job_;
}

bool CalibrationSupervisor::kalibr_job_cancel(int64_t camera_id) {
    std::shared_ptr<KalibrJob> job;
    {
        std::lock_guard lk(mu_);
        if (!kalibr_job_ || kalibr_job_camera_ != camera_id) return false;
        if (kalibr_job_->status().state != SubprocessState::Running) return false;
        job = kalibr_job_;
    }
    job->cancel();
    return true;
}

std::string CalibrationSupervisor::build_suggested_command(
    const std::filesystem::path& dataset_root,
    double                       focal_length_mm,
    uint32_t                     sensor_width_px) const {
    // Single command that brings Colima up, runs Kalibr, and brings Colima
    // back down afterward (only if calibrate.sh started it).
    const std::filesystem::path script = GW_KALIBR_CALIBRATE_SCRIPT;
    const auto lens = derive_kalibr_lens(
        focal_length_mm,
        static_cast<double>(GW_SENSOR_PIXEL_PITCH_MM),
        sensor_width_px);

    std::ostringstream os;
    os << "GW_KALIBR_FOCAL_HINT=" << lens.focal_hint_px
       << " " << sh_quote(script)
       << " " << sh_quote(dataset_root)
       << " " << lens.model;
    return os.str();
}

// ---------------------------------------------------------------------------
// Extrinsics (camera-IMU) sessions
// ---------------------------------------------------------------------------

namespace {

std::string make_topic(size_t cam_index) {
    return "/cam" + std::to_string(cam_index) + "/image_raw";
}

std::string make_frame_id(size_t cam_index) {
    return "cam" + std::to_string(cam_index);
}

// Minimal JSON string escaping for the manifest (names are user-controlled).
std::string json_escape(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n";  break;
            case '\t': out += "\\t";  break;
            default:   out.push_back(c);
        }
    }
    return out;
}

void write_text_file(const std::filesystem::path& path, const std::string& text) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out.is_open()) {
        throw CalibrationError("cannot write " + path.string());
    }
    out << text;
}

}  // namespace

ExtrinsicsSessionStatus
CalibrationSupervisor::start_extrinsics(const std::vector<int64_t>& camera_ids) {
    std::lock_guard lk(mu_);

    // --- Validation: everything before any side effect. ---
    if (camera_ids.empty() || camera_ids.size() > 2) {
        throw CalibrationError("extrinsics sessions take 1 or 2 cameras");
    }
    if (std::set<int64_t>(camera_ids.begin(), camera_ids.end()).size() !=
        camera_ids.size()) {
        throw CalibrationError("duplicate camera ids");
    }
    if (ext_session_) {
        throw CalibrationError("an extrinsics session is already active");
    }

    const auto teensy = teensy_.status();
    if (!teensy.connected)           throw CalibrationError("Teensy not connected");
    if (!teensy.armed)               throw CalibrationError("Teensy not armed — arm hardware sync first");
    if (!teensy.telemetry_connected) throw CalibrationError("Teensy telemetry interface not connected (fw=2 required)");
    if (!teensy.imu_ok)              throw CalibrationError("IMU not healthy (heartbeat reports imu_ok=0)");

    std::vector<ExtrinsicsParticipant> cams;
    std::vector<gw::CameraInputSpec>   specs;
    std::vector<uint32_t>              hints;
    std::string                        model;
    std::optional<std::string>         single_cam_camchain;  // generated below

    for (size_t i = 0; i < camera_ids.size(); ++i) {
        const int64_t id = camera_ids[i];
        if (sessions_.count(id)) {
            throw CalibrationError("camera " + std::to_string(id) +
                                   " has an active intrinsics session");
        }
        const auto row = repository_.get(id);
        if (!row) throw CalibrationError("camera " + std::to_string(id) + " not found");
        if (!row->hardware_sync_enabled) {
            throw CalibrationError(
                "camera " + std::to_string(id) +
                " is not hardware-synced — extrinsics bags need Teensy-clock stamps");
        }
        gw::FrameChannel* ch = cameras_.frame_channel_for(id);
        if (!ch) throw CalibrationError("camera " + std::to_string(id) + " is offline");

        ExtrinsicsParticipant p;
        p.camera_id       = id;
        p.topic           = make_topic(i);
        p.frame_id        = make_frame_id(i);
        p.focal_length_mm = row->focal_length_mm;
        p.sensor_width_px = live_sensor_width(id);

        const auto lens = derive_kalibr_lens(
            p.focal_length_mm, static_cast<double>(GW_SENSOR_PIXEL_PITCH_MM),
            p.sensor_width_px);
        if (model.empty()) {
            model = lens.model;
        } else if (model != lens.model) {
            // Kalibr takes one --models per topic but a VIO pair mixing
            // radtan/equi lenses is operator error — reject loudly.
            throw CalibrationError("cameras derive different Kalibr models (" +
                                   model + " vs " + lens.model + ")");
        }
        hints.push_back(lens.focal_hint_px);

        // Single-camera flow: Kalibr needs a camchain with this camera's
        // stored intrinsics. Validate it parses and matches the live mode.
        if (camera_ids.size() == 1) {
            if (!row->calibration_json) {
                throw CalibrationError(
                    "camera has no stored intrinsics — run intrinsics "
                    "calibration first");
            }
            gw::calib::Camchain chain;
            try {
                chain = gw::calib::parse_camchain(*row->calibration_json);
            } catch (const std::exception& e) {
                throw CalibrationError(
                    std::string("stored intrinsics do not parse: ") + e.what());
            }
            auto entry = chain.cameras.front().second;
            const auto mode = cameras_.current_mode_for(id);
            if (mode && mode->width && mode->height &&
                (entry.intrinsics.resolution[0] != static_cast<uint32_t>(*mode->width) ||
                 entry.intrinsics.resolution[1] != static_cast<uint32_t>(*mode->height))) {
                throw CalibrationError(
                    "stored intrinsics resolution (" +
                    std::to_string(entry.intrinsics.resolution[0]) + "x" +
                    std::to_string(entry.intrinsics.resolution[1]) +
                    ") does not match the camera's current mode — recalibrate "
                    "intrinsics first");
            }
            entry.intrinsics.rostopic = make_topic(0);
            entry.imu.reset();  // plain camchain input — no stale extrinsics
            single_cam_camchain = gw::calib::serialize_single_camera(entry);
        }

        cams.push_back(std::move(p));
        specs.push_back(gw::CameraInputSpec{ch, cams.back().topic, cams.back().frame_id});
    }

    // --- Side effects. ---
    ExtrinsicsSession s;
    s.session_id     = make_session_id();
    s.root           = root_ / s.session_id;
    s.cams           = std::move(cams);
    s.model          = model;
    s.focal_hints_px = std::move(hints);
    s.started_at     = std::chrono::steady_clock::now();

    std::error_code ec;
    std::filesystem::create_directories(s.root, ec);
    if (ec) throw CalibrationError("create_directories failed: " + ec.message());

    // manifest.json — records the camera_id → topic mapping the job uses to
    // route per-camera results back to rows.
    {
        std::ostringstream m;
        m << "{\n  \"session_id\": \"" << json_escape(s.session_id) << "\",\n"
          << "  \"type\": \"extrinsics\",\n"
          << "  \"model\": \"" << json_escape(s.model) << "\",\n"
          << "  \"imu_topic\": \"/imu0\",\n"
          << "  \"cameras\": [\n";
        for (size_t i = 0; i < s.cams.size(); ++i) {
            m << "    {\"camera_id\": " << s.cams[i].camera_id
              << ", \"topic\": \"" << json_escape(s.cams[i].topic) << "\"}"
              << (i + 1 < s.cams.size() ? ",\n" : "\n");
        }
        m << "  ]\n}\n";
        write_text_file(s.root / "manifest.json", m.str());
    }

    // imu.yaml — Kalibr's IMU noise model from the configured (datasheet)
    // values with the customary safety inflation.
    {
        const auto cfg = imu_config_.get();
        std::ostringstream y;
        y << "rostopic: /imu0\n"
          << "update_rate: " << cfg.rate_hz << "\n"
          << "accelerometer_noise_density: " << cfg.accel_noise_density * kImuNoiseInflation << "\n"
          << "accelerometer_random_walk: "   << cfg.accel_random_walk   * kImuNoiseInflation << "\n"
          << "gyroscope_noise_density: "     << cfg.gyro_noise_density  * kImuNoiseInflation << "\n"
          << "gyroscope_random_walk: "       << cfg.gyro_random_walk    * kImuNoiseInflation << "\n";
        write_text_file(s.root / "imu.yaml", y.str());
    }

    if (single_cam_camchain) {
        write_text_file(s.root / "camchain.yaml", *single_cam_camchain);
    }

    s.recorder = std::make_unique<gw::MultiTopicBagRecorder>(
        s.root, std::filesystem::path(GW_KALIBR_TARGET_DEFAULT), std::move(specs),
        &teensy_.imu_bus());
    s.recorder->start();  // may throw on filesystem error

    ext_session_ = std::move(s);
    return extrinsics_status_locked(*ext_session_);
}

ExtrinsicsSessionResult CalibrationSupervisor::stop_extrinsics() {
    ExtrinsicsSession s;
    {
        std::lock_guard lk(mu_);
        if (!ext_session_) {
            throw CalibrationError("no active extrinsics session");
        }
        s = std::move(*ext_session_);
        ext_session_.reset();
    }

    // stop() outside the supervisor mutex — it joins worker threads.
    if (s.recorder) s.recorder->stop();

    const auto now = std::chrono::steady_clock::now();
    ExtrinsicsSessionResult r;
    r.session_id     = s.session_id;
    r.path           = s.root;
    r.model          = s.model;
    r.focal_hints_px = s.focal_hints_px;
    r.elapsed_ms     = std::chrono::duration_cast<std::chrono::milliseconds>(
                           now - s.started_at).count();
    for (size_t i = 0; i < s.cams.size(); ++i) {
        const auto stats = s.recorder ? s.recorder->camera_stats(i)
                                      : gw::MultiTopicBagRecorder::CameraStats{};
        r.cameras.push_back({s.cams[i].camera_id, s.cams[i].topic,
                             stats.written, stats.dropped});
    }
    r.imu_written = s.recorder ? s.recorder->imu_written() : 0;
    r.imu_dropped = s.recorder ? s.recorder->imu_dropped() : 0;

    {
        std::ostringstream os;
        os << "GW_KALIBR_FOCAL_HINT='";
        for (size_t i = 0; i < r.focal_hints_px.size(); ++i) {
            if (i) os << ' ';
            os << r.focal_hints_px[i];
        }
        os << "' " << sh_quote(std::filesystem::path(GW_KALIBR_CALIBRATE_IMU_SCRIPT))
           << " " << sh_quote(r.path)
           << " " << r.cameras.size()
           << " " << r.model;
        r.suggested_command = os.str();
    }
    return r;
}

std::optional<ExtrinsicsSessionStatus> CalibrationSupervisor::extrinsics_status() {
    std::lock_guard lk(mu_);
    if (!ext_session_) return std::nullopt;
    return extrinsics_status_locked(*ext_session_);
}

ExtrinsicsSessionStatus
CalibrationSupervisor::extrinsics_status_locked(const ExtrinsicsSession& s) const {
    ExtrinsicsSessionStatus st;
    st.session_id = s.session_id;
    st.path       = s.root;
    st.elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - s.started_at).count();
    for (size_t i = 0; i < s.cams.size(); ++i) {
        const auto stats = s.recorder ? s.recorder->camera_stats(i)
                                      : gw::MultiTopicBagRecorder::CameraStats{};
        st.cameras.push_back({s.cams[i].camera_id, s.cams[i].topic,
                              stats.written, stats.dropped});
    }
    st.imu_written = s.recorder ? s.recorder->imu_written() : 0;
    st.imu_dropped = s.recorder ? s.recorder->imu_dropped() : 0;
    return st;
}

CalibrationJobStatus
CalibrationSupervisor::start_imu_job(const ExtrinsicsSessionResult& result) {
    std::shared_ptr<KalibrImuJob> job;
    {
        std::lock_guard lk(mu_);
        reap_or_throw_if_job_running_locked();

        std::vector<int64_t> camera_ids;
        camera_ids.reserve(result.cameras.size());
        for (const auto& c : result.cameras) camera_ids.push_back(c.camera_id);

        job = std::make_shared<KalibrImuJob>(
            repository_, std::move(camera_ids), result.path,
            std::filesystem::path(GW_KALIBR_CALIBRATE_IMU_SCRIPT),
            result.model, result.focal_hints_px, result.session_id);
        job->set_on_stored([this](int64_t id) { cameras_.on_camera_updated(id); });
        job->start();  // throws on fork/pipe failure; lock released on unwind

        imu_job_ = job;
    }
    return job->status();
}

std::optional<CalibrationJobStatus> CalibrationSupervisor::imu_job_status() {
    std::shared_ptr<KalibrImuJob> job;
    {
        std::lock_guard lk(mu_);
        if (!imu_job_) return std::nullopt;
        job = imu_job_;
    }
    return job->status();
}

std::shared_ptr<KalibrImuJob> CalibrationSupervisor::imu_job_handle() {
    std::lock_guard lk(mu_);
    return imu_job_;
}

bool CalibrationSupervisor::imu_job_cancel() {
    std::shared_ptr<KalibrImuJob> job;
    {
        std::lock_guard lk(mu_);
        if (!imu_job_) return false;
        if (imu_job_->status().state != SubprocessState::Running) return false;
        job = imu_job_;
    }
    job->cancel();
    return true;
}

}  // namespace gw::server
