#include "server/vio_supervisor.hpp"

#include <iostream>
#include <utility>

#include "calibration/calibration_store.hpp"
#include "consumer/consumer.hpp"
#include "server/camera_repository.hpp"
#include "server/imu_config_repository.hpp"
#include "server/teensy_manager.hpp"
#include "server/vio_config_repository.hpp"
#include "vio/vio_config_builder.hpp"

namespace gw::server {

namespace {

struct VioCameraRow {
    Camera                              row;
    gw::calib::CamchainEntry            entry;
    std::optional<gw::calib::GuessworkMeta> meta;
};

// Parses a vio camera's stored extrinsics block. Returns the gating-failure
// reason, or nullopt on success (out filled).
std::optional<std::string> load_vio_camera(const Camera& row, VioCameraRow& out) {
    if (!row.imu_extrinsics_json) {
        return "camera '" + row.name + "': no camera-IMU extrinsics";
    }
    try {
        auto chain = gw::calib::parse_camchain(*row.imu_extrinsics_json);
        out.entry  = chain.cameras.front().second;
    } catch (const std::exception& e) {
        return "camera '" + row.name + "': extrinsics unparseable: " + e.what();
    }
    if (!out.entry.imu) {
        return "camera '" + row.name + "': extrinsics missing T_cam_imu";
    }
    try {
        out.meta = gw::calib::parse_guesswork_meta(*row.imu_extrinsics_json);
    } catch (...) {
        // Quality metadata is best-effort.
    }
    out.row = row;
    return std::nullopt;
}

}  // namespace

VioSupervisor::VioSupervisor(CameraRepository&    cameras,
                             ImuConfigRepository& imu_config,
                             VioConfigRepository& vio_config,
                             TeensyManager&       teensy)
    : cameras_(cameras),
      imu_config_(imu_config),
      vio_config_(vio_config),
      teensy_(teensy),
      bus_(std::make_shared<gw::vio::VioBus>()),
      pairer_(std::make_shared<gw::vio::StereoSyncPairer>()),
      epoch_(std::make_shared<std::atomic<uint64_t>>(0)) {}

VioSupervisor::~VioSupervisor() {
    std::unique_ptr<gw::vio::OpenVinsRunner> runner;
    {
        std::lock_guard lk(mu_);
        runner = std::move(runner_);
    }
    runner.reset();  // shuts the pairer + joins outside our lock
}

std::shared_ptr<gw::IConsumer> VioSupervisor::make_consumer(const Camera& row) {
    std::lock_guard lk(mu_);

    gw::vio::StereoSyncPairer::Side side;
    if (row.role && *row.role == "vio_left") {
        side = gw::vio::StereoSyncPairer::kLeft;
    } else if (row.role && *row.role == "vio_right") {
        side = gw::vio::StereoSyncPairer::kRight;
    } else {
        feeders_.erase(row.id);
        ensure_runner_locked();
        return nullptr;
    }

    auto feeder = std::make_shared<gw::vio::VioFeederConsumer>(
        side, pairer_, vio_flip_180(row));
    feeders_[row.id] = feeder;

    // Calibration uploads / role changes re-run the factory via
    // on_camera_updated — re-evaluate the runner here so the rebuild is
    // automatic (fingerprint guard makes the two per-camera invocations
    // cause at most one rebuild).
    ensure_runner_locked();
    return feeder;
}

void VioSupervisor::reload() {
    std::lock_guard lk(mu_);
    ensure_runner_locked();
}

void VioSupervisor::restart() {
    std::lock_guard lk(mu_);
    if (runner_) runner_->request_reinit();
}

void VioSupervisor::ensure_runner_locked() {
    // Gather gating inputs.
    std::optional<Camera> left_row, right_row;
    for (const auto& row : cameras_.list_all()) {
        if (!row.role) continue;
        if (*row.role == "vio_left") left_row = row;
        if (*row.role == "vio_right") right_row = row;
    }

    const auto vio_cfg = vio_config_.get();
    const auto imu_cfg = imu_config_.get();

    std::string reason = "ok";
    std::optional<Fingerprint> fp;
    gw::vio::VioRunnerConfig runner_cfg;
    bool buildable = false;

    do {
        if (!vio_cfg.enabled) {
            reason = "disabled";
            break;
        }
        if (!left_row && !right_row) {
            reason = "no cameras with roles vio_left/vio_right";
            break;
        }
        if (!left_row) {
            reason = "no camera with role vio_left";
            break;
        }
        if (!right_row) {
            reason = "no camera with role vio_right";
            break;
        }

        VioCameraRow left, right;
        if (auto err = load_vio_camera(*left_row, left)) { reason = *err; break; }
        if (auto err = load_vio_camera(*right_row, right)) { reason = *err; break; }

        // Calibration-quality gate: a bad camera-IMU calibration silently
        // destroys VIO — refuse to run on one.
        for (const auto* c : {&left, &right}) {
            if (c->meta && c->meta->reprojection_error_std_px &&
                *c->meta->reprojection_error_std_px > vio_cfg.max_reproj_std_px) {
                reason = "camera '" + c->row.name + "': calibration quality " +
                         std::to_string(*c->meta->reprojection_error_std_px) +
                         " px std exceeds the " +
                         std::to_string(vio_cfg.max_reproj_std_px) + " px gate";
                break;
            }
        }
        if (reason != "ok") break;

        gw::vio::VioImuNoise noise;
        noise.gyro_noise_density  = imu_cfg.gyro_noise_density;
        noise.gyro_random_walk    = imu_cfg.gyro_random_walk;
        noise.accel_noise_density = imu_cfg.accel_noise_density;
        noise.accel_random_walk   = imu_cfg.accel_random_walk;

        gw::vio::VioTuning tuning;
        tuning.num_pts        = static_cast<int>(vio_cfg.num_pts);
        tuning.fast_threshold = static_cast<int>(vio_cfg.fast_threshold);
        tuning.downsample     = vio_cfg.downsample;

        try {
            runner_cfg = gw::vio::build_runner_config(left.entry, right.entry,
                                                      noise, tuning);
        } catch (const std::exception& e) {
            reason = e.what();
            break;
        }

        fp = Fingerprint{
            left_row->id,
            right_row->id,
            left_row->extrinsics_calibrated_at.value_or(0),
            right_row->extrinsics_calibrated_at.value_or(0),
            vio_cfg.updated_at,
            imu_cfg.updated_at,
        };
        buildable = true;
    } while (false);

    reason_ = reason;

    if (!buildable) {
        if (runner_) {
            std::cerr << "VioSupervisor: stopping runner (" << reason << ")\n";
            runner_.reset();
            fingerprint_.reset();
        }
        return;
    }
    if (runner_ && fingerprint_ == fp) return;  // unchanged

    std::cerr << "VioSupervisor: " << (runner_ ? "rebuilding" : "starting")
              << " OpenVINS runner\n";
    runner_.reset();  // joins the old thread first
    pairer_->reset();

    gw::vio::VioReinitPolicy policy;
    policy.auto_reinit   = vio_cfg.auto_reinit;
    policy.min_features  = static_cast<int>(vio_cfg.reinit_min_features);
    policy.window_frames = static_cast<int>(vio_cfg.reinit_window_frames);
    policy.max_pos_std_m = vio_cfg.reinit_max_pos_std_m;

    try {
        runner_ = std::make_unique<gw::vio::OpenVinsRunner>(
            runner_cfg, policy, pairer_, teensy_.imu_bus(), bus_, epoch_);
        fingerprint_ = fp;
    } catch (const std::exception& e) {
        reason_ = std::string("runner construction failed: ") + e.what();
        fingerprint_.reset();
    }
}

VioStatus VioSupervisor::status() {
    VioStatus st;

    std::map<int64_t, std::shared_ptr<gw::vio::VioFeederConsumer>> live;
    {
        std::lock_guard lk(mu_);
        st.reason  = reason_;
        st.running = runner_ != nullptr;
        if (runner_) {
            const auto s        = runner_->snapshot();
            st.initialized      = s.initialized;
            st.phase            = s.phase;
            st.epoch            = s.epoch;
            st.reinits          = s.reinits;
            st.freq_hz          = s.freq_hz;
            st.tracked_features = s.tracked_features;
            st.cov_pos_std_m    = s.cov_pos_std_m;
            st.last             = s.last;
            st.frames_fed       = s.frames_fed;
            st.imu_fed          = s.imu_fed;
            st.imu_bus_dropped  = s.imu_bus_dropped;
        }
        for (auto it = feeders_.begin(); it != feeders_.end();) {
            if (auto f = it->second.lock()) {
                live[it->first] = std::move(f);
                ++it;
            } else {
                it = feeders_.erase(it);
            }
        }
    }
    st.pair_counters = pairer_->counters();

    try {
        st.enabled = vio_config_.get().enabled;
    } catch (...) {}
    st.imu_rate_hz = teensy_.status().imu_rate_hz;

    for (const auto& row : cameras_.list_all()) {
        if (!row.role || (*row.role != "vio_left" && *row.role != "vio_right")) {
            continue;
        }
        VioStatus::CameraEntry e;
        e.camera_id      = row.id;
        e.name           = row.name;
        e.role           = *row.role;
        e.feeder_running = live.count(row.id) > 0;
        if (row.imu_extrinsics_json) {
            try {
                if (const auto meta =
                        gw::calib::parse_guesswork_meta(*row.imu_extrinsics_json)) {
                    e.reproj_std_px = meta->reprojection_error_std_px;
                }
            } catch (...) {}
        }
        st.cameras.push_back(std::move(e));
    }
    return st;
}

}  // namespace gw::server
