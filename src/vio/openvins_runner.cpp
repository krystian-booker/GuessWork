#include "vio/openvins_runner.hpp"

#include <cam/CamEqui.h>
#include <cam/CamRadtan.h>
#include <core/VioManager.h>
#include <core/VioManagerOptions.h>
#include <state/State.h>
#include <state/StateHelper.h>
#include <utils/print.h>
#include <utils/quat_ops.h>
#include <utils/sensor_data.h>

#include <opencv2/core.hpp>

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

namespace gw::vio {

namespace {

constexpr double  kNsToS = 1e-9;
// IMU coverage margin past the camera stamp before feeding the pair.
constexpr int64_t kImuLeadNs = 50'000'000;  // 50 ms
// An inter-pair gap this large means a camera went away (USB unplug) or
// the trigger stopped — propagating across it would be garbage; reinit.
constexpr int64_t kMaxPairGapNs = 2'000'000'000;  // 2 s

// Maps our plain VioRunnerConfig onto ov_msckf::VioManagerOptions.
// IMPORTANT: upstream's YAML loader (not VioManager) halves intrinsics +
// resolution when downsampling — since we bypass YAML, we do it here.
ov_msckf::VioManagerOptions to_vio_manager_options(const VioRunnerConfig& cfg) {
    ov_msckf::VioManagerOptions opt;

    opt.state_options.num_cameras                  = 2;
    opt.state_options.do_fej                       = true;
    opt.state_options.do_calib_camera_pose         = false;
    opt.state_options.do_calib_camera_intrinsics   = false;
    opt.state_options.do_calib_camera_timeoffset   = false;
    opt.state_options.max_clone_size               = cfg.max_clone_size;

    opt.use_stereo  = true;
    opt.use_klt     = true;
    opt.use_aruco   = false;  // built with ENABLE_ARUCO_TAGS=OFF — required
    opt.num_pts     = cfg.num_pts;
    opt.fast_threshold = cfg.fast_threshold;
    opt.downsample_cameras  = cfg.downsample;
    opt.num_opencv_threads  = cfg.num_opencv_threads;
    opt.calib_camimu_dt     = cfg.calib_camimu_dt;
    opt.gravity_mag         = cfg.gravity_mag;
    opt.try_zupt            = false;  // ZUPT tuning deferred to the rig

    opt.imu_noises.sigma_w    = cfg.sigma_w;
    opt.imu_noises.sigma_w_2  = cfg.sigma_w * cfg.sigma_w;
    opt.imu_noises.sigma_wb   = cfg.sigma_wb;
    opt.imu_noises.sigma_wb_2 = cfg.sigma_wb * cfg.sigma_wb;
    opt.imu_noises.sigma_a    = cfg.sigma_a;
    opt.imu_noises.sigma_a_2  = cfg.sigma_a * cfg.sigma_a;
    opt.imu_noises.sigma_ab   = cfg.sigma_ab;
    opt.imu_noises.sigma_ab_2 = cfg.sigma_ab * cfg.sigma_ab;

    opt.msckf_options.sigma_pix = cfg.sigma_pix;

    const VioCameraConfig* cams[2] = {&cfg.left, &cfg.right};
    for (size_t i = 0; i < 2; ++i) {
        const auto& c = *cams[i];
        double fx = c.fxfycxcy[0], fy = c.fxfycxcy[1];
        double cx = c.fxfycxcy[2], cy = c.fxfycxcy[3];
        int    w  = static_cast<int>(c.wh[0]);
        int    h  = static_cast<int>(c.wh[1]);
        if (cfg.downsample) {
            fx /= 2.0; fy /= 2.0; cx /= 2.0; cy /= 2.0;
            w /= 2;    h /= 2;
        }
        Eigen::MatrixXd intr(8, 1);
        intr << fx, fy, cx, cy, c.dist[0], c.dist[1], c.dist[2], c.dist[3];
        std::shared_ptr<ov_core::CamBase> cam;
        if (c.equidistant) {
            cam = std::make_shared<ov_core::CamEqui>(w, h);
        } else {
            cam = std::make_shared<ov_core::CamRadtan>(w, h);
        }
        cam->set_value(intr);
        opt.camera_intrinsics[i] = cam;

        // Kalibr T_cam_imu is T_ItoC; OpenVINS wants q_ItoC (JPL) + p_IinC —
        // exactly the rotation/translation of T_cam_imu, NO inversion. Use
        // upstream's own rot_2_quat for the JPL convention.
        Eigen::Matrix3d R_ItoC;
        for (int r = 0; r < 3; ++r)
            for (int col = 0; col < 3; ++col) R_ItoC(r, col) = c.T_cam_imu[r][col];
        Eigen::VectorXd ext(7);
        ext.head<4>() = ov_core::rot_2_quat(R_ItoC);
        ext.tail<3>() << c.T_cam_imu[0][3], c.T_cam_imu[1][3], c.T_cam_imu[2][3];
        opt.camera_extrinsics[i] = ext;

        // The initializer carries its own copies of the camera maps.
        opt.init_options.camera_intrinsics[i] = cam;
        opt.init_options.camera_extrinsics[i] = ext;
    }

    opt.init_options.num_cameras        = 2;
    opt.init_options.use_stereo         = true;
    opt.init_options.downsample_cameras = cfg.downsample;
    opt.init_options.sigma_w            = cfg.sigma_w;
    opt.init_options.sigma_wb           = cfg.sigma_wb;
    opt.init_options.sigma_a            = cfg.sigma_a;
    opt.init_options.sigma_ab           = cfg.sigma_ab;
    opt.init_options.sigma_pix          = cfg.sigma_pix;
    opt.init_options.gravity_mag        = cfg.gravity_mag;
    opt.init_options.calib_camimu_dt    = cfg.calib_camimu_dt;
    opt.init_options.init_window_time   = cfg.init_window_time;
    opt.init_options.init_imu_thresh    = cfg.init_imu_thresh;
    opt.init_options.init_dyn_use       = false;

    return opt;
}

}  // namespace

struct OpenVinsRunner::Impl {
    VioRunnerConfig cfg;
    VioReinitPolicy policy;

    std::shared_ptr<StereoSyncPairer>      pairer;
    gw::MeasurementBus<gw::ImuSample>&     imu_bus;
    gw::MeasurementBus<gw::ImuSample>::SubscriberHandle imu_sub;
    std::shared_ptr<VioBus>                out_bus;
    std::shared_ptr<std::atomic<uint64_t>> epoch;

    std::unique_ptr<ov_msckf::VioManager> vio;
    std::thread                           thread;
    std::atomic<bool>                     reinit_requested{false};

    // Runner-thread-only state.
    int64_t last_imu_ns       = 0;
    int64_t last_pair_ns      = 0;
    int     low_feature_count = 0;

    // Stats (snapshot reads under stats_mu / atomics).
    std::atomic<uint64_t> frames_fed{0};
    std::atomic<uint64_t> imu_fed{0};
    std::atomic<uint64_t> reinits{0};
    mutable std::mutex          stats_mu;
    std::optional<VioOdometry>  last_out;
    double                      cov_pos_std_m = 0.0;
    double                      freq_hz       = 0.0;
    uint64_t                    rate_count    = 0;
    std::chrono::steady_clock::time_point rate_t0{};
    std::atomic<bool>           initialized{false};

    Impl(VioRunnerConfig c, VioReinitPolicy p, std::shared_ptr<StereoSyncPairer> pr,
         gw::MeasurementBus<gw::ImuSample>& bus, std::shared_ptr<VioBus> out,
         std::shared_ptr<std::atomic<uint64_t>> ep)
        : cfg(std::move(c)),
          policy(p),
          pairer(std::move(pr)),
          imu_bus(bus),
          out_bus(std::move(out)),
          epoch(std::move(ep)) {}

    void build_vio() {
        auto opt = to_vio_manager_options(cfg);
        vio      = std::make_unique<ov_msckf::VioManager>(opt);
        last_imu_ns       = 0;
        last_pair_ns      = 0;
        low_feature_count = 0;
        initialized.store(false, std::memory_order_relaxed);
        epoch->fetch_add(1, std::memory_order_relaxed);
    }

    void run();
    void feed_imu_until(int64_t t_ns);
    void process_pair(const StereoPair& pair);
};

void OpenVinsRunner::Impl::feed_imu_until(int64_t t_ns) {
    const int64_t target = t_ns + kImuLeadNs;
    gw::ImuSample s;
    // Opportunistic drain first, then block until coverage.
    while (last_imu_ns < target) {
        if (!imu_bus.try_pop(imu_sub, s)) {
            if (!imu_bus.wait_pop(imu_sub, s)) return;  // unsubscribed
        }
        ov_core::ImuData d;
        d.timestamp = static_cast<double>(s.t_ns) * kNsToS;
        d.wm << s.gyro[0], s.gyro[1], s.gyro[2];
        d.am << s.accel[0], s.accel[1], s.accel[2];
        vio->feed_measurement_imu(d);
        last_imu_ns = static_cast<int64_t>(s.t_ns);
        imu_fed.fetch_add(1, std::memory_order_relaxed);
    }
}

void OpenVinsRunner::Impl::process_pair(const StereoPair& pair) {
    // Gap = camera went away and came back; the filter state is stale.
    if (last_pair_ns != 0 && pair.t_ns - last_pair_ns > kMaxPairGapNs) {
        std::cerr << "OpenVinsRunner: " << (pair.t_ns - last_pair_ns) / 1'000'000
                  << " ms frame gap — reinitializing\n";
        build_vio();
        reinits.fetch_add(1, std::memory_order_relaxed);
    }
    last_pair_ns = pair.t_ns;

    feed_imu_until(pair.t_ns);

    const uint32_t w = cfg.left.wh[0], h = cfg.left.wh[1];
    if (pair.left.width != w || pair.left.height != h ||
        pair.right.width != cfg.right.wh[0] || pair.right.height != cfg.right.wh[1]) {
        return;  // resolution mismatch vs calibration — skip silently (rare)
    }

    ov_core::CameraData cam;
    cam.timestamp  = static_cast<double>(pair.t_ns) * kNsToS;
    cam.sensor_ids = {0, 1};
    // Zero-copy wraps; feed_measurement_camera is synchronous and the pair
    // outlives the call. (OpenVINS clones internally during histogram
    // equalization; switch to .clone() here if sanitizers ever object.)
    cam.images.emplace_back(static_cast<int>(pair.left.height),
                            static_cast<int>(pair.left.width), CV_8UC1,
                            const_cast<uint8_t*>(pair.left.pixels.data()));
    cam.images.emplace_back(static_cast<int>(pair.right.height),
                            static_cast<int>(pair.right.width), CV_8UC1,
                            const_cast<uint8_t*>(pair.right.pixels.data()));
    cam.masks.push_back(cv::Mat::zeros(static_cast<int>(pair.left.height),
                                       static_cast<int>(pair.left.width), CV_8UC1));
    cam.masks.push_back(cv::Mat::zeros(static_cast<int>(pair.right.height),
                                       static_cast<int>(pair.right.width), CV_8UC1));

    vio->feed_measurement_camera(cam);
    frames_fed.fetch_add(1, std::memory_order_relaxed);

    const bool init = vio->initialized();
    initialized.store(init, std::memory_order_relaxed);
    if (!init) return;

    const auto state = vio->get_state();

    // Pose: state->_imu gives R_GtoI and p_IinG; publish T_odom_imu.
    const Eigen::Matrix3d R_GtoI = state->_imu->Rot();
    const Eigen::Vector3d p_IinG = state->_imu->pos();
    gw::apriltag::Mat4 T = gw::apriltag::mat4_identity();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) T[r][c] = R_GtoI(c, r);  // R_ItoG = Rᵀ
        T[r][3] = p_IinG(r);
    }

    // Marginal covariance, order [δθ, δp] (left-JPL error) → body tangent.
    std::vector<std::shared_ptr<ov_type::Type>> vars{state->_imu->pose()};
    const Eigen::MatrixXd P =
        ov_msckf::StateHelper::get_marginal_covariance(state, vars);
    std::array<double, 36> P_arr{};
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c) P_arr[r * 6 + c] = P(r, c);
    gw::apriltag::Mat3 R_GtoI_arr{};
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) R_GtoI_arr[r][c] = R_GtoI(r, c);
    const auto cov = cov_ov_to_body_tangent(R_GtoI_arr, P_arr);

    VioOdometry odo;
    odo.t_ns             = pair.t_ns;
    odo.epoch            = epoch->load(std::memory_order_relaxed);
    odo.initialized      = true;
    odo.T_odom_imu       = T;
    odo.cov              = cov;
    odo.tracked_features =
        static_cast<uint32_t>(vio->get_good_features_MSCKF().size());
    out_bus->publish(odo);

    const double pos_std =
        std::sqrt(std::max({cov[3 * 6 + 3], cov[4 * 6 + 4], cov[5 * 6 + 5]}));

    {
        std::lock_guard lk(stats_mu);
        last_out      = odo;
        cov_pos_std_m = pos_std;
        ++rate_count;
        const auto now = std::chrono::steady_clock::now();
        const auto dt  = now - rate_t0;
        if (dt >= std::chrono::seconds(1)) {
            freq_hz = static_cast<double>(rate_count) /
                      std::chrono::duration<double>(dt).count();
            rate_count = 0;
            rate_t0    = now;
        }
    }

    // Divergence checks → auto-reinit.
    if (policy.auto_reinit) {
        bool diverged = false;
        if (odo.tracked_features < static_cast<uint32_t>(policy.min_features)) {
            if (++low_feature_count >= policy.window_frames) {
                std::cerr << "OpenVinsRunner: tracked features collapsed ("
                          << odo.tracked_features << ") — reinitializing\n";
                diverged = true;
            }
        } else {
            low_feature_count = 0;
        }
        if (pos_std > policy.max_pos_std_m) {
            std::cerr << "OpenVinsRunner: position covariance exploded ("
                      << pos_std << " m) — reinitializing\n";
            diverged = true;
        }
        if (diverged) {
            build_vio();
            reinits.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

void OpenVinsRunner::Impl::run() {
    StereoPair pair;
    while (pairer->wait_pop(pair)) {
        if (reinit_requested.exchange(false)) {
            build_vio();
            reinits.fetch_add(1, std::memory_order_relaxed);
        }
        process_pair(pair);
    }
}

OpenVinsRunner::OpenVinsRunner(VioRunnerConfig                    cfg,
                               VioReinitPolicy                    policy,
                               std::shared_ptr<StereoSyncPairer>  pairer,
                               gw::MeasurementBus<gw::ImuSample>& imu_bus,
                               std::shared_ptr<VioBus>            out_bus,
                               std::shared_ptr<std::atomic<uint64_t>> epoch)
    : impl_(std::make_unique<Impl>(std::move(cfg), policy, std::move(pairer),
                                   imu_bus, std::move(out_bus), std::move(epoch))) {
    ov_core::Printer::setPrintLevel(ov_core::Printer::PrintLevel::WARNING);
    impl_->imu_sub = impl_->imu_bus.subscribe(/*capacity=*/4096);  // ~10 s @400 Hz
    impl_->rate_t0 = std::chrono::steady_clock::now();
    impl_->build_vio();
    impl_->thread = std::thread([this] { impl_->run(); });
}

OpenVinsRunner::~OpenVinsRunner() {
    impl_->pairer->shutdown();                 // wakes wait_pop → loop exits
    impl_->imu_bus.unsubscribe(impl_->imu_sub);  // wakes a blocked IMU wait
    if (impl_->thread.joinable()) impl_->thread.join();
}

void OpenVinsRunner::request_reinit() {
    impl_->reinit_requested.store(true, std::memory_order_relaxed);
}

OpenVinsRunner::Snapshot OpenVinsRunner::snapshot() const {
    Snapshot s;
    s.initialized = impl_->initialized.load(std::memory_order_relaxed);
    s.phase       = s.initialized ? "tracking" : "initializing";
    s.epoch       = impl_->epoch->load(std::memory_order_relaxed);
    s.reinits     = impl_->reinits.load(std::memory_order_relaxed);
    s.frames_fed  = impl_->frames_fed.load(std::memory_order_relaxed);
    s.imu_fed     = impl_->imu_fed.load(std::memory_order_relaxed);
    s.imu_bus_dropped = impl_->imu_bus.dropped(impl_->imu_sub);
    {
        std::lock_guard lk(impl_->stats_mu);
        s.freq_hz          = impl_->freq_hz;
        s.cov_pos_std_m    = impl_->cov_pos_std_m;
        s.last             = impl_->last_out;
        s.tracked_features = impl_->last_out ? impl_->last_out->tracked_features : 0;
    }
    return s;
}

}  // namespace gw::vio
