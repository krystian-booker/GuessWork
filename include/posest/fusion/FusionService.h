#pragma once

#include <atomic>
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "posest/MeasurementBus.h"
#include "posest/fusion/IFusionOutputSink.h"
#include "posest/runtime/RuntimeConfig.h"
#include "posest/util/LatencyHistogram.h"

namespace posest::fusion {

struct FusionStats {
    std::uint64_t measurements_processed{0};
    std::uint64_t stale_measurements{0};
    // Most recent timestamp accepted across any measurement type. Per-type
    // monotonicity is enforced internally with one cursor per Measurement
    // variant alternative; this field is informational only.
    std::optional<Timestamp> last_measurement_time;
    // VIO factor traffic. Skipped-no-tracking exists so the placeholder
    // pipeline's tracking_ok=false samples are observable without polluting
    // measurements_processed.
    std::uint64_t measurements_vio_processed{0};
    std::uint64_t measurements_vio_skipped_no_tracking{0};

    // Phase C: IMU preintegration counters. Only ever non-zero when the
    // preintegration feature flag is on AND the state machine has reached
    // kRunning.
    std::uint64_t imu_factors_committed{0};
    std::uint64_t imu_out_of_order{0};
    std::uint64_t imu_resets{0};
    std::uint64_t keyframes_committed{0};
    std::uint64_t bias_calibrations_completed{0};

    // F-3: chassis samples dropped because their planar speed exceeded
    // FusionConfig::max_chassis_speed_mps. Counted once per drop; the
    // dropped sample's timestamp still advances the per-type cursor so a
    // burst of garbage doesn't strand subsequent good samples behind it.
    std::uint64_t chassis_speed_gated{0};

    // F-1: live config reload counters. *_applied counts every successful
    // applyConfig call (the worker observed and swapped a pending config);
    // *_structural_skipped is incremented per call when the new config
    // changed any field that requires a graph rebuild (and was therefore
    // ignored on the live path — the affected fields keep their old values
    // until the daemon restarts).
    std::uint64_t config_reloads_applied{0};
    std::uint64_t config_reloads_structural_skipped{0};

    // F-4: chassis-vs-IMU velocity disagreement detector. Phase-C-only.
    // *_events counts every keyframe whose chassis-mean body velocity
    // disagreed with the IMU-predicted body velocity by more than
    // FusionConfig::slip_disagreement_mps; *_inflations counts the keyframes
    // that fired the inflation path (after the 2-step hysteresis) and OR'd
    // kFusionStatusSlipDetected into the published estimate.
    std::uint64_t slip_disagreement_events{0};
    std::uint64_t slip_disagreement_inflations{0};

    // F-5: ToF unary z-prior counters. _added counts every pose key that
    // received the prior; the four _skipped_* variants split by the gate
    // that rejected the addition so an operator can see which knob is
    // misconfigured (no sample yet, last sample stale, range_status non-
    // zero, or distance outside the expected band).
    std::uint64_t tof_z_priors_added{0};
    std::uint64_t tof_z_priors_skipped_no_sample{0};
    std::uint64_t tof_z_priors_skipped_stale{0};
    std::uint64_t tof_z_priors_skipped_invalid_range{0};
    std::uint64_t tof_z_priors_skipped_out_of_band{0};

    // Phase E: per-stage latency observability.
    //
    // graph_update_us measures bus-pop → ISAM2.update() return for graph-
    // mutating measurements only (chassis/vision/VIO). IMU samples bypass
    // this since they don't update the graph.
    //
    // publish_us measures the synchronous sink fan-out (TeensyService etc.)
    // — should stay in single-digit microseconds since sinks only enqueue.
    //
    // graph_factor_count / graph_variable_count are snapshots of the live
    // ISAM2 graph captured after the most recent update; they grow over the
    // life of a match until Phase C's marginalization window kicks in.
    util::LatencyHistogram::Snapshot graph_update_us;
    util::LatencyHistogram::Snapshot publish_us;
    std::size_t graph_factor_count{0};
    std::size_t graph_variable_count{0};
    // Wall-clock microseconds-since-epoch of the last successful update.
    // Useful for detecting a stalled fusion worker without pulling time
    // through DaemonHealth itself.
    std::int64_t last_update_wall_clock_us{0};
};

inline constexpr std::uint32_t kFusionStatusInitializing = 1u << 0u;
inline constexpr std::uint32_t kFusionStatusVisionUnavailable = 1u << 1u;
inline constexpr std::uint32_t kFusionStatusMarginalUnavailable = 1u << 2u;
inline constexpr std::uint32_t kFusionStatusOptimizerError = 1u << 3u;
inline constexpr std::uint32_t kFusionStatusDegradedInput = 1u << 4u;
inline constexpr std::uint32_t kFusionStatusShockInflated = 1u << 5u;
inline constexpr std::uint32_t kFusionStatusChassisGap = 1u << 6u;
// Set when the RIO flagged the chassis sample with kChassisStatusSlip (i.e.
// it observed wheel slip). Independent of kFusionStatusShockInflated; both
// can fire on the same step. Triggers chassis-sigma inflation regardless of
// IMU shock detection.
inline constexpr std::uint32_t kFusionStatusSlipReported = 1u << 7u;
// Phase C: IMU preintegration status surface. Inert when
// FusionConfig::enable_imu_preintegration is false.
//
// kFusionStatusImuGap         — IMU samples were missing for longer than
//                               max_imu_gap_seconds; preintegrator was reset
//                               and the affected keyframe interval fell back
//                               to chassis-only.
// kFusionStatusBiasUnverified — boot-time stationary calibration window
//                               timed out without enough quiet samples.
//                               Fusion runs with the persisted bias seed.
// kFusionStatusAwaitingFieldFix — graph is initialized in body frame but
//                               vision has not yet anchored it; ImuFactors
//                               are suppressed because nav frame is undefined.
// kFusionStatusSlipDetected   — chassis-vs-IMU velocity disagreement crossed
//                               slip_disagreement_mps. Distinct from
//                               kFusionStatusSlipReported (RIO-driven).
inline constexpr std::uint32_t kFusionStatusImuGap = 1u << 8u;
inline constexpr std::uint32_t kFusionStatusBiasUnverified = 1u << 9u;
inline constexpr std::uint32_t kFusionStatusAwaitingFieldFix = 1u << 10u;
inline constexpr std::uint32_t kFusionStatusSlipDetected = 1u << 11u;

struct FusionConfig {
    // Process noise on the per-Δt BetweenFactor built from integrated
    // ChassisSpeeds. Order matches gtsam Pose3 tangent: [rx, ry, rz, tx, ty, tz].
    std::array<double, 6> chassis_sigmas{0.05, 0.05, 0.05, 0.02, 0.02, 0.02};
    std::array<double, 6> origin_prior_sigmas{10.0, 10.0, 3.14, 10.0, 10.0, 10.0};

    // Fires the shock branch when |a - g_local| over imu_window_seconds
    // exceeds this. 50 m/s^2 (~5g) is past normal driving accel for an FRC
    // chassis but well below collision/wheel-slip transients.
    double shock_threshold_mps2{50.0};
    // Fires the free-fall branch when |a| drops below this. The robot in
    // air-time over a bump or ramp reads ~0; normal driving never does.
    double freefall_threshold_mps2{3.0};
    // Multiplier applied to every chassis_sigma when either branch fires.
    double shock_inflation_factor{100.0};
    // IMU samples older than this (relative to the current chassis sample)
    // are pruned from the shock-detection window.
    double imu_window_seconds{0.05};
    // ChassisSpeeds samples spaced more than this apart skip the integration
    // step entirely (kFusionStatusChassisGap is set).
    double max_chassis_dt_seconds{0.5};
    // Local gravity vector in the IMU's robot frame. Subtracted before the
    // shock-magnitude test. Override for tilted IMU mounts.
    Vec3 gravity_local_mps2{0.0, 0.0, 9.80665};

    // VioMeasurement → BetweenFactor<Pose3> ingestion. On by default — the
    // platform's mono camera + IMU + ToF stack is sized to feed Kimera, and
    // shipping with this off silently dropped VIO output before it reached
    // the graph. Operators who deliberately want chassis-only fusion (e.g.
    // during a regression bisect) can flip the column to 0 in the DB; the
    // per-sample tracking_ok=false gate continues to short-circuit
    // placeholder publishers.
    bool enable_vio{true};
    // Fallback sigmas applied when a VioMeasurement arrives with a non-PD or
    // all-zero covariance. Order matches Pose3 tangent: [rx, ry, rz, tx, ty, tz].
    std::array<double, 6> vio_default_sigmas{0.05, 0.05, 0.05, 0.02, 0.02, 0.02};

    // Phase B: Huber kernel constant on the chassis BetweenFactor (in
    // standardized residual units). Wired into the noise model in Phase B.
    double huber_k{1.5};

    // Phase C: IMU preintegration. All fields below are inert until the
    // feature flag is true and the Phase C code path lands. See
    // docs/features/fusion-service.md §8 and the IMU preintegration design
    // memo in the implementation plan.
    bool enable_imu_preintegration{false};
    Pose3d imu_extrinsic_body_to_imu;
    double accel_noise_sigma{0.05};
    double gyro_noise_sigma{0.001};
    double accel_bias_rw_sigma{1e-4};
    double gyro_bias_rw_sigma{1e-5};
    double integration_cov_sigma{1e-8};
    std::array<double, 6> persisted_bias{};
    double bias_calibration_seconds{1.5};
    double bias_calibration_chassis_threshold{0.02};
    double max_keyframe_dt_seconds{0.020};
    double max_imu_gap_seconds{0.100};
    std::uint32_t marginalize_keyframe_window{500};
    double slip_disagreement_mps{1.0};

    // F-2: soft floor-constraint prior on every new pose key, pinning
    // z / roll / pitch toward zero so the always-grounded platform invariant
    // is encoded in the graph. Sigmas are [σ_z (m), σ_roll (rad), σ_pitch (rad)].
    bool enable_floor_constraint{true};
    std::array<double, 3> floor_constraint_sigmas{0.01, 0.0087, 0.0087};

    // F-3: drop chassis samples whose planar speed exceeds this bound, and
    // surface the drop as kFusionStatusDegradedInput. Sized for the platform's
    // 5.2 m/s ceiling with margin.
    double max_chassis_speed_mps{6.5};

    // F-4: ToF-driven unary z-prior on every new pose key. Stacks with the
    // F-2 floor constraint. Validity is checked at factor-add time using
    // the cached ToFSample's range_status, freshness, and the configured
    // expected-range band on RuntimeConfig::vio. tof_grounded_distance_m
    // is the calibration baseline used to convert distance to body-z; it
    // mirrors VioConfig::tof_grounded_distance_m so the live-edit path can
    // hand both into the backend. On by default — the platform ships with
    // a downward-facing ToF beside the mono camera and the prior is the
    // only direct metric anchor for body-z. Per-sample range_status and
    // freshness gates still drop bad readings; a misconfigured ToF surfaces
    // via the four tof_z_priors_skipped_* counters.
    bool enable_tof_z_prior{true};
    double tof_z_prior_sigma_m{0.02};
    double tof_z_prior_max_age_s{0.05};
    double tof_grounded_distance_m{0.10};
    // ToF sanity band — copied from VioConfig at buildFusionConfig() time.
    // A reading outside [min, max] (or with range_status != 0) skips the
    // prior. Defaults match VioConfig's defaults so a fresh FusionConfig
    // is internally consistent.
    double tof_expected_min_m{0.05};
    double tof_expected_max_m{4.0};
};

FusionConfig buildFusionConfig(const runtime::RuntimeConfig& runtime_config);

struct FusionBackend;

class FusionService final {
public:
    explicit FusionService(MeasurementBus& measurement_bus, FusionConfig config = {});
    ~FusionService();

    FusionService(const FusionService&) = delete;
    FusionService& operator=(const FusionService&) = delete;

    void addOutputSink(std::shared_ptr<IFusionOutputSink> sink);
    void start();
    void stop();

    FusionStats stats() const;
    std::optional<FusedPoseEstimate> latestEstimate() const;

    // F-7: snapshot of the current IMU bias as [ax, ay, az, gx, gy, gz].
    // Returns nullopt when the bias has not yet been set by a successful
    // boot stationary calibration — i.e. it is still the persisted seed (or
    // identity for a fresh DB). The daemon consults this on clean shutdown
    // before writing back to RuntimeConfig.fusion.persisted_bias.
    std::optional<std::array<double, 6>> currentBiasIfTrusted() const;

    // F-1: live reload of fusion config. Called by the daemon's "config
    // saved" callback after WebService writes a new RuntimeConfig to SQLite.
    // The caller passes the freshly-built fusion::FusionConfig; the worker
    // picks it up at the top of the next process() iteration. Live-
    // reloadable fields (sigmas, thresholds, max_chassis_speed_mps, etc.)
    // are swapped under the worker mutex; structural fields
    // (enable_imu_preintegration, enable_vio, IMU extrinsic, IMU noise
    // sigmas, persisted_bias, bias_calibration_seconds, enable_floor_constraint)
    // are silently ignored on the live path — config_reloads_structural_skipped
    // counts those so callers can see the restart-required state.
    void applyConfig(FusionConfig new_config);

private:
    void runLoop();
    void process(const Measurement& measurement);
    bool isSupportedMeasurement(const Measurement& measurement) const;
    // Per-type monotonicity gate: rejects a measurement whose timestamp is
    // older than the previous one of *the same variant alternative*.
    // Different types track independent cursors so a chassis sample at t=10
    // does not silently drop an AprilTag observation at t=9.
    bool acceptTimestamp(const Measurement& measurement, Timestamp timestamp);
    // publish_started is captured by the caller right before calling this
    // method so the publish-stage histogram measures sink fan-out only,
    // not the time spent doing the graph update upstream.
    void publishEstimate(
        FusedPoseEstimate estimate,
        std::chrono::steady_clock::time_point publish_started);

    static constexpr std::size_t kMeasurementTypeCount = 6;

    MeasurementBus& measurement_bus_;
    std::unique_ptr<FusionBackend> backend_;
    mutable std::mutex mu_;
    std::vector<std::shared_ptr<IFusionOutputSink>> sinks_;
    FusionStats stats_;
    // Live histograms; snapshot()ed into FusionStats inside stats(). Both are
    // mutated only by the worker thread under mu_, snapshotted by readers
    // under the same lock.
    util::LatencyHistogram graph_update_hist_;
    util::LatencyHistogram publish_hist_;
    // Live graph sizes, refreshed after every successful backend update.
    std::size_t graph_factor_count_{0};
    std::size_t graph_variable_count_{0};
    std::int64_t last_update_wall_clock_us_{0};
    // One cursor per Measurement variant alternative — keyed by Measurement::index().
    std::array<std::optional<Timestamp>, kMeasurementTypeCount> per_type_cursors_;
    std::optional<FusedPoseEstimate> latest_estimate_;
    // F-1: pending-config hand-off. Web thread writes under mu_; worker
    // pops at the top of every process() iteration via consumePendingConfig.
    std::optional<FusionConfig> pending_config_;
    std::thread worker_;
    std::atomic<bool> running_{false};
};

}  // namespace posest::fusion
