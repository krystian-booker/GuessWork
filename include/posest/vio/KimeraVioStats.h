#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace posest::vio {

// Snapshot returned by KimeraVioConsumer::stats() and surfaced
// polymorphically through IVisionPipeline::pipelineStats() so the daemon
// can fan it onto DaemonHealth without a dynamic_cast per pipeline type.
//
// Lives in its own header (rather than next to KimeraVioConsumer) because
// posest/pipelines/PipelineStats.h includes it for the variant alternative
// — pulling in the full consumer header would re-export GTSAM transitively.
struct KimeraVioStats {
    // Populated only when surfaced via pipelineStats(); KimeraVioConsumer
    // stamps it from ConsumerBase::id() at snapshot time. Empty when
    // returned from the internal stats() accessor used by tests.
    std::string pipeline_id;

    std::uint64_t frames_pushed{0};
    std::uint64_t frames_dropped_backpressure{0};
    std::uint64_t imu_pushed{0};
    std::uint64_t imu_dropped_backpressure{0};
    std::uint64_t imu_buffer_overflow{0};
    std::uint64_t outputs_received{0};
    std::uint64_t outputs_published{0};
    std::uint64_t outputs_skipped_first{0};
    std::uint64_t outputs_skipped_no_tracking{0};
    std::uint64_t outputs_inflated_airborne{0};
    std::uint64_t ground_distance_missing{0};
    // Outputs whose published VioMeasurement carried a ToF
    // ground_distance_m value (i.e. the Frame had a ToF reading and the
    // teensy_time_us snapshot survived eviction). Compared against
    // outputs_published this is the operator's signal that the camera↔
    // ToF time alignment is reaching downstream factor consumers — if
    // outputs_published rises but this stays at zero, the ToFSampleCache
    // wiring is broken upstream of KimeraVioConsumer.
    std::uint64_t outputs_with_ground_distance{0};
    // Live-config bookkeeping. Mirrors FusionService's
    // config_reloads_applied / config_reloads_structural_skipped split
    // so a website save that touches a structural field is observable
    // instead of silently no-op.
    std::uint64_t config_reloads_applied{0};
    std::uint64_t config_reloads_structural_skipped{0};

    // Phase 2 carpet-scenario telemetry. Both counters are zero unless
    // KimeraVioConfig::preprocess_clahe is on. frames_clahe_applied
    // increments per frame that actually went through cv::CLAHE;
    // frames_clahe_skipped_low_texture increments when the variance-of
    // -Laplacian gate suppressed the application (image too flat / too
    // noisy for CLAHE to help). The ratio is the operator's signal for
    // tuning clahe_min_variance_laplacian.
    std::uint64_t frames_clahe_applied{0};
    std::uint64_t frames_clahe_skipped_low_texture{0};

    // Phase 2 backend-output telemetry. last_landmark_count is the raw
    // value from Kimera's most recent BackendOutput; landmark_count_avg
    // is an exponential moving average over the same series (alpha 0.1,
    // ~10-frame smoothing) so the operator gets a steadier signal than
    // the per-frame value. outputs_below_landmark_floor counts outputs
    // whose landmark_count was strictly less than
    // KimeraVioConfig::landmark_count_floor — independent of Kimera's
    // own tracking_ok heuristic, this is the metric that catches the
    // CLAHE-amplified-FPN regime (lots of landmarks reported but they
    // die in optimization rather than survive).
    std::int32_t last_landmark_count{0};
    double landmark_count_avg{0.0};
    std::uint64_t outputs_below_landmark_floor{0};

    // Milliseconds since the last published VioMeasurement, computed at
    // snapshot time relative to steady_clock::now(). nullopt before the
    // first output is published. The watchdog signal that catches a
    // silent autoinit failure or a Kimera spin that never yields output:
    // when this number stops growing, VIO is healthy; when it grows
    // unbounded, the operator (or a website-side alarm) sees the stall.
    std::optional<std::int64_t> last_output_age_ms;
};

}  // namespace posest::vio
