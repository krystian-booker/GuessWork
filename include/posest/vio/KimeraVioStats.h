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
    // Live-config bookkeeping. Mirrors FusionService's
    // config_reloads_applied / config_reloads_structural_skipped split
    // so a website save that touches a structural field is observable
    // instead of silently no-op.
    std::uint64_t config_reloads_applied{0};
    std::uint64_t config_reloads_structural_skipped{0};

    // Milliseconds since the last published VioMeasurement, computed at
    // snapshot time relative to steady_clock::now(). nullopt before the
    // first output is published. The watchdog signal that catches a
    // silent autoinit failure or a Kimera spin that never yields output:
    // when this number stops growing, VIO is healthy; when it grows
    // unbounded, the operator (or a website-side alarm) sees the stall.
    std::optional<std::int64_t> last_output_age_ms;
};

}  // namespace posest::vio
