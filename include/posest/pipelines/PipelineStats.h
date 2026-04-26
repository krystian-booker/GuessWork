#pragma once

#include <cstdint>
#include <string>
#include <variant>

namespace posest::pipelines {

// Per-pipeline runtime counters. Mirrors the `FusionStats` shape: a small
// POD-ish struct read out under the pipeline's own lock via stats(). Surfaced
// on `DaemonHealth::apriltag_pipelines` and consumed by the daemon's
// `--health` JSON dump.
struct AprilTagPipelineStats {
    std::string pipeline_id;
    std::uint64_t frames_processed{0};
    std::uint64_t frames_no_detection{0};
    std::uint64_t frames_dropped_no_calibration{0};
    std::uint64_t frames_dropped_by_ambiguity{0};
    std::uint64_t frames_solved_single{0};
    std::uint64_t frames_solved_multi{0};
    // Mailbox drops as observed by the consumer's LatestFrameSlot. Read from
    // ConsumerBase::droppedByMailbox() each time stats() is called so the
    // value stays consistent with the rest of the snapshot.
    std::uint64_t mailbox_drops{0};
    std::int64_t  last_solve_latency_us{0};
    std::int64_t  max_solve_latency_us{0};
    double        last_reprojection_rms_px{0.0};
};

// Discriminated stats payload returned polymorphically by IVisionPipeline.
// Default-constructed monostate means "this pipeline has no telemetry to
// report" — placeholder pipelines, future Charuco/Aruco/etc. without
// dedicated counters yet.
using PipelineStatsValue = std::variant<std::monostate, AprilTagPipelineStats>;

}  // namespace posest::pipelines
