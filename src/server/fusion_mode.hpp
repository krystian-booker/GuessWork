#pragma once

#include <cstdint>
#include <optional>

namespace gw::server {

// Source freshness threshold for the degraded-modes matrix
// (docs/pose_pipeline.md — the matrix rows mirror these cases 1:1).
inline constexpr int64_t kFusionFreshMs = 500;

// Pure derivation of the human-readable fusion mode from source ages.
// nullopt age = never seen = stale. vio_ok additionally requires
// vio_enabled (T_robot_imu configured).
inline const char* derive_fusion_mode(bool                   initialized,
                                      std::optional<int64_t> tag_age_ms,
                                      std::optional<int64_t> vio_age_ms,
                                      std::optional<int64_t> odom_age_ms,
                                      bool                   vio_enabled,
                                      bool                   collision_mode) {
    const auto fresh = [](const std::optional<int64_t>& age) {
        return age && *age < kFusionFreshMs;
    };
    if (!initialized) return "uninitialized";
    if (collision_mode) return "collision";

    const bool tags = fresh(tag_age_ms);
    const bool vio  = vio_enabled && fresh(vio_age_ms);
    const bool odom = fresh(odom_age_ms);

    if (!tags) return "dead_reckoning";  // drifting on whatever relatives remain
    if (vio && odom) return "nominal";
    if (!vio && odom) return "no_vio";
    if (vio && !odom) return "no_odom";
    return "tags_only";
}

}  // namespace gw::server
