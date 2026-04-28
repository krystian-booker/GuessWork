#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>

namespace posest::vio {

// Tracks whether the chassis is currently airborne, based on a stream of
// downward-facing ToF distance readings. The fusion graph must ignore VIO
// factors during airborne intervals (the downward camera is blurry and the
// metric scale collapses) and for a brief settle window after landing while
// the chassis stops ringing on its suspension.
//
// Thresholds use a Schmitt-trigger style hysteresis band so a single noisy
// sample near the boundary doesn't flap the state. See AirborneThresholds.
enum class AirborneState : std::uint8_t {
    kGrounded = 0,
    kAirborne = 1,
    kSettling = 2,
};

struct AirborneThresholds {
    // Upper threshold: distance above this is unambiguously airborne.
    // Default 0.15 m = 6 inches — chassis ground clearance is 4 inches, so
    // this gives ~2 inches of headroom over carpet/tile irregularities.
    double above_m{0.15};

    // Lower threshold: re-entering this band starts the settle timer. Tighter
    // than `above_m` to provide hysteresis. 0.127 m = 5 inches.
    double below_m{0.127};

    // Time the chassis stays in kSettling after dropping below `below_m`,
    // measured on steady_clock so it does not depend on frame cadence.
    // 50 ms covers the suspension ringdown observed under 15 g impulses.
    std::chrono::steady_clock::duration settle{std::chrono::milliseconds(50)};
};

class AirborneTracker {
public:
    AirborneTracker() = default;
    explicit AirborneTracker(AirborneThresholds thresholds)
        : thresholds_(thresholds) {}

    // Feed one ToF reading and return the current state. Pass std::nullopt
    // when the ToF reading is missing for this frame (e.g. ToFSampleCache
    // miss): the tracker holds its current state and bumps a counter so
    // the caller can observe ToF dropouts without making policy decisions
    // here.
    AirborneState update(std::optional<double> ground_distance_m,
                         std::chrono::steady_clock::time_point now);

    AirborneState state() const { return state_; }

    // Number of update() calls that received std::nullopt. Used by the
    // VIO consumer to expose a `ground_distance_missing_count` stat so a
    // broken ToFSampleCache is observable from the outside.
    std::uint64_t missingCount() const { return missing_count_; }

    const AirborneThresholds& thresholds() const { return thresholds_; }

    // Live-tunable swap of the hysteresis thresholds. Preserves the
    // current state and any in-flight settle deadline; the next update()
    // is what actually sees the new thresholds. Caller must serialize
    // calls with update() — the AirborneTracker owns no lock and is
    // single-threaded by contract.
    void setThresholds(AirborneThresholds t) { thresholds_ = t; }

private:
    AirborneThresholds thresholds_{};
    AirborneState state_{AirborneState::kGrounded};
    std::chrono::steady_clock::time_point settle_deadline_{};
    std::uint64_t missing_count_{0};
};

// Inflate a 6×6 covariance matrix (gtsam Pose3 tangent order
// [rx, ry, rz, tx, ty, tz], row-major) according to the airborne state.
// Diagonal entries are scaled — off-diagonals are left untouched so a
// well-conditioned input remains symmetric and PSD. Each diagonal entry
// is capped at `cap` to prevent ISAM2's information-matrix inversion
// from overflowing (1e6 is a safe ceiling: ~1000× the largest sane VIO
// sigma², well under the f64 condition limit).
//
// kGrounded → identity (input returned unchanged).
// kAirborne, kSettling → diagonal entries multiplied by `inflation_factor`
// then clamped to `cap`.
std::array<double, 36> inflate(const std::array<double, 36>& covariance,
                               AirborneState state,
                               double inflation_factor,
                               double cap);

}  // namespace posest::vio
