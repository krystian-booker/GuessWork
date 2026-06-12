#pragma once

#include <vector>

namespace gw {

// Overlapping Allan-deviation analysis of a static inertial rate signal,
// extracting the two Kalibr-convention noise parameters:
//
//   noise_density (N): white-noise level — the σ(τ) = N/√τ line (slope −1/2
//     in log-log) evaluated at τ = 1 s. Units: input-unit/√Hz, i.e.
//     rad/s/√Hz for gyro (rad/s input) and m/s²/√Hz for accel.
//   random_walk (K): rate-random-walk level — the σ(τ) = K·√(τ/3) line
//     (slope +1/2) evaluated at τ = 3 s. Units: rad/s²/√Hz | m/s³/√Hz.
//
// Region detection is slope-based (longest contiguous run of points whose
// local log-log slope is within ±0.2 of the target); a region with fewer
// than 3 points yields a best-effort value with its `ok` flag false — the
// result is never NaN/inf. A credible K fit needs hours of static data
// (≥3 h minimum, overnight recommended) — callers warn on short inputs.
//
// References: IEEE Std 952, the Kalibr IMU-noise-model wiki, rpng/kalibr_allan.

struct AllanPoint {
    double tau_s = 0.0;
    double sigma = 0.0;  // Allan deviation at this τ
};

struct AllanResult {
    std::vector<AllanPoint> curve;  // ~30 log-spaced taus

    double noise_density = 0.0;
    double random_walk   = 0.0;

    bool   noise_density_ok = false;  // ≥3 contiguous points, slope ∈ [−0.7,−0.3]
    bool   random_walk_ok   = false;  // ≥3 contiguous points, slope ∈ [0.3, 0.7]
    double fit_quality      = 0.0;    // min over the two regions, 0..1
};

// Pure, single-threaded. Requires samples.size() >= 100 (returns an empty
// curve with both `ok` flags false otherwise).
AllanResult compute_allan(const std::vector<float>& samples, double rate_hz);

}  // namespace gw
