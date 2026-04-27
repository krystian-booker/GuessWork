#pragma once

#include <chrono>
#include <cstddef>
#include <string>

#include "posest/vio/AirborneCovariance.h"

namespace posest::vio {

// Strategy for converting Kimera's absolute pose marginal covariance
// into the relative-motion covariance attached to a BetweenFactor.
//
// kAbsolute   — pass through unchanged. Wrong in principle (the
//               BetweenFactor wants relative covariance) but harmless
//               in practice if Kimera's smoother is well-conditioned;
//               useful as a baseline.
// kDelta      — Σ_rel ≈ clamp_psd(Σ_curr − Σ_prev). More correct but
//               brittle near singularities. Falls back to default
//               sigmas via FusionService's PSD check at line 723.
// kScaled     — Σ_rel = α · Σ_curr with α∈(0,1]. Pragmatic compromise
//               and the default; α=0.8 leaves enough information to
//               keep the factor strong without claiming impossible
//               precision on a one-shot delta.
enum class CovarianceStrategy : std::uint8_t {
    kAbsolute = 0,
    kDelta = 1,
    kScaled = 2,
};

struct KimeraVioConfig {
    // Filesystem path to the directory holding FrontendParams.yaml,
    // BackendParams.yaml, ImuParams.yaml. Resolved by KimeraBackend at
    // start(); ignored by FakeVioBackend.
    std::string param_dir;

    // Camera identifier stamped onto every emitted VioMeasurement.
    // Should match the producer that feeds frames into this consumer
    // (FusionService treats camera_id as a routing tag; mismatched IDs
    // will silently route the measurement to the wrong factor chain).
    std::string camera_id{"vio"};

    AirborneThresholds airborne;

    // Diagonal multiplier applied when airborne or settling. 1e3 is
    // already 1000× any sane VIO sigma²; combined with `inflation_cap`
    // at 1e6 the factor's information matrix entries collapse to ≤ 1e-6
    // — a well-conditioned "near zero" that ISAM2 absorbs without
    // numerical pain. Anything larger risks overflow when the
    // information matrix is inverted internally.
    double inflation_factor{1.0e3};
    double inflation_cap{1.0e6};

    CovarianceStrategy covariance_strategy{CovarianceStrategy::kScaled};
    // Used only when covariance_strategy == kScaled.
    double covariance_scale_alpha{0.8};

    // Bound on the in-flight IMU buffer between the IMU bus drainer
    // thread (B) and the frame worker (A). At 1 kHz IMU and ~120 Hz
    // frames there are typically ~8 IMU samples per frame; 1024 caps
    // the buffer at ~1 second of IMU even if frames stop entirely. On
    // overflow the oldest sample is dropped.
    std::size_t imu_buffer_capacity{1024};

    // Bound on the recent (teensy_time_us → AirborneState) lookup
    // table consumed by the output callback. Sized for ~0.5 s of
    // 120 Hz frames; older entries are trimmed.
    std::size_t airborne_lookup_capacity{64};
};

}  // namespace posest::vio
