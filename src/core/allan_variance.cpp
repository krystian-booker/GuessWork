#include "core/allan_variance.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace gw {

namespace {

struct RegionFit {
    double value       = 0.0;  // fitted line evaluated at the anchor τ
    bool   ok          = false;
    double fit_quality = 0.0;
};

// Fixed-slope least-squares fit in log10 space over curve[begin, end):
// log10 σ = b + slope·log10 τ ⇒ b = mean(log10 σ − slope·log10 τ).
// Returns the line's value at τ = anchor_tau and a residual-based quality.
RegionFit fit_fixed_slope(const std::vector<AllanPoint>& curve, size_t begin,
                          size_t end, double slope, double anchor_tau) {
    RegionFit out;
    const size_t n = end - begin;
    if (n == 0) return out;

    double b = 0.0;
    for (size_t i = begin; i < end; ++i) {
        b += std::log10(curve[i].sigma) - slope * std::log10(curve[i].tau_s);
    }
    b /= static_cast<double>(n);

    double ss = 0.0;
    for (size_t i = begin; i < end; ++i) {
        const double r = std::log10(curve[i].sigma) -
                         (b + slope * std::log10(curve[i].tau_s));
        ss += r * r;
    }
    const double rms = std::sqrt(ss / static_cast<double>(n));

    out.value       = std::pow(10.0, b + slope * std::log10(anchor_tau));
    out.ok          = n >= 3;
    out.fit_quality = std::clamp(1.0 - rms / 0.2, 0.0, 1.0);
    return out;
}

// Longest contiguous run of curve points whose local log-log slope lies in
// [lo, hi], restricted to indices ≥ from. Slopes are central differences
// (forward/backward at the ends). Returns [begin, end); empty when none.
// On a tie, the earlier run wins (prefers small τ — right for the N region).
std::pair<size_t, size_t> longest_slope_run(const std::vector<AllanPoint>& curve,
                                            const std::vector<double>& slope,
                                            double lo, double hi, size_t from) {
    size_t best_b = 0, best_e = 0;
    size_t run_b = from;
    for (size_t i = from; i <= curve.size(); ++i) {
        const bool in = i < curve.size() && slope[i] >= lo && slope[i] <= hi;
        if (!in) {
            if (i - run_b > best_e - best_b) {
                best_b = run_b;
                best_e = i;
            }
            run_b = i + 1;
        }
    }
    return {best_b, best_e};
}

// Best-effort fallback: the 3 points whose local slope is closest to target.
// Used when no qualifying run exists so the API never returns NaN — callers
// see ok=false and treat the value as a rough bound.
std::pair<size_t, size_t> closest_slope_window(const std::vector<double>& slope,
                                               double target, size_t from,
                                               size_t n_points) {
    if (n_points < from + 3) return {from, n_points};
    size_t best  = from;
    double bestd = 1e18;
    for (size_t i = from; i + 3 <= n_points; ++i) {
        double d = 0;
        for (size_t j = i; j < i + 3; ++j) d += std::abs(slope[j] - target);
        if (d < bestd) {
            bestd = d;
            best  = i;
        }
    }
    return {best, best + 3};
}

}  // namespace

AllanResult compute_allan(const std::vector<float>& samples, double rate_hz) {
    AllanResult res;
    const size_t n = samples.size();
    if (n < 100 || rate_hz <= 0.0) return res;

    const double dt = 1.0 / rate_hz;

    // Cumulative integral θ[k] = dt·Σ_{i<k} y[i] in double — float
    // accumulation over millions of samples loses the small differences the
    // estimator lives on.
    std::vector<double> theta(n + 1);
    theta[0] = 0.0;
    for (size_t i = 0; i < n; ++i) {
        theta[i + 1] = theta[i] + dt * static_cast<double>(samples[i]);
    }

    // ~30 log-spaced cluster sizes m ∈ [1, n/10], each needing N−2m ≥ 8
    // overlapping triples.
    std::vector<size_t> ms;
    const double m_max = static_cast<double>(n) / 10.0;
    for (int i = 0; i < 30; ++i) {
        const double f = static_cast<double>(i) / 29.0;
        const size_t m =
            static_cast<size_t>(std::round(std::pow(m_max, f)));
        if (m < 1 || n < 2 * m + 8) continue;
        if (!ms.empty() && ms.back() == m) continue;
        ms.push_back(m);
    }
    if (ms.size() < 2) return res;

    for (const size_t m : ms) {
        const double tau = static_cast<double>(m) * dt;
        double       sum = 0.0;
        const size_t k_end = n - 2 * m;  // inclusive
        for (size_t k = 0; k <= k_end; ++k) {
            const double d = theta[k + 2 * m] - 2.0 * theta[k + m] + theta[k];
            sum += d * d;
        }
        const double avar =
            sum / (2.0 * tau * tau * static_cast<double>(k_end + 1));
        res.curve.push_back({tau, std::sqrt(avar)});
    }

    // Local log-log slopes (central differences).
    const size_t np = res.curve.size();
    std::vector<double> slope(np, 0.0);
    const auto lg = [&](size_t i) {
        return std::make_pair(std::log10(res.curve[i].tau_s),
                              std::log10(res.curve[i].sigma));
    };
    for (size_t i = 0; i < np; ++i) {
        const size_t a = i == 0 ? 0 : i - 1;
        const size_t b = i + 1 >= np ? np - 1 : i + 1;
        const auto [xa, ya] = lg(a);
        const auto [xb, yb] = lg(b);
        slope[i] = (xb - xa) > 0 ? (yb - ya) / (xb - xa) : 0.0;
    }

    // N region: white noise, slope −1/2, prefer the small-τ end.
    auto [nb, ne] = longest_slope_run(res.curve, slope, -0.7, -0.3, 0);
    bool n_run_ok = ne - nb >= 3;
    if (!n_run_ok) std::tie(nb, ne) = closest_slope_window(slope, -0.5, 0, np);
    const RegionFit nfit = fit_fixed_slope(res.curve, nb, ne, -0.5, 1.0);
    res.noise_density    = nfit.value;
    res.noise_density_ok = n_run_ok && nfit.ok;

    // K region: rate random walk, slope +1/2, at τ beyond the N region.
    auto [kb, ke] = longest_slope_run(res.curve, slope, 0.3, 0.7, ne);
    bool k_run_ok = ke - kb >= 3;
    if (!k_run_ok) std::tie(kb, ke) = closest_slope_window(slope, 0.5, ne, np);
    // σ(τ) = K·√(τ/3) ⇒ the +1/2 line evaluated at τ = 3 equals K.
    const RegionFit kfit = fit_fixed_slope(res.curve, kb, ke, 0.5, 3.0);
    res.random_walk    = kfit.value;
    res.random_walk_ok = k_run_ok && kfit.ok;

    res.fit_quality = std::min(nfit.fit_quality, kfit.fit_quality);
    return res;
}

}  // namespace gw
