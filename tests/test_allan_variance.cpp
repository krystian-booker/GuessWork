#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "core/allan_variance.hpp"

namespace gw {

namespace {

struct Lcg {
    uint64_t state = 0x9E3779B97F4A7C15ull;
    uint64_t next() {
        state = state * 6364136223846793005ull + 1442695040888963407ull;
        return state >> 33;
    }
    double uniform() { return static_cast<double>(next()) / 2147483648.0; }
    double gauss() {  // Irwin–Hall ≈ N(0,1)
        double s = 0;
        for (int i = 0; i < 12; ++i) s += uniform();
        return s - 6.0;
    }
};

// White noise of density N (unit/√Hz): per-sample σ = N·√rate.
std::vector<float> white_noise(Lcg& rng, double n_density, double rate_hz,
                               double seconds) {
    const size_t n     = static_cast<size_t>(rate_hz * seconds);
    const double sigma = n_density * std::sqrt(rate_hz);
    std::vector<float> out(n);
    for (auto& v : out) v = static_cast<float>(sigma * rng.gauss());
    return out;
}

}  // namespace

TEST(AllanVarianceTest, WhiteNoiseRecoversDensity) {
    Lcg rng;
    const double N = 0.01;
    const auto samples = white_noise(rng, N, 400.0, 200.0);
    const auto res = compute_allan(samples, 400.0);

    ASSERT_TRUE(res.noise_density_ok);
    EXPECT_NEAR(res.noise_density, N, 0.1 * N);
    EXPECT_GT(res.fit_quality, 0.0);
}

TEST(AllanVarianceTest, MixtureRecoversBoth) {
    Lcg rng;
    const double N = 0.01, K = 0.005;  // crossover τ = 3·(N/K)² ≈ 12 s
    const double rate = 400.0, seconds = 400.0;
    const size_t n  = static_cast<size_t>(rate * seconds);
    const double dt = 1.0 / rate;

    std::vector<float> samples(n);
    double bias = 0.0;
    for (size_t i = 0; i < n; ++i) {
        bias += K * std::sqrt(dt) * rng.gauss();  // rate random walk
        samples[i] = static_cast<float>(N * std::sqrt(rate) * rng.gauss() + bias);
    }
    const auto res = compute_allan(samples, rate);

    ASSERT_TRUE(res.noise_density_ok);
    EXPECT_NEAR(res.noise_density, N, 0.1 * N);
    ASSERT_TRUE(res.random_walk_ok);
    EXPECT_NEAR(res.random_walk, K, 0.25 * K);  // K converges slower than N
}

TEST(AllanVarianceTest, ShortInputNoBlowup) {
    Lcg rng;
    const auto samples = white_noise(rng, 0.01, 400.0, 0.375);  // 150 samples
    const auto res = compute_allan(samples, 400.0);

    // No qualifying +1/2 region in 0.375 s of data — but nothing explodes.
    EXPECT_FALSE(res.random_walk_ok);
    EXPECT_TRUE(std::isfinite(res.noise_density));
    EXPECT_TRUE(std::isfinite(res.random_walk));
    EXPECT_TRUE(std::isfinite(res.fit_quality));
    for (const auto& p : res.curve) {
        EXPECT_TRUE(std::isfinite(p.sigma));
        EXPECT_GT(p.tau_s, 0.0);
    }
}

TEST(AllanVarianceTest, TooFewSamplesReturnsEmpty) {
    const std::vector<float> samples(50, 0.1f);
    const auto res = compute_allan(samples, 400.0);
    EXPECT_TRUE(res.curve.empty());
    EXPECT_FALSE(res.noise_density_ok);
    EXPECT_FALSE(res.random_walk_ok);
}

TEST(AllanVarianceTest, WhiteNoiseCurveSlopeIsMinusHalf) {
    Lcg rng;
    const auto samples = white_noise(rng, 0.02, 400.0, 100.0);
    const auto res = compute_allan(samples, 400.0);
    ASSERT_GE(res.curve.size(), 6u);

    // Average slope over the small-τ half of the curve.
    const size_t half = res.curve.size() / 2;
    const double s =
        (std::log10(res.curve[half].sigma) - std::log10(res.curve[0].sigma)) /
        (std::log10(res.curve[half].tau_s) - std::log10(res.curve[0].tau_s));
    EXPECT_NEAR(s, -0.5, 0.1);
}

}  // namespace gw
