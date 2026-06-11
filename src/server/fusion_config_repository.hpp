#pragma once

#include <cstdint>
#include <optional>

namespace gw::server {

class Database;

// The single fusion_config row — GTSAM fusion-engine tunables. Sigma units:
// odom_sigma_* are 1σ velocity error (m/s, rad/s; interval noise scales
// linearly with dt), vio_sigma_* are random-walk densities (rad/√s, m/√s).
struct FusionConfig {
    bool    enabled              = true;
    double  lag_s                = 2.0;
    int64_t min_state_dt_ms      = 25;
    int64_t output_hz            = 100;
    int64_t max_extrapolation_ms = 150;
    double  tag_gate_chi2        = 22.46;
    double  tag_huber_k          = 1.345;
    double  vio_huber_k          = 1.345;
    double  odom_cauchy_k        = 0.5;
    double  odom_sigma_vx        = 0.05;
    double  odom_sigma_vy        = 0.05;
    double  odom_sigma_omega     = 0.05;
    double  vio_sigma_rot        = 0.01;
    double  vio_sigma_trans      = 0.01;
    double  collision_inflation  = 10.0;
    int64_t collision_window     = 20;
    double  reinit_pos_std_m     = 1.0;
    int64_t updated_at           = 0;  // unix seconds
};

// Partial update. nullopt fields stay unchanged.
struct FusionConfigUpdate {
    std::optional<bool>    enabled;
    std::optional<double>  lag_s;
    std::optional<int64_t> min_state_dt_ms;
    std::optional<int64_t> output_hz;
    std::optional<int64_t> max_extrapolation_ms;
    std::optional<double>  tag_gate_chi2;
    std::optional<double>  tag_huber_k;
    std::optional<double>  vio_huber_k;
    std::optional<double>  odom_cauchy_k;
    std::optional<double>  odom_sigma_vx;
    std::optional<double>  odom_sigma_vy;
    std::optional<double>  odom_sigma_omega;
    std::optional<double>  vio_sigma_rot;
    std::optional<double>  vio_sigma_trans;
    std::optional<double>  collision_inflation;
    std::optional<int64_t> collision_window;
    std::optional<double>  reinit_pos_std_m;

    bool empty() const {
        return !enabled && !lag_s && !min_state_dt_ms && !output_hz &&
               !max_extrapolation_ms && !tag_gate_chi2 && !tag_huber_k &&
               !vio_huber_k && !odom_cauchy_k && !odom_sigma_vx &&
               !odom_sigma_vy && !odom_sigma_omega && !vio_sigma_rot &&
               !vio_sigma_trans && !collision_inflation && !collision_window &&
               !reinit_pos_std_m;
    }
};

// Accessor for the single-row fusion_config table (seeded at schema
// creation). Column CHECK violations surface as std::runtime_error from
// update().
class FusionConfigRepository {
public:
    explicit FusionConfigRepository(Database& db) : db_(db) {}

    FusionConfig get();
    FusionConfig update(const FusionConfigUpdate& patch);

private:
    Database& db_;
};

}  // namespace gw::server
