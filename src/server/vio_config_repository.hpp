#pragma once

#include <cstdint>
#include <optional>

namespace gw::server {

class Database;

// The single vio_config row — OpenVINS tunables. Which cameras participate
// comes from cameras.role; the IMU noise model from imu_config.
struct VioConfig {
    bool    enabled              = true;
    int64_t num_pts              = 150;
    int64_t fast_threshold       = 20;
    bool    downsample           = true;
    double  max_reproj_std_px    = 1.0;
    bool    auto_reinit          = true;
    int64_t reinit_min_features  = 15;
    int64_t reinit_window_frames = 15;
    double  reinit_max_pos_std_m = 2.0;
    int64_t updated_at           = 0;  // unix seconds
};

// Partial update. nullopt fields stay unchanged.
struct VioConfigUpdate {
    std::optional<bool>    enabled;
    std::optional<int64_t> num_pts;
    std::optional<int64_t> fast_threshold;
    std::optional<bool>    downsample;
    std::optional<double>  max_reproj_std_px;
    std::optional<bool>    auto_reinit;
    std::optional<int64_t> reinit_min_features;
    std::optional<int64_t> reinit_window_frames;
    std::optional<double>  reinit_max_pos_std_m;

    bool empty() const {
        return !enabled && !num_pts && !fast_threshold && !downsample &&
               !max_reproj_std_px && !auto_reinit && !reinit_min_features &&
               !reinit_window_frames && !reinit_max_pos_std_m;
    }
};

// Accessor for the single-row vio_config table (seeded at schema creation).
// Column CHECK violations surface as std::runtime_error from update().
class VioConfigRepository {
public:
    explicit VioConfigRepository(Database& db) : db_(db) {}

    VioConfig get();
    VioConfig update(const VioConfigUpdate& patch);

private:
    Database& db_;
};

}  // namespace gw::server
