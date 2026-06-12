#pragma once

#include <cstdint>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "core/allan_variance.hpp"
#include "server/imu_log_recorder.hpp"

namespace gw::server {

class ImuConfigRepository;
class TeensyManager;

// Fully integrated Allan-variance IMU noise refinement: record a long static
// IMU log (ImuLogRecorder), analyze it server-side (gw::compute_allan per
// axis), and apply the suggested Kalibr-convention noise values into
// imu_config. Driven by the /api/imu/allan/* routes.
class ImuAllanService {
public:
    ImuAllanService(TeensyManager& teensy, ImuConfigRepository& imu_config,
                    std::filesystem::path log_dir);

    bool start_recording(int64_t duration_s, std::string& err);
    bool stop_recording();
    ImuLogRecorder::Status recording_status() const;

    struct Analysis {
        std::string file;
        uint64_t    samples    = 0;
        double      duration_s = 0.0;
        double      rate_hz    = 0.0;

        // Suggested imu_config values = worst-axis (max), Kalibr units.
        double accel_noise_density = 0.0;  // m/s²/√Hz
        double accel_random_walk   = 0.0;  // m/s³/√Hz
        double gyro_noise_density  = 0.0;  // rad/s/√Hz
        double gyro_random_walk    = 0.0;  // rad/s²/√Hz

        // ax ay az gx gy gz (curves included — ~30 points each).
        gw::AllanResult axes[6];

        std::vector<std::string> warnings;
        int64_t                  analyzed_at = 0;  // unix seconds
    };

    // Synchronous (~1–2 s for an 8 h / 11 M-sample log). `file` is a
    // basename inside log_dir; empty selects the newest log. Throws
    // std::runtime_error on missing/unreadable/too-short input.
    Analysis analyze(const std::string& file);

    std::optional<Analysis> last_analysis() const;

    // Writes the last analysis's suggested values into imu_config. The
    // caller is responsible for fusion.reload(). false (+err) when no
    // analysis exists.
    bool apply(std::string& err);

private:
    TeensyManager&       teensy_;
    ImuConfigRepository& imu_config_;
    ImuLogRecorder       recorder_;

    mutable std::mutex      mu_;
    std::optional<Analysis> last_;
};

}  // namespace gw::server
