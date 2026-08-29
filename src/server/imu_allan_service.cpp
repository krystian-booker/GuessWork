#include "server/imu_allan_service.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <stdexcept>

#include "server/imu_config_repository.hpp"

namespace gw::server {

namespace {

constexpr size_t kRecordBytes = 32;  // mirrors imu_log_recorder.cpp

// Static-ness heuristics — appended as warnings, never hard failures (the
// operator decides; a non-static recording just produces inflated values).
constexpr double kGyroMeanWarn  = 0.02;  // rad/s
constexpr double kGyroStdWarn   = 0.05;  // rad/s
constexpr double kAccelNormWarn = 0.5;   // m/s² off 9.81
constexpr double kMinCredibleS  = 3.0 * 3600.0;

}  // namespace

ImuAllanService::ImuAllanService(SyncControllerManager&        controller,
                                 ImuConfigRepository&  imu_config,
                                 std::filesystem::path log_dir)
    : controller_(controller),
      imu_config_(imu_config),
      recorder_(controller, std::move(log_dir)) {}

bool ImuAllanService::start_recording(int64_t duration_s, std::string& err) {
    return recorder_.start(duration_s, err);
}

bool ImuAllanService::stop_recording() { return recorder_.stop(); }

ImuLogRecorder::Status ImuAllanService::recording_status() const {
    return recorder_.status();
}

ImuAllanService::Analysis ImuAllanService::analyze(const std::string& file) {
    if (recorder_.status().recording) {
        throw std::runtime_error("recording in progress — stop it first");
    }

    std::filesystem::path path;
    if (file.empty()) {
        const auto newest = recorder_.newest_log();
        if (!newest) throw std::runtime_error("no IMU logs recorded yet");
        path = *newest;
    } else {
        if (file.find('/') != std::string::npos) {
            throw std::runtime_error("file must be a basename inside the log dir");
        }
        path = recorder_.dir() / file;
    }

    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) {
        throw std::runtime_error("cannot open " + path.string());
    }

    Analysis a;
    a.file = path.filename().string();

    // One pass loads all six channels (8 h ≈ 6 × 46 MB of floats — fine);
    // the Allan θ scratch (the big allocation) is per-axis and sequential.
    std::vector<float> chan[6];
    uint64_t first_t = 0, last_t = 0;
    uint8_t  rec[kRecordBytes];
    while (in.read(reinterpret_cast<char*>(rec), kRecordBytes)) {
        uint64_t t_ns;
        float    v[6];
        std::memcpy(&t_ns, rec, 8);
        std::memcpy(v, rec + 8, 24);
        if (first_t == 0) first_t = t_ns;
        last_t = t_ns;
        for (int i = 0; i < 6; ++i) chan[i].push_back(v[i]);
        ++a.samples;
    }
    if (a.samples < 1000) {
        throw std::runtime_error("log too short to analyze (" +
                                 std::to_string(a.samples) + " samples)");
    }

    a.duration_s = static_cast<double>(last_t - first_t) * 1e-9;
    a.rate_hz    = a.duration_s > 0
                       ? static_cast<double>(a.samples - 1) / a.duration_s
                       : 0.0;
    if (a.rate_hz <= 0) throw std::runtime_error("non-monotonic timestamps in log");

    // Static-ness heuristics.
    double mean[6] = {}, var[6] = {};
    for (int i = 0; i < 6; ++i) {
        for (float v : chan[i]) mean[i] += v;
        mean[i] /= static_cast<double>(a.samples);
        for (float v : chan[i]) var[i] += (v - mean[i]) * (v - mean[i]);
        var[i] /= static_cast<double>(a.samples);
    }
    for (int g = 3; g < 6; ++g) {
        if (std::abs(mean[g]) > kGyroMeanWarn || std::sqrt(var[g]) > kGyroStdWarn) {
            a.warnings.push_back("motion suspected on gyro axis " +
                                 std::to_string(g - 3) +
                                 " — data may not be static");
        }
    }
    const double g_norm = std::sqrt(mean[0] * mean[0] + mean[1] * mean[1] +
                                    mean[2] * mean[2]);
    if (std::abs(g_norm - 9.81) > kAccelNormWarn) {
        a.warnings.push_back("accel norm " + std::to_string(g_norm) +
                             " m/s² far from gravity — check mounting/units");
    }
    if (a.duration_s < kMinCredibleS) {
        a.warnings.push_back(
            "recording shorter than 3 h — random-walk fit unreliable "
            "(record overnight for credible values)");
    }

    for (int i = 0; i < 6; ++i) {
        a.axes[i] = gw::compute_allan(chan[i], a.rate_hz);
        chan[i].clear();
        chan[i].shrink_to_fit();
        const char* axis_names[6] = {"accel_x", "accel_y", "accel_z",
                                     "gyro_x",  "gyro_y",  "gyro_z"};
        if (!a.axes[i].noise_density_ok) {
            a.warnings.push_back(std::string(axis_names[i]) +
                                 ": no clean white-noise region in the Allan "
                                 "curve — noise_density is a rough bound");
        }
        if (!a.axes[i].random_walk_ok) {
            a.warnings.push_back(std::string(axis_names[i]) +
                                 ": no clean +1/2-slope region — random_walk "
                                 "is a rough bound");
        }
    }

    // Suggested config = worst axis (max) per quantity — conservative.
    for (int i = 0; i < 3; ++i) {
        a.accel_noise_density =
            std::max(a.accel_noise_density, a.axes[i].noise_density);
        a.accel_random_walk =
            std::max(a.accel_random_walk, a.axes[i].random_walk);
        a.gyro_noise_density =
            std::max(a.gyro_noise_density, a.axes[i + 3].noise_density);
        a.gyro_random_walk =
            std::max(a.gyro_random_walk, a.axes[i + 3].random_walk);
    }
    a.analyzed_at = std::chrono::duration_cast<std::chrono::seconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count();

    std::lock_guard lk(mu_);
    last_ = a;
    return a;
}

std::optional<ImuAllanService::Analysis> ImuAllanService::last_analysis() const {
    std::lock_guard lk(mu_);
    return last_;
}

bool ImuAllanService::apply(std::string& err) {
    std::optional<Analysis> a;
    {
        std::lock_guard lk(mu_);
        a = last_;
    }
    if (!a) {
        err = "no analysis to apply";
        return false;
    }
    ImuConfigUpdate patch;
    patch.accel_noise_density = a->accel_noise_density;
    patch.accel_random_walk   = a->accel_random_walk;
    patch.gyro_noise_density  = a->gyro_noise_density;
    patch.gyro_random_walk    = a->gyro_random_walk;
    try {
        imu_config_.update(patch);
    } catch (const std::exception& e) {
        err = e.what();
        return false;
    }
    return true;
}

}  // namespace gw::server
