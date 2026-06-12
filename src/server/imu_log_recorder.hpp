#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>

namespace gw::server {

class TeensyManager;

// Records the raw IMU stream to disk for Allan-variance analysis. One file
// per recording: <dir>/<unix_ts>.bin of fixed 32-byte little-endian records
//   u64 t_ns | f32 ax ay az | f32 gx gy gz
// (no header — the layout is also parsed by ImuAllanService::analyze).
//
// One drain thread per recording (MultiTopicBagRecorder pattern): subscribes
// teensy.imu_bus() with a 4096 ring, wait_pop loop into a buffered ofstream,
// auto-stops when the Teensy-clock span reaches the requested duration.
class ImuLogRecorder {
public:
    ImuLogRecorder(TeensyManager& teensy, std::filesystem::path dir);
    ~ImuLogRecorder();  // stops + joins

    ImuLogRecorder(const ImuLogRecorder&)            = delete;
    ImuLogRecorder& operator=(const ImuLogRecorder&) = delete;

    struct Status {
        bool                       recording = false;
        std::optional<std::string> file;  // current, or last finished
        uint64_t                   samples = 0;
        uint64_t                   bytes   = 0;
        double                     rate_hz = 0.0;  // ~1 s window
        int64_t                    remaining_s = 0;
    };

    static constexpr int64_t kMaxDurationS = 86'400;

    // false (+err) when already recording / bad duration / file unopenable.
    bool start(int64_t duration_s, std::string& err);
    // false when idle.
    bool stop();

    Status status() const;

    // Newest .bin in dir by filename (unix timestamp). nullopt when none.
    std::optional<std::filesystem::path> newest_log() const;

    const std::filesystem::path& dir() const { return dir_; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    TeensyManager&        teensy_;
    std::filesystem::path dir_;
};

}  // namespace gw::server
