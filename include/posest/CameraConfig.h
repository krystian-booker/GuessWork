#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace posest {

struct CameraFormatConfig {
    int width = 1920;
    int height = 1080;
    double fps = 30.0;
    std::string pixel_format = "mjpeg";
};

struct CameraControlEntry {
    std::string name;
    std::int32_t value;
};

enum class TriggerMode : std::uint8_t {
    FreeRun = 0,    // backend drives its own exposure cadence
    External = 1,   // hardware trigger pulse on a GPIO/strobe line
    Software = 2,   // host issues a software trigger per frame
};

const char* triggerModeToString(TriggerMode mode) noexcept;
std::optional<TriggerMode> triggerModeFromString(std::string_view text) noexcept;

// Reconnect policy applied by CameraProducer when grabFrame reports a
// transient I/O error (e.g. USB device unplugged). interval_ms == 0 disables
// reconnect entirely (the capture thread exits, matching pre-feature behavior).
// max_attempts == 0 means retry forever (mirrors TeensyConfig semantics).
struct ReconnectPolicy {
    std::uint32_t interval_ms = 1000;
    std::uint32_t max_attempts = 0;
};

struct CameraConfig {
    std::string id;
    std::string type;
    std::string device;
    bool enabled = true;
    CameraFormatConfig format;
    std::vector<CameraControlEntry> controls;
    TriggerMode trigger_mode = TriggerMode::FreeRun;
    ReconnectPolicy reconnect{};
};

}  // namespace posest
