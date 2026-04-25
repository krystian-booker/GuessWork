#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "posest/CameraConfig.h"

namespace posest {

enum class ConnectionState : std::uint8_t {
    Disconnected = 0,
    Connecting = 1,
    Streaming = 2,
    Failed = 3,
};

const char* connectionStateToString(ConnectionState state) noexcept;

enum class ControlValueType : std::uint8_t {
    Integer = 0,
    Boolean = 1,
    Menu = 2,
};

struct ControlMenuEntry {
    std::int32_t value = 0;
    std::string label;
};

struct ControlDescriptor {
    std::string name;
    ControlValueType type = ControlValueType::Integer;
    std::int32_t min = 0;
    std::int32_t max = 0;
    std::int32_t step = 1;
    std::int32_t default_value = 0;
    std::optional<std::int32_t> current_value;
    bool read_only = false;
    bool auto_mode = false;
    std::string auto_companion;
    std::vector<ControlMenuEntry> menu_entries;
};

struct FrameRateRange {
    double min_fps = 0.0;
    double max_fps = 0.0;
    double step_fps = 0.0;            // 0 = continuous range
    std::vector<double> discrete;     // populated only for discrete enumerations
};

struct FrameSizeOption {
    int width = 0;
    int height = 0;
    std::vector<FrameRateRange> rates;
};

struct PixelFormatOption {
    std::string name;                 // canonical (e.g. "mjpeg", "yuyv", "grey")
    std::uint32_t fourcc = 0;         // backend-defined; 0 if not applicable
    std::vector<FrameSizeOption> sizes;
};

enum class DeviceHintKind : std::uint8_t {
    DevicePath = 0,
    SerialNumber = 1,
    NetworkAddress = 2,
    DiscoveryName = 3,
};

struct DeviceHint {
    DeviceHintKind kind = DeviceHintKind::DevicePath;
    std::string description;
};

struct LiveStats {
    ConnectionState state = ConnectionState::Disconnected;
    std::uint64_t disconnect_count = 0;
    std::uint64_t reconnect_attempts = 0;
    std::uint64_t successful_connects = 0;
    std::optional<std::chrono::steady_clock::time_point> last_frame_time;
    double measured_fps = 0.0;
    std::string last_error;
};

// Telemetry-friendly per-camera snapshot, keyed by camera id.
struct CameraLiveStats {
    std::string camera_id;
    LiveStats live;
};

struct CameraCapabilities {
    std::string camera_id;
    std::string backend;
    DeviceHint device_hint;

    std::vector<PixelFormatOption> pixel_formats;
    std::vector<ControlDescriptor> controls;
    std::vector<TriggerMode> trigger_modes;

    bool supports_set_control = false;
    bool supports_get_control = false;
    bool supports_reconnect = false;
    bool supports_set_trigger_mode = false;

    std::optional<CameraFormatConfig> current_format;
    TriggerMode current_trigger_mode = TriggerMode::FreeRun;

    LiveStats live;
};

}  // namespace posest
