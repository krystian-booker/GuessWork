#include "posest/CameraCapabilities.h"
#include "posest/CameraConfig.h"

namespace posest {

const char* connectionStateToString(ConnectionState state) noexcept {
    switch (state) {
        case ConnectionState::Disconnected: return "disconnected";
        case ConnectionState::Connecting:   return "connecting";
        case ConnectionState::Streaming:    return "streaming";
        case ConnectionState::Failed:       return "failed";
    }
    return "disconnected";
}

const char* triggerModeToString(TriggerMode mode) noexcept {
    switch (mode) {
        case TriggerMode::FreeRun:  return "free_run";
        case TriggerMode::External: return "external";
        case TriggerMode::Software: return "software";
    }
    return "free_run";
}

std::optional<TriggerMode> triggerModeFromString(std::string_view text) noexcept {
    if (text == "free_run") return TriggerMode::FreeRun;
    if (text == "external") return TriggerMode::External;
    if (text == "software") return TriggerMode::Software;
    return std::nullopt;
}

}  // namespace posest
