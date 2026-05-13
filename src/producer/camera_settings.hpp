#pragma once

#include <optional>
#include <string>

namespace gw {

// Current applied state for the live-tunable camera settings, also reused as
// the "patch" type for apply_settings_live(): a nullopt field means "leave
// untouched" on input and "node not exposed by this camera" on output.
//
// Auto→manual seeding contract: when {gain,exposure}_auto goes to false AND no
// explicit value is provided in the same patch, the producer reads the current
// camera-side value and writes it back as the manual setting before flipping
// auto off. The actually-applied value is returned in the same struct.
struct CameraSettingsValues {
    std::optional<bool>   gain_auto;
    std::optional<double> gain;
    std::optional<bool>   exposure_auto;
    std::optional<double> exposure;
};

using CameraSettingsPatch = CameraSettingsValues;

struct CameraSettingRange {
    double      min = 0;
    double      max = 0;
    std::string unit;
};

// Min/max/unit for each per-camera setting, cached after start(). A nullopt
// entry means the camera does not expose that float node.
struct CameraSettingsLimits {
    std::optional<CameraSettingRange> gain;     // dB
    std::optional<CameraSettingRange> exposure; // us
};

}  // namespace gw
