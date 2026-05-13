#pragma once

#include <optional>
#include <string>
#include <vector>

// Forward-declare CameraPtr without pulling the SDK headers in. The .cpp does
// include the SDK.
namespace Spinnaker {
class CameraPtr;
}

namespace gw {

struct VideoModeOption {
    std::string           name;          // GenICam symbolic, e.g. "Mode0"
    std::string           display_name;  // human label, may equal name
    std::string           description;
    // Resolution / frame-rate probed by briefly switching into the mode during
    // enumeration. nullopt if the camera failed to report them.
    std::optional<int>    width;
    std::optional<int>    height;
    std::optional<double> max_fps;
};

struct VideoModeList {
    bool                         supported = false;  // false if node not present
    std::optional<std::string>   current;            // currently-selected symbolic
    std::vector<VideoModeOption> options;
};

// Reads VideoMode enum entries from a camera. Both variants are safe to call on
// cameras that don't expose VideoMode — they return {supported=false, ...}.
//
// `_standalone` performs Init / DeInit around the read; it is intended for use
// from contexts where the camera is not already initialized (e.g. supervisor
// querying an available-serial). Spinnaker exceptions thrown during Init
// propagate, but DeInit is best-effort and never throws.
//
// `_initialized` assumes the caller already called Init() and will eventually
// call DeInit(). It is intended for use from inside SpinnakerProducer::start().
VideoModeList enumerate_video_modes_standalone(Spinnaker::CameraPtr cam);
VideoModeList enumerate_video_modes_initialized(Spinnaker::CameraPtr cam);

}  // namespace gw
