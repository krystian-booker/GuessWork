#pragma once

#include <vector>

#include "posest/CameraCapabilities.h"

namespace posest::v4l2 {

// Walks /sys/class/video4linux/video* and /dev/v4l/by-id/* to discover
// available V4L2 capture devices. Returns one CameraCapabilities entry per
// usable device (must support V4L2_CAP_VIDEO_CAPTURE + V4L2_CAP_STREAMING).
//
// camera_id is left empty (the UI/CLI assigns one); device_hint.description
// holds the most stable path it could find (preferring /dev/v4l/by-id).
std::vector<CameraCapabilities> enumerateDevices();

}  // namespace posest::v4l2
