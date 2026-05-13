#pragma once

// This header pulls in the full Spinnaker SDK and is intended to be included
// ONLY by translation units that need to construct a SpinnakerCameraBinding
// (i.e. the CameraSupervisor). Everyone else uses spinnaker_producer.hpp.

#include "producer/spinnaker_producer.hpp"

#include <SpinnakerPlatform.h>
#undef  SPINNAKER_DEPRECATED_CLASS
#define SPINNAKER_DEPRECATED_CLASS(msg) class SPINNAKER_API

#include <Spinnaker.h>

namespace gw {

struct SpinnakerCameraBinding {
    Spinnaker::SystemPtr system;
    Spinnaker::CameraPtr cam;
};

}  // namespace gw
