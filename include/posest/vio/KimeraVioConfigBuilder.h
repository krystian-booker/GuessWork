#pragma once

#include "posest/runtime/RuntimeConfig.h"
#include "posest/vio/KimeraVioConfig.h"

namespace posest::vio {

// Translate the persisted runtime::RuntimeConfig into the live
// KimeraVioConfig consumed by KimeraVioConsumer. Mirrors
// fusion::buildFusionConfig (FusionService.cpp). The camera_id field is
// not persisted on the kimera_vio_config row — it is derived from the
// hardware-layer vio_config.vio_camera_id so the two stay in lockstep.
KimeraVioConfig buildKimeraVioConfig(const runtime::RuntimeConfig& runtime_config);

}  // namespace posest::vio
