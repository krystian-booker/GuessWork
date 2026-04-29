#include "posest/vio/KimeraVioConfigBuilder.h"

namespace posest::vio {

KimeraVioConfig buildKimeraVioConfig(const runtime::RuntimeConfig& runtime_config) {
    KimeraVioConfig out = runtime_config.kimera_vio;
    if (!runtime_config.vio.vio_camera_id.empty()) {
        out.camera_id = runtime_config.vio.vio_camera_id;
    }
    // ir_led_enabled lives on the hardware-layer VioConfig but the
    // consumer needs to see it to gate CLAHE preprocessing. Mirror it
    // here rather than threading the full RuntimeConfig through; the
    // builder is the canonical translation point between persisted and
    // live config and runs again on every save callback.
    out.ir_led_enabled = runtime_config.vio.ir_led_enabled;
    return out;
}

}  // namespace posest::vio
