#include "posest/vio/KimeraVioConfigBuilder.h"

namespace posest::vio {

KimeraVioConfig buildKimeraVioConfig(const runtime::RuntimeConfig& runtime_config) {
    KimeraVioConfig out = runtime_config.kimera_vio;
    if (!runtime_config.vio.vio_camera_id.empty()) {
        out.camera_id = runtime_config.vio.vio_camera_id;
    }
    return out;
}

}  // namespace posest::vio
