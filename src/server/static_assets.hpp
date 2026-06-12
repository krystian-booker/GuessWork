#pragma once

#include <crow.h>

namespace gw::server {

// Registers GET routes that serve the embedded React bundle. In Debug builds
// (no GW_HAS_EMBEDDED_WEB define), this is a no-op: the binary only exposes
// the JSON API and the React dev server is expected to run separately.
void register_static_routes(crow::SimpleApp& app);

// True when this binary embeds the React bundle (GW_BUILD_WEB=ON). The define
// is PRIVATE to gw_server, so callers (e.g. the startup banner in main) probe
// it at runtime through this.
bool has_embedded_web();

}  // namespace gw::server
