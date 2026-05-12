#pragma once

#include <crow.h>

namespace gw::server {

// Registers GET routes that serve the embedded React bundle. In Debug builds
// (no GW_HAS_EMBEDDED_WEB define), this is a no-op: the binary only exposes
// the JSON API and the React dev server is expected to run separately.
void register_static_routes(crow::SimpleApp& app);

}  // namespace gw::server
