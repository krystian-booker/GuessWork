#pragma once

#include <crow.h>

namespace gw::server {

class CameraRepository;

void register_camera_routes(crow::SimpleApp& app, CameraRepository& repo);

}  // namespace gw::server
