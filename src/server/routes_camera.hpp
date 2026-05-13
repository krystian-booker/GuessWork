#pragma once

#include <crow.h>

namespace gw::server {

class CameraRepository;
class CameraSupervisor;

void register_camera_routes(crow::SimpleApp&  app,
                            CameraRepository& repo,
                            CameraSupervisor& supervisor);

}  // namespace gw::server
