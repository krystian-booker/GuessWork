#pragma once

#include <crow.h>

namespace gw::server {

class CameraRepository;
class CalibrationSupervisor;
class CameraSupervisor;

void register_calibration_routes(crow::SimpleApp&       app,
                                 CameraRepository&      repo,
                                 CalibrationSupervisor& calib,
                                 CameraSupervisor&      supervisor);

}  // namespace gw::server
