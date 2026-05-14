#pragma once

#include <crow.h>

namespace gw::server {

class CameraRepository;
class CalibrationSupervisor;

void register_calibration_routes(crow::SimpleApp&       app,
                                 CameraRepository&      repo,
                                 CalibrationSupervisor& calib);

}  // namespace gw::server
