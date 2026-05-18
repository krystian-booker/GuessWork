#pragma once

#include <crow.h>

namespace gw::server {

class TriggerGroupRepository;
class TeensyManager;

void register_hardware_sync_routes(crow::SimpleApp&        app,
                                   TriggerGroupRepository& repo,
                                   TeensyManager&          teensy);

}  // namespace gw::server
