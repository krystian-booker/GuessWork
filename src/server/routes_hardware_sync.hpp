#pragma once

#include <crow.h>

namespace gw::server {

class TriggerGroupRepository;
class SyncControllerManager;

void register_hardware_sync_routes(crow::SimpleApp&        app,
                                   TriggerGroupRepository& repo,
                                   SyncControllerManager&  controller);

}  // namespace gw::server
