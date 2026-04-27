#pragma once

#include <functional>

#include "posest/config/IConfigStore.h"
#include "posest/runtime/TelemetrySnapshot.h"

namespace posest::runtime {

class WebService final {
public:
    // F-1: on_saved fires after a successful saveRuntimeConfig. Daemon
    // installs a callback that hands the new config to FusionService (and
    // future subsystems) so live-reloadable knobs take effect without a
    // restart. The callback is invoked synchronously on the saving thread —
    // implementations must return quickly and not block the web request.
    using ConfigSavedCallback = std::function<void(const RuntimeConfig&)>;

    explicit WebService(
        config::IConfigStore& config_store,
        ConfigSavedCallback on_saved = {});

    RuntimeConfig getConfig() const;
    void stageConfig(RuntimeConfig config);

    TelemetrySnapshot telemetry() const;
    void updateTelemetry(TelemetrySnapshot snapshot);

private:
    config::IConfigStore& config_store_;
    ConfigSavedCallback on_saved_;
    TelemetrySnapshot telemetry_;
};

}  // namespace posest::runtime
