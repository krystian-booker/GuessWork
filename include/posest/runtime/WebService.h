#pragma once

#include "posest/config/IConfigStore.h"
#include "posest/runtime/TelemetrySnapshot.h"

namespace posest::runtime {

class WebService final {
public:
    explicit WebService(config::IConfigStore& config_store);

    RuntimeConfig getConfig() const;
    void stageConfig(RuntimeConfig config);

    TelemetrySnapshot telemetry() const;
    void updateTelemetry(TelemetrySnapshot snapshot);

private:
    config::IConfigStore& config_store_;
    TelemetrySnapshot telemetry_;
};

}  // namespace posest::runtime
