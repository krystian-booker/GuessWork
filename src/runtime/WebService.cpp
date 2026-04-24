#include "posest/runtime/WebService.h"

#include <utility>

namespace posest::runtime {

WebService::WebService(config::IConfigStore& config_store)
    : config_store_(config_store) {}

RuntimeConfig WebService::getConfig() const {
    return config_store_.loadRuntimeConfig();
}

void WebService::stageConfig(RuntimeConfig config) {
    config_store_.saveRuntimeConfig(config);
}

TelemetrySnapshot WebService::telemetry() const {
    return telemetry_;
}

void WebService::updateTelemetry(TelemetrySnapshot snapshot) {
    telemetry_ = std::move(snapshot);
}

}  // namespace posest::runtime
