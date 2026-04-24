#include "posest/config/InMemoryConfigStore.h"

#include <utility>

namespace posest::config {

InMemoryConfigStore::InMemoryConfigStore(runtime::RuntimeConfig config)
    : config_(std::move(config)) {}

runtime::RuntimeConfig InMemoryConfigStore::loadRuntimeConfig() const {
    std::lock_guard<std::mutex> g(mu_);
    return config_;
}

void InMemoryConfigStore::saveRuntimeConfig(const runtime::RuntimeConfig& config) {
    std::lock_guard<std::mutex> g(mu_);
    config_ = config;
}

}  // namespace posest::config
