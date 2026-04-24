#pragma once

#include "posest/runtime/RuntimeConfig.h"

namespace posest::config {

class IConfigStore {
public:
    virtual ~IConfigStore() = default;

    virtual runtime::RuntimeConfig loadRuntimeConfig() const = 0;
    virtual void saveRuntimeConfig(const runtime::RuntimeConfig& config) = 0;
};

}  // namespace posest::config
