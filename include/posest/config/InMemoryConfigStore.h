#pragma once

#include <mutex>

#include "posest/config/IConfigStore.h"

namespace posest::config {

class InMemoryConfigStore final : public IConfigStore {
public:
    explicit InMemoryConfigStore(runtime::RuntimeConfig config = {});

    runtime::RuntimeConfig loadRuntimeConfig() const override;
    void saveRuntimeConfig(const runtime::RuntimeConfig& config) override;

private:
    mutable std::mutex mu_;
    runtime::RuntimeConfig config_;
};

}  // namespace posest::config
