#pragma once

#include <stdint.h>

#include "sync_controller_protocol.h"

namespace gw_fw {

class Bmi088 {
public:
    bool begin();
    bool poll(gw_sync::ImuRecord& sample);
    bool ok() const { return ok_; }
    uint32_t sample_count() const { return sample_count_; }
    uint32_t drop_count() const { return drop_count_; }

private:
    bool ok_ = false;
    uint32_t next_sample_us_ = 0;
    uint32_t sample_count_ = 0;
    uint32_t drop_count_ = 0;
};

}  // namespace gw_fw

