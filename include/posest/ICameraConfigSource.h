#pragma once

#include <vector>

#include "posest/CameraConfig.h"

namespace posest {

class ICameraConfigSource {
public:
    virtual ~ICameraConfigSource() = default;
    virtual std::vector<CameraConfig> loadAll() const = 0;
};

}  // namespace posest
