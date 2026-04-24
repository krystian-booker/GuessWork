#pragma once

#include "posest/MeasurementTypes.h"

namespace posest::fusion {

class IFusionOutputSink {
public:
    virtual ~IFusionOutputSink() = default;
    virtual void publish(FusedPoseEstimate estimate) = 0;
};

}  // namespace posest::fusion
