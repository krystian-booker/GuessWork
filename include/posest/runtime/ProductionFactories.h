#pragma once

#include <cstdint>
#include <functional>
#include <memory>

#include "posest/MeasurementBus.h"
#include "posest/Timestamp.h"
#include "posest/runtime/Factories.h"

namespace posest::runtime {

class ProductionCameraFactory final : public ICameraBackendFactory {
public:
    std::shared_ptr<IFrameProducer> createCamera(const CameraConfig& config) override;
    std::vector<CameraCapabilities> enumerateAvailable() override;
};

class ProductionPipelineFactory final : public IPipelineFactory {
public:
    using TeensyTimeConverter =
        std::function<Timestamp(std::uint64_t teensy_time_us, Timestamp fallback)>;

    std::shared_ptr<IVisionPipeline> createPipeline(
        const PipelineConfig& config,
        IMeasurementSink& measurement_sink) override;
    std::shared_ptr<IVisionPipeline> createPipeline(
        const PipelineConfig& config,
        IMeasurementSink& measurement_sink,
        const RuntimeConfig& runtime_config) override;

    // Provide the dependencies the "vio" pipeline branch needs. The
    // daemon calls this once after constructing the dedicated VIO IMU
    // bus and the TeensyService time converter. Tests that build a
    // "vio" pipeline must call this with stub values; absent context,
    // createPipeline throws when it sees type=="vio".
    void setVioContext(MeasurementBus& imu_vio_bus,
                       TeensyTimeConverter time_converter);

private:
    MeasurementBus* imu_vio_bus_{nullptr};
    TeensyTimeConverter time_converter_{};
};

}  // namespace posest::runtime
