#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <variant>
#include <vector>

#include "posest/IFrameConsumer.h"
#include "posest/MeasurementBus.h"
#include "posest/runtime/RuntimeConfig.h"

namespace posest::calibration {

struct CalibrationRecorderConfig {
    std::filesystem::path output_dir;
    std::vector<std::string> camera_ids;
    std::vector<runtime::CameraTriggerConfig> camera_triggers;
    double duration_s{0.0};
    std::int64_t trigger_to_exposure_center_us{0};
    double min_trigger_match_fraction{1.0};
};

struct CalibrationRecorderStats {
    std::uint64_t frames_seen{0};
    std::uint64_t frames_recorded{0};
    std::uint64_t frames_without_trigger{0};
    std::uint64_t imu_samples_recorded{0};
    std::uint64_t trigger_events_recorded{0};
};

class CalibrationRecorder final : public IFrameConsumer, public IMeasurementSink {
public:
    explicit CalibrationRecorder(CalibrationRecorderConfig config);
    ~CalibrationRecorder() override;

    CalibrationRecorder(const CalibrationRecorder&) = delete;
    CalibrationRecorder& operator=(const CalibrationRecorder&) = delete;

    const std::string& id() const override { return id_; }
    void start() override;
    void stop() override;
    void deliver(FramePtr frame) override;

    bool publish(AprilTagObservation observation) override;
    bool publish(VioMeasurement measurement) override;
    bool publish(ImuSample sample) override;
    bool publish(ChassisSpeedsSample sample) override;
    bool publish(CameraTriggerEvent event) override;

    CalibrationRecorderStats stats() const;
    void throwIfUnacceptable() const;

private:
    using Event = std::variant<FramePtr, ImuSample, CameraTriggerEvent>;

    struct TriggerMatch {
        bool matched{false};
        CameraTriggerEvent event;
    };

    void workerLoop();
    void processFrame(const Frame& frame);
    void processImu(const ImuSample& sample);
    void processTrigger(const CameraTriggerEvent& event);
    TriggerMatch matchTrigger(const Frame& frame) const;
    void writeSessionJson() const;
    bool selectedCamera(const std::string& camera_id) const;
    void enqueue(Event event);

    static std::uint64_t steadyMicros(Timestamp timestamp);

    std::string id_{"calibration_recorder"};
    CalibrationRecorderConfig config_;
    std::unordered_map<std::string, std::int32_t> camera_to_pin_;
    mutable std::mutex mu_;
    std::condition_variable cv_;
    std::deque<Event> queue_;
    std::unordered_map<std::int32_t, CameraTriggerEvent> latest_trigger_by_pin_;
    std::thread worker_;
    std::ofstream frames_csv_;
    std::ofstream imu_csv_;
    std::ofstream trigger_csv_;
    CalibrationRecorderStats stats_;
    bool started_{false};
    bool stop_requested_{false};
};

}  // namespace posest::calibration
