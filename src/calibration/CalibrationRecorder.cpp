#include "posest/calibration/CalibrationRecorder.h"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>

namespace posest::calibration {

namespace {

std::filesystem::path imagePath(
    const std::filesystem::path& output_dir,
    const std::string& camera_id,
    std::uint64_t sequence) {
    return output_dir / "images" / (camera_id + "_" + std::to_string(sequence) + ".png");
}

}  // namespace

CalibrationRecorder::CalibrationRecorder(CalibrationRecorderConfig config)
    : config_(std::move(config)) {
    if (config_.output_dir.empty()) {
        throw std::invalid_argument("CalibrationRecorder requires an output directory");
    }
    for (const auto& trigger : config_.camera_triggers) {
        if (trigger.enabled) {
            camera_to_pin_[trigger.camera_id] = trigger.teensy_pin;
        }
    }
}

CalibrationRecorder::~CalibrationRecorder() {
    stop();
}

void CalibrationRecorder::start() {
    std::lock_guard<std::mutex> g(mu_);
    if (started_) {
        return;
    }
    std::filesystem::create_directories(config_.output_dir / "images");
    frames_csv_.open(config_.output_dir / "frames.csv");
    imu_csv_.open(config_.output_dir / "imu.csv");
    trigger_csv_.open(config_.output_dir / "trigger_events.csv");
    if (!frames_csv_ || !imu_csv_ || !trigger_csv_) {
        throw std::runtime_error("failed to open calibration dataset CSV files");
    }
    frames_csv_ << "camera_id,frame_sequence,image_path,ros_timestamp_us,"
                   "capture_timestamp_us,trigger_timestamp_us,trigger_sequence,"
                   "trigger_pin,matched\n";
    imu_csv_ << "timestamp_us,accel_x_mps2,accel_y_mps2,accel_z_mps2,"
                "gyro_x_radps,gyro_y_radps,gyro_z_radps,temperature_c,status_flags\n";
    trigger_csv_ << "timestamp_us,teensy_time_us,pin,trigger_sequence,status_flags\n";
    stop_requested_ = false;
    started_ = true;
    worker_ = std::thread(&CalibrationRecorder::workerLoop, this);
}

void CalibrationRecorder::stop() {
    {
        std::lock_guard<std::mutex> g(mu_);
        if (!started_) {
            return;
        }
        stop_requested_ = true;
        cv_.notify_all();
    }
    if (worker_.joinable()) {
        worker_.join();
    }
    writeSessionJson();
    std::lock_guard<std::mutex> g(mu_);
    started_ = false;
    frames_csv_.close();
    imu_csv_.close();
    trigger_csv_.close();
}

void CalibrationRecorder::deliver(FramePtr frame) {
    if (!frame || !selectedCamera(frame->camera_id)) {
        return;
    }
    enqueue(std::move(frame));
}

bool CalibrationRecorder::publish(AprilTagObservation /*observation*/) {
    return true;
}

bool CalibrationRecorder::publish(VioMeasurement /*measurement*/) {
    return true;
}

bool CalibrationRecorder::publish(ImuSample sample) {
    enqueue(std::move(sample));
    return true;
}

bool CalibrationRecorder::publish(WheelOdometrySample /*sample*/) {
    return true;
}

bool CalibrationRecorder::publish(RobotOdometrySample /*sample*/) {
    return true;
}

bool CalibrationRecorder::publish(CameraTriggerEvent event) {
    enqueue(std::move(event));
    return true;
}

CalibrationRecorderStats CalibrationRecorder::stats() const {
    std::lock_guard<std::mutex> g(mu_);
    return stats_;
}

void CalibrationRecorder::throwIfUnacceptable() const {
    const auto snapshot = stats();
    if (snapshot.frames_seen == 0) {
        throw std::runtime_error("calibration recording captured no frames");
    }
    const double matched_fraction =
        static_cast<double>(snapshot.frames_recorded) /
        static_cast<double>(snapshot.frames_seen);
    if (matched_fraction < config_.min_trigger_match_fraction) {
        throw std::runtime_error("calibration recording did not match enough frames to triggers");
    }
}

void CalibrationRecorder::workerLoop() {
    while (true) {
        Event event;
        {
            std::unique_lock<std::mutex> lock(mu_);
            cv_.wait(lock, [&] { return stop_requested_ || !queue_.empty(); });
            if (queue_.empty()) {
                if (stop_requested_) {
                    return;
                }
                continue;
            }
            event = std::move(queue_.front());
            queue_.pop_front();
        }

        std::visit(
            [this](const auto& value) {
                using T = std::decay_t<decltype(value)>;
                if constexpr (std::is_same_v<T, FramePtr>) {
                    if (value) processFrame(*value);
                } else if constexpr (std::is_same_v<T, ImuSample>) {
                    processImu(value);
                } else if constexpr (std::is_same_v<T, CameraTriggerEvent>) {
                    processTrigger(value);
                }
            },
            event);
    }
}

void CalibrationRecorder::processFrame(const Frame& frame) {
    TriggerMatch match;
    {
        std::lock_guard<std::mutex> g(mu_);
        ++stats_.frames_seen;
        match = matchTrigger(frame);
        if (!match.matched) {
            ++stats_.frames_without_trigger;
        }
    }

    const auto path = imagePath(config_.output_dir, frame.camera_id, frame.sequence);
    cv::imwrite(path.string(), frame.image);

    const std::uint64_t capture_us = steadyMicros(frame.capture_time);
    const std::uint64_t trigger_us = match.matched ? steadyMicros(match.event.timestamp) : 0u;
    const std::uint64_t ros_us = match.matched
        ? static_cast<std::uint64_t>(
              static_cast<std::int64_t>(trigger_us) + config_.trigger_to_exposure_center_us)
        : capture_us;

    std::lock_guard<std::mutex> g(mu_);
    frames_csv_ << frame.camera_id << "," << frame.sequence << ","
                << std::filesystem::relative(path, config_.output_dir).string() << ","
                << ros_us << "," << capture_us << "," << trigger_us << ","
                << (match.matched ? match.event.trigger_sequence : 0u) << ","
                << (match.matched ? match.event.pin : -1) << ","
                << (match.matched ? 1 : 0) << "\n";
    if (match.matched) {
        ++stats_.frames_recorded;
    }
}

void CalibrationRecorder::processImu(const ImuSample& sample) {
    std::lock_guard<std::mutex> g(mu_);
    imu_csv_ << steadyMicros(sample.timestamp) << ","
             << sample.accel_mps2.x << "," << sample.accel_mps2.y << ","
             << sample.accel_mps2.z << "," << sample.gyro_radps.x << ","
             << sample.gyro_radps.y << "," << sample.gyro_radps.z << ","
             << sample.temperature_c.value_or(0.0) << "," << sample.status_flags << "\n";
    ++stats_.imu_samples_recorded;
}

void CalibrationRecorder::processTrigger(const CameraTriggerEvent& event) {
    std::lock_guard<std::mutex> g(mu_);
    latest_trigger_by_pin_[event.pin] = event;
    trigger_csv_ << steadyMicros(event.timestamp) << "," << event.teensy_time_us << ","
                 << event.pin << "," << event.trigger_sequence << ","
                 << event.status_flags << "\n";
    ++stats_.trigger_events_recorded;
}

CalibrationRecorder::TriggerMatch CalibrationRecorder::matchTrigger(const Frame& frame) const {
    const auto pin_it = camera_to_pin_.find(frame.camera_id);
    if (pin_it == camera_to_pin_.end()) {
        return {};
    }
    const auto trigger_it = latest_trigger_by_pin_.find(pin_it->second);
    if (trigger_it == latest_trigger_by_pin_.end()) {
        return {};
    }
    if (trigger_it->second.timestamp > frame.capture_time) {
        return {};
    }
    return {true, trigger_it->second};
}

void CalibrationRecorder::writeSessionJson() const {
    const auto snapshot = stats();
    nlohmann::json session = {
        {"duration_s", config_.duration_s},
        {"camera_ids", config_.camera_ids},
        {"trigger_to_exposure_center_us", config_.trigger_to_exposure_center_us},
        {"frames_seen", snapshot.frames_seen},
        {"frames_recorded", snapshot.frames_recorded},
        {"frames_without_trigger", snapshot.frames_without_trigger},
        {"imu_samples_recorded", snapshot.imu_samples_recorded},
        {"trigger_events_recorded", snapshot.trigger_events_recorded},
    };
    std::ofstream out(config_.output_dir / "session.json");
    out << session.dump(2) << "\n";
}

bool CalibrationRecorder::selectedCamera(const std::string& camera_id) const {
    return config_.camera_ids.empty() ||
           std::find(config_.camera_ids.begin(), config_.camera_ids.end(), camera_id) !=
               config_.camera_ids.end();
}

void CalibrationRecorder::enqueue(Event event) {
    {
        std::lock_guard<std::mutex> g(mu_);
        if (!started_ || stop_requested_) {
            return;
        }
        queue_.push_back(std::move(event));
    }
    cv_.notify_one();
}

std::uint64_t CalibrationRecorder::steadyMicros(Timestamp timestamp) {
    const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
        timestamp.time_since_epoch());
    return static_cast<std::uint64_t>(micros.count());
}

}  // namespace posest::calibration
