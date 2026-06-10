#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace gw::server {

class Database;

struct Camera {
    int64_t                    id         = 0;
    std::string                name;
    std::string                serial;
    double                     focal_length_mm = 0.0;  // lens focal length; drives Kalibr focal hint + model
    std::optional<std::string> mode;       // GenICam VideoMode symbolic, or unset
    // Live-tunable settings. Each unset (nullopt) means "use camera default" —
    // we don't touch the corresponding GenICam node at start.
    std::optional<bool>        gain_auto;
    std::optional<double>      gain;
    std::optional<bool>        exposure_auto;
    std::optional<double>      exposure;
    // Last uploaded Kalibr camchain YAML and the wall-clock second at which
    // it was stored. Both unset = uncalibrated.
    std::optional<std::string> calibration_json;
    std::optional<int64_t>     calibrated_at;
    // Hardware-trigger ("slave") mode. When true, the producer configures the
    // camera to fire on a rising edge on Line0 / OPTO_IN and routes its frame
    // timestamps from the matching Teensy output's pulse stream.
    bool                       hardware_sync_enabled = false;
    // Physical Teensy output the camera is wired to (1..6). Required when
    // hardware_sync_enabled is true; ignored otherwise. Unique across cameras.
    std::optional<int64_t>     trigger_output_pin;
    // Pipeline role: 'apriltag' | 'vio_left' | 'vio_right'. Unset = camera is
    // registered but not wired to a detection/VIO consumer.
    std::optional<std::string> role;
    // The camera's block of a Kalibr camchain-imucam result (cam0-keyed
    // YAML: intrinsics + T_cam_imu + timeshift, T_cn_cnm1 where present).
    std::optional<std::string> imu_extrinsics_json;
    std::optional<int64_t>     extrinsics_calibrated_at;
    int64_t                    created_at = 0;  // unix seconds
};

// Partial update payload. All fields are optional; an entirely-empty struct is
// a no-op. The repository preserves any field left at nullopt.
struct CameraUpdate {
    std::optional<std::string> name;
    std::optional<double>      focal_length_mm;
    std::optional<std::string> mode;
    std::optional<bool>        gain_auto;
    std::optional<double>      gain;
    std::optional<bool>        exposure_auto;
    std::optional<double>      exposure;
    std::optional<bool>        hardware_sync_enabled;
    // Nested optional: outer = "in patch", inner = "value (nullopt → SQL NULL)".
    // The route layer clears the pin (inner nullopt) when hw-sync is disabled.
    std::optional<std::optional<int64_t>> trigger_output_pin;
    // Same nested-optional pattern: inner nullopt clears the role.
    std::optional<std::optional<std::string>> role;

    bool empty() const {
        return !name && !focal_length_mm && !mode
            && !gain_auto && !gain && !exposure_auto && !exposure
            && !hardware_sync_enabled && !trigger_output_pin && !role;
    }
};

// Thrown when a write violates the UNIQUE(name) constraint. Route layer maps
// this to HTTP 409 Conflict.
class DuplicateNameError : public std::runtime_error {
public:
    explicit DuplicateNameError(const std::string& name)
        : std::runtime_error("camera name already exists: " + name) {}
};

// Thrown when an insert violates the UNIQUE index on `serial`. The route layer
// maps this to HTTP 409 Conflict.
class DuplicateSerialError : public std::runtime_error {
public:
    explicit DuplicateSerialError(const std::string& serial)
        : std::runtime_error("camera serial already mapped: " + serial) {}
};

// Thrown when a write would assign a trigger output pin already claimed by
// another camera (UNIQUE(trigger_output_pin) violation). Maps to HTTP 409.
class DuplicateTriggerOutputPinError : public std::runtime_error {
public:
    explicit DuplicateTriggerOutputPinError(int64_t pin)
        : std::runtime_error("trigger_output_pin already in use: " + std::to_string(pin)) {}
};

class CameraRepository {
public:
    explicit CameraRepository(Database& db) : db_(db) {}

    std::vector<Camera>   list_all();
    std::optional<Camera> get(int64_t id);
    std::optional<Camera> find_by_serial(std::string_view serial);

    // Throws DuplicateNameError, DuplicateSerialError, or
    // DuplicateTriggerOutputPinError on UNIQUE conflict. focal_length_mm is
    // required (a positive lens focal length in mm, typically 1–50); range
    // validation lives in the route layer. hardware_sync_enabled defaults to
    // false; trigger_output_pin is ignored unless hw-sync is on.
    Camera                create(std::string_view                name,
                                 std::string_view                serial,
                                 double                          focal_length_mm,
                                 std::optional<std::string_view> mode = std::nullopt,
                                 bool                            hardware_sync_enabled = false,
                                 std::optional<int64_t>          trigger_output_pin = std::nullopt);

    // Partial update. Any field of CameraUpdate may be nullopt to leave it
    // unchanged; an entirely-empty patch returns the row as-is. Serial cannot
    // be changed. Returns the updated row, or std::nullopt if id does not
    // exist. Throws DuplicateNameError on UNIQUE conflict.
    std::optional<Camera> update(int64_t id, const CameraUpdate& patch);

    // Returns true if a row was deleted.
    bool                  remove(int64_t id);

    // Stores the intrinsic calibration text (Kalibr camchain YAML) against
    // the camera row and stamps calibrated_at with the current unix second.
    // Returns the updated row, or nullopt if id doesn't exist.
    std::optional<Camera> set_calibration(int64_t id, std::string_view text);

    // Clears both calibration_json and calibrated_at. Returns true if a row was
    // touched; false if id doesn't exist.
    bool                  clear_calibration(int64_t id);

    // Stores the camera's IMU-extrinsics block (cam0-keyed camchain-imucam
    // YAML) and stamps extrinsics_calibrated_at with the current unix second.
    // Returns the updated row, or nullopt if id doesn't exist.
    std::optional<Camera> set_imu_extrinsics(int64_t id, std::string_view text);

    // Clears imu_extrinsics_json and extrinsics_calibrated_at. Returns true
    // if a row was touched.
    bool                  clear_imu_extrinsics(int64_t id);

private:
    Database& db_;
};

}  // namespace gw::server
