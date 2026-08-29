#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "core/imu_types.hpp"
#include "core/measurement_bus.hpp"
#include "producer/pulse_stamper.hpp"

namespace gw::server {

// Owns the one-port USB connection to the MicoAir F405 V2 sync controller.
// The controller is the timestamp authority for camera trigger events and the
// onboard BMI088. Configuration remains host-authoritative and is atomically
// re-pushed after every reconnect.
class SyncControllerManager : public gw::IPulseStamper {
public:
    struct GroupConfig {
        std::string          name;
        double               fps;
        std::vector<uint8_t> output_pins;  // stable logical outputs 1..6
    };

    struct Status {
        bool                       connected = false;
        std::optional<std::string> port;
        std::optional<std::string> board;
        bool                       armed = false;
        std::optional<int64_t>     last_pulse_age_ms;
        uint64_t                   total_pulses = 0;
        std::optional<std::string> last_error;
        std::optional<int>         firmware_version;
        std::optional<int>         protocol_version;
        uint32_t                   reset_reason = 0;

        bool                   imu_ok = false;
        double                 imu_rate_hz = 0.0;
        std::optional<int64_t> imu_last_sample_age_ms;
        uint64_t               imu_samples = 0;
        uint64_t               imu_fw_drops = 0;
        uint64_t               imu_crc_errors = 0;
        uint64_t               trigger_fw_drops = 0;
        uint64_t               usb_errors = 0;

        bool     sync_healthy = false;
        double   sync_offset_us = 0.0;  // host - controller
        double   sync_drift_ppm = 0.0;
        uint64_t sync_samples = 0;
        uint64_t sync_resets = 0;
    };

    explicit SyncControllerManager(
        std::string device_glob = "/dev/cu.usbmodem*");
    ~SyncControllerManager();

    SyncControllerManager(const SyncControllerManager&) = delete;
    SyncControllerManager& operator=(const SyncControllerManager&) = delete;

    void start();
    void stop();
    Status status() const;

    bool push_config(const std::vector<GroupConfig>& groups, std::string& err);
    bool stop_outputs(std::string& err);
    bool clear_config(std::string& err);
    bool test_output(uint8_t output_pin, std::string& err);

    uint64_t pop_pulse_ns(uint8_t output_pin,
                          uint64_t camera_frame_id) override;
    void reset_pin_state(uint8_t output_pin) override;

    gw::MeasurementBus<gw::ImuSample>& imu_bus();

    std::optional<uint64_t> host_to_controller_ns(uint64_t host_ns) const;
    std::optional<uint64_t> controller_to_host_ns(uint64_t controller_ns) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace gw::server
