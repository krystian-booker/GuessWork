#pragma once

#include <cstddef>
#include <cstdint>

#include "FirmwareConfig.h"
#include "Protocol.h"

namespace posest::firmware {

class CameraTriggerScheduler final {
public:
    void begin();
    void apply(
        const CameraTriggerCommand* commands,
        std::size_t count,
        std::uint32_t now_us);
    void update(std::uint32_t now_us);
    void disableAll();

    std::uint32_t statusFlags() const { return status_flags_; }

private:
    struct Channel {
        bool enabled{false};
        bool high{false};
        std::uint8_t pin{0};
        std::uint32_t period_us{0};
        std::uint32_t pulse_width_us{0};
        std::uint32_t next_rise_us{0};
        std::uint32_t fall_us{0};
    };

    static bool isAllowedPin(std::int32_t pin);
    static bool due(std::uint32_t now_us, std::uint32_t target_us);
    static std::uint32_t normalizePhase(std::int64_t phase_offset_us, std::uint32_t period_us);
    static std::uint32_t periodFromRate(double rate_hz);

    Channel channels_[kMaxCameraSyncOutputs]{};
    std::uint32_t status_flags_{0};
};

}  // namespace posest::firmware
