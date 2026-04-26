#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace posest::teensy {

// Windowed filter for host <-> Teensy time sync samples.
//
// Each Sample is one host-side measurement of (teensy_midpoint, offset,
// round_trip). The filter:
//   - Bootstraps with the first kBootstrap samples accepted unconditionally so
//     the round-trip baseline can be measured.
//   - Once bootstrapped, rejects any sample whose round_trip exceeds
//     `min_rtt * 2 + 1 ms` (with a 1 ms floor so very-low-RTT links don't gate
//     themselves out).
//   - Maintains an EMA over accepted offsets with alpha = 1/8.
//   - Maintains a linear regression slope over (teensy_midpoint, offset) on
//     the most recent accepted samples; the slope is exposed as skew.
//
// `apply(teensy_time_us)` returns `teensy_time_us + ema_offset + skew * (t -
// anchor)` so the host can interpolate between sync responses without the
// offset jumping discretely.
class TimeSyncFilter final {
public:
    struct Sample {
        std::int64_t teensy_midpoint_us{0};
        std::int64_t offset_us{0};
        std::uint64_t round_trip_us{0};
    };

    static constexpr std::size_t kRingSize = 9;
    static constexpr std::size_t kBootstrap = 3;
    static constexpr std::uint64_t kMinRttFloorUs = 1000;

    bool update(const Sample& sample);

    bool established() const { return established_; }
    std::int64_t offsetUs() const { return offset_ema_us_; }
    double skewPerUs() const { return skew_per_us_; }
    double skewPpm() const { return skew_per_us_ * 1.0e6; }
    std::int64_t anchorTeensyUs() const { return anchor_teensy_us_; }
    std::uint64_t accepted() const { return accepted_; }
    std::uint64_t rejected() const { return rejected_; }
    std::uint64_t lastRoundTripUs() const { return last_rtt_us_; }

    std::int64_t apply(std::int64_t teensy_time_us) const;

    void reset();

private:
    void recomputeRegression();

    std::array<Sample, kRingSize> samples_{};
    std::size_t count_{0};
    std::size_t head_{0};
    std::int64_t offset_ema_us_{0};
    std::int64_t anchor_teensy_us_{0};
    double skew_per_us_{0.0};
    std::uint64_t accepted_{0};
    std::uint64_t rejected_{0};
    std::uint64_t last_rtt_us_{0};
    bool established_{false};
};

}  // namespace posest::teensy
