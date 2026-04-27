#include "posest/vio/AirborneCovariance.h"

#include <algorithm>

namespace posest::vio {

AirborneState AirborneTracker::update(
    std::optional<double> ground_distance_m,
    std::chrono::steady_clock::time_point now) {
    if (!ground_distance_m.has_value()) {
        ++missing_count_;
        // No reading: hold current state. We still let an in-flight
        // settle timer expire so a dropout at the tail end of a landing
        // doesn't pin us in kSettling forever.
        if (state_ == AirborneState::kSettling && now >= settle_deadline_) {
            state_ = AirborneState::kGrounded;
        }
        return state_;
    }

    const double d = *ground_distance_m;

    if (d > thresholds_.above_m) {
        state_ = AirborneState::kAirborne;
        return state_;
    }

    // Below `above_m`. If we were airborne and just crossed below the
    // tighter `below_m` threshold, start the settle timer. Otherwise stay
    // in whatever state we were (hysteresis: a sample in the [below_m,
    // above_m] band does not by itself land us).
    if (state_ == AirborneState::kAirborne && d < thresholds_.below_m) {
        state_ = AirborneState::kSettling;
        settle_deadline_ = now + thresholds_.settle;
        return state_;
    }

    if (state_ == AirborneState::kSettling) {
        if (now >= settle_deadline_) {
            state_ = AirborneState::kGrounded;
        }
        // Else: still settling, regardless of current distance.
        return state_;
    }

    // kGrounded with a sample in either the hysteresis band or below
    // `below_m`: stay grounded. Nothing to do.
    return state_;
}

std::array<double, 36> inflate(const std::array<double, 36>& covariance,
                               AirborneState state,
                               double inflation_factor,
                               double cap) {
    if (state == AirborneState::kGrounded) {
        return covariance;
    }
    std::array<double, 36> out = covariance;
    for (int i = 0; i < 6; ++i) {
        const std::size_t idx = static_cast<std::size_t>(i * 6 + i);
        const double scaled = out[idx] * inflation_factor;
        out[idx] = std::min(scaled, cap);
    }
    return out;
}

}  // namespace posest::vio
