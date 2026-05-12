#pragma once

#include <cstdint>

namespace gw {

// Monotonic host clock based on mach_absolute_time, exposed as nanoseconds.
// Used by producers to stamp host_capture_ns and by consumers/probes for latency.
class Clock {
public:
    static uint64_t now_ns();
};

}  // namespace gw
