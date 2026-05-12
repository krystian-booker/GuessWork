#include "core/clock.hpp"

#include <mach/mach_time.h>

namespace gw {

namespace {

mach_timebase_info_data_t load_timebase() {
    mach_timebase_info_data_t tb{};
    mach_timebase_info(&tb);
    return tb;
}

}  // namespace

uint64_t Clock::now_ns() {
    static const mach_timebase_info_data_t tb = load_timebase();
    const uint64_t                         t  = mach_absolute_time();
    return (t * tb.numer) / tb.denom;
}

}  // namespace gw
