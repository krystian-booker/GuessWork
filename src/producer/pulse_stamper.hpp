#pragma once

#include <cstdint>

namespace gw {

// Abstraction the SpinnakerProducer uses to source per-frame timestamps when
// the camera is in hardware-trigger slave mode. The implementation
// (`gw::server::SyncControllerManager`) lives in the server layer; this interface is
// defined in `gw::` so the producer library doesn't pull in the server layer.
//
// Contract:
//   - `pop_pulse_ns(pin, camera_frame_id)` returns the rising-edge timestamp
//     (in nanoseconds on the source clock) for the next-expected pulse on
//     the given sync controller output pin. The implementation tracks per-pin state
//     so subsequent calls advance through the pulse stream; gaps in
//     camera_frame_id cause matching pulses to be discarded so the streams
//     stay aligned across dropped camera frames.
//   - Returns 0 when no pulse is currently available (e.g. the trigger
//     device isn't connected yet, or the producer outran the pulse stream).
//     The caller should fall back to the camera's own timestamp.
//   - `reset_pin_state(pin)` is called when a producer (re)arms a hw-sync
//     camera so the next frame seeds a fresh baseline.
class IPulseStamper {
public:
    virtual ~IPulseStamper() = default;
    virtual uint64_t pop_pulse_ns(uint8_t pin, uint64_t camera_frame_id) = 0;
    virtual void     reset_pin_state(uint8_t pin) = 0;
};

}  // namespace gw
