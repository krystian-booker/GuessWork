#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "posest/ProducerBase.h"

namespace posest::mock {

struct MockProducerConfig {
    std::string id = "mock_producer";
    int width = 320;
    int height = 240;
    double target_fps = 60.0;
    // When true (default), the producer supplies its own capture_time (the
    // scheduled deadline of the frame), exercising the subclass-timestamp
    // path and giving tests a deterministic reference. When false, captureOne
    // leaves the timestamp unset so the base falls back to steady_clock::now().
    bool supply_timestamp = true;
};

// Deterministic synthetic frame source. Paces itself with sleep_until to avoid
// drift. The image is a single-channel Mat whose pixels are filled with a
// value derived from the frame index, so consumers can sanity-check the
// content; the authoritative sequence number lives on the Frame struct.
class MockProducer final : public ProducerBase {
public:
    explicit MockProducer(MockProducerConfig cfg);
    // Required by the ProducerBase contract: stop the capture thread before
    // this leaf's members go out of scope (~ProducerBase aborts otherwise).
    ~MockProducer() override { stop(); }

protected:
    bool captureOne(
        cv::Mat& out,
        std::optional<std::chrono::steady_clock::time_point>& out_capture_time) override;

private:
    MockProducerConfig cfg_;
    std::chrono::steady_clock::duration period_;
    std::chrono::steady_clock::time_point next_deadline_{};
    bool first_{true};
    std::uint64_t frame_idx_{0};
};

}  // namespace posest::mock
