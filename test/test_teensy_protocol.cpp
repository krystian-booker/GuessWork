#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "posest/teensy/Protocol.h"

TEST(TeensyProtocol, EncodesAndDecodesFrameRoundTrip) {
    posest::teensy::Frame frame;
    frame.type = posest::teensy::MessageType::ImuSample;
    frame.sequence = 42;
    frame.payload = {1, 2, 3, 4};

    const auto bytes = posest::teensy::encodeFrame(frame);
    const auto decoded = posest::teensy::decodeFrame(bytes);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->frame.type, frame.type);
    EXPECT_EQ(decoded->frame.sequence, frame.sequence);
    EXPECT_EQ(decoded->frame.payload, frame.payload);
    EXPECT_EQ(decoded->bytes_consumed, bytes.size());
}

TEST(TeensyProtocol, RejectsCorruptCrc) {
    posest::teensy::Frame frame;
    frame.type = posest::teensy::MessageType::CanRx;
    frame.payload = {9, 8, 7};

    auto bytes = posest::teensy::encodeFrame(frame);
    bytes.back() ^= 0xFFu;

    EXPECT_FALSE(posest::teensy::decodeFrame(bytes).has_value());
}

TEST(TeensyProtocol, StreamDecoderHandlesChunksAndSequenceGaps) {
    posest::teensy::Frame a;
    a.sequence = 1;
    a.payload = {1};
    posest::teensy::Frame b;
    b.sequence = 3;
    b.payload = {2};

    auto bytes_a = posest::teensy::encodeFrame(a);
    auto bytes_b = posest::teensy::encodeFrame(b);
    std::vector<std::uint8_t> combined;
    combined.insert(combined.end(), bytes_a.begin(), bytes_a.end());
    combined.insert(combined.end(), bytes_b.begin(), bytes_b.end());

    posest::teensy::StreamDecoder decoder;
    const auto frames = decoder.push(combined.data(), combined.size());

    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[0].sequence, 1u);
    EXPECT_EQ(frames[1].sequence, 3u);
    EXPECT_EQ(decoder.stats().sequence_gaps, 1u);
}
