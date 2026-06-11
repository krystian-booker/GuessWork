#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

#include "server/telemetry_decoder.hpp"

namespace gw::server {

namespace {

// Mirror of the firmware's framing (firmware/src/binary_proto.{h,cpp}) so
// the tests build golden frames independently of the decoder under test.
uint16_t crc16_ccitt(const uint8_t* data, size_t n) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

void put_u32(std::vector<uint8_t>& v, uint32_t x) {
    v.push_back(static_cast<uint8_t>(x));
    v.push_back(static_cast<uint8_t>(x >> 8));
    v.push_back(static_cast<uint8_t>(x >> 16));
    v.push_back(static_cast<uint8_t>(x >> 24));
}
void put_u64(std::vector<uint8_t>& v, uint64_t x) {
    put_u32(v, static_cast<uint32_t>(x));
    put_u32(v, static_cast<uint32_t>(x >> 32));
}
void put_f32(std::vector<uint8_t>& v, float f) {
    uint32_t bits;
    std::memcpy(&bits, &f, sizeof(bits));
    put_u32(v, bits);
}

std::vector<uint8_t> frame(uint8_t type, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> f{0xA5, 0x5A, type,
                           static_cast<uint8_t>(payload.size())};
    f.insert(f.end(), payload.begin(), payload.end());
    const uint16_t crc = crc16_ccitt(f.data() + 2, payload.size() + 2);
    f.push_back(static_cast<uint8_t>(crc));
    f.push_back(static_cast<uint8_t>(crc >> 8));
    return f;
}

std::vector<uint8_t> imu_batch_frame(
        const std::vector<std::pair<uint64_t, std::array<float, 6>>>& samples) {
    std::vector<uint8_t> payload;
    payload.push_back(static_cast<uint8_t>(samples.size()));
    for (const auto& [t_us, vals] : samples) {
        put_u64(payload, t_us);
        for (float v : vals) put_f32(payload, v);
    }
    return frame(0x01, payload);
}

std::vector<uint8_t> heartbeat_frame(uint64_t t_us, bool imu_ok,
                                     uint32_t samples, uint32_t drops) {
    std::vector<uint8_t> payload;
    put_u64(payload, t_us);
    payload.push_back(imu_ok ? 0x01 : 0x00);
    put_u32(payload, samples);
    put_u32(payload, drops);
    return frame(0x02, payload);
}

// fw=3 heartbeat: the fw=2 prefix plus the CAN extension (binary_proto.h).
std::vector<uint8_t> heartbeat_v3_frame(uint64_t t_us, uint8_t flags,
                                        uint32_t imu_samples, uint32_t imu_drops,
                                        uint32_t can_rx, uint32_t can_rx_drops,
                                        uint32_t odom_tx_drops, uint32_t pose_tx,
                                        uint8_t can_mode) {
    std::vector<uint8_t> payload;
    put_u64(payload, t_us);
    payload.push_back(flags);
    put_u32(payload, imu_samples);
    put_u32(payload, imu_drops);
    put_u32(payload, can_rx);
    put_u32(payload, can_rx_drops);
    put_u32(payload, odom_tx_drops);
    put_u32(payload, pose_tx);
    payload.push_back(can_mode);
    return frame(0x02, payload);
}

std::vector<uint8_t> odom_payload(uint64_t t_arrival_us, uint64_t rio_time_us,
                                  float vx, float vy, float omega,
                                  uint16_t status_flags, uint8_t counter,
                                  uint8_t mode) {
    std::vector<uint8_t> payload;
    put_u64(payload, t_arrival_us);
    put_u64(payload, rio_time_us);
    put_f32(payload, vx);
    put_f32(payload, vy);
    put_f32(payload, omega);
    payload.push_back(static_cast<uint8_t>(status_flags));
    payload.push_back(static_cast<uint8_t>(status_flags >> 8));
    payload.push_back(counter);
    payload.push_back(mode);
    return payload;
}

struct Collector {
    std::vector<gw::ImuSample>               imu;
    std::vector<TelemetryDecoder::Heartbeat> hb;
    std::vector<TelemetryDecoder::Odom>      odom;

    void attach(TelemetryDecoder& d) {
        d.on_imu       = [this](const gw::ImuSample& s) { imu.push_back(s); };
        d.on_heartbeat = [this](const TelemetryDecoder::Heartbeat& h) {
            hb.push_back(h);
        };
        d.on_odom = [this](const TelemetryDecoder::Odom& o) {
            odom.push_back(o);
        };
    }
};

}  // namespace

TEST(TelemetryDecoderTest, DecodesSingleImuBatch) {
    TelemetryDecoder d;
    Collector c;
    c.attach(d);

    const auto f = imu_batch_frame({
        {123456789ull, {1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f}},
        {123459289ull, {-9.81f, 0.0f, 0.5f, -0.1f, 0.0f, 2000.0f}},
    });
    d.feed(f.data(), f.size());

    ASSERT_EQ(c.imu.size(), 2u);
    EXPECT_EQ(c.imu[0].t_ns, 123456789ull * 1000);
    EXPECT_FLOAT_EQ(c.imu[0].accel[0], 1.0f);
    EXPECT_FLOAT_EQ(c.imu[0].gyro[2], 0.3f);
    EXPECT_EQ(c.imu[1].t_ns, 123459289ull * 1000);
    EXPECT_FLOAT_EQ(c.imu[1].accel[0], -9.81f);
    EXPECT_FLOAT_EQ(c.imu[1].gyro[2], 2000.0f);
    EXPECT_EQ(d.stats().packets, 1u);
    EXPECT_EQ(d.stats().imu_samples, 2u);
    EXPECT_EQ(d.stats().crc_errors, 0u);
}

TEST(TelemetryDecoderTest, Decodes64BitTimestampPastU32Wrap) {
    TelemetryDecoder d;
    Collector c;
    c.attach(d);

    // > 2^32 µs — exactly the value that broke the old 32-bit protocol.
    const uint64_t t_us = (1ull << 32) + 42;
    const auto f = imu_batch_frame({{t_us, {0, 0, 9.81f, 0, 0, 0}}});
    d.feed(f.data(), f.size());

    ASSERT_EQ(c.imu.size(), 1u);
    EXPECT_EQ(c.imu[0].t_ns, t_us * 1000);
}

TEST(TelemetryDecoderTest, DecodesHeartbeat) {
    TelemetryDecoder d;
    Collector c;
    c.attach(d);

    const auto f = heartbeat_frame(5'000'000, true, 2000, 3);
    d.feed(f.data(), f.size());

    ASSERT_EQ(c.hb.size(), 1u);
    EXPECT_EQ(c.hb[0].t_us, 5'000'000ull);
    EXPECT_TRUE(c.hb[0].imu_ok);
    EXPECT_EQ(c.hb[0].imu_samples, 2000u);
    EXPECT_EQ(c.hb[0].imu_drops, 3u);
}

TEST(TelemetryDecoderTest, ReassemblesAcrossArbitraryChunkBoundaries) {
    TelemetryDecoder d;
    Collector c;
    c.attach(d);

    const auto f = imu_batch_frame({{1000, {1, 2, 3, 4, 5, 6}}});
    // Feed one byte at a time — worst-case fragmentation.
    for (uint8_t b : f) d.feed(&b, 1);

    ASSERT_EQ(c.imu.size(), 1u);
    EXPECT_EQ(c.imu[0].t_ns, 1000ull * 1000);
}

TEST(TelemetryDecoderTest, ResyncsAfterLeadingGarbage) {
    TelemetryDecoder d;
    Collector c;
    c.attach(d);

    std::vector<uint8_t> stream = {'R', 'E', 'A', 'D', 'Y', 0xA5, 0x00, 0xFF};
    const auto f = heartbeat_frame(77, false, 0, 0);
    stream.insert(stream.end(), f.begin(), f.end());
    d.feed(stream.data(), stream.size());

    ASSERT_EQ(c.hb.size(), 1u);
    EXPECT_EQ(c.hb[0].t_us, 77ull);
    EXPECT_GT(d.stats().bytes_skipped, 0u);
}

TEST(TelemetryDecoderTest, DropsCorruptFrameAndRecoverNext) {
    TelemetryDecoder d;
    Collector c;
    c.attach(d);

    auto bad = imu_batch_frame({{1000, {1, 2, 3, 4, 5, 6}}});
    bad[10] ^= 0xFF;  // corrupt a payload byte → CRC mismatch
    const auto good = heartbeat_frame(99, true, 1, 0);

    std::vector<uint8_t> stream;
    stream.insert(stream.end(), bad.begin(), bad.end());
    stream.insert(stream.end(), good.begin(), good.end());
    d.feed(stream.data(), stream.size());

    EXPECT_TRUE(c.imu.empty());
    ASSERT_EQ(c.hb.size(), 1u);
    EXPECT_EQ(c.hb[0].t_us, 99ull);
    EXPECT_GE(d.stats().crc_errors, 1u);
}

TEST(TelemetryDecoderTest, SkipsUnknownTypeWithValidCrc) {
    TelemetryDecoder d;
    Collector c;
    c.attach(d);

    const auto unknown = frame(0x7E, {1, 2, 3});
    const auto good    = heartbeat_frame(11, true, 5, 0);
    std::vector<uint8_t> stream;
    stream.insert(stream.end(), unknown.begin(), unknown.end());
    stream.insert(stream.end(), good.begin(), good.end());
    d.feed(stream.data(), stream.size());

    EXPECT_EQ(d.stats().unknown_types, 1u);
    ASSERT_EQ(c.hb.size(), 1u);
}

TEST(TelemetryDecoderTest, DecodesFw2HeartbeatWithoutCanFields) {
    TelemetryDecoder d;
    Collector c;
    c.attach(d);

    const auto f = heartbeat_frame(5'000'000, true, 2000, 3);  // len 17
    d.feed(f.data(), f.size());

    ASSERT_EQ(c.hb.size(), 1u);
    EXPECT_FALSE(c.hb[0].can_present);
    EXPECT_FALSE(c.hb[0].can_ok);
    EXPECT_EQ(c.hb[0].can_rx, 0u);
}

TEST(TelemetryDecoderTest, DecodesFw3HeartbeatCanExtension) {
    TelemetryDecoder d;
    Collector c;
    c.attach(d);

    // flags: imu_ok | can_ok.
    const auto f = heartbeat_v3_frame(9'000'000, 0x03, 4000, 1,
                                      12345, 7, 2, 99, /*can_mode=*/1);
    d.feed(f.data(), f.size());

    ASSERT_EQ(c.hb.size(), 1u);
    EXPECT_TRUE(c.hb[0].imu_ok);
    EXPECT_TRUE(c.hb[0].can_present);
    EXPECT_TRUE(c.hb[0].can_ok);
    EXPECT_EQ(c.hb[0].imu_samples, 4000u);
    EXPECT_EQ(c.hb[0].can_rx, 12345u);
    EXPECT_EQ(c.hb[0].can_rx_drops, 7u);
    EXPECT_EQ(c.hb[0].odom_tx_drops, 2u);
    EXPECT_EQ(c.hb[0].pose_tx, 99u);
    EXPECT_EQ(c.hb[0].can_mode, 1);
}

TEST(TelemetryDecoderTest, DecodesOdomPacket) {
    TelemetryDecoder d;
    Collector c;
    c.attach(d);

    const auto f = frame(0x03, odom_payload(123'456'789ull, 55'000'000ull,
                                            1.5f, -0.25f, 0.75f,
                                            0x0003, 42, /*mode=*/2));
    d.feed(f.data(), f.size());

    ASSERT_EQ(c.odom.size(), 1u);
    EXPECT_EQ(c.odom[0].t_arrival_us, 123'456'789ull);
    EXPECT_EQ(c.odom[0].rio_time_us, 55'000'000ull);
    EXPECT_FLOAT_EQ(c.odom[0].vx, 1.5f);
    EXPECT_FLOAT_EQ(c.odom[0].vy, -0.25f);
    EXPECT_FLOAT_EQ(c.odom[0].omega, 0.75f);
    EXPECT_EQ(c.odom[0].status_flags, 0x0003);
    EXPECT_EQ(c.odom[0].counter, 42);
    EXPECT_EQ(c.odom[0].mode, 2);
    EXPECT_EQ(d.stats().odom_packets, 1u);
}

TEST(TelemetryDecoderTest, DecodesOdomWithUnknownRioTime) {
    TelemetryDecoder d;
    Collector c;
    c.attach(d);

    const auto f = frame(0x03, odom_payload(1'000ull, /*rio=*/0ull,
                                            0.0f, 0.0f, 0.0f, 0, 1, 1));
    d.feed(f.data(), f.size());

    ASSERT_EQ(c.odom.size(), 1u);
    EXPECT_EQ(c.odom[0].rio_time_us, 0ull);  // 0 = unknown, per contract
}

TEST(TelemetryDecoderTest, DropsWrongLengthOdomWithValidCrc) {
    TelemetryDecoder d;
    Collector c;
    c.attach(d);

    auto payload = odom_payload(1'000ull, 2'000ull, 1, 2, 3, 0, 1, 1);
    payload.pop_back();  // 31 bytes — valid CRC, wrong length
    const auto bad  = frame(0x03, payload);
    const auto good = heartbeat_frame(7, true, 0, 0);
    std::vector<uint8_t> stream;
    stream.insert(stream.end(), bad.begin(), bad.end());
    stream.insert(stream.end(), good.begin(), good.end());
    d.feed(stream.data(), stream.size());

    EXPECT_TRUE(c.odom.empty());
    EXPECT_EQ(d.stats().odom_packets, 0u);
    ASSERT_EQ(c.hb.size(), 1u);  // stream recovered
}

TEST(TelemetryDecoderTest, OdomInterleavedWithImuAcrossFragments) {
    TelemetryDecoder d;
    Collector c;
    c.attach(d);

    std::vector<uint8_t> stream;
    const auto imu1 = imu_batch_frame({{1000, {1, 2, 3, 4, 5, 6}}});
    const auto odo  = frame(0x03, odom_payload(2'000ull, 1'500ull,
                                               0.5f, 0.0f, -0.5f, 0, 9, 1));
    const auto imu2 = imu_batch_frame({{3000, {6, 5, 4, 3, 2, 1}}});
    stream.insert(stream.end(), imu1.begin(), imu1.end());
    stream.insert(stream.end(), odo.begin(), odo.end());
    stream.insert(stream.end(), imu2.begin(), imu2.end());

    // Worst-case fragmentation: byte at a time.
    for (uint8_t b : stream) d.feed(&b, 1);

    ASSERT_EQ(c.imu.size(), 2u);
    ASSERT_EQ(c.odom.size(), 1u);
    EXPECT_EQ(c.odom[0].counter, 9);
    EXPECT_EQ(d.stats().packets, 3u);
}

TEST(TelemetryDecoderTest, RejectsMalformedBatchCount) {
    TelemetryDecoder d;
    Collector c;
    c.attach(d);

    // count says 3 but payload only carries 1 record. CRC is valid, so the
    // frame is consumed; the batch itself is dropped.
    std::vector<uint8_t> payload;
    payload.push_back(3);
    put_u64(payload, 1);
    for (int i = 0; i < 6; ++i) put_f32(payload, 0.0f);
    const auto f = frame(0x01, payload);
    d.feed(f.data(), f.size());

    EXPECT_TRUE(c.imu.empty());
    EXPECT_EQ(d.stats().packets, 1u);
}

}  // namespace gw::server
