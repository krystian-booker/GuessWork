#include <gtest/gtest.h>

#include <vector>

#include "server/sync_protocol_decoder.hpp"

namespace gw::server {
namespace {

std::vector<uint8_t> make_frame(gw_sync::MessageType type,
                                const std::vector<uint8_t>& payload,
                                uint16_t request_id = 0) {
    std::vector<uint8_t> out(gw_sync::kMaxFrameBytes);
    const size_t n = gw_sync::build_frame(type, request_id, payload.data(),
                                           static_cast<uint16_t>(payload.size()),
                                           out.data());
    out.resize(n);
    return out;
}

std::vector<uint8_t> imu_frame(uint64_t t_us) {
    std::vector<uint8_t> payload(4 + 2 * 32);
    payload[0] = 2;
    size_t off = 4;
    for (int sample = 0; sample < 2; ++sample) {
        off = gw_sync::put_u64(payload.data(), off, t_us + sample * 2500);
        for (float value : {1.f, 2.f, 9.8f, 0.1f, 0.2f, 0.3f}) {
            off = gw_sync::put_f32(payload.data(), off, value + sample);
        }
    }
    return make_frame(gw_sync::MessageType::ImuBatch, payload);
}

std::vector<uint8_t> heartbeat_frame(uint64_t t_us) {
    std::vector<uint8_t> payload(32);
    size_t off = 0;
    off = gw_sync::put_u64(payload.data(), off, t_us);
    off = gw_sync::put_u32(payload.data(), off,
                           gw_sync::kHeartbeatImuOk | gw_sync::kHeartbeatArmed);
    off = gw_sync::put_u32(payload.data(), off, 400);
    off = gw_sync::put_u32(payload.data(), off, 1);
    off = gw_sync::put_u32(payload.data(), off, 2);
    gw_sync::put_u32(payload.data(), off, 3);
    return make_frame(gw_sync::MessageType::Heartbeat, payload);
}

}  // namespace

TEST(SyncProtocolDecoderTest, DecodesInterleavedFramesAcrossByteFragments) {
    SyncProtocolDecoder decoder;
    std::vector<gw::ImuSample> imu;
    std::vector<gw_sync::Heartbeat> heartbeats;
    std::vector<gw_sync::TriggerEvent> triggers;
    decoder.on_imu = [&](const auto& v) { imu.push_back(v); };
    decoder.on_heartbeat = [&](const auto& v) { heartbeats.push_back(v); };
    decoder.on_trigger = [&](const auto& v) { triggers.push_back(v); };

    std::vector<uint8_t> trigger_payload(16);
    trigger_payload[0] = 2;
    gw_sync::put_u32(trigger_payload.data(), 4, 99);
    gw_sync::put_u64(trigger_payload.data(), 8, (1ull << 32) + 42);

    std::vector<uint8_t> stream = {'x', 'x'};
    for (auto frame : {imu_frame(123'000),
                       make_frame(gw_sync::MessageType::Trigger, trigger_payload),
                       heartbeat_frame(999'000)}) {
        stream.insert(stream.end(), frame.begin(), frame.end());
    }
    for (const uint8_t byte : stream) decoder.feed(&byte, 1);

    ASSERT_EQ(imu.size(), 2u);
    EXPECT_EQ(imu[0].t_ns, 123'000'000u);
    EXPECT_FLOAT_EQ(imu[1].gyro[2], 1.3f);
    ASSERT_EQ(triggers.size(), 1u);
    EXPECT_EQ(triggers[0].slot, 2);
    EXPECT_EQ(triggers[0].index, 99u);
    EXPECT_EQ(triggers[0].t_us, (1ull << 32) + 42);
    ASSERT_EQ(heartbeats.size(), 1u);
    EXPECT_EQ(heartbeats[0].usb_errors, 3u);
    EXPECT_EQ(decoder.stats().packets, 3u);
    EXPECT_EQ(decoder.stats().bytes_skipped, 2u);
}

TEST(SyncProtocolDecoderTest, DecodesDeviceInfoAndAckRequestIds) {
    SyncProtocolDecoder decoder;
    uint16_t info_id = 0;
    std::optional<SyncProtocolDecoder::Ack> ack;
    decoder.on_device_info = [&](uint16_t id, const auto&) { info_id = id; };
    decoder.on_ack = [&](const auto& value) { ack = value; };

    gw_sync::DeviceInfo info;
    info.board_id = gw_sync::kBoardIdMicoAirF405V2;
    uint8_t info_payload[28];
    gw_sync::encode_device_info(info, info_payload);
    auto info_frame = make_frame(gw_sync::MessageType::DeviceInfo,
                                 {info_payload, info_payload + 28}, 41);
    const std::vector<uint8_t> ack_payload = {
        static_cast<uint8_t>(gw_sync::MessageType::Arm),
        static_cast<uint8_t>(gw_sync::AckStatus::Ok), 0, 0};
    auto ack_frame = make_frame(gw_sync::MessageType::Ack, ack_payload, 42);
    decoder.feed(info_frame.data(), info_frame.size());
    decoder.feed(ack_frame.data(), ack_frame.size());
    EXPECT_EQ(info_id, 41);
    ASSERT_TRUE(ack.has_value());
    EXPECT_EQ(ack->request_id, 42);
    EXPECT_EQ(ack->command, gw_sync::MessageType::Arm);
}

TEST(SyncProtocolDecoderTest, RejectsCorruptionAndRecovers) {
    SyncProtocolDecoder decoder;
    int heartbeats = 0;
    decoder.on_heartbeat = [&](const auto&) { ++heartbeats; };
    auto bad = imu_frame(1);
    bad[12] ^= 0x80;
    const auto good = heartbeat_frame(2);
    bad.insert(bad.end(), good.begin(), good.end());
    decoder.feed(bad.data(), bad.size());
    EXPECT_EQ(heartbeats, 1);
    EXPECT_EQ(decoder.stats().crc_errors, 1u);
}

}  // namespace gw::server

