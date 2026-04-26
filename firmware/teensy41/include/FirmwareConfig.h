#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace posest::firmware {

constexpr std::size_t kMaxCameraSyncOutputs = 6;
constexpr std::array<std::uint8_t, kMaxCameraSyncOutputs> kCameraSyncPins{
    2, 3, 4, 5, 6, 7};
constexpr std::uint8_t kStatusLedPin = 13;
constexpr std::uint32_t kDefaultSerialBaud = 921600;
constexpr std::uint32_t kHealthPublishIntervalUs = 100000;

constexpr std::uint8_t kBmi088SpiMisoPin = 1;
constexpr std::uint8_t kBmi088SpiMosiPin = 26;
constexpr std::uint8_t kBmi088SpiSckPin = 27;
constexpr std::uint8_t kBmi088AccelCsPin = 8;
constexpr std::uint8_t kBmi088GyroCsPin = 9;
constexpr std::uint8_t kBmi088ProtocolSelectPin = 14;
constexpr std::uint8_t kBmi088Int1Pin = 15;
constexpr std::uint8_t kBmi088Int2Pin = 16;

constexpr std::uint32_t kErrorCrcFailure = 1u << 0u;
constexpr std::uint32_t kErrorUnsupportedCommand = 1u << 1u;
constexpr std::uint32_t kErrorInvalidPayload = 1u << 2u;
constexpr std::uint32_t kErrorCanUnsupported = 1u << 3u;
constexpr std::uint32_t kErrorImuInitFailure = 1u << 4u;
constexpr std::uint32_t kErrorImuSpiFailure = 1u << 5u;
constexpr std::uint32_t kErrorImuMissedDrdy = 1u << 6u;
constexpr std::uint32_t kErrorImuSampleOverrun = 1u << 7u;
constexpr std::uint32_t kErrorImuAccelSaturated = 1u << 8u;
constexpr std::uint32_t kErrorImuGyroSaturated = 1u << 9u;
constexpr std::uint32_t kErrorImuSelfTestFailure = 1u << 10u;

constexpr std::uint32_t kTriggerUnsupportedCount = 1u << 0u;
constexpr std::uint32_t kTriggerInvalidPin = 1u << 1u;
constexpr std::uint32_t kTriggerDuplicatePin = 1u << 2u;
constexpr std::uint32_t kTriggerInvalidRate = 1u << 3u;
constexpr std::uint32_t kTriggerPulseTooWide = 1u << 4u;
constexpr std::uint32_t kTriggerOverrun = 1u << 5u;

constexpr std::uint32_t kCanInitFailure = 1u << 0u;
constexpr std::uint32_t kCanBusOff = 1u << 1u;
constexpr std::uint32_t kCanRxOverrun = 1u << 2u;
constexpr std::uint32_t kCanTxOverrun = 1u << 3u;
constexpr std::uint32_t kCanFrameTruncated = 1u << 4u;
constexpr std::uint32_t kCanRxDecodeFailure = 1u << 5u;

// Firmware-internal CanBridge status_flags_ bits. These never cross USB; the
// wire-visible bits live in Protocol.h (kHealthRio*, kStatus*).
constexpr std::uint32_t kRioStatusPingRejected = 1u << 1u;

}  // namespace posest::firmware
