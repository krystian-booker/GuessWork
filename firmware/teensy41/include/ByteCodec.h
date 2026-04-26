#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace posest::firmware {

inline void appendU16(std::uint8_t* out, std::size_t& offset, std::uint16_t value) {
    out[offset++] = static_cast<std::uint8_t>(value & 0xFFu);
    out[offset++] = static_cast<std::uint8_t>((value >> 8u) & 0xFFu);
}

inline void appendU32(std::uint8_t* out, std::size_t& offset, std::uint32_t value) {
    for (int shift = 0; shift <= 24; shift += 8) {
        out[offset++] = static_cast<std::uint8_t>(
            (value >> static_cast<unsigned>(shift)) & 0xFFu);
    }
}

inline void appendU64(std::uint8_t* out, std::size_t& offset, std::uint64_t value) {
    for (int shift = 0; shift <= 56; shift += 8) {
        out[offset++] = static_cast<std::uint8_t>(
            (value >> static_cast<unsigned>(shift)) & 0xFFu);
    }
}

inline void appendI64(std::uint8_t* out, std::size_t& offset, std::int64_t value) {
    appendU64(out, offset, static_cast<std::uint64_t>(value));
}

inline void appendDouble(std::uint8_t* out, std::size_t& offset, double value) {
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(value));
    appendU64(out, offset, bits);
}

inline std::uint16_t readU16(const std::uint8_t* bytes, std::size_t offset) {
    return static_cast<std::uint16_t>(bytes[offset]) |
           static_cast<std::uint16_t>(
               static_cast<std::uint16_t>(bytes[offset + 1]) << 8u);
}

inline std::uint32_t readU32(const std::uint8_t* bytes, std::size_t offset) {
    return static_cast<std::uint32_t>(bytes[offset]) |
           (static_cast<std::uint32_t>(bytes[offset + 1]) << 8u) |
           (static_cast<std::uint32_t>(bytes[offset + 2]) << 16u) |
           (static_cast<std::uint32_t>(bytes[offset + 3]) << 24u);
}

inline std::uint64_t readU64(const std::uint8_t* bytes, std::size_t offset) {
    std::uint64_t value = 0;
    for (int shift = 0; shift <= 56; shift += 8) {
        value |= static_cast<std::uint64_t>(
                     bytes[offset + static_cast<std::size_t>(shift / 8)])
                 << static_cast<unsigned>(shift);
    }
    return value;
}

inline std::int64_t readI64(const std::uint8_t* bytes, std::size_t offset) {
    return static_cast<std::int64_t>(readU64(bytes, offset));
}

inline double readDouble(const std::uint8_t* bytes, std::size_t offset) {
    const std::uint64_t bits = readU64(bytes, offset);
    double value = 0.0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

}  // namespace posest::firmware
