#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace posest::teensy {

class ISerialTransport {
public:
    virtual ~ISerialTransport() = default;

    virtual void open(const std::string& path, std::uint32_t baud_rate) = 0;
    virtual void close() = 0;
    virtual bool isOpen() const = 0;
    virtual std::size_t read(
        std::uint8_t* buffer,
        std::size_t capacity,
        std::chrono::milliseconds timeout) = 0;
    virtual void write(const std::vector<std::uint8_t>& bytes) = 0;
};

std::unique_ptr<ISerialTransport> makePosixSerialTransport();

}  // namespace posest::teensy
