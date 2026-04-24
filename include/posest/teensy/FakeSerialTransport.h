#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>
#include <memory>
#include <vector>

#include "posest/teensy/SerialTransport.h"

namespace posest::teensy {

class FakeSerialTransport final : public ISerialTransport {
public:
    struct State;

    FakeSerialTransport();
    explicit FakeSerialTransport(std::shared_ptr<State> state);

    void open(const std::string& path, std::uint32_t baud_rate) override;
    void close() override;
    bool isOpen() const override;
    std::size_t read(
        std::uint8_t* buffer,
        std::size_t capacity,
        std::chrono::milliseconds timeout) override;
    void write(const std::vector<std::uint8_t>& bytes) override;

    void pushReadBytes(std::vector<std::uint8_t> bytes);
    void failNextOpens(std::size_t count);
    void failNextReads(std::size_t count);
    void failNextWrites(std::size_t count);

    bool waitForOpenCount(std::size_t count, std::chrono::milliseconds timeout) const;
    bool waitForWriteCount(std::size_t count, std::chrono::milliseconds timeout) const;

    std::size_t openCount() const;
    std::size_t closeCount() const;
    std::vector<std::vector<std::uint8_t>> writes() const;
    std::shared_ptr<State> sharedState() const;

private:
    std::shared_ptr<State> state_;
};

}  // namespace posest::teensy
