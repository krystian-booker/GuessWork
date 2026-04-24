#include "posest/teensy/FakeSerialTransport.h"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace posest::teensy {

struct FakeSerialTransport::State {
    mutable std::mutex mu;
    mutable std::condition_variable cv;
    bool open{false};
    std::string last_path;
    std::uint32_t last_baud_rate{0};
    std::size_t open_count{0};
    std::size_t close_count{0};
    std::size_t fail_opens_remaining{0};
    std::size_t fail_reads_remaining{0};
    std::size_t fail_writes_remaining{0};
    std::deque<std::vector<std::uint8_t>> read_chunks;
    std::vector<std::vector<std::uint8_t>> writes;
};

FakeSerialTransport::FakeSerialTransport()
    : state_(std::make_shared<State>()) {}

FakeSerialTransport::FakeSerialTransport(std::shared_ptr<State> state)
    : state_(std::move(state)) {
    if (!state_) {
        throw std::invalid_argument("FakeSerialTransport requires shared state");
    }
}

void FakeSerialTransport::open(const std::string& path, std::uint32_t baud_rate) {
    std::lock_guard<std::mutex> g(state_->mu);
    if (state_->fail_opens_remaining > 0) {
        --state_->fail_opens_remaining;
        throw std::runtime_error("fake serial open failure");
    }
    state_->open = true;
    state_->last_path = path;
    state_->last_baud_rate = baud_rate;
    ++state_->open_count;
    state_->cv.notify_all();
}

void FakeSerialTransport::close() {
    std::lock_guard<std::mutex> g(state_->mu);
    if (state_->open) {
        state_->open = false;
        ++state_->close_count;
    }
    state_->cv.notify_all();
}

bool FakeSerialTransport::isOpen() const {
    std::lock_guard<std::mutex> g(state_->mu);
    return state_->open;
}

std::size_t FakeSerialTransport::read(
    std::uint8_t* buffer,
    std::size_t capacity,
    std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(state_->mu);
    if (state_->fail_reads_remaining > 0) {
        --state_->fail_reads_remaining;
        throw std::runtime_error("fake serial read failure");
    }

    state_->cv.wait_for(
        lock,
        timeout,
        [&] { return !state_->open || !state_->read_chunks.empty(); });
    if (!state_->open || state_->read_chunks.empty()) {
        return 0;
    }

    auto& chunk = state_->read_chunks.front();
    const std::size_t n = std::min(capacity, chunk.size());
    std::copy_n(chunk.begin(), n, buffer);
    chunk.erase(chunk.begin(), chunk.begin() + static_cast<std::ptrdiff_t>(n));
    if (chunk.empty()) {
        state_->read_chunks.pop_front();
    }
    return n;
}

void FakeSerialTransport::write(const std::vector<std::uint8_t>& bytes) {
    std::lock_guard<std::mutex> g(state_->mu);
    if (!state_->open) {
        throw std::runtime_error("fake serial write while closed");
    }
    if (state_->fail_writes_remaining > 0) {
        --state_->fail_writes_remaining;
        throw std::runtime_error("fake serial write failure");
    }
    state_->writes.push_back(bytes);
    state_->cv.notify_all();
}

void FakeSerialTransport::pushReadBytes(std::vector<std::uint8_t> bytes) {
    std::lock_guard<std::mutex> g(state_->mu);
    state_->read_chunks.push_back(std::move(bytes));
    state_->cv.notify_all();
}

void FakeSerialTransport::failNextOpens(std::size_t count) {
    std::lock_guard<std::mutex> g(state_->mu);
    state_->fail_opens_remaining += count;
}

void FakeSerialTransport::failNextReads(std::size_t count) {
    std::lock_guard<std::mutex> g(state_->mu);
    state_->fail_reads_remaining += count;
    state_->cv.notify_all();
}

void FakeSerialTransport::failNextWrites(std::size_t count) {
    std::lock_guard<std::mutex> g(state_->mu);
    state_->fail_writes_remaining += count;
}

bool FakeSerialTransport::waitForOpenCount(
    std::size_t count,
    std::chrono::milliseconds timeout) const {
    std::unique_lock<std::mutex> lock(state_->mu);
    return state_->cv.wait_for(lock, timeout, [&] { return state_->open_count >= count; });
}

bool FakeSerialTransport::waitForWriteCount(
    std::size_t count,
    std::chrono::milliseconds timeout) const {
    std::unique_lock<std::mutex> lock(state_->mu);
    return state_->cv.wait_for(lock, timeout, [&] { return state_->writes.size() >= count; });
}

std::size_t FakeSerialTransport::openCount() const {
    std::lock_guard<std::mutex> g(state_->mu);
    return state_->open_count;
}

std::size_t FakeSerialTransport::closeCount() const {
    std::lock_guard<std::mutex> g(state_->mu);
    return state_->close_count;
}

std::vector<std::vector<std::uint8_t>> FakeSerialTransport::writes() const {
    std::lock_guard<std::mutex> g(state_->mu);
    return state_->writes;
}

std::shared_ptr<FakeSerialTransport::State> FakeSerialTransport::sharedState() const {
    return state_;
}

}  // namespace posest::teensy
