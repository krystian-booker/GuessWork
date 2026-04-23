#include "posest/ConsumerBase.h"

#include <utility>

namespace posest {

ConsumerBase::ConsumerBase(std::string id) : id_(std::move(id)) {}

ConsumerBase::~ConsumerBase() {
    stop();
}

void ConsumerBase::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return;
    }
    worker_ = std::thread(&ConsumerBase::runLoop, this);
}

void ConsumerBase::stop() {
    bool expected = true;
    if (!running_.compare_exchange_strong(expected, false)) {
        return;
    }
    slot_.shutdown();
    if (worker_.joinable()) {
        worker_.join();
    }
}

void ConsumerBase::deliver(FramePtr frame) {
    if (!running_.load(std::memory_order_acquire)) {
        return;
    }
    slot_.put(std::move(frame));
}

void ConsumerBase::runLoop() {
    while (true) {
        FramePtr f = slot_.take();
        if (!f) {
            return;  // shutdown
        }
        process(*f);
    }
}

}  // namespace posest
