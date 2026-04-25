#include "posest/ProducerBase.h"

#include <chrono>
#include <cstdio>
#include <exception>
#include <utility>

namespace posest {

ProducerBase::ProducerBase(std::string id) : id_(std::move(id)) {}

ProducerBase::~ProducerBase() {
    stop();
}

void ProducerBase::addConsumer(std::shared_ptr<IFrameConsumer> consumer) {
    std::lock_guard<std::mutex> g(consumers_mu_);
    consumers_.push_back(std::move(consumer));
}

void ProducerBase::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return;
    }
    worker_ = std::thread(&ProducerBase::runLoop, this);
}

void ProducerBase::stop() {
    bool expected = true;
    if (!running_.compare_exchange_strong(expected, false)) {
        return;
    }
    if (worker_.joinable()) {
        worker_.join();
    }
}

void ProducerBase::runLoop() {
    // Snapshot the consumer list once. We don't support hot-add during capture
    // (wiring is expected to be complete before start()); a mutex is still
    // used for the initial read so addConsumer calls before start() are safe.
    std::vector<std::shared_ptr<IFrameConsumer>> subscribers;
    {
        std::lock_guard<std::mutex> g(consumers_mu_);
        subscribers = consumers_;
    }

    while (running_.load(std::memory_order_acquire)) {
        cv::Mat image;
        std::optional<std::chrono::steady_clock::time_point> subclass_ts;
        bool ok = false;
        try {
            ok = captureOne(image, subclass_ts);
        } catch (const std::exception& e) {
            // Defensive: capture loops must never propagate. Log and exit
            // cleanly so the worker can be joined; subclasses (e.g.
            // CameraProducer) own their own retry/reconnect policy.
            std::fprintf(stderr,
                         "%s: capture loop terminated by exception: %s\n",
                         id_.c_str(), e.what());
            break;
        } catch (...) {
            std::fprintf(stderr,
                         "%s: capture loop terminated by unknown exception\n",
                         id_.c_str());
            break;
        }
        if (!ok) {
            break;
        }

        auto frame = std::make_shared<Frame>();
        frame->capture_time =
            subclass_ts.value_or(std::chrono::steady_clock::now());
        frame->sequence = next_sequence_++;
        frame->camera_id = id_;
        frame->image = std::move(image);

        FramePtr pub = frame;
        produced_.fetch_add(1, std::memory_order_relaxed);

        for (auto& c : subscribers) {
            c->deliver(pub);
        }
    }
}

}  // namespace posest
