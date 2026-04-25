#include "posest/ProducerBase.h"

#include <algorithm>
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

bool ProducerBase::removeConsumer(const std::shared_ptr<IFrameConsumer>& consumer) {
    std::lock_guard<std::mutex> g(consumers_mu_);
    auto it = std::find(consumers_.begin(), consumers_.end(), consumer);
    if (it == consumers_.end()) {
        return false;
    }
    consumers_.erase(it);
    return true;
}

void ProducerBase::start() {
    // Only Idle → Running is permitted. EndOfStream/Failed are terminal:
    // once a capture loop has exited on its own, the operator must construct
    // a new producer rather than re-arming this one. Running → Running is a
    // no-op (idempotent start).
    ProducerState expected = ProducerState::Idle;
    if (!state_.compare_exchange_strong(
            expected, ProducerState::Running, std::memory_order_acq_rel)) {
        return;
    }
    worker_ = std::thread(&ProducerBase::runLoop, this);
}

void ProducerBase::stop() {
    // Tell a still-running loop to exit, but also handle the terminal cases
    // (EndOfStream/Failed) where the worker has already exited on its own —
    // we still need to join the thread and return to Idle so the destructor
    // can run cleanly.
    ProducerState prior = state_.exchange(ProducerState::Idle, std::memory_order_acq_rel);
    if (prior == ProducerState::Idle) {
        return;
    }
    if (worker_.joinable()) {
        worker_.join();
    }
}

void ProducerBase::runLoop() {
    // Per-iteration subscriber snapshot. Refreshed inside the loop so that
    // addConsumer / removeConsumer take effect for the very next frame without
    // requiring a producer restart.
    std::vector<std::shared_ptr<IFrameConsumer>> subscribers;

    // Helper: only publish a terminal state if we still own the Running state.
    // If stop() has already moved us to Idle, leave it there — the operator
    // explicitly asked for shutdown and shouldn't see EndOfStream/Failed after.
    auto setTerminalIfStillRunning = [this](ProducerState terminal) {
        ProducerState expected = ProducerState::Running;
        state_.compare_exchange_strong(
            expected, terminal, std::memory_order_acq_rel);
    };

    while (state_.load(std::memory_order_acquire) == ProducerState::Running) {
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
            setTerminalIfStillRunning(ProducerState::Failed);
            return;
        } catch (...) {
            std::fprintf(stderr,
                         "%s: capture loop terminated by unknown exception\n",
                         id_.c_str());
            setTerminalIfStillRunning(ProducerState::Failed);
            return;
        }
        if (!ok) {
            setTerminalIfStillRunning(ProducerState::EndOfStream);
            return;
        }

        auto frame = std::make_shared<Frame>();
        frame->capture_time =
            subclass_ts.value_or(std::chrono::steady_clock::now());
        frame->sequence = next_sequence_++;
        frame->camera_id = id_;
        frame->image = std::move(image);

        FramePtr pub = frame;
        produced_.fetch_add(1, std::memory_order_relaxed);

        {
            std::lock_guard<std::mutex> g(consumers_mu_);
            subscribers = consumers_;
        }
        for (auto& c : subscribers) {
            c->deliver(pub);
        }
    }
}

}  // namespace posest
