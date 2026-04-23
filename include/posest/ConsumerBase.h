#pragma once

#include <atomic>
#include <string>
#include <thread>

#include "posest/Frame.h"
#include "posest/IFrameConsumer.h"
#include "posest/LatestFrameSlot.h"

namespace posest {

// Reusable base implementing the thread + mailbox plumbing.
//
// Subclasses implement process(const Frame&), which runs on the consumer's
// own worker thread. Subclasses must NOT assume they see every frame: the
// mailbox drops unread frames when a newer one arrives.
class ConsumerBase : public IFrameConsumer {
public:
    explicit ConsumerBase(std::string id);
    ~ConsumerBase() override;

    ConsumerBase(const ConsumerBase&) = delete;
    ConsumerBase& operator=(const ConsumerBase&) = delete;

    const std::string& id() const override { return id_; }

    void start() override;
    void stop() override;

    void deliver(FramePtr frame) override;

    std::uint64_t droppedByMailbox() const { return slot_.droppedCount(); }

protected:
    virtual void process(const Frame& frame) = 0;

private:
    void runLoop();

    std::string id_;
    LatestFrameSlot slot_;
    std::thread worker_;
    std::atomic<bool> running_{false};
};

}  // namespace posest
