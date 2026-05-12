#pragma once

#include <cstdint>
#include <memory>

namespace gw::server {

class PipelineStatsView;
class StreamConsumer;
class CameraRepository;

// Thin wrapper around a Crow application. The Crow SDK headers are heavyweight
// (asio, boost-style metaprogramming) so we hide them behind a Pimpl: consumers
// of this header don't pay the compile-time cost.
class HttpServer {
public:
    HttpServer(uint16_t           port,
               PipelineStatsView& stats,
               StreamConsumer&    stream,
               CameraRepository&  cameras);
    ~HttpServer();

    HttpServer(const HttpServer&)            = delete;
    HttpServer& operator=(const HttpServer&) = delete;

    // Blocks until stop() is called from another thread.
    void run();

    // Asks Crow to shut down. Safe to call from a signal handler.
    void stop();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace gw::server
