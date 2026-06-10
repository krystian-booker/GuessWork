#pragma once

#include <chrono>
#include <cstdint>
#include <memory>

namespace gw::server {

class CameraRepository;
class CameraSupervisor;
class CalibrationSupervisor;
class TriggerGroupRepository;
class TeensyManager;
class ImuConfigRepository;

// Thin wrapper around a Crow application. The Crow SDK headers are heavyweight
// (asio, boost-style metaprogramming) so we hide them behind a Pimpl: consumers
// of this header don't pay the compile-time cost.
class HttpServer {
public:
    HttpServer(uint16_t                              port,
               CameraSupervisor&                     supervisor,
               CameraRepository&                     cameras,
               CalibrationSupervisor&                calibration,
               TriggerGroupRepository&               trigger_groups,
               TeensyManager&                        teensy,
               ImuConfigRepository&                  imu_config,
               std::chrono::steady_clock::time_point started_at);
    ~HttpServer();

    HttpServer(const HttpServer&)            = delete;
    HttpServer& operator=(const HttpServer&) = delete;

    void run();
    void stop();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace gw::server
