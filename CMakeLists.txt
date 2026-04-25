cmake_minimum_required(VERSION 3.20)
project(posest LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion)

find_package(OpenCV REQUIRED)
find_package(GTest REQUIRED)
find_package(SQLite3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(apriltag REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(GTSAM REQUIRED)

if(TARGET gtsam)
    set(POSEST_GTSAM_TARGET gtsam)
elseif(TARGET gtsam::gtsam)
    set(POSEST_GTSAM_TARGET gtsam::gtsam)
else()
    message(FATAL_ERROR "GTSAM package did not provide a known CMake target")
endif()

# --- posest_core: interfaces, Frame, LatestFrameSlot, ProducerBase, ConsumerBase
add_library(posest_core
    src/core/LatestFrameSlot.cpp
    src/core/ProducerBase.cpp
    src/core/ConsumerBase.cpp
    src/core/MeasurementBus.cpp
    src/core/CameraTriggerCache.cpp
)
target_include_directories(posest_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(posest_core PUBLIC opencv::opencv)
target_compile_features(posest_core PUBLIC cxx_std_20)

# --- posest_camera: generic camera abstractions
add_library(posest_camera
    src/camera/CameraConfig.cpp
    src/camera/CameraProducer.cpp
)
target_link_libraries(posest_camera PUBLIC posest_core)

# --- posest_config: runtime config store contracts + SQLite schema
add_library(posest_config
    src/config/CalibrationParsers.cpp
    src/config/ConfigValidator.cpp
    src/config/InMemoryConfigStore.cpp
    src/config/SqliteConfigStore.cpp
    src/config/SqliteSchema.cpp
)
target_link_libraries(posest_config PUBLIC posest_core posest_camera SQLite::SQLite3)
target_link_libraries(posest_config PUBLIC nlohmann_json::nlohmann_json)
target_link_libraries(posest_config PUBLIC yaml-cpp::yaml-cpp)

# --- posest_runtime: graph wiring + web/control facade
add_library(posest_runtime
    src/runtime/RuntimeGraph.cpp
    src/runtime/WebService.cpp
)
target_link_libraries(posest_runtime PUBLIC posest_core posest_camera posest_config)

# --- posest_calibration: dataset recording and Kalibr Docker command helpers
add_library(posest_calibration
    src/calibration/CalibrationRecorder.cpp
)
target_link_libraries(posest_calibration PUBLIC
    posest_core
    posest_camera
    posest_runtime
    nlohmann_json::nlohmann_json
)

# --- posest_fusion: measurement ingestion + fusion output boundary
add_library(posest_fusion
    src/fusion/FusionService.cpp
)
target_link_libraries(posest_fusion PUBLIC posest_core posest_runtime ${POSEST_GTSAM_TARGET})

# --- posest_teensy: USB protocol codec + fusion output adapter
add_library(posest_teensy
    src/teensy/Protocol.cpp
    src/teensy/SerialTransport.cpp
    src/teensy/FakeSerialTransport.cpp
    src/teensy/TeensyService.cpp
)
target_link_libraries(posest_teensy PUBLIC posest_core posest_fusion posest_runtime)

# --- posest_pipelines: vision pipeline implementations
add_library(posest_pipelines
    src/pipelines/VisionPipelineBase.cpp
    src/pipelines/PlaceholderPipelines.cpp
    src/pipelines/AprilTagPipeline.cpp
    src/pipelines/AprilTagPoseSolver.cpp
)
target_link_libraries(posest_pipelines PUBLIC
    posest_core
    posest_runtime
    nlohmann_json::nlohmann_json
    apriltag::apriltag
)

# --- posest_v4l2: V4L2 camera backend (Linux only)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_library(posest_v4l2
        src/v4l2/V4L2Producer.cpp
        src/v4l2/V4L2DeviceEnumerator.cpp
    )
    target_link_libraries(posest_v4l2 PUBLIC posest_camera)
endif()

# --- posest_daemon_lib: daemon lifecycle, production factories, health
add_library(posest_daemon_lib
    src/runtime/Daemon.cpp
    src/runtime/ProductionFactories.cpp
)
target_link_libraries(posest_daemon_lib PUBLIC
    posest_config
    posest_runtime
    posest_calibration
    posest_fusion
    posest_teensy
    posest_pipelines
    nlohmann_json::nlohmann_json
)
if(TARGET posest_v4l2)
    target_compile_definitions(posest_daemon_lib PRIVATE POSEST_HAS_V4L2)
    target_link_libraries(posest_daemon_lib PUBLIC posest_v4l2)
endif()

# --- posest_daemon: single process runtime entrypoint
add_executable(posest_daemon app/posest_daemon.cpp)
target_link_libraries(posest_daemon PRIVATE posest_daemon_lib)

# --- posest_mock: MockProducer / MockConsumer / MockCameraProducer (used by tests)
add_library(posest_mock
    src/mock/MockProducer.cpp
    src/mock/MockConsumer.cpp
    src/mock/MockCameraProducer.cpp
)
target_link_libraries(posest_mock PUBLIC posest_core posest_camera)

# --- tests
enable_testing()
add_executable(posest_tests
    test/test_latest_frame_slot.cpp
    test/test_producer_base.cpp
    test/test_camera_trigger_cache.cpp
    test/test_mock_pipeline.cpp
    test/test_measurement_bus.cpp
    test/test_runtime_graph.cpp
    test/test_pipelines.cpp
    test/test_apriltag_pose_solver.cpp
    test/test_teensy_protocol.cpp
    test/test_teensy_service.cpp
    test/test_calibration_recorder.cpp
    test/test_config_schema.cpp
    test/test_camera_producer.cpp
    test/test_fusion_service.cpp
    test/test_daemon.cpp
)
target_link_libraries(posest_tests PRIVATE
    posest_mock
    posest_camera
    posest_config
    posest_runtime
    posest_calibration
    posest_fusion
    posest_teensy
    posest_pipelines
    posest_daemon_lib
    GTest::gtest
    GTest::gtest_main
)

if(TARGET posest_v4l2)
    target_sources(posest_tests PRIVATE test/test_v4l2_producer.cpp)
    target_link_libraries(posest_tests PRIVATE posest_v4l2)
endif()

include(GoogleTest)
gtest_discover_tests(posest_tests)
