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
    src/core/ToFSampleCache.cpp
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
    src/calibration/CalibrationTargetWriter.cpp
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

# --- posest_vio_core: airborne covariance gate, IVioBackend, FakeVioBackend,
# KimeraVioConsumer. Always built so tests + Linux CI can exercise the
# consumer against FakeVioBackend without needing a Kimera-VIO install.

# Embed the static Kimera YAML templates as raw string literals so the
# runtime KimeraParamWriter can lay down a self-contained param_dir
# without needing to look up the install prefix at runtime. CMake reads
# each source YAML once at configure time and substitutes its contents
# into a generated header. Touching any of the three YAMLs reruns
# CMake on the next build.
set(POSEST_KIMERA_FRONTEND_YAML_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/share/posest/kimera/FrontendParams.yaml)
set(POSEST_KIMERA_BACKEND_YAML_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/share/posest/kimera/BackendParams.yaml)
set(POSEST_KIMERA_IMU_YAML_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/share/posest/kimera/ImuParams.yaml)
file(READ ${POSEST_KIMERA_FRONTEND_YAML_PATH} POSEST_KIMERA_FRONTEND_YAML)
file(READ ${POSEST_KIMERA_BACKEND_YAML_PATH} POSEST_KIMERA_BACKEND_YAML)
file(READ ${POSEST_KIMERA_IMU_YAML_PATH} POSEST_KIMERA_IMU_TEMPLATE_YAML)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    ${POSEST_KIMERA_FRONTEND_YAML_PATH}
    ${POSEST_KIMERA_BACKEND_YAML_PATH}
    ${POSEST_KIMERA_IMU_YAML_PATH})
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vio/KimeraStaticParams.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/posest_vio_core_gen/posest/vio/KimeraStaticParams.h
    @ONLY)

add_library(posest_vio_core
    src/vio/AirborneCovariance.cpp
    src/vio/FakeVioBackend.cpp
    src/vio/KimeraParamWriter.cpp
    src/vio/KimeraVioConfigBuilder.cpp
    src/vio/KimeraVioConsumer.cpp
)
target_include_directories(posest_vio_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/posest_vio_core_gen>
)
target_link_libraries(posest_vio_core PUBLIC
    posest_core
    ${POSEST_GTSAM_TARGET}
)

# --- posest_vio_kimera: real Kimera-VIO backend adapter. Defaults ON only
# on Apple (Mac Mini target); opt-in on Linux for developers who have
# Kimera-VIO installed locally. cmake/FindKimeraVIO.cmake provides the
# KimeraVIO::KimeraVIO target since Kimera does not ship a portable
# Config.cmake.
option(POSEST_BUILD_VIO "Build Kimera-VIO backend adapter" ${APPLE})
if(POSEST_BUILD_VIO)
    list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
    find_package(KimeraVIO REQUIRED)
    add_library(posest_vio_kimera
        src/vio/KimeraBackend.cpp
    )
    target_link_libraries(posest_vio_kimera PUBLIC
        posest_vio_core
        KimeraVIO::KimeraVIO
    )
endif()

# Install the Kimera YAML parameter files alongside the daemon binary.
# KimeraVioConfig::param_dir resolves to <install-prefix>/share/posest/kimera
# in production deployments.
install(
    DIRECTORY share/posest/kimera
    DESTINATION share/posest
    FILES_MATCHING PATTERN "*.yaml"
)

# --- posest_teensy: USB protocol codec + fusion output adapter
add_library(posest_teensy
    src/teensy/Protocol.cpp
    src/teensy/SerialTransport.cpp
    src/teensy/FakeSerialTransport.cpp
    src/teensy/TeensyService.cpp
    src/teensy/TimeSyncFilter.cpp
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
    posest_vio_core
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
    test/test_latency_histogram.cpp
    test/test_latest_frame_slot.cpp
    test/test_producer_base.cpp
    test/test_camera_trigger_cache.cpp
    test/test_tof_sample_cache.cpp
    test/test_mock_pipeline.cpp
    test/test_measurement_bus.cpp
    test/test_runtime_graph.cpp
    test/test_pipelines.cpp
    test/test_apriltag_pose_solver.cpp
    test/test_teensy_protocol.cpp
    test/test_teensy_protocol_golden.cpp
    test/test_teensy_service.cpp
    test/test_time_sync_filter.cpp
    test/test_calibrate_end_to_end.cpp
    test/test_calibration_parsers.cpp
    test/test_calibration_recorder.cpp
    test/test_calibration_target_writer.cpp
    test/test_config_schema.cpp
    test/test_kalibr_dataset_lifecycle.cpp
    test/test_camera_producer.cpp
    test/test_fusion_service.cpp
    test/test_daemon.cpp
    test/test_airborne_covariance.cpp
    test/test_kimera_param_writer.cpp
    test/test_kimera_vio_consumer.cpp
    test/test_tee_sink.cpp
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
    posest_vio_core
    posest_daemon_lib
    GTest::gtest
    GTest::gtest_main
)

target_compile_definitions(posest_tests PRIVATE
    POSEST_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/test/fixtures"
)

if(TARGET posest_v4l2)
    target_sources(posest_tests PRIVATE test/test_v4l2_producer.cpp)
    target_link_libraries(posest_tests PRIVATE posest_v4l2)
endif()

if(POSEST_BUILD_VIO)
    target_sources(posest_tests PRIVATE test/test_kimera_backend.cpp)
    target_link_libraries(posest_tests PRIVATE posest_vio_kimera)
endif()

include(GoogleTest)
gtest_discover_tests(posest_tests)

find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
    add_test(
        NAME protocol_constants_drift
        COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_SOURCE_DIR}/scripts/check_protocol_constants.py
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
endif()
