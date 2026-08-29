cmake_minimum_required(VERSION 3.20)
# VERSION is propagated to vendored deps that use CMAKE_PROJECT_VERSION
# (notably libsrtp, which set_target_properties()s its lib with it).
project(GuessWork VERSION 0.1.0 LANGUAGES CXX OBJCXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_OBJCXX_STANDARD 20)
set(CMAKE_OBJCXX_STANDARD_REQUIRED ON)

# Honor FetchContent download timestamps so cached archives don't trip the
# "stale download" warning on newer CMake.
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

# Apple Silicon / macOS only
if(NOT APPLE)
    message(FATAL_ERROR "GuessWork only supports macOS / Apple Silicon")
endif()

# Sanitizer options for debug/test builds
option(GW_ENABLE_TSAN "Build with ThreadSanitizer" OFF)
option(GW_ENABLE_ASAN "Build with AddressSanitizer" OFF)

# Whether to build the React bundle and embed it into the binary.
# Default: ON for Release, OFF for everything else. Override with
# -DGW_BUILD_WEB=ON/OFF at configure time.
if(NOT DEFINED GW_BUILD_WEB)
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        set(GW_BUILD_WEB ON)
    else()
        set(GW_BUILD_WEB OFF)
    endif()
endif()
option(GW_BUILD_WEB "Build and embed the React bundle into the binary" ${GW_BUILD_WEB})

# Whether to clone and build the Basalt VIO toolchain as part of our build.
# OFF by default because the first-time build is 15-30 min (vcpkg bootstraps
# the full transitive dep tree). Calibration moved to Kalibr (Dockerized);
# this option only exists for future VIO experimentation with basalt_vio.
option(GW_BUILD_BASALT "Clone and build Basalt for VIO" OFF)

if(GW_ENABLE_TSAN)
    add_compile_options(-fsanitize=thread -g -O1 -fno-omit-frame-pointer)
    add_link_options(-fsanitize=thread)
endif()
if(GW_ENABLE_ASAN)
    add_compile_options(-fsanitize=address,undefined -g -O1 -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)
# Objective-C++ files use ARC.
add_compile_options($<$<COMPILE_LANGUAGE:OBJCXX>:-fobjc-arc>)

include(FetchContent)

# ----------------------------------------------------------------------------
# Core library
# ----------------------------------------------------------------------------
add_library(gw_core
    src/core/clock.cpp
    src/core/frame.cpp
    src/core/frame_pool.cpp
    src/core/frame_channel.cpp
    src/core/clock_sync.cpp
    src/core/attitude_filter.cpp
    src/core/allan_variance.cpp
)
target_include_directories(gw_core PUBLIC src)
target_link_libraries(gw_core PUBLIC
    "-framework CoreFoundation"
    "-framework CoreVideo"
    "-framework IOSurface"
)

# ----------------------------------------------------------------------------
# Spinnaker SDK (system-installed under /usr/local)
#
# GW_STUB_SPINNAKER=ON builds without the SDK: SpinnakerProducer and
# CameraSupervisor are replaced by stubs that report every camera offline.
# For dev machines / CI that can't install the license-gated FLIR SDK —
# never for the robot.
# ----------------------------------------------------------------------------
option(GW_STUB_SPINNAKER
    "Build without the FLIR Spinnaker SDK (all cameras permanently offline)" OFF)

if(NOT GW_STUB_SPINNAKER AND NOT EXISTS /usr/local/include/spinnaker)
    message(FATAL_ERROR
        "Spinnaker SDK not found at /usr/local/include/spinnaker. Install it "
        "(see install.sh) or configure with -DGW_STUB_SPINNAKER=ON for a "
        "camera-less dev/CI build.")
endif()

if(GW_STUB_SPINNAKER)
    add_library(gw_spinnaker INTERFACE)  # empty: nothing links the SDK
else()
    add_library(gw_spinnaker INTERFACE)
    target_include_directories(gw_spinnaker INTERFACE /usr/local/include/spinnaker)
    target_link_libraries(gw_spinnaker INTERFACE
        /usr/local/lib/libSpinnaker.dylib
        # libSpinnaker.dylib resolves libGenApi*, libGCBase*, etc. via @rpath.
        # Add /usr/local/lib to the executable's runtime search path.
        "-Wl,-rpath,/usr/local/lib"
    )
endif()

# ----------------------------------------------------------------------------
# Producer / Consumer / App libraries
# ----------------------------------------------------------------------------
if(GW_STUB_SPINNAKER)
    add_library(gw_producer src/producer/spinnaker_stub.cpp)
    target_compile_definitions(gw_producer PUBLIC GW_STUB_SPINNAKER=1)
else()
    add_library(gw_producer
        src/producer/spinnaker_producer.cpp
        src/producer/spinnaker_user_buffer_pool.cpp
        src/producer/spinnaker_video_modes.cpp
    )
endif()
target_link_libraries(gw_producer PUBLIC gw_core gw_spinnaker)

add_library(gw_consumer
    src/consumer/preview_consumer.mm
    src/consumer/rosbag_writer.cpp
    src/consumer/rosbag_recording_consumer.cpp
    src/consumer/multi_topic_bag_recorder.cpp
)
target_link_libraries(gw_consumer PUBLIC gw_core
    "-framework Metal"
    "-framework QuartzCore"
    "-framework CoreVideo"
)

# ----------------------------------------------------------------------------
# Video encoder (Mono8 -> NV12 + H.264 via VideoToolbox)
# ----------------------------------------------------------------------------
add_library(gw_encoder
    src/encoder/mono8_to_nv12.cpp
    src/encoder/h264_encoder.cpp
)
target_include_directories(gw_encoder PUBLIC src)
target_link_libraries(gw_encoder PUBLIC
    "-framework Accelerate"
    "-framework CoreFoundation"
    "-framework CoreMedia"
    "-framework CoreVideo"
    "-framework VideoToolbox"
)

# ----------------------------------------------------------------------------
# WebRTC stack (libdatachannel + vendored libsrtp + libjuice + mbedtls + plog).
# Pulled before Crow so the FetchContent settings don't clash.
# ----------------------------------------------------------------------------
include(${CMAKE_SOURCE_DIR}/cmake/libdatachannel.cmake)

# ----------------------------------------------------------------------------
# Crow HTTP framework (fetched). Crow needs standalone Asio at configure time;
# fetch it ourselves and point Crow's Findasio.cmake at the source tree.
# ----------------------------------------------------------------------------
FetchContent_Declare(
    asio
    URL https://github.com/chriskohlhoff/asio/archive/refs/tags/asio-1-30-2.zip
)
FetchContent_MakeAvailable(asio)
set(ASIO_INCLUDE_DIR "${asio_SOURCE_DIR}/asio/include" CACHE PATH "" FORCE)

set(CROW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(CROW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    crow
    URL https://github.com/CrowCpp/Crow/archive/refs/tags/v1.2.0.zip
)
FetchContent_MakeAvailable(crow)

# ----------------------------------------------------------------------------
# SQLite (system-installed; macOS ships sqlite3 in the SDK)
# ----------------------------------------------------------------------------
find_package(SQLite3 REQUIRED)
# FindSQLite3 resolves the header inside an SDK's usr/include. When that SDK
# differs from the compiler's implicit sysroot (e.g. Xcode.app vs
# CommandLineTools), the resulting `-isystem <SDK>/usr/include` jumps ahead
# of libc++'s headers and every <cstddef>-style include breaks. sqlite3.h is
# on the implicit search path anyway — drop the interface include dir.
set_target_properties(SQLite3::SQLite3 PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "")

# ----------------------------------------------------------------------------
# yaml-cpp (fetched) + calibration library. gw_calibration parses/serializes
# Kalibr camchain YAML into typed structs; yaml-cpp stays a PRIVATE
# dependency so consumers (gw_server today, gw_apriltag/gw_vio later) never
# see its headers.
# ----------------------------------------------------------------------------
set(YAML_CPP_BUILD_TESTS   OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS   OFF CACHE BOOL "" FORCE)
set(YAML_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    yaml-cpp
    URL https://github.com/jbeder/yaml-cpp/archive/refs/tags/0.8.0.zip
)
FetchContent_MakeAvailable(yaml-cpp)

add_library(gw_calibration
    src/calibration/calibration_store.cpp
)
target_include_directories(gw_calibration PUBLIC src)
target_link_libraries(gw_calibration PRIVATE yaml-cpp::yaml-cpp)

# ----------------------------------------------------------------------------
# AprilTag detection library. The AprilRobotics detector is fetched and
# built statically; OpenCV (Homebrew) provides solvePnP/undistortPoints.
# Neither appears in gw_apriltag's public headers — consumers see only std
# types and gw::calib::Mat4.
# ----------------------------------------------------------------------------
find_package(OpenCV REQUIRED COMPONENTS core calib3d)

# Scope the apriltag build options as normal variables (shadowing any cache
# entries) so they don't leak into other FetchContent'd projects.
set(BUILD_SHARED_LIBS OFF)
set(BUILD_EXAMPLES OFF)
set(BUILD_PYTHON_WRAPPER OFF)
set(BUILD_TESTING OFF)
set(ASAN OFF)
FetchContent_Declare(
    apriltag
    URL https://github.com/AprilRobotics/apriltag/archive/refs/tags/v3.4.5.zip
)
FetchContent_MakeAvailable(apriltag)
unset(BUILD_SHARED_LIBS)
unset(BUILD_EXAMPLES)
unset(BUILD_PYTHON_WRAPPER)
unset(BUILD_TESTING)
unset(ASAN)
# Plain C library — exempt it from our -Wall -Wextra -Wpedantic. Always
# optimize it: an -O0 detector costs ~50 ms/frame at 2048×1536 (vs ~10 ms at
# -O2), blowing the latency budget even in Debug builds of our own code.
target_compile_options(apriltag PRIVATE -w -O2)

add_library(gw_apriltag
    src/apriltag/pose_math.cpp
    src/apriltag/field_layout.cpp
    src/apriltag/tag_pose_estimator.cpp
    src/apriltag/apriltag_consumer.cpp
)
target_include_directories(gw_apriltag PUBLIC src)
target_link_libraries(gw_apriltag
    PUBLIC  gw_core gw_calibration
    PRIVATE apriltag yaml-cpp::yaml-cpp ${OpenCV_LIBS}
)
target_include_directories(gw_apriltag PRIVATE ${OpenCV_INCLUDE_DIRS})

# ----------------------------------------------------------------------------
# VIO library (OpenVINS stereo+IMU MSCKF, built from source — see
# cmake/openvins.cmake for the pin and the macOS arm64 fixes). OpenVINS is
# GPL-3.0; distributed guesswork binaries are effectively GPLv3.
# ----------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
# VioManager.h pulls in boost/filesystem.hpp — our TUs need the headers (the
# OpenVINS dylib itself already links the Boost libraries).
find_package(Boost REQUIRED COMPONENTS filesystem)
include(${CMAKE_SOURCE_DIR}/cmake/openvins.cmake)

add_library(gw_vio
    src/vio/stereo_sync_consumer.cpp
    src/vio/vio_config_builder.cpp
    src/vio/openvins_runner.cpp
)
target_include_directories(gw_vio PUBLIC src)
target_link_libraries(gw_vio
    PUBLIC  gw_core gw_calibration gw_apriltag
    PRIVATE ov_msckf_lib Eigen3::Eigen Boost::filesystem ${OpenCV_LIBS}
)
target_include_directories(gw_vio PRIVATE ${OpenCV_INCLUDE_DIRS})

# ----------------------------------------------------------------------------
# Fusion library (GTSAM fixed-lag smoother, built from source — see
# cmake/gtsam.cmake for the 4.3a1 pin rationale). GTSAM is BSD.
# ----------------------------------------------------------------------------
include(${CMAKE_SOURCE_DIR}/cmake/gtsam.cmake)

add_library(gw_fusion
    src/fusion/fusion_engine.cpp
    src/fusion/sync_clock_now.cpp
)
target_include_directories(gw_fusion PUBLIC src)
target_link_libraries(gw_fusion
    PUBLIC  gw_core gw_calibration gw_apriltag
    PRIVATE gtsam_lib Eigen3::Eigen
)

# ----------------------------------------------------------------------------
# HTTP server library
# ----------------------------------------------------------------------------
# CameraSupervisor is the one server TU that talks to the Spinnaker SDK;
# stub builds swap in the offline-only implementation.
if(GW_STUB_SPINNAKER)
    set(GW_CAMERA_SUPERVISOR_SRC src/server/camera_supervisor_stub.cpp)
else()
    set(GW_CAMERA_SUPERVISOR_SRC src/server/camera_supervisor.cpp)
endif()

add_library(gw_server
    ${GW_CAMERA_SUPERVISOR_SRC}
    src/server/calibration_supervisor.cpp
    src/server/subprocess_job.cpp
    src/server/kalibr_job.cpp
    src/server/kalibr_imu_job.cpp
    src/server/routes_status.cpp
    src/server/routes_stream.cpp
    src/server/routes_camera.cpp
    src/server/routes_calibration.cpp
    src/server/routes_hardware_sync.cpp
    src/server/static_assets.cpp
    src/server/http_server.cpp
    src/server/stream_consumer.cpp
    src/server/webrtc_peer.cpp
    src/server/database.cpp
    src/server/camera_repository.cpp
    src/server/trigger_group_repository.cpp
    src/server/imu_config_repository.cpp
    src/server/routes_imu.cpp
    src/server/field_layout_repository.cpp
    src/server/apriltag_supervisor.cpp
    src/server/routes_apriltag.cpp
    src/server/vio_config_repository.cpp
    src/server/vio_supervisor.cpp
    src/server/routes_vio.cpp
    src/server/net_config_repository.cpp
    src/server/routes_robot.cpp
    src/net/robot_link.cpp
    src/server/fusion_config_repository.cpp
    src/server/fusion_supervisor.cpp
    src/server/routes_fusion.cpp
    src/server/imu_log_recorder.cpp
    src/server/imu_allan_service.cpp
    src/server/imu_attitude_service.cpp
    src/server/config_snapshot.cpp
    src/server/routes_config.cpp
    src/server/sync_controller_manager.cpp
    src/server/sync_protocol_decoder.cpp
)
target_link_libraries(gw_server PUBLIC gw_core gw_producer gw_encoder gw_consumer gw_calibration gw_apriltag gw_vio gw_fusion Crow::Crow LibDataChannel::LibDataChannel SQLite3::SQLite3)
# Host and firmware compile the same freestanding USB wire contract.
target_include_directories(gw_server PRIVATE ${PROJECT_SOURCE_DIR})

# Crow's headers trip a few -Wpedantic warnings (notably the pre-C++23
# `operator"" _method` literal operator form). Silence them on the server lib
# without weakening the project-wide flags.
target_compile_options(gw_server PRIVATE
    -Wno-pedantic
    -Wno-deprecated-literal-operator
)

# ----------------------------------------------------------------------------
# Basalt VIO (opt-in source build via ExternalProject)
# ----------------------------------------------------------------------------
# Calibration moved to Kalibr (Dockerized) — this option only exists so a
# future VIO experiment can pull basalt's `basalt_vio` binary without doing
# a separate install. No app code reads anything basalt.cmake produces today.
if(GW_BUILD_BASALT)
    include(${CMAKE_SOURCE_DIR}/cmake/basalt.cmake)
endif()

# ----------------------------------------------------------------------------
# Kalibr (Dockerized) — compile-time defines for the suggested-command builder.
# The image is built out-of-band (docker/kalibr/build.sh, invoked from
# install.sh); the supervisor suggests `docker/kalibr/calibrate.sh <session>
# <model>`, which manages the Colima lifecycle so the VM is only up while
# Kalibr is actually running.
# ----------------------------------------------------------------------------
set(GW_KALIBR_DOCKER_IMAGE     "guesswork/kalibr:latest"
    CACHE STRING   "Docker image tag for the Kalibr container")
set(GW_KALIBR_TARGET_DEFAULT   "${CMAKE_SOURCE_DIR}/data/kalibr/aprilgrid_6x6.yaml"
    CACHE FILEPATH "Default Kalibr AprilGrid target YAML")
set(GW_KALIBR_CALIBRATE_SCRIPT "${CMAKE_SOURCE_DIR}/docker/kalibr/calibrate.sh"
    CACHE FILEPATH "Script that wraps colima start + docker run + colima stop")
set(GW_KALIBR_CALIBRATE_IMU_SCRIPT "${CMAKE_SOURCE_DIR}/docker/kalibr/calibrate_imu.sh"
    CACHE FILEPATH "Camera-IMU extrinsics script (kalibr_calibrate_imu_camera)")
set(GW_FIELD_LAYOUT_DEFAULT "${CMAKE_SOURCE_DIR}/data/field_layouts/2026-rebuilt-welded.json"
    CACHE FILEPATH "WPILib AprilTagFieldLayout JSON seeded into field_layouts on first boot")
if(NOT EXISTS "${GW_KALIBR_TARGET_DEFAULT}")
    message(FATAL_ERROR
        "GW_KALIBR_TARGET_DEFAULT not found at ${GW_KALIBR_TARGET_DEFAULT}")
endif()
if(NOT EXISTS "${GW_KALIBR_CALIBRATE_SCRIPT}")
    message(FATAL_ERROR
        "GW_KALIBR_CALIBRATE_SCRIPT not found at ${GW_KALIBR_CALIBRATE_SCRIPT}")
endif()
if(NOT EXISTS "${GW_KALIBR_CALIBRATE_IMU_SCRIPT}")
    message(FATAL_ERROR
        "GW_KALIBR_CALIBRATE_IMU_SCRIPT not found at ${GW_KALIBR_CALIBRATE_IMU_SCRIPT}")
endif()
if(NOT EXISTS "${GW_FIELD_LAYOUT_DEFAULT}")
    message(FATAL_ERROR
        "GW_FIELD_LAYOUT_DEFAULT not found at ${GW_FIELD_LAYOUT_DEFAULT}")
endif()

# Sensor pixel pitch (mm) — used by the supervisor to convert per-camera
# lens focal length in mm into a pixel focal-length hint for Kalibr. Default
# matches the Sony IMX264 (1/1.8" @ 2048×1536) this app targets today.
set(GW_SENSOR_PIXEL_PITCH_MM "0.00345" CACHE STRING
    "Camera sensor pixel pitch in millimeters (default: Sony IMX264 / 1/1.8\" @ 2048x1536)")

target_compile_definitions(gw_server PRIVATE
    GW_KALIBR_DOCKER_IMAGE="${GW_KALIBR_DOCKER_IMAGE}"
    GW_KALIBR_TARGET_DEFAULT="${GW_KALIBR_TARGET_DEFAULT}"
    GW_KALIBR_CALIBRATE_SCRIPT="${GW_KALIBR_CALIBRATE_SCRIPT}"
    GW_KALIBR_CALIBRATE_IMU_SCRIPT="${GW_KALIBR_CALIBRATE_IMU_SCRIPT}"
    GW_SENSOR_PIXEL_PITCH_MM=${GW_SENSOR_PIXEL_PITCH_MM}
    GW_FIELD_LAYOUT_DEFAULT="${GW_FIELD_LAYOUT_DEFAULT}"
)

# ----------------------------------------------------------------------------
# Embedded React bundle (Release / opt-in)
# ----------------------------------------------------------------------------
if(GW_BUILD_WEB)
    FetchContent_Declare(
        cmrc
        GIT_REPOSITORY https://github.com/vector-of-bool/cmrc.git
        GIT_TAG        2.0.1
    )
    FetchContent_MakeAvailable(cmrc)

    include(${CMAKE_SOURCE_DIR}/cmake/embed_web.cmake)
    gw_embed_web(gw_server)
endif()

# ----------------------------------------------------------------------------
# Main (headless) executable: pipeline + HTTP server
# ----------------------------------------------------------------------------
add_executable(guesswork apps/guesswork/main.cpp)
target_link_libraries(guesswork PRIVATE
    gw_server
    gw_producer
    gw_core
)

# ----------------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------------
option(GW_BUILD_TESTS "Build unit tests" ON)
if(GW_BUILD_TESTS)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    enable_testing()
    add_executable(gw_tests
        tests/test_clock.cpp
        tests/test_frame_pool.cpp
        tests/test_frame_channel.cpp
        tests/test_mono8_copy.cpp
        tests/test_mono8_to_nv12.cpp
        tests/test_h264_encoder.cpp
        tests/test_database.cpp
        tests/test_camera_repository.cpp
        tests/test_camera_repository_calibration.cpp
        tests/test_rosbag_writer.cpp
        tests/test_rosbag_recording_consumer.cpp
        tests/test_subprocess_job.cpp
        tests/test_sync_protocol_decoder.cpp
        tests/test_measurement_bus.cpp
        tests/test_imu_config_repository.cpp
        tests/test_calibration_store.cpp
        tests/test_multi_topic_bag_recorder.cpp
        tests/test_kalibr_imu_job.cpp
        tests/test_pose_math.cpp
        tests/test_field_layout.cpp
        tests/test_tag_pose_estimator.cpp
        tests/test_apriltag_consumer.cpp
        tests/test_field_layout_repository.cpp
        tests/test_stereo_sync_pairer.cpp
        tests/test_vio_config_builder.cpp
        tests/test_vio_config_repository.cpp
        tests/test_sync_controller_protocol.cpp
        tests/test_calibration_supervisor.cpp
        tests/test_apriltag_supervisor.cpp
        tests/test_attitude_filter.cpp
        tests/test_sync_controller_manager.cpp
        tests/test_trigger_group_repository.cpp
        tests/test_udp_payloads.cpp
        tests/test_clock_sync.cpp
        tests/test_robot_link.cpp
        tests/test_net_config_repository.cpp
        tests/test_controller_now.cpp
        tests/test_fusion_engine.cpp
        tests/test_fusion_config_repository.cpp
        tests/test_latency_stats.cpp
        tests/test_allan_variance.cpp
        tests/test_fusion_mode.cpp
        tests/test_fusion_supervisor.cpp
        tests/test_imu_log_recorder.cpp
        tests/test_config_snapshot.cpp
    )
    target_link_libraries(gw_tests PRIVATE gw_core gw_consumer gw_encoder gw_server gw_apriltag gw_vio gw_fusion GTest::gtest_main)
    # The estimator tests build synthetic projections with OpenCV directly,
    # the consumer test renders a real tag bitmap via the apriltag library,
    # and the field-layout test parses the shipped season fixture.
    # Protocol tests compile the same freestanding USB wire-contract header as
    # the STM32 firmware.
    target_include_directories(gw_tests PRIVATE ${OpenCV_INCLUDE_DIRS}
                                                ${PROJECT_SOURCE_DIR})
    target_link_libraries(gw_tests PRIVATE ${OpenCV_LIBS} apriltag)
    target_compile_definitions(gw_tests PRIVATE
        GW_FIELD_LAYOUT_DEFAULT="${GW_FIELD_LAYOUT_DEFAULT}")
    include(GoogleTest)
    # A cold-cache Debug gw_tests can take >5 s (the default) just to load
    # its dylibs for --gtest_list_tests on first run.
    gtest_discover_tests(gw_tests DISCOVERY_TIMEOUT 60)
endif()
