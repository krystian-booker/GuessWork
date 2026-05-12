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
)
target_include_directories(gw_core PUBLIC src)
target_link_libraries(gw_core PUBLIC
    "-framework CoreFoundation"
    "-framework CoreVideo"
    "-framework IOSurface"
)

# ----------------------------------------------------------------------------
# Spinnaker SDK (system-installed under /usr/local)
# ----------------------------------------------------------------------------
add_library(gw_spinnaker INTERFACE)
target_include_directories(gw_spinnaker INTERFACE /usr/local/include/spinnaker)
target_link_libraries(gw_spinnaker INTERFACE
    /usr/local/lib/libSpinnaker.dylib
    # libSpinnaker.dylib resolves libGenApi*, libGCBase*, etc. via @rpath.
    # Add /usr/local/lib to the executable's runtime search path.
    "-Wl,-rpath,/usr/local/lib"
)

# ----------------------------------------------------------------------------
# Producer / Consumer / App libraries
# ----------------------------------------------------------------------------
add_library(gw_producer
    src/producer/spinnaker_producer.cpp
    src/producer/spinnaker_user_buffer_pool.cpp
)
target_link_libraries(gw_producer PUBLIC gw_core gw_spinnaker)

add_library(gw_consumer
    src/consumer/preview_consumer.mm
)
target_link_libraries(gw_consumer PUBLIC gw_core
    "-framework Metal"
    "-framework QuartzCore"
    "-framework CoreVideo"
)

add_library(gw_app
    src/app/preview_window.mm
)
target_link_libraries(gw_app PUBLIC gw_core
    "-framework AppKit"
    "-framework QuartzCore"
    "-framework Metal"
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

# ----------------------------------------------------------------------------
# HTTP server library
# ----------------------------------------------------------------------------
add_library(gw_server
    src/server/pipeline_stats.cpp
    src/server/routes_status.cpp
    src/server/routes_stream.cpp
    src/server/routes_camera.cpp
    src/server/static_assets.cpp
    src/server/http_server.cpp
    src/server/stream_consumer.cpp
    src/server/webrtc_peer.cpp
    src/server/database.cpp
    src/server/camera_repository.cpp
)
target_link_libraries(gw_server PUBLIC gw_core gw_producer gw_encoder Crow::Crow LibDataChannel::LibDataChannel SQLite3::SQLite3)

# Crow's headers trip a few -Wpedantic warnings (notably the pre-C++23
# `operator"" _method` literal operator form). Silence them on the server lib
# without weakening the project-wide flags.
target_compile_options(gw_server PRIVATE
    -Wno-pedantic
    -Wno-deprecated-literal-operator
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
# Test preview app (windowed Metal preview, equivalent to the old main.mm)
# ----------------------------------------------------------------------------
add_executable(test_preview apps/test_preview/main.mm)
target_link_libraries(test_preview PRIVATE
    gw_app
    gw_consumer
    gw_producer
    gw_core
    "-framework AppKit"
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
        tests/test_mono8_to_nv12.cpp
        tests/test_h264_encoder.cpp
        tests/test_database.cpp
        tests/test_camera_repository.cpp
    )
    target_link_libraries(gw_tests PRIVATE gw_core gw_encoder gw_server GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(gw_tests)
endif()
