cmake_minimum_required(VERSION 3.20)
project(GuessWork LANGUAGES CXX OBJCXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_OBJCXX_STANDARD 20)
set(CMAKE_OBJCXX_STANDARD_REQUIRED ON)

# Apple Silicon / macOS only
if(NOT APPLE)
    message(FATAL_ERROR "GuessWork only supports macOS / Apple Silicon")
endif()

# Sanitizer options for debug/test builds
option(GW_ENABLE_TSAN "Build with ThreadSanitizer" OFF)
option(GW_ENABLE_ASAN "Build with AddressSanitizer" OFF)

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
# Main executable
# ----------------------------------------------------------------------------
add_executable(guesswork src/main.mm)
target_link_libraries(guesswork PRIVATE
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
    include(FetchContent)
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
    )
    target_link_libraries(gw_tests PRIVATE gw_core GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(gw_tests)
endif()
