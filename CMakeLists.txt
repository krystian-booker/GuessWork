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

# --- posest_core: interfaces, Frame, LatestFrameSlot, ProducerBase, ConsumerBase
add_library(posest_core
    src/core/LatestFrameSlot.cpp
    src/core/ProducerBase.cpp
    src/core/ConsumerBase.cpp
)
target_include_directories(posest_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(posest_core PUBLIC opencv::opencv)
target_compile_features(posest_core PUBLIC cxx_std_20)

# --- posest_camera: generic camera abstractions + XML config
find_package(pugixml REQUIRED)

add_library(posest_camera
    src/camera/XmlCameraConfigSource.cpp
    src/camera/CameraProducer.cpp
)
target_link_libraries(posest_camera PUBLIC posest_core PRIVATE pugixml::pugixml)

# --- posest_v4l2: V4L2 camera backend (Linux only)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_library(posest_v4l2
        src/v4l2/V4L2Producer.cpp
    )
    target_link_libraries(posest_v4l2 PUBLIC posest_camera)
endif()

# --- posest_mock: MockProducer / MockConsumer (used by tests)
add_library(posest_mock
    src/mock/MockProducer.cpp
    src/mock/MockConsumer.cpp
)
target_link_libraries(posest_mock PUBLIC posest_core)

# --- camera_config_tool (Linux only, needs V4L2 headers)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_executable(camera_config_tool app/camera_config_tool.cpp)
    target_link_libraries(camera_config_tool PRIVATE posest_core)
endif()

# --- tests
enable_testing()
add_executable(posest_tests
    test/test_latest_frame_slot.cpp
    test/test_producer_base.cpp
    test/test_mock_pipeline.cpp
    test/test_camera_config.cpp
)
target_link_libraries(posest_tests PRIVATE posest_mock posest_camera GTest::gtest GTest::gtest_main)

if(TARGET posest_v4l2)
    target_sources(posest_tests PRIVATE test/test_v4l2_producer.cpp)
    target_link_libraries(posest_tests PRIVATE posest_v4l2)
endif()

include(GoogleTest)
gtest_discover_tests(posest_tests)
