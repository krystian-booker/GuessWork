# OpenVINS (stereo+IMU MSCKF) — ROS-free source build via ExternalProject.
#
# Configures only the ov_msckf subdirectory; its cmake/ROS1.cmake globs
# ov_core + ov_init + ov_msckf sources into ONE shared library
# (libov_msckf_lib.dylib) and installs headers under include/open_vins/.
#
# Pinned to a master SHA rather than the v2.7 release: v2.7 predates the
# ceres::Manifold migration and does not compile against Homebrew's
# Ceres 2.2 (upstream issue #385).
#
# macOS arm64 fixes (verified by the Phase 4 spike build):
#   - Boost 1.90 dropped the legacy boost_system compiled stub (Boost.System
#     is header-only since 1.69), so `find_package(Boost COMPONENTS system …)`
#     fails — PATCH_COMMAND removes the component.
#   - Newer libc++ no longer transitively provides assert(); upstream is
#     missing <cassert> includes — fixed with `-include cassert`.
#   - Runtime: the build uses ENABLE_ARUCO_TAGS=OFF, so VioManagerOptions
#     must set use_aruco=false (handled in openvins_runner.cpp).
#
# License note: OpenVINS is GPL-3.0 — a distributed guesswork binary that
# links it is effectively GPLv3.
#
# Deps (Homebrew): eigen, boost, ceres-solver, opencv.

include(ExternalProject)

set(OPENVINS_GIT_TAG "69488123ed9362dd44b6f28e7f4680abbff1442b" CACHE STRING
    "OpenVINS commit to build (master with the Ceres 2.2 Manifold fix)")
set(OPENVINS_INSTALL_DIR "${CMAKE_BINARY_DIR}/openvins-install" CACHE PATH
    "OpenVINS install prefix")

ExternalProject_Add(openvins_external
    GIT_REPOSITORY      https://github.com/rpng/open_vins.git
    GIT_TAG             ${OPENVINS_GIT_TAG}
    GIT_SHALLOW         FALSE
    UPDATE_DISCONNECTED TRUE
    SOURCE_SUBDIR       ov_msckf
    PATCH_COMMAND
        sed -i "" -e
        "s/find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)/find_package(Boost REQUIRED COMPONENTS filesystem thread date_time)/"
        <SOURCE_DIR>/ov_msckf/CMakeLists.txt
    CMAKE_GENERATOR     Ninja
    CMAKE_ARGS
        -DCMAKE_BUILD_TYPE=Release
        -DENABLE_ROS=OFF
        -DENABLE_ARUCO_TAGS=OFF
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5
        -DCMAKE_CXX_FLAGS=-include\ cassert
        -DCMAKE_INSTALL_PREFIX=${OPENVINS_INSTALL_DIR}
    INSTALL_DIR         ${OPENVINS_INSTALL_DIR}
    BUILD_BYPRODUCTS    ${OPENVINS_INSTALL_DIR}/lib/libov_msckf_lib.dylib
)

# INTERFACE_INCLUDE_DIRECTORIES must exist at configure time.
file(MAKE_DIRECTORY ${OPENVINS_INSTALL_DIR}/include/open_vins)

add_library(ov_msckf_lib SHARED IMPORTED GLOBAL)
set_target_properties(ov_msckf_lib PROPERTIES
    IMPORTED_LOCATION             ${OPENVINS_INSTALL_DIR}/lib/libov_msckf_lib.dylib
    INTERFACE_INCLUDE_DIRECTORIES ${OPENVINS_INSTALL_DIR}/include/open_vins
)
target_link_options(ov_msckf_lib INTERFACE "-Wl,-rpath,${OPENVINS_INSTALL_DIR}/lib")
add_dependencies(ov_msckf_lib openvins_external)
