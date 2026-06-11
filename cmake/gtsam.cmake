# GTSAM (factor-graph backend for the Phase 6 fusion engine) — source build
# via ExternalProject; no Homebrew formula exists.
#
# Pinned to 4.3a1 (prerelease) rather than the stable 4.2.x line, for two
# verified reasons:
#   - 4.2.x hard-requires Boost; Homebrew Boost 1.90 removed legacy compiled
#     stubs and already broke one dependency build (OpenVINS — see
#     openvins.cmake). At 4.3a1 the two flags below give a fully Boost-free
#     build. Fallback if that ever regresses: set both Boost flags ON —
#     4.3a1's cmake/HandleBoost.cmake requests no `system` component, so
#     Boost 1.90 is safe either way.
#   - IncrementalFixedLagSmoother moved from gtsam_unstable into core gtsam
#     at 4.3a1 (the gtsam_unstable header is now a deprecation shim), so we
#     build with GTSAM_BUILD_UNSTABLE=OFF.
#
# Other options: system Eigen (same Homebrew Eigen as gw_vio — avoids mixing
# two Eigen versions across our TUs), TBB off (deterministic single-thread
# solves), nested dissection off (drops the vendored metis dylib; our graphs
# are tiny chains). GTSAM_POSE3_EXPMAP/GTSAM_ROT3_EXPMAP default ON — Pose3
# retract = full SE(3) expmap with right perturbation, exactly the covariance
# convention TagPoseMeasurement/VioOdometry already use. Do not touch them.
#
# macOS arm64 fix (same class as the OpenVINS one): newer libc++ no longer
# transitively provides assert(); gtsam/geometry/SL4.cpp uses assert without
# including <cassert> — fixed with `-include cassert`.
#
# License: GTSAM is BSD — no effect on guesswork binary distribution.

include(ExternalProject)

set(GTSAM_GIT_TAG "4.3a1" CACHE STRING
    "GTSAM release tag (Boost-free era; IncrementalFixedLagSmoother in core)")
set(GTSAM_INSTALL_DIR "${CMAKE_BINARY_DIR}/gtsam-install" CACHE PATH
    "GTSAM install prefix")

ExternalProject_Add(gtsam_external
    GIT_REPOSITORY      https://github.com/borglab/gtsam.git
    GIT_TAG             ${GTSAM_GIT_TAG}
    GIT_SHALLOW         TRUE
    UPDATE_DISCONNECTED TRUE
    CMAKE_GENERATOR     Ninja
    CMAKE_ARGS
        -DCMAKE_BUILD_TYPE=Release
        -DCMAKE_INSTALL_PREFIX=${GTSAM_INSTALL_DIR}
        -DCMAKE_CXX_FLAGS=-include\ cassert
        -DGTSAM_ENABLE_BOOST_SERIALIZATION=OFF
        -DGTSAM_USE_BOOST_FEATURES=OFF
        -DGTSAM_BUILD_UNSTABLE=OFF
        -DGTSAM_BUILD_PYTHON=OFF
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF
        -DGTSAM_BUILD_TESTS=OFF
        -DGTSAM_BUILD_TIMING_ALWAYS=OFF
        -DGTSAM_BUILD_TYPE_POSTFIXES=OFF
        -DGTSAM_WITH_TBB=OFF
        -DGTSAM_USE_SYSTEM_EIGEN=ON
        -DGTSAM_SUPPORT_NESTED_DISSECTION=OFF
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
    INSTALL_DIR         ${GTSAM_INSTALL_DIR}
    BUILD_BYPRODUCTS    ${GTSAM_INSTALL_DIR}/lib/libgtsam.dylib
)

# INTERFACE_INCLUDE_DIRECTORIES must exist at configure time.
file(MAKE_DIRECTORY ${GTSAM_INSTALL_DIR}/include)

add_library(gtsam_lib SHARED IMPORTED GLOBAL)
set_target_properties(gtsam_lib PROPERTIES
    IMPORTED_LOCATION             ${GTSAM_INSTALL_DIR}/lib/libgtsam.dylib
    INTERFACE_INCLUDE_DIRECTORIES ${GTSAM_INSTALL_DIR}/include
)
target_link_options(gtsam_lib INTERFACE "-Wl,-rpath,${GTSAM_INSTALL_DIR}/lib")
add_dependencies(gtsam_lib gtsam_external)
