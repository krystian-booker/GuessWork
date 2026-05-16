# Build the Basalt VIO toolchain from source as part of our CMake build.
# Driven entirely via ExternalProject_Add so that Basalt's vcpkg-managed
# dependencies (Eigen, Sophus, TBB, Pangolin, OpenCV, fmt, ...) do NOT
# pollute our target tree — we only consume the installed artifacts.
#
# Calibration moved to Kalibr (Dockerized); the app no longer reads anything
# this module produces. The option exists so a future VIO experiment can
# pull in basalt_vio without a separate install.
#
# First-time build cost: 15-30 min (vcpkg bootstraps and builds the whole
# transitive dep tree). Cached for subsequent builds; pass
# `-DGW_BUILD_BASALT=OFF` (the default) to skip the build entirely.

include(ExternalProject)

# Basalt requires CMake 3.24+ for its own configure step. We don't bump our
# top-level minimum (3.20) — ExternalProject_Add launches Basalt's CMake in a
# separate process so its requirement is checked independently of ours.
find_program(BASALT_NINJA ninja REQUIRED
    DOC "Ninja is required to build Basalt (its CMake presets are Ninja-only)")

set(BASALT_INSTALL_DIR "${CMAKE_BINARY_DIR}/basalt-install"
    CACHE PATH "Install prefix for the embedded Basalt build")

# Pin to a known-good revision. Bump when Basalt ships a tagged release with
# fixes we need. Master is moving but Apple Silicon support is in.
set(BASALT_GIT_TAG "master"
    CACHE STRING "Basalt git revision (tag, branch, or commit)")

ExternalProject_Add(basalt_external
    GIT_REPOSITORY         https://gitlab.com/VladyslavUsenko/basalt.git
    GIT_TAG                ${BASALT_GIT_TAG}
    GIT_SUBMODULES_RECURSE TRUE
    # Avoid re-fetching submodules on every reconfigure (vcpkg is huge).
    UPDATE_DISCONNECTED    TRUE
    CMAKE_GENERATOR        Ninja
    CMAKE_ARGS
        -DCMAKE_BUILD_TYPE=Release
        -DCMAKE_INSTALL_PREFIX=${BASALT_INSTALL_DIR}
        # Use Basalt's bundled vcpkg manifest. Path is relative to the cloned
        # source tree (Basalt vendors vcpkg as `thirdparty/vcpkg`).
        -DCMAKE_TOOLCHAIN_FILE=<SOURCE_DIR>/thirdparty/vcpkg/scripts/buildsystems/vcpkg.cmake
    INSTALL_DIR            ${BASALT_INSTALL_DIR}
    BUILD_BYPRODUCTS       ${BASALT_INSTALL_DIR}/bin/basalt_vio
)
