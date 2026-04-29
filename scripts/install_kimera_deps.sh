#!/usr/bin/env bash
#
# install_kimera_deps.sh — build and install Kimera-VIO and its
# dependency chain into /usr/local on Ubuntu 20.04 / 22.04 / 24.04.
#
# Idempotent: rerun safely. Each step short-circuits if the
# install artifact already exists. To force a rebuild, delete the
# corresponding lib (e.g. `sudo rm /usr/local/lib/libgtsam.so*`)
# and rerun.
#
# Refs are pinned to upstream master HEADs — bump the *_REF
# constants below deliberately, not casually. Test the full
# posest build after any bump.
#
# Environment overrides:
#   PREFIX   install prefix             default: /usr/local
#   WORKDIR  scratch build directory    default: ~/.posest_kimera_build
#   JOBS     parallel build jobs        default: $(nproc)
#   SKIP_APT set to 1 to skip apt step  default: unset
#
# Source: distilled from
# https://github.com/MIT-SPARK/Kimera-VIO/blob/master/docs/kimera_vio_install.md

set -euo pipefail

readonly PREFIX="${PREFIX:-/usr/local}"
readonly WORKDIR="${WORKDIR:-${HOME}/.posest_kimera_build}"
readonly JOBS="${JOBS:-$(nproc)}"

readonly GTSAM_REF=4.2
readonly OPENGV_REF=master
readonly DBOW2_REF=master
readonly KIMERA_RPGO_REF=master
readonly KIMERA_VIO_REF=master

# CMake 4.x dropped compatibility with cmake_minimum_required < 3.5,
# which several upstream deps (notably GTSAM 4.2) still declare. The
# CMake-suggested escape hatch is to set CMAKE_POLICY_VERSION_MINIMUM
# at configure time. Apply it to every dep configure to make this
# script work on Ubuntu 25.10 / modern macOS / any host with CMake
# 4.x. Harmless on older CMake (the variable is silently ignored).
readonly CMAKE_POLICY_FALLBACK="-DCMAKE_POLICY_VERSION_MINIMUM=3.5"

# Boost.Math 1.90 (Ubuntu 25.10) ships
# /usr/include/boost/math/special_functions/trunc.hpp using
# boost::math::enable_if_t without including the header that
# defines it (boost/math/tools/type_traits.hpp). The fix is to
# force every TU to pre-include that header — but it itself
# references std::underlying_type_t etc., so <type_traits> has
# to come first. Order matters: -include flags are processed in
# command-line order. Affects KimeraRPGO + KimeraVIO, which pull
# GTSAM's GncOptimizer.h → boost::math::quantile → trunc.hpp.
readonly BOOST_MATH_FIX_CXXFLAGS="-include type_traits -include boost/math/tools/type_traits.hpp"

# OpenCV 4.10 (Ubuntu 24.04+) stopped pulling <opencv2/viz.hpp> in
# transitively from the headers Kimera-VIO already includes. Several
# TUs (RgbdCamera.cpp, EurocPlayground.cpp, Mesh.cpp, MeshOptimization.cpp)
# use cv::viz::Color / cv::viz::Mesh / cv::viz::WMesh without including
# the umbrella header, so they fail with "'viz' is not a member of 'cv'".
# Force-include the header on every C++ TU. Harmless on TUs that already
# include it (header guards) and on older OpenCV (header still exists).
# Requires libopencv-viz-dev on the host.
#
# CMake's compiler-detection probe runs BEFORE find_package(OpenCV)
# adds the OpenCV include dir, so the bare -include cannot resolve the
# header. We have to put -I on the same flag string. pkg-config gives
# us "-I/usr/include/opencv4" (or whatever the host uses); fall back
# to the Ubuntu default if pkg-config is missing.
readonly OPENCV_INCLUDE_FLAGS="$(pkg-config --cflags-only-I opencv4 2>/dev/null || echo -I/usr/include/opencv4)"
readonly OPENCV_VIZ_FIX_CXXFLAGS="${OPENCV_INCLUDE_FLAGS} -include opencv2/viz.hpp"

log() { printf '\n[install_kimera_deps] %s\n' "$*"; }

ensure_workdir() {
    mkdir -p "${WORKDIR}"
}

clone_or_update() {
    local repo_url="$1"
    local ref="$2"
    local dest="$3"
    if [ -d "${dest}/.git" ]; then
        log "updating $(basename "${dest}") (ref=${ref})"
        git -C "${dest}" fetch --depth 1 origin "${ref}"
        git -C "${dest}" checkout "${ref}"
    else
        log "cloning $(basename "${dest}") (ref=${ref})"
        git clone --depth 1 --branch "${ref}" "${repo_url}" "${dest}"
    fi
}

step_apt() {
    if [ "${SKIP_APT:-0}" = "1" ]; then
        log "SKIP_APT=1 set; skipping apt step"
        return 0
    fi
    log "installing apt prerequisites"
    sudo apt-get update
    # Core build + BLAS/LAPACK + sparse linalg + Boost/TBB + image libs +
    # logging. libopenblas-dev replaces libatlas-base-dev (retired in
    # newer Ubuntu releases); libmetis-dev replaces libparmetis-dev for
    # GTSAM's non-MPI graph partitioning use.
    sudo apt-get install -y \
        cmake build-essential pkg-config unzip git \
        libboost-all-dev libtbb-dev \
        libjpeg-dev libpng-dev libtiff-dev \
        libgtk-3-dev gfortran \
        libopenblas-dev liblapack-dev \
        libmetis-dev libsuitesparse-dev \
        libgflags-dev libgoogle-glog-dev \
        libopencv-dev libopencv-contrib-dev
    # VTK package name varies by Ubuntu release: libvtk7-dev on 20.04,
    # libvtk9-dev on 22.04+. Try newest first; ignore failure (Kimera's
    # core build does not strictly require VTK).
    sudo apt-get install -y libvtk9-dev \
        || sudo apt-get install -y libvtk7-dev \
        || true
}

step_gtsam() {
    if [ -f "${PREFIX}/lib/libgtsam.so" ] || [ -f "${PREFIX}/lib/libgtsam.dylib" ]; then
        log "GTSAM already installed under ${PREFIX} — skipping"
        return 0
    fi
    log "building GTSAM ${GTSAM_REF}"
    clone_or_update https://github.com/borglab/gtsam.git "${GTSAM_REF}" "${WORKDIR}/gtsam"
    # Boost ≥1.69 made boost::system header-only and dropped libboost_system.so.
    # GTSAM 4.2 was tagged before that and still requires Boost_SYSTEM_LIBRARY
    # to be set; on Ubuntu 25.10 (Boost 1.90) the find_package call fails. Drop
    # the system component from HandleBoost.cmake — it's header-only now, so
    # nothing to link, and GTSAM's source still picks up <boost/system/*.hpp>
    # transitively. Reset the file first so a previously botched run heals,
    # then match " system " (with both surrounding spaces) so the substring
    # inside "filesystem " can't be eaten on subsequent runs.
    local handle_boost="${WORKDIR}/gtsam/cmake/HandleBoost.cmake"
    git -C "${WORKDIR}/gtsam" checkout -- cmake/HandleBoost.cmake
    sed -i '/^set(BOOST_FIND_MINIMUM_COMPONENTS/s/ system / /' "${handle_boost}"
    sed -i 's/NOT Boost_SYSTEM_LIBRARY OR //' "${handle_boost}"
    sed -i '/^  Boost::system$/d' "${handle_boost}"
    cmake -S "${WORKDIR}/gtsam" -B "${WORKDIR}/gtsam/build" \
        ${CMAKE_POLICY_FALLBACK} \
        -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
        -DCMAKE_BUILD_TYPE=Release \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_POSE3_EXPMAP=ON \
        -DGTSAM_ROT3_EXPMAP=ON \
        -DGTSAM_TANGENT_PREINTEGRATION=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_PYTHON=OFF
    cmake --build "${WORKDIR}/gtsam/build" -j "${JOBS}"
    sudo cmake --install "${WORKDIR}/gtsam/build"
}

step_opengv() {
    if [ -f "${PREFIX}/lib/libopengv.a" ] || [ -f "${PREFIX}/lib/libopengv.so" ]; then
        log "OpenGV already installed under ${PREFIX} — skipping"
        return 0
    fi
    log "building OpenGV (${OPENGV_REF})"
    clone_or_update https://github.com/laurentkneip/opengv.git "${OPENGV_REF}" "${WORKDIR}/opengv"
    cmake -S "${WORKDIR}/opengv" -B "${WORKDIR}/opengv/build" \
        ${CMAKE_POLICY_FALLBACK} \
        -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
        -DCMAKE_BUILD_TYPE=Release
    cmake --build "${WORKDIR}/opengv/build" -j "${JOBS}"
    sudo cmake --install "${WORKDIR}/opengv/build"
}

step_dbow2() {
    if [ -f "${PREFIX}/lib/libDBoW2.so" ] || [ -f "${PREFIX}/lib/libDBoW2.dylib" ]; then
        log "DBoW2 already installed under ${PREFIX} — skipping"
        return 0
    fi
    log "building DBoW2 (${DBOW2_REF})"
    clone_or_update https://github.com/dorian3d/DBoW2.git "${DBOW2_REF}" "${WORKDIR}/DBoW2"
    cmake -S "${WORKDIR}/DBoW2" -B "${WORKDIR}/DBoW2/build" \
        ${CMAKE_POLICY_FALLBACK} \
        -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
        -DCMAKE_BUILD_TYPE=Release
    cmake --build "${WORKDIR}/DBoW2/build" -j "${JOBS}"
    sudo cmake --install "${WORKDIR}/DBoW2/build"
}

step_kimera_rpgo() {
    if [ -f "${PREFIX}/lib/libKimeraRPGO.so" ] || [ -f "${PREFIX}/lib/libKimeraRPGO.dylib" ]; then
        log "Kimera-RPGO already installed under ${PREFIX} — skipping"
        return 0
    fi
    log "building Kimera-RPGO (${KIMERA_RPGO_REF})"
    clone_or_update https://github.com/MIT-SPARK/Kimera-RPGO.git "${KIMERA_RPGO_REF}" "${WORKDIR}/Kimera-RPGO"
    # Wipe any previous failed configure so the new flags take effect.
    rm -rf "${WORKDIR}/Kimera-RPGO/build"
    # KimeraRPGO's CMakeLists pins CMAKE_CXX_STANDARD=11 via a plain
    # set() that shadows any -DCMAKE_CXX_STANDARD passed at configure.
    # Boost.Math 1.90 needs ≥14 for std::is_final / enable_if_t /
    # underlying_type_t. Patch the CMakeLists in-place to bump it to
    # 17 before configuring. Idempotent: re-running matches the
    # already-bumped form (`STANDARD 17`) and is a no-op.
    sed -i 's/set(CMAKE_CXX_STANDARD 11)/set(CMAKE_CXX_STANDARD 17)/' \
        "${WORKDIR}/Kimera-RPGO/CMakeLists.txt"
    cmake -S "${WORKDIR}/Kimera-RPGO" -B "${WORKDIR}/Kimera-RPGO/build" \
        ${CMAKE_POLICY_FALLBACK} \
        -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_FLAGS="${BOOST_MATH_FIX_CXXFLAGS}"
    cmake --build "${WORKDIR}/Kimera-RPGO/build" -j "${JOBS}"
    sudo cmake --install "${WORKDIR}/Kimera-RPGO/build"
}

step_kimera_vio() {
    if [ -f "${PREFIX}/lib/libKimeraVIO.so" ] || [ -f "${PREFIX}/lib/libKimeraVIO.dylib" ]; then
        log "Kimera-VIO already installed under ${PREFIX} — skipping"
        return 0
    fi
    log "building Kimera-VIO (${KIMERA_VIO_REF})"
    clone_or_update https://github.com/MIT-SPARK/Kimera-VIO.git "${KIMERA_VIO_REF}" "${WORKDIR}/Kimera-VIO"
    rm -rf "${WORKDIR}/Kimera-VIO/build"
    cmake -S "${WORKDIR}/Kimera-VIO" -B "${WORKDIR}/Kimera-VIO/build" \
        ${CMAKE_POLICY_FALLBACK} \
        -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
        -DCMAKE_BUILD_TYPE=Release \
        -DKIMERA_BUILD_TESTS=OFF \
        -DKIMERA_BUILD_EXAMPLES=OFF \
        -DCMAKE_CXX_STANDARD=17 \
        -DCMAKE_CXX_STANDARD_REQUIRED=ON \
        -DCMAKE_CXX_FLAGS="${BOOST_MATH_FIX_CXXFLAGS} ${OPENCV_VIZ_FIX_CXXFLAGS}"
    cmake --build "${WORKDIR}/Kimera-VIO/build" -j "${JOBS}"
    sudo cmake --install "${WORKDIR}/Kimera-VIO/build"
}

main() {
    ensure_workdir
    step_apt
    step_gtsam
    step_opengv
    step_dbow2
    step_kimera_rpgo
    step_kimera_vio
    sudo ldconfig
    log "Kimera-VIO + dependencies installed under ${PREFIX}."
    log "Build scratch left in ${WORKDIR} — safe to delete."
    log "Next: conan install . --build=missing -s build_type=Release"
}

main "$@"
