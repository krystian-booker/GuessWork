# FindKimeraVIO.cmake
#
# Custom find module for MIT-SPARK/Kimera-VIO. Kimera does not ship a
# portable Config.cmake — its CMake exports a target only when the
# downstream consumer was built inside Kimera's own source tree.
#
# Variables consumed:
#   KimeraVIO_ROOT  — install prefix (or build dir) of Kimera-VIO.
#                     Headers are expected at $ROOT/include/kimera-vio,
#                     library at $ROOT/lib/libKimeraVIO.{a,dylib,so}.
#                     Falls back to standard system paths if unset.
#
# Provides on success:
#   KimeraVIO_FOUND
#   KimeraVIO_INCLUDE_DIRS
#   KimeraVIO_LIBRARIES
#   imported target KimeraVIO::KimeraVIO
#
# Notes:
#   - Kimera links DBoW2, OpenGV, glog, gflags. They must be installed
#     and visible to the linker. On Apple use Homebrew (`brew install
#     glog gflags`) and build OpenGV/DBoW2 from source. On Linux, glog
#     and gflags are typically available via apt.
#   - Kimera's GTSAM dependency is satisfied by the project's existing
#     find_package(GTSAM) — they must agree on version (4.2). If
#     Kimera was built against a different GTSAM, expect ABI surprises.

find_path(KimeraVIO_INCLUDE_DIR
    NAMES kimera-vio/pipeline/Pipeline.h kimera-vio/Pipeline.h
    HINTS
        ${KimeraVIO_ROOT}/include
        $ENV{KimeraVIO_ROOT}/include
    PATHS
        /usr/local/include
        /opt/homebrew/include
)

find_library(KimeraVIO_LIBRARY
    NAMES KimeraVIO kimera_vio
    HINTS
        ${KimeraVIO_ROOT}/lib
        $ENV{KimeraVIO_ROOT}/lib
    PATHS
        /usr/local/lib
        /opt/homebrew/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(KimeraVIO
    REQUIRED_VARS KimeraVIO_INCLUDE_DIR KimeraVIO_LIBRARY
    FAIL_MESSAGE "Kimera-VIO headers/library not found in /usr/local, \
/opt/homebrew, or \$KimeraVIO_ROOT. Run scripts/install_kimera_deps.sh to \
build the Kimera-VIO dependency chain from source, or set \
-DPOSEST_BUILD_VIO=OFF to skip the Kimera adapter (the daemon falls back to \
FakeVioBackend). See README.md."
)

if(KimeraVIO_FOUND AND NOT TARGET KimeraVIO::KimeraVIO)
    add_library(KimeraVIO::KimeraVIO UNKNOWN IMPORTED)
    set_target_properties(KimeraVIO::KimeraVIO PROPERTIES
        IMPORTED_LOCATION "${KimeraVIO_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${KimeraVIO_INCLUDE_DIR}"
    )
    set(KimeraVIO_INCLUDE_DIRS ${KimeraVIO_INCLUDE_DIR})
    set(KimeraVIO_LIBRARIES ${KimeraVIO_LIBRARY})
endif()

mark_as_advanced(KimeraVIO_INCLUDE_DIR KimeraVIO_LIBRARY)
