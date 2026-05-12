# Fetch and configure libdatachannel for our embedded WebRTC use case.
#
# We use the bundled vendored deps (libjuice, libsrtp, mbedtls, plog) to keep
# the build self-contained: nothing on the host system needs to be installed.
include(FetchContent)

# libdatachannel's vendored deps (especially plog) carry very old
# cmake_minimum_required lines that fail to configure on modern CMake. Set
# the policy floor to satisfy them; affects only the fetched deps' scope.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "" FORCE)

set(NO_EXAMPLES   ON  CACHE BOOL "" FORCE)
set(NO_TESTS      ON  CACHE BOOL "" FORCE)
set(NO_WEBSOCKET  ON  CACHE BOOL "" FORCE)  # we use Crow for HTTP/WS instead
set(USE_MBEDTLS    OFF CACHE BOOL "" FORCE)  # use OpenSSL (homebrew) instead
set(ENABLE_MBEDTLS OFF CACHE BOOL "" FORCE)  # propagated to bundled libsrtp
set(ENABLE_OPENSSL ON  CACHE BOOL "" FORCE)  # propagated to bundled libsrtp
set(USE_NICE      OFF CACHE BOOL "" FORCE)  # use bundled libjuice for ICE
set(NO_MEDIA      OFF CACHE BOOL "" FORCE)  # we need RTP/H264 packetizer
set(PREFER_SYSTEM_LIB OFF CACHE BOOL "" FORCE)

# Point at homebrew OpenSSL on Apple Silicon.
if(APPLE AND EXISTS "/opt/homebrew/opt/openssl@3")
    list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew/opt/openssl@3")
    set(OPENSSL_ROOT_DIR "/opt/homebrew/opt/openssl@3" CACHE PATH "" FORCE)
endif()

FetchContent_Declare(
    libdatachannel
    GIT_REPOSITORY https://github.com/paullouisageneau/libdatachannel.git
    GIT_TAG        v0.21.2
    GIT_SUBMODULES_RECURSE TRUE
)
FetchContent_MakeAvailable(libdatachannel)
