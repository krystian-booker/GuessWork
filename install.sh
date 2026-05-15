#!/usr/bin/env bash
# GuessWork bootstrap installer.
#
# Installs every dependency needed to build and run this repository on a fresh
# macOS / Apple Silicon machine, then configures and builds the project.
#
# Usage:
#   ./install.sh              # full install + Debug build into ./build
#   ./install.sh --release    # Release build (also enables the embedded React bundle)
#   ./install.sh --no-build   # install deps only, skip cmake build
#
# Basalt is a hard runtime requirement, so the first CMake configure always
# passes -DGW_BUILD_BASALT=ON. That triggers a 15-30 min vcpkg bootstrap on a
# fresh checkout; subsequent builds reuse the cached basalt-install/ tree.
#
# Idempotent: re-running is safe; existing installs are detected and skipped.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_ROOT"

BUILD_TYPE="Debug"
BUILD_DIR="build"
DO_BUILD=1
EMBED_WEB="OFF"

for arg in "$@"; do
    case "$arg" in
        --release)    BUILD_TYPE="Release"; EMBED_WEB="ON" ;;
        --no-build)   DO_BUILD=0 ;;
        -h|--help)
            sed -n '2,15p' "$0"
            exit 0
            ;;
        *) echo "unknown flag: $arg" >&2; exit 2 ;;
    esac
done

log()  { printf '\033[1;34m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m!!\033[0m  %s\n' "$*" >&2; }
fail() { printf '\033[1;31mxx\033[0m  %s\n' "$*" >&2; exit 1; }

# ---------------------------------------------------------------------------
# Platform sanity check
# ---------------------------------------------------------------------------
[[ "$(uname -s)" == "Darwin" ]] || fail "GuessWork only builds on macOS."
[[ "$(uname -m)" == "arm64" ]] || warn "Detected non-arm64 ($(uname -m)). The project targets Apple Silicon; Intel Macs may not work."

# ---------------------------------------------------------------------------
# Xcode Command Line Tools
# ---------------------------------------------------------------------------
if ! xcode-select -p >/dev/null 2>&1; then
    log "Installing Xcode Command Line Tools (a GUI dialog will appear)…"
    xcode-select --install || true
    fail "Re-run install.sh after the Command Line Tools install completes."
fi
log "Xcode CLT: $(xcode-select -p)"

# ---------------------------------------------------------------------------
# Homebrew
# ---------------------------------------------------------------------------
if ! command -v brew >/dev/null 2>&1; then
    log "Installing Homebrew…"
    NONINTERACTIVE=1 /bin/bash -c \
        "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    # Make brew available in the current shell.
    eval "$(/opt/homebrew/bin/brew shellenv)"
fi
log "Homebrew: $(brew --prefix)"

# ---------------------------------------------------------------------------
# Homebrew packages
# ---------------------------------------------------------------------------
BREW_PKGS=(
    cmake          # build system (need >= 3.24 for Basalt's CMake step)
    ninja          # required by Basalt's CMake presets
    pkg-config     # used by find_library/find_path lookups
    openssl@3      # libdatachannel WebRTC stack
    libspng        # PNG encoder for RecordingConsumer
    node           # web/ frontend build (npm + Vite)
)

log "Installing Homebrew packages: ${BREW_PKGS[*]}"
brew update
for pkg in "${BREW_PKGS[@]}"; do
    if brew list --formula --versions "$pkg" >/dev/null 2>&1; then
        log "  $pkg already installed"
    else
        brew install "$pkg"
    fi
done

# CMake version check (Basalt requires >= 3.24).
CMAKE_VER="$(cmake --version | head -n1 | awk '{print $3}')"
log "cmake: $CMAKE_VER"

# ---------------------------------------------------------------------------
# Spinnaker SDK (manual install — FLIR/Teledyne requires a license click-through)
# ---------------------------------------------------------------------------
SPINNAKER_LIB="/usr/local/lib/libSpinnaker.dylib"
SPINNAKER_INC="/usr/local/include/spinnaker"

if [[ ! -f "$SPINNAKER_LIB" || ! -d "$SPINNAKER_INC" ]]; then
    cat <<EOF >&2

------------------------------------------------------------------------
Spinnaker SDK not found at /usr/local/{lib,include}/.

GuessWork links against FLIR/Teledyne's Spinnaker SDK, which can't be
distributed via Homebrew and requires accepting a license. To install:

  1. Sign in / create an account at:
       https://www.teledynevisionsolutions.com/products/spinnaker-sdk/
  2. Download the macOS ARM64 SDK package.
  3. Run the installer; it places headers at /usr/local/include/spinnaker
     and dylibs at /usr/local/lib (libSpinnaker.dylib, libGenApi*, …).
  4. Re-run ./install.sh.

Skipping the CMake build for now; everything else is set up.
------------------------------------------------------------------------
EOF
    DO_BUILD=0
else
    log "Spinnaker SDK: $SPINNAKER_LIB"
fi

# ---------------------------------------------------------------------------
# Web frontend deps
# ---------------------------------------------------------------------------
if [[ -d "$REPO_ROOT/web" ]]; then
    log "Installing web/ npm dependencies…"
    (cd "$REPO_ROOT/web" && npm install)
fi

# ---------------------------------------------------------------------------
# Optional: pre-install Playwright browsers for e2e tests.
# Cheap if already cached; skip silently on failure (network-restricted CI).
# ---------------------------------------------------------------------------
if [[ -d "$REPO_ROOT/web" ]] && command -v npx >/dev/null 2>&1; then
    log "Installing Playwright browsers (for web/ e2e tests)…"
    (cd "$REPO_ROOT/web" && npx --yes playwright install --with-deps chromium) || \
        warn "Playwright install failed — e2e tests won't run, but the rest of the build is fine."
fi

# ---------------------------------------------------------------------------
# CMake configure + build
# ---------------------------------------------------------------------------
if [[ $DO_BUILD -eq 1 ]]; then
    # Only force GW_BUILD_BASALT=ON on a fresh build tree. Once Basalt has been
    # built into $BUILD_DIR/basalt-install/, the cached cache var keeps it on,
    # and forcing the flag again would re-run ExternalProject's configure step.
    BASALT_FLAG=()
    if [[ ! -x "$REPO_ROOT/$BUILD_DIR/basalt-install/bin/basalt_calibrate" ]]; then
        BASALT_FLAG=(-DGW_BUILD_BASALT=ON)
        log "Basalt not yet built — first configure will trigger the 15-30 min vcpkg bootstrap."
    else
        log "Basalt already built at $BUILD_DIR/basalt-install/ — reusing."
    fi

    cmake \
        -S "$REPO_ROOT" \
        -B "$REPO_ROOT/$BUILD_DIR" \
        -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        -DGW_BUILD_WEB="$EMBED_WEB" \
        "${BASALT_FLAG[@]}"
    log "Building $BUILD_TYPE into $BUILD_DIR/ …"
    cmake --build "$REPO_ROOT/$BUILD_DIR" -j

    log "Build complete: $REPO_ROOT/$BUILD_DIR/guesswork"
else
    log "Skipping CMake build (deps-only install)."
fi

cat <<EOF

Next steps:
  • Run the server:           ./$BUILD_DIR/guesswork
  • Frontend dev (separate):  cd web && npm run dev   (proxies /api -> :8080)
  • Run unit tests:           ctest --test-dir $BUILD_DIR --output-on-failure
  • Reset the DB if schema changed:  ./$BUILD_DIR/guesswork --reset-db
EOF
