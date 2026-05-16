#!/usr/bin/env bash
# GuessWork bootstrap installer.
#
# Installs every dependency needed to build and run this repository on a fresh
# macOS / Apple Silicon machine, then configures and builds the project.
#
# Usage:
#   ./install.sh                # full install + Debug build into ./build
#   ./install.sh --release      # Release build (also enables the embedded React bundle)
#   ./install.sh --no-build     # install deps only, skip cmake build
#   ./install.sh --with-basalt  # opt in to building Basalt for future VIO work
#
# Calibration runs in a Docker container (Kalibr); this script installs
# Colima + the docker CLI, and if Colima is already running, builds the
# Kalibr image. If Colima isn't up the script just prints how to start it.
#
# Idempotent: re-running is safe; existing installs are detected and skipped.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_ROOT"

BUILD_TYPE="Debug"
BUILD_DIR="build"
DO_BUILD=1
EMBED_WEB="OFF"
WITH_BASALT=0

for arg in "$@"; do
    case "$arg" in
        --release)     BUILD_TYPE="Release"; EMBED_WEB="ON" ;;
        --no-build)    DO_BUILD=0 ;;
        --with-basalt) WITH_BASALT=1 ;;
        -h|--help)
            sed -n '2,18p' "$0"
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
    cmake          # build system
    ninja          # required by Basalt's CMake presets (opt-in via --with-basalt)
    pkg-config     # used by find_library/find_path lookups
    openssl@3      # libdatachannel WebRTC stack
    node           # web/ frontend build (npm + Vite)
    colima         # Linux VM that runs the Kalibr container
    docker         # docker CLI; talks to Colima's docker socket
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

# CMake version check (Basalt requires >= 3.24 when --with-basalt is used).
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
# Kalibr container — build once so the per-session runner doesn't have to
# wait 15-30 min on its first invocation. The runner (docker/kalibr/
# calibrate.sh) is what brings Colima up/down on demand at calibration time,
# so we don't keep the VM running here — we start it only long enough to
# build the image, then stop it back down. Skipped if the image is already
# present (Docker Desktop or a pre-built cache).
# ---------------------------------------------------------------------------
KALIBR_IMAGE="${GW_KALIBR_IMAGE:-guesswork/kalibr:latest}"
# Default Colima VM resources. The Kalibr image build is memory-intensive
# (Eigen-heavy templates) and the upstream Colima default of 2 CPU / 2 GB
# OOM-segfaults during numpy_eigen. 4/8 is a safe minimum on a 16 GB Mac.
# Override per-host via env vars.
COLIMA_CPU="${COLIMA_CPU:-4}"
COLIMA_MEMORY="${COLIMA_MEMORY:-8}"
COLIMA_DISK="${COLIMA_DISK:-60}"

# On Apple Silicon, use macOS Virtualization.framework + Rosetta 2 for amd64
# emulation. The qemu fallback is much slower AND has known bugs that
# segfault python3.8 during the Kalibr image build's apt-get triggers.
COLIMA_EXTRA=()
if [[ "$(uname -m)" == "arm64" ]]; then
    if ! arch -arch x86_64 /usr/bin/true >/dev/null 2>&1; then
        log "Installing Rosetta 2 (required for amd64 emulation under Colima)…"
        softwareupdate --install-rosetta --agree-to-license
    fi
    COLIMA_EXTRA=(--vm-type=vz --vz-rosetta)
fi

HAD_DOCKER=1
if ! docker info >/dev/null 2>&1; then
    HAD_DOCKER=0
    log "Starting Colima (cpu=$COLIMA_CPU memory=${COLIMA_MEMORY}G disk=${COLIMA_DISK}G ${COLIMA_EXTRA[*]:-}) just to build the Kalibr image…"
    # `colima start` ignores --cpu/--memory/--disk and the vz/rosetta flags
    # if a profile already exists — the values only apply on first boot. If
    # the user has a too-small or qemu-backed existing profile, they need:
    # `colima delete && rerun install.sh`.
    colima start --cpu "$COLIMA_CPU" --memory "$COLIMA_MEMORY" --disk "$COLIMA_DISK" "${COLIMA_EXTRA[@]}"
    if ! docker info >/dev/null 2>&1; then
        fail "Docker still not reachable after \`colima start\`. Run \`colima status\` to diagnose."
    fi
fi
if docker image inspect "$KALIBR_IMAGE" >/dev/null 2>&1; then
    log "Kalibr image $KALIBR_IMAGE already present — skipping build."
else
    log "Building Kalibr image $KALIBR_IMAGE (first run takes 15-30 min under amd64 emulation)…"
    "$REPO_ROOT/docker/kalibr/build.sh"
fi
if [[ $HAD_DOCKER -eq 0 ]]; then
    log "Stopping Colima — the per-session runner will bring it back up when you calibrate."
    colima stop || true
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
    EXTRA_FLAGS=()
    if [[ $WITH_BASALT -eq 1 ]]; then
        EXTRA_FLAGS+=(-DGW_BUILD_BASALT=ON)
        log "Enabling -DGW_BUILD_BASALT=ON (15-30 min first-time vcpkg bootstrap)."
    fi

    cmake \
        -S "$REPO_ROOT" \
        -B "$REPO_ROOT/$BUILD_DIR" \
        -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        -DGW_BUILD_WEB="$EMBED_WEB" \
        "${EXTRA_FLAGS[@]}"
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
