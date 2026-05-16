#!/usr/bin/env bash
# Build the Kalibr Docker image used by GuessWork's calibration page.
#
# Tag defaults to `guesswork/kalibr:latest` — matches the tag the
# CalibrationSupervisor bakes into the suggested command. Override with
# GW_KALIBR_IMAGE=<tag> ./build.sh if you need to push to a registry.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${GW_KALIBR_IMAGE:-guesswork/kalibr:latest}"
# Parallelism for the `catkin build` step inside the image. Default 2 keeps
# peak memory low so numpy_eigen doesn't OOM-segfault cc1plus. Bump if the
# Colima VM has >= 12 GB RAM allocated.
JOBS="${GW_KALIBR_BUILD_JOBS:-2}"

if ! command -v docker >/dev/null 2>&1; then
    echo "docker not found on PATH. Install via:" >&2
    echo "  brew install colima docker && colima start" >&2
    exit 1
fi
if ! docker info >/dev/null 2>&1; then
    echo "docker daemon not reachable. Start it with:" >&2
    echo "  colima start" >&2
    exit 1
fi

echo "==> Building $IMAGE (first run takes 15-30 min under amd64 emulation; jobs=$JOBS)…"
exec docker build --platform=linux/amd64 \
    --build-arg "KALIBR_BUILD_JOBS=$JOBS" \
    -t "$IMAGE" -f "$HERE/Dockerfile" "$HERE"
