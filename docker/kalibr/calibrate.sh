#!/usr/bin/env bash
# Run Kalibr against one GuessWork recording session, managing the Docker
# host (Colima) lifecycle so it's only running while Kalibr runs.
#
# Usage:
#   ./docker/kalibr/calibrate.sh <session-dir> [pinhole-radtan|pinhole-equi] [bag-freq-hz]
#
# <session-dir> must contain calibration.bag + target.yaml (both produced by
# the recording page). The model defaults to pinhole-radtan; pass
# pinhole-equi for fisheye / wide-FOV lenses.
#
# bag-freq-hz subsamples the bag down to that target rate before Kalibr
# extracts corners — 4 Hz (the default) gives ~120-240 well-spread views
# from a typical 30-60 s recording, which is plenty for intrinsics and
# keeps per-worker memory well under the Colima VM's allocation. Pass 0 to
# pass every frame through (only useful with a beefier VM and slow capture).
#
# Env vars:
#   GW_KALIBR_FOCAL_HINT=<pixels>
#     Optional focal-length hint in pixels for Kalibr's optimizer. Kalibr's
#     auto-init via Zhang's method occasionally fails on real-world bags
#     even with healthy detections; when it does, calibration falls back to
#     all-NaN intrinsics. Setting this env var enables Kalibr's manual-init
#     fallback (which the entrypoint otherwise leaves off because docker
#     run --rm has no TTY for the prompt). A rough estimate is fine — the
#     optimizer refines from there.
#       focal_px ≈ (image_width / 2) / tan(HFOV_deg / 2 * π / 180)
#     e.g. 2048-wide sensor + ~70° HFOV → ~1450 pixels.
#
# Behavior:
#   - If the Docker socket is reachable already (Colima or Docker Desktop is
#     up), we use it and leave it as-is afterward.
#   - If nothing is serving the socket, we run `colima start` before Kalibr
#     and `colima stop` on exit (success, failure, or Ctrl-C — via trap).
#   - The Kalibr image must already be built; run `docker/kalibr/build.sh`
#     once if `docker image inspect guesswork/kalibr:latest` fails.
#
# On success, `camchain-calibration.yaml` is written next to the bag. Upload
# it via the calibration page to store it against the camera row.

set -euo pipefail

SESSION="${1:-}"
MODEL="${2:-pinhole-radtan}"
BAG_FREQ="${3:-4}"
IMAGE="${GW_KALIBR_IMAGE:-guesswork/kalibr:latest}"
FOCAL_HINT="${GW_KALIBR_FOCAL_HINT:-}"
# Colima VM resources if we end up starting the VM ourselves. Only applied on
# the *first* `colima start` — an existing profile keeps whatever it was
# created with. Match install.sh so a fresh user gets one consistent VM.
COLIMA_CPU="${COLIMA_CPU:-4}"
COLIMA_MEMORY="${COLIMA_MEMORY:-8}"
COLIMA_DISK="${COLIMA_DISK:-60}"
# On Apple Silicon use macOS Virtualization.framework + Rosetta for amd64
# emulation (qemu is slower and has known bugs that break the Kalibr build).
COLIMA_EXTRA=()
if [[ "$(uname -m)" == "arm64" ]]; then
    COLIMA_EXTRA=(--vm-type=vz --vz-rosetta)
fi

usage() {
    sed -n '2,21p' "$0" >&2
    exit 2
}

[[ -n "$SESSION" ]] || usage
[[ -d "$SESSION" ]]                    || { echo "session dir not found: $SESSION"   >&2; exit 2; }
[[ -f "$SESSION/calibration.bag" ]]    || { echo "missing $SESSION/calibration.bag"  >&2; exit 2; }
[[ -f "$SESSION/target.yaml" ]]        || { echo "missing $SESSION/target.yaml"      >&2; exit 2; }

case "$MODEL" in
    pinhole-radtan|pinhole-equi) ;;
    *) echo "model must be pinhole-radtan or pinhole-equi (got: $MODEL)" >&2; exit 2 ;;
esac

# Validate BAG_FREQ as a non-negative integer (0 = pass-through, else the
# target rate in Hz that Kalibr subsamples the bag to).
if ! [[ "$BAG_FREQ" =~ ^[0-9]+$ ]]; then
    echo "bag-freq-hz must be a non-negative integer (got: $BAG_FREQ)" >&2; exit 2
fi

# Focal hint: positive number (allow decimals). Empty = no hint, use auto-init.
if [[ -n "$FOCAL_HINT" ]]; then
    if ! [[ "$FOCAL_HINT" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
        echo "GW_KALIBR_FOCAL_HINT must be a positive number (got: $FOCAL_HINT)" >&2
        exit 2
    fi
fi

# Resolve to an absolute path so the docker -v mount is unambiguous regardless
# of cwd; readlink -f isn't on macOS by default, so use cd / pwd.
SESSION_ABS="$(cd "$SESSION" && pwd)"

log() { printf '\033[1;34m==>\033[0m %s\n' "$*"; }

# Track whether *we* started Colima so we only stop what we started. If
# Docker Desktop or a pre-existing Colima profile was already serving the
# socket, we leave it running.
STARTED_COLIMA=0
cleanup() {
    if [[ $STARTED_COLIMA -eq 1 ]]; then
        log "stopping colima (we started it for this run)…"
        colima stop || true
    fi
}
trap cleanup EXIT

if ! docker info >/dev/null 2>&1; then
    if ! command -v colima >/dev/null 2>&1; then
        echo "docker socket not reachable and \`colima\` not on PATH." >&2
        echo "Install with: brew install colima docker" >&2
        exit 2
    fi
    log "docker socket not reachable — starting colima (cpu=$COLIMA_CPU memory=${COLIMA_MEMORY}G ${COLIMA_EXTRA[*]:-})…"
    colima start --cpu "$COLIMA_CPU" --memory "$COLIMA_MEMORY" --disk "$COLIMA_DISK" "${COLIMA_EXTRA[@]}"
    STARTED_COLIMA=1
fi

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "Kalibr image $IMAGE not built." >&2
    echo "Run docker/kalibr/build.sh first (15-30 min, cached after)." >&2
    exit 2
fi

FREQ_ARGS=()
if [[ "$BAG_FREQ" -gt 0 ]]; then
    FREQ_ARGS=(--bag-freq "$BAG_FREQ")
    desc="model=$MODEL, subsampled to ${BAG_FREQ} Hz"
else
    desc="model=$MODEL, full bag"
fi

# Manual focal-length init path: when a hint is provided we run docker with
# `-i` (keep stdin open) and pipe the hint into Kalibr's manual-init prompt.
# The env var inside the container is what activates the prompt; otherwise
# the entrypoint leaves it unset and Kalibr's auto-init is the only path.
if [[ -n "$FOCAL_HINT" ]]; then
    log "running Kalibr on $SESSION_ABS ($desc, focal-hint=${FOCAL_HINT} px)…"
    printf '%s\n' "$FOCAL_HINT" | docker run --rm -i \
        -e KALIBR_MANUAL_FOCAL_LENGTH_INIT=1 \
        -v "$SESSION_ABS":/data \
        -w /data \
        "$IMAGE" \
        rosrun kalibr kalibr_calibrate_cameras \
            --bag /data/calibration.bag \
            --topics /cam0/image_raw \
            --models "$MODEL" \
            --target /data/target.yaml \
            "${FREQ_ARGS[@]}"
else
    log "running Kalibr on $SESSION_ABS ($desc)…"
    docker run --rm \
        -v "$SESSION_ABS":/data \
        -w /data \
        "$IMAGE" \
        rosrun kalibr kalibr_calibrate_cameras \
            --bag /data/calibration.bag \
            --topics /cam0/image_raw \
            --models "$MODEL" \
            --target /data/target.yaml \
            "${FREQ_ARGS[@]}"
fi

# Kalibr writes <bag-basename>-camchain.yaml; for calibration.bag that's
# calibration-camchain.yaml. Surface it explicitly so the user can copy the
# path straight into the upload button.
RESULT="$SESSION_ABS/calibration-camchain.yaml"
if [[ -f "$RESULT" ]]; then
    log "done. camchain: $RESULT"
else
    log "Kalibr finished but $RESULT was not produced — check the output above."
    exit 1
fi
