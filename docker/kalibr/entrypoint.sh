#!/usr/bin/env bash
# Image entrypoint. Sources the catkin workspace's setup.bash so any command
# the caller passes (rosrun kalibr ..., python3 ..., bash) finds ROS on PATH,
# then exec's that command.
#
# Critical: setup.bash inspects its own positional args via `$@`, so we have
# to clear them before sourcing, then restore them for the final exec.

set -e

saved=("$@")
set --
# shellcheck disable=SC1091
source "$WORKSPACE/devel/setup.bash"
cd "$WORKSPACE"

if [[ ${#saved[@]} -eq 0 ]]; then
    exec /bin/bash
fi
exec "${saved[@]}"
