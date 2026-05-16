# Kalibr container

Used by the camera calibration page (`/cameras/<id>/calibrate`) to run
`kalibr_calibrate_cameras` against the rosbag the app records.

## Provenance

`Dockerfile` is adapted from Kalibr's upstream
[`Dockerfile_ros1_20_04`](https://github.com/ethz-asl/kalibr/blob/master/Dockerfile_ros1_20_04)
pinned to commit `1f60227442d25e36365ef5f72cd80b9666d73467` (March 2024).

The only divergence from upstream is that we `git clone` Kalibr inside the
build instead of using the whole repo as the build context, so the build
context stays this directory only.

## Build

`install.sh` invokes this for you on a fresh setup. To rebuild manually:

```bash
./build.sh                   # builds guesswork/kalibr:latest (~15-30 min on first run)
```

## Resource tuning

The Kalibr build is memory-hungry — `numpy_eigen` instantiates a lot of
Eigen templates and will OOM-segfault `cc1plus` if the VM is too small or
parallelism is too high. Both knobs default conservatively:

- **VM:** install.sh and calibrate.sh start Colima with `--cpu 4 --memory 8
  --disk 60`. Override with `COLIMA_CPU` / `COLIMA_MEMORY` / `COLIMA_DISK`
  env vars before running either script. Values only apply on *first*
  `colima start`; if an existing too-small profile is the problem, run
  `colima delete` and then `./install.sh` (or any other script that brings
  Colima up).
- **Image build:** `catkin build -j2` inside the container. Bump for a
  larger VM with `GW_KALIBR_BUILD_JOBS=<N> ./build.sh`.

## Architecture note

The image is amd64-only because Kalibr depends on amd64 ROS Noetic debs.
On Apple Silicon, Colima must use Rosetta 2 for amd64 emulation — `qemu`
(the default) segfaults python3.8 during the apt-get triggers in the
Kalibr image build. `install.sh` and `calibrate.sh` both detect arm64 and
pass `--vm-type=vz --vz-rosetta` automatically; Rosetta is installed by
`install.sh` if missing.

If you have an existing qemu-backed Colima profile from before this fix,
recreate it:

```bash
colima delete
./install.sh   # boots a new Rosetta-backed VM
```

## Update the pinned commit

Bump `KALIBR_GIT_REF` in `Dockerfile`, rebuild, smoke-test that the
suggested calibration command still runs end-to-end, then update the SHA in
this README.
