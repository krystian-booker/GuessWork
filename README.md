# GuessWork

Robot pose-estimation daemon. A single Linux/macOS process drains
frames from one or more cameras through vision pipelines
(AprilTag, Kimera-VIO), fuses the resulting measurements with
IMU + wheel/odometry data via a GTSAM factor graph, and ships
fused poses to a Teensy 4.1 companion over USB. SQLite holds
the runtime config; a small web facade exposes live edits and
health.

## Repository layout

- `src/`, `include/posest/` — daemon code, split by area
  (`core`, `camera`, `config`, `runtime`, `pipelines`, `fusion`,
  `vio`, `teensy`, `v4l2`, `calibration`).
- `firmware/teensy41/` — PlatformIO firmware for the Teensy
  companion.
- `share/posest/kimera/` — static Kimera-VIO YAML templates the
  daemon expands at boot (`KimeraParamWriter`).
- `scripts/` — one-shot installers and tooling (Kimera-VIO
  dependency install, protocol-constant drift check).
- `test/` — GoogleTest suite.
- `docs/features/` — deeper architecture per feature.
- `CLAUDE.md` — code style + conventions.

## Linux setup (Ubuntu 20.04 / 22.04 / 24.04)

The build pulls most third-party libraries through Conan, but
**OpenCV, GTSAM, and the rest of the Kimera-VIO dependency chain
(OpenGV, DBoW2, Kimera-RPGO, Kimera-VIO) are installed system-wide
and are not on Conan Center.** The same OpenCV and GTSAM installs
back `posest_core` / `posest_fusion`; running a second copy of
either through Conan would put two physically distinct
`libopencv_core.so.410`s (or two GTSAMs) in the daemon binary and
corrupt cv::Mat / gtsam types at the ABI seam. Conan still owns
the rest of the dependency closure (gtest, sqlite3, nlohmann_json,
apriltag, yaml-cpp).

### One-shot install

```bash
bash scripts/install_kimera_deps.sh
```

This:
1. Installs apt prerequisites (`cmake`, `libboost-all-dev`,
   `libtbb-dev`, `libgflags-dev`, `libgoogle-glog-dev`,
   `libopencv-dev`, `libvtk{7,9}-dev`, etc.).
2. Builds and installs **GTSAM 4.2** to `/usr/local` with the
   flags Kimera requires
   (`POSE3_EXPMAP=ON`, `ROT3_EXPMAP=ON`,
   `TANGENT_PREINTEGRATION=OFF`, `USE_SYSTEM_EIGEN=ON`).
3. Builds and installs **OpenGV**, **DBoW2**, **Kimera-RPGO**,
   and **Kimera-VIO** (master branches; pin the `*_REF`
   constants in the script for reproducibility).
4. Runs `sudo ldconfig`.

First run takes ~30–45 min on a recent multi-core machine.
Subsequent runs short-circuit any step whose install artifact
already exists. Override `PREFIX`, `WORKDIR`, or `JOBS` via env
vars; pass `SKIP_APT=1` to skip the apt step on hosts where
you've already installed the system packages.

### Conan + CMake

After the install script succeeds:

```bash
conan install . --build=missing -s build_type=Release
cmake --preset conan-release
cmake --build --preset conan-release -j
ctest --preset conan-release --output-on-failure
```

`conan install` writes a `CMakeUserPresets.json` (gitignored)
that wires the conan toolchain into the `conan-release` preset.

### Manual fallback (non-Ubuntu, custom prefix, or step-by-step)

If `scripts/install_kimera_deps.sh` does not match your
environment, follow the upstream Kimera-VIO install guide
manually:
<https://github.com/MIT-SPARK/Kimera-VIO/blob/master/docs/kimera_vio_install.md>

Key requirements:
- GTSAM **must** be built with `GTSAM_POSE3_EXPMAP=ON`,
  `GTSAM_ROT3_EXPMAP=ON`,
  `GTSAM_TANGENT_PREINTEGRATION=OFF`,
  `GTSAM_USE_SYSTEM_EIGEN=ON`. TBB optional but recommended.
- Install order matters: GTSAM → OpenGV → DBoW2 → Kimera-RPGO
  → Kimera-VIO. Each later step finds the earlier one via
  `find_package` against the install prefix.
- `cmake/FindKimeraVIO.cmake` searches `/usr/local`,
  `/opt/homebrew`, and `$KimeraVIO_ROOT`. Set
  `KimeraVIO_ROOT=/your/prefix` if you install elsewhere.

## macOS setup

```bash
brew install cmake boost tbb vtk gflags glog opencv
```

Then build GTSAM, OpenGV, DBoW2, Kimera-RPGO, and Kimera-VIO
from source per the upstream guide above, installing into
`/opt/homebrew` (Apple Silicon) or `/usr/local` (Intel).
`scripts/install_kimera_deps.sh` is Linux-only today; a
sibling macOS installer is a follow-up.

After Kimera installs:

```bash
conan install . --build=missing -s build_type=Release
cmake --preset conan-release
cmake --build --preset conan-release -j
ctest --preset conan-release --output-on-failure
```

## Building & running tests

Iterate on a single GTest case (faster than `ctest`):

```bash
./build/Release/posest_tests --gtest_filter='MockPipeline.SlowConsumerDropsButProducerIsNotBlocked'
```

Re-run a flaky test under load:

```bash
./build/Release/posest_tests --gtest_filter='X.*' --gtest_repeat=10
```

## Running the daemon

```bash
./build/Release/posest_daemon --config /path/to/posest.db
```

Useful flags:

- `--health-once` — print the health JSON snapshot and exit.
- `--health-interval-ms 250` — emit a health snapshot every
  250 ms while running.
- `--help` — full usage including the one-shot calibration
  subcommands (`calibrate-camera`, `import-field-layout`,
  `record-kalibr-dataset`, `make-kalibr-bag`,
  `calibrate-camera-imu`).

## Disabling Kimera-VIO at build time

If you don't need the real Kimera backend (CI, dev sandboxes,
machines where the Kimera dependency chain isn't installed):

```bash
cmake -DPOSEST_BUILD_VIO=OFF --preset conan-release
cmake --build --preset conan-release -j
```

The daemon keeps building. The "vio" pipeline slot drops to
`FakeVioBackend` (a deterministic synthetic-trajectory backend
covered by the test suite). All non-Kimera tests stay green;
the gated `KimeraBackend.*` smoke tests are simply not
compiled. Note that GTSAM still has to be available — either
system-installed (recommended, see above) or via your own
local build — because `posest_fusion` links it directly.

## Teensy 4.1 firmware

Firmware lives under `firmware/teensy41/` (PlatformIO). The
host-side wire format in `include/posest/teensy/Protocol.h` is
authoritative — change there first, then mirror in firmware:

```bash
pio run -d firmware/teensy41
pio run -d firmware/teensy41 -t upload
pio device monitor -d firmware/teensy41 -b 921600
```

## Where to read next

- `docs/features/` — feature-level architecture
  (producer/consumer pipeline, camera producer, calibration,
  fusion, VIO).
- `CLAUDE.md` — code style, naming, and the latency contract
  that the producer/consumer plumbing must honour.
