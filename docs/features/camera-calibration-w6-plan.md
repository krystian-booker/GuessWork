# W6 — End-to-End Orchestrator Subcommand (Detailed Plan)

## Context

W1–W5 turned the calibration backend into a complete plumbing chain:
target catalog (W1), quality gate + thresholds (W2), multi-camera Kalibr
ingestion (W3), conditional IMU sync (W4), and a list/delete CLI for
dataset rows (W5). The result is a robust three-step CLI: operators run
`record-kalibr-dataset → make-kalibr-bag → calibrate-camera`, optionally
followed by `calibrate-camera-imu`.

That works, but it's still three (or four) shell invocations the operator
has to chain by hand. Each step has its own set of flags. There is no
single function the future HTTP server's `POST /api/calibrate` endpoint
can call — it would have to recreate the chaining logic itself, with
every flag-translation it implies.

W6 introduces **`calibrate-camera-end-to-end`**: one CLI invocation that
records → bags → runs Kalibr → optionally runs Kalibr camera-IMU →
ingests + gates + persists. Crucially:

- The orchestrator function (not the CLI shell) is the single entry
  point the HTTP layer will eventually wrap. The legacy three-step CLI
  stays — power users keep their tooling.
- Every Kalibr / Docker shell-out becomes injectable so an end-to-end
  test can pre-stage Kalibr fixture YAMLs and exercise the full
  orchestrator with a fake `system()`. This is the piece that's been
  conspicuously missing since W1; it lands here so the orchestrator
  ships with end-to-end coverage rather than just unit-tested
  building blocks.

## Decisions baked in

- **Default keeps the dataset on disk.** After a successful run the
  recording directory and `kalibr_datasets` row stick around. The
  operator opts into deletion with `--cleanup-dataset`, which routes
  through the same code path as `delete-kalibr-dataset --remove-files`
  (W5). Re-running Kalibr against the same recording is a real workflow
  for tuning; defaulting to delete would make it impossible.
- **`--imu PATH` is required from the operator** when `--mode=intrinsic+imu`.
  Same contract as the existing `calibrate-camera-imu` subcommand. No
  auto-generation from `TeensyConfig.imu`; sensor noise densities for
  Kalibr are calibration-grade, not fusion-grade, and the operator
  knows their hardware.
- **Partial failure persists what passed.** If the intrinsic step's
  W2 gate clears and Kalibr converged, those rows are saved before
  the camera-IMU step runs. An IMU-step failure exits non-zero with
  the underlying error; the operator can re-run just
  `calibrate-camera-imu` against the same dataset without redoing
  the multi-minute intrinsic optimization.

## Critical files to modify

| File | What changes |
|---|---|
| `include/posest/runtime/Daemon.h` | New `DaemonCommand::CalibrateCameraEndToEnd`; new `CalibrateCameraEndToEndOptions` struct (vectors for camera ids/topics/poses, target_id, version, output_dir, duration_s, mode, imu_path, force, max_reprojection_rms_px, docker_image, require_imu, cleanup_dataset); new field on `DaemonOptions`. Public free function `setSystemImplForTesting(std::function<int(const char*)>)` + `resetSystemImplForTesting()` so tests can swap the shell-out. |
| `src/runtime/Daemon.cpp` | New `g_system` function pointer in the anonymous namespace (default `::system`). Replace every `std::system(cmd.c_str())` call site (4 today: `CalibrateCamera`, `MakeKalibrBag`, `CalibrateCameraImu` × 2 invocations) with `g_system(cmd.c_str())`. Definitions for `setSystemImplForTesting` / `resetSystemImplForTesting`. Subcommand parser branch + flag parsing for `--mode`, `--cleanup-dataset`. New top-level helper `runCalibrationEndToEnd(options, store, camera_factory)` that orchestrates; new `DaemonCommand::CalibrateCameraEndToEnd` dispatch. `daemonUsage` line. |
| `test/test_calibrate_end_to_end.cpp` (new) | End-to-end orchestrator test using a real `SqliteConfigStore`, `UnusedCameraFactory` adapted to a stubbed `IFrameProducer`, `setSystemImplForTesting` to capture-and-skip Docker invocations, and pre-staged `camchain_two_cameras.yaml` + `results-cam_two_cameras.txt` (already in W2/W3 fixtures) at the path the parser expects. Asserts: success persists 2 calibrations + 2 extrinsics + 1 cam-to-cam; failed-RMS fixture throws and persists nothing; `--cleanup-dataset` removes both row and directory; `--mode=intrinsic+imu` invokes the camera-IMU command sequence. |
| `test/test_daemon.cpp` | Parse tests for the new subcommand; rejection tests for missing required flags and missing `--imu` when mode is `intrinsic+imu`. |

## Step-by-step

### Step 1 — Option struct + enum value

In `include/posest/runtime/Daemon.h`:

```cpp
enum class DaemonCommand {
    Run,
    CalibrateCamera,
    ImportFieldLayout,
    RecordKalibrDataset,
    MakeKalibrBag,
    CalibrateCameraImu,
    ImportCalibrationTarget,
    ListKalibrDatasets,
    DeleteKalibrDataset,
    CalibrateCameraEndToEnd,         // NEW
};

enum class CalibrationMode {
    Intrinsic,
    IntrinsicAndImu,
};

struct CalibrateCameraEndToEndOptions {
    // Per-camera (W3 shape, vectors index-aligned).
    std::vector<std::string> camera_ids;
    std::vector<std::string> topics;
    std::vector<Pose3d> camera_to_robots;

    // Recording + Kalibr inputs.
    std::string target_id;                     // W1; required
    std::filesystem::path output_dir;          // dataset + Kalibr outputs
    std::string version;
    double duration_s{0.0};
    ImuRequirement require_imu{ImuRequirement::Auto};

    // Mode + IMU-cam inputs.
    CalibrationMode mode{CalibrationMode::Intrinsic};
    std::filesystem::path imu_path;            // required iff IntrinsicAndImu

    // W2 quality gate overrides.
    bool force{false};
    std::optional<double> max_reprojection_rms_px;

    // Docker image override (resolved through resolvedKalibrDockerImage
    // when empty).
    std::string docker_image;

    // After success, remove the dataset directory + kalibr_datasets row.
    bool cleanup_dataset{false};
};
```

Add to `DaemonOptions`:

```cpp
CalibrateCameraEndToEndOptions calibrate_camera_end_to_end;
```

Public helpers for tests, declared in `Daemon.h`:

```cpp
// W6: swap the shell-out used by every Docker invocation. Returns the
// previous implementation. Tests call setSystemImplForTesting() at
// SetUp and resetSystemImplForTesting() at TearDown.
using SystemImpl = std::function<int(const char*)>;
SystemImpl setSystemImplForTesting(SystemImpl impl);
void resetSystemImplForTesting();
```

### Step 2 — `g_system` injection point

In the anonymous namespace at the top of `src/runtime/Daemon.cpp`:

```cpp
SystemImpl& systemImplStorage() {
    static SystemImpl impl = [](const char* cmd) { return std::system(cmd); };
    return impl;
}

int runShell(const char* cmd) { return systemImplStorage()(cmd); }
```

(A function-local static avoids static-init-order pitfalls and is
trivially thread-safe in C++11+.)

Definitions of the public helpers, at namespace scope:

```cpp
SystemImpl setSystemImplForTesting(SystemImpl impl) {
    auto previous = systemImplStorage();
    systemImplStorage() = std::move(impl);
    return previous;
}

void resetSystemImplForTesting() {
    systemImplStorage() = [](const char* cmd) { return std::system(cmd); };
}
```

Replace every `std::system(cmd.c_str())` call site in `runConfigCommand`
with `runShell(cmd.c_str())`. Today there are four:

- `CalibrateCamera` branch — Kalibr cameras docker
- `MakeKalibrBag` branch — make_rosbag.py docker
- `CalibrateCameraImu` branch — make_rosbag.py docker
- `CalibrateCameraImu` branch — kalibr_calibrate_imu_camera docker

After this swap every existing test path keeps working (default impl
calls `std::system`) and the orchestrator gains injectability for free.

### Step 3 — CLI parsing

In `parseDaemonOptions`, add the subcommand recognizer next to the
others:

```cpp
} else if (arg == "calibrate-camera-end-to-end") {
    if (options.command != DaemonCommand::Run) {
        throw std::invalid_argument("only one subcommand may be provided");
    }
    options.command = DaemonCommand::CalibrateCameraEndToEnd;
}
```

Two new flag branches (the rest reuse existing `--camera-id`/`--topic`/
`--camera-to-robot`/`--target-id`/`--output-dir`/`--version`/`--imu`/
`--require-imu`/`--max-reprojection-rms-px`/`--force`/`--docker-image`/
`--duration-s` parsers, with each one tee-ing into the new options
struct's vector / scalar field):

```cpp
} else if (arg == "--mode") {
    const std::string value = requireValue(i, argc, argv, arg);
    if (value == "intrinsic") {
        options.calibrate_camera_end_to_end.mode =
            CalibrationMode::Intrinsic;
    } else if (value == "intrinsic+imu") {
        options.calibrate_camera_end_to_end.mode =
            CalibrationMode::IntrinsicAndImu;
    } else {
        throw std::invalid_argument(
            "--mode must be one of {intrinsic, intrinsic+imu}, got: " + value);
    }
} else if (arg == "--cleanup-dataset") {
    options.calibrate_camera_end_to_end.cleanup_dataset = true;
}
```

For each existing flag the orchestrator reuses, the parser fans out:
e.g. `--camera-id` already pushes to
`record_kalibr_dataset.camera_ids` and `calibrate_camera.camera_ids`;
extend it to also push to
`calibrate_camera_end_to_end.camera_ids`. Same idiom for `--topic`,
`--camera-to-robot`, `--target-id`, `--output-dir`, `--version`,
`--imu`, `--require-imu`, `--duration-s`, `--max-reprojection-rms-px`,
`--force`, `--docker-image`. Net effect: a single CLI invocation with
all these flags populates each subcommand's struct, and only the active
command's struct is read at dispatch time.

Post-parse validation:

```cpp
} else if (options.command == DaemonCommand::CalibrateCameraEndToEnd) {
    const auto& cmd = options.calibrate_camera_end_to_end;
    const auto n = cmd.camera_ids.size();
    if (n == 0 ||
        cmd.topics.size() != n ||
        cmd.camera_to_robots.size() != n ||
        cmd.target_id.empty() ||
        cmd.output_dir.empty() ||
        cmd.version.empty() ||
        !(cmd.duration_s > 0.0)) {
        throw std::invalid_argument(
            "calibrate-camera-end-to-end requires equal-count "
            "--camera-id/--topic/--camera-to-robot, plus --target-id, "
            "--output-dir, --version, --duration-s");
    }
    if (cmd.mode == CalibrationMode::IntrinsicAndImu &&
        cmd.imu_path.empty()) {
        throw std::invalid_argument(
            "calibrate-camera-end-to-end --mode=intrinsic+imu requires --imu PATH");
    }
}
```

### Step 4 — `daemonUsage` line

Append to `daemonUsage()`:

```
posest_daemon calibrate-camera-end-to-end --config PATH \
    --camera-id ID... --topic T... --camera-to-robot x,y,z,r,p,y... \
    --target-id ID --output-dir DIR --version V --duration-s N \
    [--mode={intrinsic,intrinsic+imu}] [--imu IMU.yaml] \
    [--require-imu={auto,yes,no}] [--max-reprojection-rms-px X] \
    [--force] [--docker-image IMAGE] [--cleanup-dataset]
```

### Step 5 — `runCalibrationEndToEnd` orchestrator

New function in `src/runtime/Daemon.cpp` (declared in the public header
so the future HTTP layer can call it directly without going through
`runConfigCommand`):

```cpp
void runCalibrationEndToEnd(
    const DaemonOptions& options,
    config::IConfigStore& config_store,
    ICameraBackendFactory& camera_factory);
```

Body, in dependency order:

1. **Recording (W4).** Load config, build a `CalibrationRecorderConfig`
   honouring `cmd.require_imu` (mirror the `RecordKalibrDataset`
   branch's resolution: Auto → use Teensy iff `serial_port` non-empty;
   Yes → require time sync; No → skip Teensy). Build
   `CalibrationRecorder`, add cameras as consumers, run for
   `cmd.duration_s`. On exit `recorder.throwIfUnacceptable()`. Then
   `rememberDataset(...)` and save config (so the dataset row exists
   regardless of subsequent failures — useful for debugging).
2. **Bag (existing).** Build `MakeKalibrBagOptions`:
   `dataset_dir = output_dir`, `bag_path = output_dir/kalibr.bag`,
   `no_imu` auto-set from `datasetHasImuSamples(output_dir)` (W4
   helper). Resolve docker image. `runShell(buildMakeKalibrBagDockerCommand(...))`.
3. **Target (W1).** If `target_path` not provided, materialize via
   `findCalibrationTarget(config, target_id)` +
   `calibration::writeKalibrTargetYaml` → `output_dir / target.yaml`.
4. **Kalibr intrinsic (W3 multi-camera).** Build
   `CalibrateCameraOptions` from the orchestrator's vectors, set
   `bag_path = output_dir/kalibr.bag`, `target_path = materialized`,
   `output_dir = output_dir`, propagate `force` /
   `max_reprojection_rms_px`. `runShell(buildKalibrDockerCommand(...))`.
5. **Parse + gate (W3 + W2).** `parseKalibrAllCameras` against
   `findKalibrCamchain(output_dir)` with the same topic→id map.
   `parseKalibrCameraResults` against `findKalibrResultsCam(...)`.
   For each camera, attach the metric matched by Kalibr label
   (`cam0`, `cam1`, ...), set `report_path = findKalibrReportCam(output_dir, "report-cam")`,
   then `throwIfUnacceptableCalibration(cal, effective_tool, force)`.
6. **Persist intrinsics + cam-to-cam (W3).** For each camera,
   `replaceCalibration` with the matching `CameraExtrinsicsConfig`.
   For each cam-to-cam edge, `replaceCameraToCameraExtrinsics`. Save
   config. **This save commits before the IMU step runs** so
   intrinsic results survive an IMU failure (the partial-failure
   policy).
7. **Camera-IMU step, only if `mode == IntrinsicAndImu`.** Build
   a `CalibrateCameraImuOptions` mirroring the orchestrator's flags
   (`dataset_dir = output_dir`, `target_path = materialized`,
   `imu_path = options.imu_path`, `version = options.version`,
   `force`, `max_reprojection_rms_px`). Run the existing
   `CalibrateCameraImu` flow inline by calling a small helper
   refactored out of `runConfigCommand`'s existing branch
   (`runCameraImuCalibration(options, config_store)` with the same
   body — the `runConfigCommand` `CalibrateCameraImu` branch becomes
   a thin wrapper). Re-loads config to pick up the just-saved
   intrinsics, runs the bag + IMU-cam dockers, parses results,
   gates per-camera with `throwIfUnacceptableCameraImu`, writes
   `replaceCameraImuCalibration`, saves.
8. **Cleanup, only if `cmd.cleanup_dataset`.** Reuse the
   `DeleteKalibrDataset` dispatch's logic (refactor the body into
   `cleanupKalibrDataset(config, store, dataset_id, remove_files=true)`
   so both code paths share it). Removes the row + directory.

Wire as the dispatch branch in `runConfigCommand`:

```cpp
if (options.command == DaemonCommand::CalibrateCameraEndToEnd) {
    runCalibrationEndToEnd(options, config_store, camera_factory);
    return;
}
```

### Step 6 — Minor refactors implied

Two small refactors fall out:

- Extract `runCameraImuCalibration` from the existing
  `CalibrateCameraImu` branch in `runConfigCommand` so it's callable
  from the orchestrator. The branch keeps working; it becomes a
  one-line wrapper.
- Extract `cleanupKalibrDataset(config_store, id, remove_files)` from
  the W5 `DeleteKalibrDataset` branch so the orchestrator's
  `--cleanup-dataset` path doesn't duplicate the delete logic.

Both are straightforward "lift the body, leave the dispatch as a
caller" moves. No behavior change for the existing CLI.

### Step 7 — Tests

**`test/test_daemon.cpp`** — parse tests:

- `ParsesCalibrateCameraEndToEndCommand` — full multi-camera flag set
  including `--mode=intrinsic+imu --imu /tmp/imu.yaml --cleanup-dataset`.
  Assert every field on the struct.
- `RejectsCalibrateCameraEndToEndMissingCounts` — 2 camera-ids, 1 topic.
- `RejectsCalibrateCameraEndToEndIntrinsicImuWithoutImuFlag` — mode set,
  `--imu` absent.
- `RejectsCalibrateCameraEndToEndUnknownMode` — `--mode=foo`.

**`test/test_calibrate_end_to_end.cpp`** (new) — full dispatch:

Test fixture sets `setSystemImplForTesting` to a lambda that:
1. Captures the command string into a `std::vector<std::string>`.
2. On a `kalibr_calibrate_cameras` command, copies a pre-staged
   `camchain_two_cameras.yaml` and `results-cam_two_cameras.txt` from
   `test/fixtures/kalibr/` to the output dir Kalibr would have written
   to (the test uses an output-dir under `temp_directory_path`).
3. On a `make_rosbag.py` command, writes a stub `kalibr.bag` (empty
   file is enough; the orchestrator never reads it).
4. On a `kalibr_calibrate_imu_camera` command, copies a pre-staged
   `camchain-imucam_two_cameras.yaml` (small new fixture) +
   `results-imucam_pass.txt` (already in W2 fixtures).
5. Returns 0 by default; the failure-path test returns non-zero or
   stages a `_fail.txt` fixture.

Camera capture is mocked by a dirt-simple `IFrameProducer` stub that
runs a thread, generates a single `Frame` with `cv::Mat::zeros(4,4,CV_8UC3)`
once, and feeds the recorder so `frames_seen >= 1` and
`throwIfUnacceptable` clears (with `min_trigger_match_fraction = 0.0`
because `--require-imu=no` in the test).

Cases:

- `IntrinsicSuccessPersistsCalibrationsAndBaseline` — two cameras,
  `--mode=intrinsic --require-imu=no`. Stage pass fixtures. Assert
  loaded config: 2 `calibrations` rows with non-zero
  `reprojection_rms_px`, 2 `camera_extrinsics`, 1
  `camera_to_camera_extrinsics`, 1 `kalibr_datasets` row.
- `FailedGateRollsBackNoCalibrations` — same shape, but stage
  `results-cam_fail.txt` (RMS ~2.27 px > 1.0 default). Assert orchestrator
  throws; assert `loaded.calibrations.empty()`. The
  `kalibr_datasets` row exists (committed early) — that's expected;
  the operator can re-run after diagnosing.
- `IntrinsicAndImuPersistsBoth` — `--mode=intrinsic+imu`, stage
  intrinsic + IMU fixtures. Assert 2 `calibrations`, 2
  `camera_imu_calibrations`.
- `IntrinsicSucceedsImuFailsKeepsIntrinsics` — intrinsic fixture
  passes; IMU fixture's RMS is above the IMU gate. Assert orchestrator
  throws; assert `loaded.calibrations.size() == 2` (intrinsic
  persisted) and `loaded.camera_imu_calibrations.empty()`.
- `CleanupDatasetRemovesRowAndDirectory` — `--cleanup-dataset`. Assert
  post-success `loaded.kalibr_datasets.empty()` and `output_dir`
  doesn't exist on disk.
- `KeepsDatasetByDefault` — without `--cleanup-dataset`. Assert
  `kalibr_datasets.size() == 1` and `output_dir` still on disk.

### Step 8 — Build + ctest

```
cmake --build --preset conan-release -j
ctest --preset conan-release --output-on-failure
```

Net delta: ≈ 4 parse tests + 6 orchestrator tests = ~10 new tests on
top of W5's 315.

## Reused functions / utilities

- `CalibrationRecorder` (`src/calibration/CalibrationRecorder.cpp`) —
  step 1, unchanged.
- `rememberDataset`, `datasetHasImuSamples`, `findCalibrationTarget`
  (`src/runtime/Daemon.cpp`) — steps 1, 2, 3 unchanged.
- `calibration::writeKalibrTargetYaml` (`src/calibration/CalibrationTargetWriter.cpp`)
  — step 3.
- `buildMakeKalibrBagDockerCommand`, `buildKalibrDockerCommand`,
  `buildCalibrateCameraImuDockerCommand` (`src/runtime/Daemon.cpp`) —
  unchanged from W3/W4.
- `parseKalibrAllCameras`, `parseKalibrCameraResults`,
  `parseKalibrCameraImuResults`,
  `parseKalibrCameraImuCalibration`,
  `findKalibrCamchain`, `findKalibrResultsCam`, `findKalibrReportCam`,
  `findKalibrImuCamchain`, `findKalibrResultsImuCam`
  (W2 + W3 + existing) — all reused unchanged.
- `throwIfUnacceptableCalibration`,
  `throwIfUnacceptableCameraImu` (W2) — reused per-camera.
- `replaceCalibration`, `replaceCameraToCameraExtrinsics`,
  `replaceCameraImuCalibration` (`src/runtime/Daemon.cpp`) —
  unchanged.
- `cleanupKalibrDataset` — extracted from W5's
  `DeleteKalibrDataset` dispatch in step 6.
- `runCameraImuCalibration` — extracted from existing
  `CalibrateCameraImu` dispatch in step 6.
- `SqliteConfigStore::saveRuntimeConfig`'s atomic transaction —
  ensures the intrinsic save in step 6 is all-or-nothing.

## Verification

1. **Build + ctest:** all 315 W5 tests + ~10 new ones pass.
2. **Parse coverage:**
   `./build/Release/posest_tests --gtest_filter='DaemonOptions*EndToEnd*'`.
3. **Orchestrator coverage:**
   `./build/Release/posest_tests --gtest_filter='CalibrateEndToEnd*'`.
4. **Manual smoke (post-merge), single camera, intrinsic-only:**
   ```
   posest_daemon import-calibration-target --target-id apgrid_88 \
       --type aprilgrid --rows 6 --cols 6 \
       --tag-size-m 0.088 --tag-spacing-ratio 0.3
   posest_daemon calibrate-camera-end-to-end \
       --camera-id cam0 --topic /posest/cam0/image_raw \
       --camera-to-robot 0.10,0,0.20,0,0,0 \
       --target-id apgrid_88 --output-dir ./datasets/cal-$(date +%s) \
       --version v1 --duration-s 60 --require-imu no
   posest_daemon list-kalibr-datasets --json | jq .
   ```
   Expect a single `kalibr_datasets` row, a `calibrations` row with
   non-zero `reprojection_rms_px`, and a `camera_extrinsics` row.
5. **Manual smoke, two cameras + IMU:**
   ```
   posest_daemon calibrate-camera-end-to-end \
       --camera-id cam0 --camera-id cam1 \
       --topic /posest/cam0/image_raw --topic /posest/cam1/image_raw \
       --camera-to-robot 0.10,0,0.20,0,0,0 \
       --camera-to-robot -0.10,0,0.20,0,0,0 \
       --target-id apgrid_88 --output-dir ./datasets/cal-imu \
       --version v1 --duration-s 90 \
       --mode intrinsic+imu --imu ./imu.yaml \
       --cleanup-dataset
   ```
   Expect 2 `calibrations`, 2 `camera_extrinsics`, 1
   `camera_to_camera_extrinsics`, 2 `camera_imu_calibrations`. After
   completion, `./datasets/cal-imu` should not exist on disk and
   `list-kalibr-datasets` should be empty.

## Out of scope for W6

- HTTP server / WebUI integration. The whole point is the orchestrator
  function is callable; the future server simply wraps it.
- `g_system` tightening (e.g., a virtual interface or per-call
  injection). The function-pointer-in-anon-namespace approach is the
  smallest swap; we revisit if a future test needs per-instance
  isolation.
- Auto-IMU YAML generation from `TeensyConfig` (requires sensor-grade
  noise densities; deferred until the operator workflow demands it).
- Camera-IMU cam-to-cam refresh from `camchain-imucam.yaml`. Kalibr
  doesn't optimize cam-to-cam in the IMU step, so the W3 rows stay
  authoritative; same out-of-scope decision as W3 documented.
