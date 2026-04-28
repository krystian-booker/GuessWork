# Camera Calibration — Backend Implementation Plan

> **Status:** W1 merged (calibration target catalog, default AprilGrid seed,
> `import-calibration-target` subcommand, `--target-id` on `calibrate-camera`,
> 8 new tests passing). W2–W6 still to land.

## Context

Today the camera intrinsic / extrinsic / camera-IMU calibration flow ships as
three CLI subcommands of `posest_daemon` (`record-kalibr-dataset` →
`make-kalibr-bag` → `calibrate-camera` / `calibrate-camera-imu`). The
recording pipeline, ROS-bag emission, Docker invocation, and SQLite
persistence of `CameraCalibrationConfig` / `CameraExtrinsicsConfig` /
`CameraImuCalibrationConfig` are all in place. The audit in
`docs/features/camera-calibration-status.md` identified six gaps that block
the user's brief from being met **before any HTTP server / website work
happens**:

1. No DB-backed calibration target catalog (the `--target` flag is a
   hand-curated YAML path; no AprilGrid default; no size config).
2. Multi-camera Kalibr output is only partially ingested — `T_cn_cnm1`
   (cam-to-cam baselines) is dropped, and a multi-camera run requires N
   separate `calibrate-camera` invocations.
3. No quality rating from Kalibr surfaces — reprojection RMS, observation
   count, and the report path are not parsed or stored, and there is no
   post-Kalibr acceptance gate (a wildly bad run silently overwrites the
   active calibration row).
4. `record-kalibr-dataset` always requires a Teensy time-sync, even for
   intrinsic-only runs — it should be conditional.
5. `kalibr_datasets` rows are write-only — no list/delete CLI; no disk
   cleanup.
6. There is no single-shot orchestrator that runs record → bag → Kalibr
   in one invocation, which both reduces operator error today and gives
   the future HTTP server a single entry point to call.

This plan is the backend work that lands those six gaps. **Out of scope**
(deferred until the website lands): HTTP server, live preview / video
streaming, AprilGrid corner-overlay UI, undistortion preview UI, "review
& activate" UI. Without a UI those features are useful only on a developer
workstation and will be re-built when the website is in place.

## Decisions baked in (from clarifying questions)

- Multi-camera intrinsic ingestion: one `calibrate-camera` invocation
  accepts repeated `--camera-id`s, parses every cam node from Kalibr's
  `camchain.yaml`, and writes both the per-camera intrinsics and the
  cam-to-cam extrinsics in a single SQLite transaction.
- Add a `calibrate-camera-end-to-end` orchestrator that chains record →
  bag → Kalibr in one subcommand. Today's three subcommands stay.
- No live preview / overlay / snapshot tool until the website lands.
- Post-Kalibr quality gate is **on by default**, with a configurable
  threshold and a `--force` override.

---

## Workstreams

Each workstream is independently land-able and passes tests in isolation.
Suggested merge order is W1 → W2 → W3 → W4 → W5 → W6 because W6 (the
end-to-end orchestrator) depends on the wiring from W1–W5.

### W1 — Calibration target catalog (DB-backed, default AprilGrid) — **DONE**

**Goal:** the operator picks a target by id; the daemon materializes the
Kalibr `target.yaml` on demand. Default AprilGrid is seeded.

**SQLite migration** (new `migration11Sql()` in
`src/config/SqliteSchema.cpp`, append to the chain in `applyMigrations()`,
bump `currentSchemaVersion()`):

```sql
CREATE TABLE calibration_targets (
    id TEXT PRIMARY KEY NOT NULL,
    type TEXT NOT NULL,                -- 'aprilgrid' | 'checkerboard' | 'circlegrid'
    rows INTEGER NOT NULL,
    cols INTEGER NOT NULL,
    tag_size_m REAL NOT NULL DEFAULT 0.0,      -- aprilgrid: tag edge length
    tag_spacing_ratio REAL NOT NULL DEFAULT 0.0, -- aprilgrid: spacing/size
    square_size_m REAL NOT NULL DEFAULT 0.0,    -- checkerboard / circlegrid
    tag_family TEXT NOT NULL DEFAULT 'tag36h11',
    notes TEXT NOT NULL DEFAULT ''
);
INSERT OR IGNORE INTO calibration_targets
    (id, type, rows, cols, tag_size_m, tag_spacing_ratio, tag_family)
    VALUES ('default_aprilgrid_6x6', 'aprilgrid', 6, 6, 0.088, 0.3, 'tag36h11');
```

**Struct + validator + load/save:**

- New `CalibrationTargetConfig` in `include/posest/runtime/RuntimeConfig.h`
  with the columns above. Add `std::vector<CalibrationTargetConfig>
  calibration_targets;` to `RuntimeConfig`.
- `ConfigValidator.cpp`: new `validateCalibrationTargets()` block
  (mirror `validateKalibrDatasets`). Require non-empty id,
  non-empty type ∈ allowed set, positive rows/cols, positive size for
  the type's required fields, no duplicate ids.
- `SqliteConfigStore.cpp`: new `loadCalibrationTargets()` + matching
  writer, called from `loadRuntimeConfig()` and `saveRuntimeConfig()`'s
  full-replacement transaction (mirror the `kalibr_datasets` block).

**Kalibr YAML emitter** (new
`include/posest/calibration/CalibrationTargetWriter.h` +
`src/calibration/CalibrationTargetWriter.cpp`):

- `writeKalibrTargetYaml(const CalibrationTargetConfig&,
  const std::filesystem::path&)` writes a Kalibr-shaped target YAML
  (e.g. `target_type: 'aprilgrid'`, `tagCols`, `tagRows`, `tagSize`,
  `tagSpacing`).

**CLI:**

- New `import-calibration-target` subcommand
  (`DaemonCommand::ImportCalibrationTarget`,
  `ImportCalibrationTargetOptions` struct in
  `include/posest/runtime/Daemon.h`):
  `--target-id ID --type TYPE --rows N --cols N [--tag-size-m X]
   [--tag-spacing-ratio X] [--square-size-m X] [--tag-family STR]
   [--from-yaml PATH]`. With `--from-yaml`, load and parse an
  existing Kalibr target.yaml into a `CalibrationTargetConfig` (new
  helper `parseKalibrTargetYaml` in `CalibrationParsers.cpp`).
- `runConfigCommand` branch: load config, upsert the target, save.
- Extend `calibrate-camera` and the new
  `calibrate-camera-end-to-end` (W6) to accept `--target-id ID`.
  Resolution order: `--target PATH` (back-door, kept for power users)
  > `--target-id ID` looked up in DB and materialized to a tempfile
  next to the dataset > error if neither.

**Tests:**

- `test_config_schema.cpp::FullRuntimeConfigRoundTripsAndReopens` —
  add a target row to `makeValidConfig()` and assert it survives.
- New `test_calibration_target_writer.cpp` — assert the emitted YAML
  has the expected Kalibr keys and round-trips through
  `parseKalibrTargetYaml`.
- `test_daemon.cpp` — `parseDaemonOptions` covers
  `import-calibration-target` and the new `--target-id` flag.

### W2 — Quality metrics + acceptance gate

**Goal:** every Kalibr run records its reprojection RMS, observation
count, coverage proxy, and a path to Kalibr's PDF/text report. A run
that exceeds a configurable threshold throws before persisting.

**SQLite migration** (new `migration12Sql()`):

```sql
ALTER TABLE calibrations
    ADD COLUMN reprojection_rms_px REAL NOT NULL DEFAULT 0.0;
ALTER TABLE calibrations
    ADD COLUMN observation_count   INTEGER NOT NULL DEFAULT 0;
ALTER TABLE calibrations
    ADD COLUMN coverage_score      REAL NOT NULL DEFAULT 0.0;
ALTER TABLE calibrations
    ADD COLUMN report_path         TEXT NOT NULL DEFAULT '';
ALTER TABLE camera_imu_calibrations
    ADD COLUMN reprojection_rms_px REAL NOT NULL DEFAULT 0.0;
ALTER TABLE camera_imu_calibrations
    ADD COLUMN gyro_rms_radps      REAL NOT NULL DEFAULT 0.0;
ALTER TABLE camera_imu_calibrations
    ADD COLUMN accel_rms_mps2      REAL NOT NULL DEFAULT 0.0;
ALTER TABLE camera_imu_calibrations
    ADD COLUMN report_path         TEXT NOT NULL DEFAULT '';
ALTER TABLE calibration_tool_config
    ADD COLUMN max_reprojection_rms_px REAL NOT NULL DEFAULT 1.5;
ALTER TABLE calibration_tool_config
    ADD COLUMN max_camera_imu_rms_px   REAL NOT NULL DEFAULT 2.0;
```

**Struct extensions:**

- Add the four new fields to `CameraCalibrationConfig` and the four
  new fields to `CameraImuCalibrationConfig` in
  `include/posest/runtime/RuntimeConfig.h`.
- Extend `CalibrationToolConfig` with the two new thresholds.
- Extend `loadRuntimeConfig` / `saveRuntimeConfig` SELECT/INSERT
  column lists in `SqliteConfigStore.cpp` to round-trip the new
  columns.

**Parser extensions** (`src/config/CalibrationParsers.cpp`):

- `parseKalibrCameraCalibration` — read optional `reprojection_error`
  (Kalibr writes it under per-camera nodes in `*results-cam.yaml`),
  optional `coverage` proxy if present. If absent, leave 0.0.
- New `parseKalibrCameraResults(path)` overload that reads
  `*results-cam.yaml`'s top-level reprojection error block and fills
  matching `CameraCalibrationConfig` rows by camera id (so the
  ingestion code in `runConfigCommand` reads both `camchain.yaml` and
  `results-cam.yaml`).
- `parseKalibrCameraImuCalibration` — read `reprojection_error`,
  `gyroscope_error` and `accelerometer_error` when present in the
  IMU-camera result.

**Acceptance gate** (new helper in `src/runtime/Daemon.cpp`):

```cpp
void throwIfUnacceptableCalibration(
    const CameraCalibrationConfig& cal,
    const CalibrationToolConfig& tool,
    bool force);
```

Throws `std::runtime_error` if `cal.reprojection_rms_px >
tool.max_reprojection_rms_px && !force`. Same shape mirrors
`CalibrationRecorder::throwIfUnacceptable`. A second helper
`throwIfUnacceptableCameraImu(...)` covers the IMU-cam path.

Wire it into `runConfigCommand`:

- `DaemonCommand::CalibrateCamera` — between `parseKalibrCameraCalibration`
  and `replaceCalibration`.
- `DaemonCommand::CalibrateCameraImu` — between
  `parseKalibrCameraImuCalibration` and `replaceCameraImuCalibration`.

Add `--force` and `--max-reprojection-rms-px X` flags to both
subcommands.

**Tests:**

- New `test/fixtures/kalibr/results_cam_pass.yaml`,
  `results_cam_fail.yaml`, `camchain_imucam_pass.yaml`,
  `results_imucam.txt` (or YAML — match what Kalibr emits today).
- New `test_calibration_parsers.cpp` cases for the new parser
  functions.
- New unit test for `throwIfUnacceptableCalibration` /
  `throwIfUnacceptableCameraImu`.
- `test_config_schema.cpp` — assert the new columns round-trip.

### W3 — Multi-camera Kalibr ingestion

**Goal:** a single `calibrate-camera` invocation ingests every camera's
intrinsics from Kalibr's `camchain.yaml` plus all cam-to-cam baselines
(`T_cn_cnm1`).

**SQLite migration** (new `migration13Sql()`):

```sql
CREATE TABLE camera_to_camera_extrinsics (
    reference_camera_id TEXT NOT NULL,
    target_camera_id    TEXT NOT NULL,
    version             TEXT NOT NULL,
    tx_m REAL NOT NULL,
    ty_m REAL NOT NULL,
    tz_m REAL NOT NULL,
    roll_rad REAL NOT NULL,
    pitch_rad REAL NOT NULL,
    yaw_rad REAL NOT NULL,
    PRIMARY KEY (reference_camera_id, target_camera_id, version),
    FOREIGN KEY (reference_camera_id) REFERENCES cameras(id) ON DELETE CASCADE,
    FOREIGN KEY (target_camera_id)    REFERENCES cameras(id) ON DELETE CASCADE
);
```

**Struct + validator + load/save:**

- `CameraToCameraExtrinsicsConfig` (Pose3d + ids + version) in
  `RuntimeConfig.h`. `std::vector<...> camera_to_camera_extrinsics;`
  on `RuntimeConfig`.
- `ConfigValidator.cpp` — both ids must reference known cameras; ids
  must differ; no duplicate `(ref, target, version)` tuples.
- `SqliteConfigStore.cpp` — load + save mirrors `camera_extrinsics`.

**Parser extension** (`src/config/CalibrationParsers.cpp`):

- New `parseKalibrAllCameras(path, version, created_at) ->
  KalibrCalibrationBundle` returning
  `{ vector<CameraCalibrationConfig> cameras;
     vector<CameraToCameraExtrinsicsConfig> cam_to_cam; }`.
- Implementation iterates the root map (replace
  `chooseKalibrCameraNode`'s single-pick with a full walk in this new
  function — keep `chooseKalibrCameraNode` for the existing single-cam
  callers). Camera id is supplied by the caller via a topic→id map
  built from the CLI `--topic` flags. `T_cn_cnm1` is read with the
  existing `poseFromKalibrMatrix` and emits a
  `CameraToCameraExtrinsicsConfig{ camN_minus_1.id, camN.id, version, … }`.

**CLI:**

- `parseDaemonOptions` (`src/runtime/Daemon.cpp`):
  - Allow repeated `--camera-id` for `calibrate-camera` (already a
    vector for `record-kalibr-dataset`; switch the
    `CalibrateCameraOptions` field from `std::string camera_id` to
    `std::vector<std::string> camera_ids` and update validation).
  - Add repeated `--topic` (vector). Required count == camera count.
  - `--camera-to-robot` becomes repeated (one per camera, in the same
    order as `--camera-id`).
- `runConfigCommand` `CalibrateCamera` branch: build the topic→id
  map, run Kalibr (no change to the docker command — Kalibr already
  takes `--topics` as a comma-separated list), call
  `parseKalibrAllCameras`, run `throwIfUnacceptableCalibration` per
  camera (W2 dependency), then in one transaction call
  `replaceCalibration` for each `(camera, extrinsics)` pair and write
  the `camera_to_camera_extrinsics` rows.
- `buildKalibrDockerCommand` — change `--topics` to take a comma-joined
  topic list.

**Tests:**

- New `test/fixtures/kalibr/camchain_two_cameras.yaml` and
  `results_cam_two_cameras.yaml`.
- `test_calibration_parsers.cpp` — assert
  `parseKalibrAllCameras` returns 2 intrinsics + 1 cam-to-cam.
- `test_daemon.cpp` — `parseDaemonOptions` accepts repeated
  `--camera-id` / `--topic` / `--camera-to-robot`.
- `test_config_schema.cpp` — round-trip the new table.

### W4 — Conditional IMU sync for `record-kalibr-dataset`

**Goal:** intrinsic-only recording does not require a working Teensy.

**Changes:**

- `RecordKalibrDatasetOptions` in
  `include/posest/runtime/Daemon.h`: add
  `enum class ImuRequirement { Auto, Yes, No }` plus
  `ImuRequirement require_imu{Auto};`.
- `parseDaemonOptions` — add `--require-imu={auto,yes,no}` flag.
- `runConfigCommand` — branch by `ImuRequirement`:
  - `Auto` is the default and currently behaves like `Yes`. Keep the
    behaviour but also let `Auto` skip the gate when no Teensy serial
    port is configured (`config.teensy.serial_port.empty()`).
  - `Yes`: keep the existing 5 s `time_sync_established` gate; throw
    on miss.
  - `No`: do not start `TeensyService` at all. The recorder simply
    won't see IMU samples or trigger events, and `frames.csv` rows
    will all be `matched=0`. Set
    `CalibrationRecorderConfig.min_trigger_match_fraction = 0.0`
    automatically when `No` is selected.
- `make_rosbag.py` already filters unmatched frames; for intrinsic-only
  datasets, switch the bag generator to write all frames using
  `capture_timestamp_us` when IMU is disabled. Add a `--no-imu` flag
  to the script and have `make-kalibr-bag` pass it through when the
  dataset's `session.json` has `imu_samples_recorded == 0`.
- `session.json` already records `imu_samples_recorded`. No schema
  change to the manifest, but extend `make-kalibr-bag` to read
  `imu_samples_recorded` and decide.

**Tests:**

- `test_calibration_recorder.cpp` — extend with a "no-imu" path that
  asserts unmatched frames are still kept when
  `min_trigger_match_fraction = 0.0`.
- `test_daemon.cpp` — `--require-imu=no` parses; `runConfigCommand`
  branch for that path doesn't construct a `TeensyService` (use a
  fake serial transport in a test daemon).

### W5 — Dataset lifecycle CLI

**Goal:** the `kalibr_datasets` table that's currently write-only gets
two new commands so operators can list and clean up. Disk cleanup is
explicit (operator-confirmed via `--remove-files`).

**No schema change.**

**CLI additions** (in `parseDaemonOptions`, with new
`DaemonCommand::ListKalibrDatasets` and
`DaemonCommand::DeleteKalibrDataset`):

- `posest_daemon list-kalibr-datasets --config PATH [--json]` — load
  config, print one row per dataset (id, path, created_at, duration,
  cameras, on-disk: yes/no). `--json` prints as a JSON array suitable
  for piping into the future website.
- `posest_daemon delete-kalibr-dataset --config PATH --id ID
  [--remove-files]` — remove the row; with `--remove-files` also
  recursively delete the dataset directory after a sanity check that
  the path is below a configured root (default
  `~/.cache/posest/datasets`).

**Tests:**

- `test_daemon.cpp` — `parseDaemonOptions` for both new subcommands.
- New `test_kalibr_dataset_lifecycle.cpp` — populate two dataset rows,
  list them (capture stdout), delete one without `--remove-files` and
  assert the row is gone but the directory survives, then delete the
  other with `--remove-files` and assert directory removal.

### W6 — End-to-end orchestrator subcommand

**Goal:** one CLI invocation does record → bag → Kalibr → store. Reuses
W1–W5 building blocks. Designed so the future HTTP server's
`POST /api/calibrate` is a thin wrapper over the same orchestrator
function.

**New subcommand** `calibrate-camera-end-to-end` with
`DaemonCommand::CalibrateCameraEndToEnd` and
`CalibrateCameraEndToEndOptions`:

```
posest_daemon calibrate-camera-end-to-end --config PATH \
    --camera-id ID [--camera-id ID ...] \
    --topic T   [--topic T ...] \
    --camera-to-robot x,y,z,r,p,y [--camera-to-robot ...] \
    --duration-s N \
    --target-id ID \
    --version VERSION \
    --output-dir DIR \
    [--require-imu={auto,yes,no}] \
    [--mode={intrinsic,intrinsic+imu}] \
    [--max-reprojection-rms-px X] [--force] \
    [--docker-image IMAGE] [--keep-dataset]
```

**Implementation** (new function
`runCalibrationEndToEnd(options, store, camera_factory)` in
`src/runtime/Daemon.cpp`):

1. Resolve the target via `--target-id` → temp `target.yaml` (W1).
2. Run the recorder block, honouring `--require-imu` (W4).
3. Run `buildMakeKalibrBagDockerCommand` (existing).
4. Run `buildKalibrDockerCommand` for the multi-cam topic list (W3).
5. If `--mode=intrinsic+imu`, additionally run
   `buildCalibrateCameraImuDockerCommand` against the just-produced
   bag (existing). Requires `--imu PATH` if mode includes IMU.
6. Parse with `parseKalibrAllCameras` + `parseKalibrCameraResults`
   (W3 + W2). Run `throwIfUnacceptableCalibration` per camera (W2).
7. In a single SQLite transaction, write all
   `CameraCalibrationConfig` + `CameraExtrinsicsConfig` +
   `CameraToCameraExtrinsicsConfig` (+ `CameraImuCalibrationConfig`
   when `intrinsic+imu`).
8. Unless `--keep-dataset`, schedule cleanup via the W5
   `delete-kalibr-dataset --remove-files` helper.

**Refactor** the existing single-step subcommands to call into the
same helpers so no logic is duplicated. The three legacy commands
stay as thin wrappers.

**Tests:**

- New `test_calibrate_end_to_end.cpp` using a `FakeSystemExec`
  (smallest-impact injection: a `std::function<int(const char*)>`
  in an anonymous namespace in `Daemon.cpp`, default `::system`,
  swappable for tests). Drive a fake camera factory + fake transport;
  pre-stage Kalibr fixture YAMLs at the path the parser expects;
  assert that on success the SQLite store contains the expected
  rows, and that on a failed RMS gate no rows are written.

---

## Critical files to modify

| File | What changes |
|---|---|
| `include/posest/runtime/RuntimeConfig.h` | New structs `CalibrationTargetConfig`, `CameraToCameraExtrinsicsConfig`; new fields on `CameraCalibrationConfig` (rms, count, coverage, report_path), `CameraImuCalibrationConfig` (rms, gyro/accel error, report_path), `CalibrationToolConfig` (rms thresholds); new vectors on `RuntimeConfig`. |
| `src/config/SqliteSchema.cpp` | New migrations 11/12/13; bump `currentSchemaVersion()`; chain in `applyMigrations()`. |
| `src/config/SqliteConfigStore.cpp` | Load/save helpers for the new tables; SELECT/INSERT column extensions on `calibrations`, `camera_imu_calibrations`, `calibration_tool_config`. |
| `src/config/ConfigValidator.cpp` | New `validateCalibrationTargets`, `validateCameraToCameraExtrinsics`; threshold-positivity checks on `CalibrationToolConfig`. |
| `src/config/CalibrationParsers.cpp` / `.h` | New `parseKalibrAllCameras`, `parseKalibrCameraResults`, `parseKalibrTargetYaml`; per-camera reprojection RMS extraction in existing parsers. |
| `include/posest/calibration/CalibrationTargetWriter.h` + `src/calibration/CalibrationTargetWriter.cpp` | New: emits Kalibr-shaped `target.yaml` from `CalibrationTargetConfig`. |
| `include/posest/runtime/Daemon.h` | New `DaemonCommand` enum values; new option structs (`ImportCalibrationTargetOptions`, `ListKalibrDatasetsOptions`, `DeleteKalibrDatasetOptions`, `CalibrateCameraEndToEndOptions`); extend `CalibrateCameraOptions` to vectors; add `ImuRequirement` and `--force` / `--max-reprojection-rms-px`. |
| `src/runtime/Daemon.cpp` | New subcommand parsing + dispatch; new helpers `throwIfUnacceptableCalibration`, `throwIfUnacceptableCameraImu`, `runCalibrationEndToEnd`, `materializeTargetYaml`; `buildKalibrDockerCommand` accepts joined topic list. Inject `g_system` function pointer for testability. |
| `src/calibration/CalibrationRecorder.cpp` | No interface change. |
| `scripts/kalibr/make_rosbag.py` | Add `--no-imu` flag; skip the IMU-message write block; keep all frames (not only `matched=1`) when `--no-imu` is set. |
| `test/test_config_schema.cpp` | Round-trip new tables/columns. |
| `test/test_daemon.cpp` | Cover all new subcommand parses + new flags. |
| `test/test_calibration_recorder.cpp` | New no-imu acceptance case. |
| `test/test_calibration_parsers.cpp` (new) | Cover new parsers against fixtures. |
| `test/test_calibration_target_writer.cpp` (new) | Round-trip `CalibrationTargetConfig` ↔ Kalibr YAML. |
| `test/test_kalibr_dataset_lifecycle.cpp` (new) | Cover `list-kalibr-datasets` / `delete-kalibr-dataset`. |
| `test/test_calibrate_end_to_end.cpp` (new) | Cover the orchestrator with `FakeSystemExec` and pre-staged YAML fixtures. |
| `test/fixtures/kalibr/` (new directory) | `camchain_single_camera.yaml`, `camchain_two_cameras.yaml`, `results_cam_pass.yaml`, `results_cam_fail.yaml`, `camchain_imucam.yaml`, `results_imucam.yaml`, `target_aprilgrid_default.yaml`. |
| `docs/features/camera-calibration-status.md` | Move every item this plan lands from "missing" to "implemented" once each workstream merges. |

## Reused functions / utilities

- `parseKalibrCameraCalibration`, `parseKalibrCameraImuCalibration`,
  `poseFromKalibrMatrix` (`src/config/CalibrationParsers.cpp`) — reused
  inside `parseKalibrAllCameras`.
- `chooseKalibrCameraNode` (`src/config/CalibrationParsers.cpp`) — kept
  as-is for the single-cam callers.
- `replaceCalibration`, `replaceCameraImuCalibration`,
  `rememberDataset`, `findKalibrCamchain`, `findKalibrImuCamchain`
  (`src/runtime/Daemon.cpp`) — reused unchanged.
- `CalibrationRecorder::throwIfUnacceptable` pattern — mirrored by
  the new `throwIfUnacceptableCalibration`.
- `parsePoseCsv` (`CalibrationParsers.cpp`) — reused for the vector
  form of `--camera-to-robot`.
- `Statement` / `Transaction` helpers in `SqliteConfigStore.cpp` —
  every new load/save reuses them (no new SQLite plumbing).
- `make_rosbag.py` — extended, not rewritten.

## Verification

End-to-end check after all six workstreams land:

1. **Migrations:**
   `cmake --build --preset conan-release -j && ctest --preset conan-release --output-on-failure`
   exercises `test_config_schema.cpp` which opens a fresh DB, walks
   v0 → current, and round-trips `makeValidConfig()` (now extended
   with target rows, cam-to-cam rows, and the new RMS columns).
2. **Parser fixtures:**
   `./build/Release/posest_tests --gtest_filter='CalibrationParsers.*'`
   asserts the new fixture YAMLs decode to expected structs.
3. **CLI parsing:**
   `./build/Release/posest_tests --gtest_filter='Daemon.*'`
   covers each new subcommand and flag.
4. **End-to-end orchestrator (no Docker required):**
   `./build/Release/posest_tests --gtest_filter='CalibrateEndToEnd.*'`
   pre-stages `camchain_two_cameras.yaml` +
   `results_cam_pass.yaml` at the path the daemon expects, runs
   `runCalibrationEndToEnd` with the fake `g_system` returning 0,
   and asserts: 2 `CameraCalibrationConfig` rows, 2
   `CameraExtrinsicsConfig` rows, 1 `CameraToCameraExtrinsicsConfig`
   row, all with non-zero `reprojection_rms_px`. Re-run with the
   `_fail.yaml` fixture and assert no rows are written and the call
   throws.
5. **Live hardware smoke (manual, by the operator after merging):**
   ```
   posest_daemon import-calibration-target --target-id apgrid_88 \
       --type aprilgrid --rows 6 --cols 6 \
       --tag-size-m 0.088 --tag-spacing-ratio 0.3
   posest_daemon calibrate-camera-end-to-end \
       --camera-id cam0 --camera-id cam1 \
       --topic /posest/cam0/image_raw --topic /posest/cam1/image_raw \
       --camera-to-robot 0.10,0,0.20,0,0,0 \
       --camera-to-robot -0.10,0,0.20,0,0,0 \
       --duration-s 60 --target-id apgrid_88 \
       --version v1 --output-dir ./datasets/cal-$(date +%s) \
       --mode intrinsic --require-imu no
   ```
   Confirm the rows in the running config (read via
   `posest_daemon --health-once`) include both cameras' intrinsics,
   the cam-to-cam baseline, and a non-zero RMS.
6. **Dataset lifecycle:**
   `posest_daemon list-kalibr-datasets --config posest.db --json`
   should return a JSON array containing the just-recorded dataset;
   `posest_daemon delete-kalibr-dataset --id <id> --remove-files`
   should remove both row and directory.
