# W3 — Multi-Camera Kalibr Ingestion (Detailed Plan)

## Context

W1 added the calibration target catalog. W2 added quality metrics + a
post-Kalibr acceptance gate. With both merged, the calibration flow now
records, runs Kalibr in Docker, and persists rated intrinsics — but only
one camera at a time.

The brief calls for "single camera or multiple cameras". Today, calibrating
N cameras requires N separate `calibrate-camera` invocations, and Kalibr's
`T_cn_cnm1` (cam-to-cam baselines emitted in the same multi-camera
`camchain.yaml`) is silently dropped. That means a multi-camera rig has
N independent calibration runs and zero record of the optimized inter-camera
geometry — the very transform Kalibr exists to produce.

W3 closes that gap on the backend, *without* touching the future HTTP/UI:

1. **One `calibrate-camera` invocation per Kalibr run.** Repeated
   `--camera-id`, `--topic`, and `--camera-to-robot` flags drive a single
   multi-camera Kalibr invocation. The result is parsed once, every
   camera's intrinsics + per-camera quality gate run, and everything
   persists in a single SQLite transaction.
2. **A new `camera_to_camera_extrinsics` table** records `T_cn_cnm1`
   exactly as Kalibr emits it.
3. **`buildKalibrDockerCommand` switches** to space-separated `--topics`
   and per-camera `--models pinhole-radtan` so the docker invocation
   matches Kalibr's documented multi-camera CLI.
4. **Single-camera invocations keep working.** A single
   `--camera-id`/`--topic`/`--camera-to-robot` parses to one-element
   vectors. No special-case code path.

## Decisions baked in

- **Cam-to-cam pose stored as Kalibr emits it.** The pose represents
  *p_cn = T · p_cnm1* (a point in cam(n-1)'s frame mapped to cam(n)'s
  frame). The row stores `reference_camera_id = cam(n-1)`,
  `target_camera_id = cam(n)`, and the raw 4×4 decomposed via
  `poseFromKalibrMatrix`. Mirrors the existing `T_cam_imu` storage
  convention in `camera_imu_calibrations`. Documented in a schema
  comment so consumers can invert if they need the opposite direction.
- **Fail-safe on partial Kalibr output.** If the operator passed N
  `--camera-id` flags and Kalibr's `camchain.yaml` returns fewer than
  N cam<i> entries (typically because a camera saw too few target
  observations to converge), `parseKalibrAllCameras` throws.
  `--force` does **not** bypass this — it's a structural mismatch,
  not a quality issue. Operator either re-runs with the failing
  camera dropped from `--camera-id`, or re-records.
- **Explicit `--camera-to-robot` per camera.** N `--camera-to-robot`
  flags are required, in the same order as `--camera-id`. Count
  mismatch is a parse error. No automatic chaining via Kalibr
  baselines (rules out a class of "Kalibr-converged-poorly + pose
  silently wrong" failures). Power users who want the
  derive-from-baseline ergonomic can revisit it after W3 lands.

## Critical files to modify

| File | What changes |
|---|---|
| `src/config/SqliteSchema.cpp` | New `migration13Sql()`; bump `currentSchemaVersion()` to 13; chain into `applyMigrations()`. |
| `include/posest/runtime/RuntimeConfig.h` | New `CameraToCameraExtrinsicsConfig` struct; new `std::vector<...> camera_to_camera_extrinsics` on `RuntimeConfig`. |
| `src/config/SqliteConfigStore.cpp` | Load/save round-trip for the new table; new `DELETE FROM camera_to_camera_extrinsics` in the save preamble. |
| `src/config/ConfigValidator.cpp` | New validation block: both ids must reference known cameras, ids must differ, no duplicate `(ref, target, version)` tuples, finite pose. |
| `include/posest/config/CalibrationParsers.h` + `src/config/CalibrationParsers.cpp` | New `KalibrCalibrationBundle` struct; new `parseKalibrAllCameras` that walks every cam<N> node. Existing `parseKalibrCameraCalibration` kept untouched for callers that still need single-camera lookup by topic. |
| `include/posest/runtime/Daemon.h` | `CalibrateCameraOptions` field renames: `camera_id`→`camera_ids` (vector), `topic`→`topics` (vector), `camera_to_robot`→`camera_to_robots` (vector). Drop the `has_camera_to_robot` bool — `camera_to_robots.size()` is the test. |
| `src/runtime/Daemon.cpp` | `parseDaemonOptions` accepts repeated `--camera-id`/`--topic`/`--camera-to-robot`; post-parse validation enforces equal non-zero counts. `buildKalibrDockerCommand` joins topics + models space-separated. `runConfigCommand` `CalibrateCamera` branch builds a topic→camera_id map, calls `parseKalibrAllCameras`, runs the W2 quality gate per-camera, persists intrinsics + extrinsics + cam-to-cam rows in one save. `daemonUsage` updated. |
| `test/test_daemon.cpp` | Migrate the two existing test sites that read singular fields. Add multi-camera parse tests + count-mismatch rejection + multi-camera `buildKalibrDockerCommand` test. |
| `test/test_calibration_parsers.cpp` | New cases for `parseKalibrAllCameras` (one-camera, two-camera-with-baseline, partial-failure rejection, unknown-topic rejection). |
| `test/test_config_schema.cpp` | Round-trip a `camera_to_camera_extrinsics` row in `makeValidConfig`; validator-rejection cases for unknown camera, self-loop, duplicate tuple. |
| `test/fixtures/kalibr/camchain_two_cameras.yaml` (new) | Two-camera Kalibr camchain with a `T_cn_cnm1` block on cam1. The W2 fixture `results-cam_two_cameras.txt` is already in place from the prior workstream. |

## Step-by-step

### Step 1 — Schema (migration 13)

Add to `src/config/SqliteSchema.cpp`. Two foreign keys to `cameras(id)`
with `ON DELETE CASCADE` (deleting either camera invalidates the row).
Composite primary key allows multiple versions per camera pair.

```cpp
const char* migration13Sql() {
    // Per-Kalibr-run cam-to-cam baselines. Pose stored exactly as Kalibr
    // emits T_cn_cnm1: p_cn = T * p_cnm1, where reference_camera_id is
    // cam(n-1) and target_camera_id is cam(n). Consumers that need the
    // opposite direction must invert.
    return R"sql(
BEGIN;

CREATE TABLE IF NOT EXISTS camera_to_camera_extrinsics (
    reference_camera_id TEXT NOT NULL,
    target_camera_id TEXT NOT NULL,
    version TEXT NOT NULL,
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

PRAGMA user_version = 13;
COMMIT;
)sql";
}
```

Wire into `applyMigrations()` after migration 12; bump
`currentSchemaVersion()` to 13.

### Step 2 — Struct + RuntimeConfig wiring

In `include/posest/runtime/RuntimeConfig.h`:

```cpp
struct CameraToCameraExtrinsicsConfig {
    std::string reference_camera_id;  // cam(n-1)
    std::string target_camera_id;     // cam(n)
    std::string version;
    // Kalibr-native T_cn_cnm1: maps a point from reference camera frame
    // to target camera frame (p_target = T * p_reference).
    Pose3d target_in_reference;
};
```

Add `std::vector<CameraToCameraExtrinsicsConfig> camera_to_camera_extrinsics;`
to `RuntimeConfig`.

### Step 3 — Validator

In `src/config/ConfigValidator.cpp`, append a new block after the existing
`camera_extrinsics` validation:

- non-empty ids; both ids must be in `cameras_enabled` map; ids must
  differ;
- non-empty version;
- no duplicate `(reference, target, version)` tuples in the vector;
- finite pose (`isFinitePose(target_in_reference)`).

### Step 4 — SqliteConfigStore round-trip

`loadRuntimeConfig`: SELECT all 9 columns ordered by
`reference_camera_id, target_camera_id, version`. Reuse the same
double-binding pattern as `camera_extrinsics`.

`saveRuntimeConfig`: add `DELETE FROM camera_to_camera_extrinsics` to the
preamble (between `camera_extrinsics` and `calibrations` deletes); add a
matching `INSERT` loop after the `camera_extrinsics` save block.

### Step 5 — Parser: `parseKalibrAllCameras`

Add to `include/posest/config/CalibrationParsers.h`:

```cpp
struct KalibrCalibrationBundle {
    std::vector<runtime::CameraCalibrationConfig> cameras;
    std::vector<runtime::CameraToCameraExtrinsicsConfig> cam_to_cam;
};

// Walks every cam<N> node in Kalibr's camchain.yaml. The topic→id map
// translates Kalibr's `rostopic` field into GuessWork camera ids. Throws
// if any rostopic is missing from the map (structural mismatch with the
// CLI), or if `topic_to_camera_id.size()` differs from the count of
// cam<N> entries Kalibr emitted (partial failure).
KalibrCalibrationBundle parseKalibrAllCameras(
    const std::filesystem::path& camchain_path,
    const std::unordered_map<std::string, std::string>& topic_to_camera_id,
    const std::string& version,
    const std::string& created_at);
```

Implementation sketch (`src/config/CalibrationParsers.cpp`):

1. `YAML::LoadFile(camchain_path)`. Iterate root entries deterministically
   by their cam-key sort order (`cam0` < `cam1` < ...). For each entry:
   a. Read `rostopic`. Look up the GuessWork camera id; throw if absent.
   b. Build a `CameraCalibrationConfig` using the same field reads as
      `parseKalibrCameraCalibration` (factor a small private helper
      `cameraConfigFromYamlNode(node, id, version, ...)` that both
      callers use; eliminates the `chooseKalibrCameraNode` duplication).
      Note: W2 metrics (rms / observation count / report path) are NOT
      filled here; callers stitch them in from `results-cam.txt` (W2's
      `parseKalibrCameraResults`).
   c. If the node has `T_cn_cnm1`: parse it via existing
      `poseFromKalibrMatrix`. The previous cam in the iteration order
      is the reference; current is the target.
2. After the loop, assert `bundle.cameras.size() ==
   topic_to_camera_id.size()` — fail-safe partial-result guard.
3. Return the bundle (cameras in iteration order; cam-to-cam rows in
   adjacent-pair order).

`chooseKalibrCameraNode` and `parseKalibrCameraCalibration` are KEPT as-is
so the W2 single-camera path continues to work and the camera-IMU
ingestion (which still uses topic-based lookup) is untouched.

### Step 6 — CLI: vector option fields + flag parsing

In `include/posest/runtime/Daemon.h` rename:

```cpp
struct CalibrateCameraOptions {
    std::vector<std::string> camera_ids;
    std::filesystem::path bag_path;
    std::filesystem::path target_path;
    std::string target_id;
    std::vector<std::string> topics;
    std::filesystem::path output_dir;
    std::string version;
    std::vector<Pose3d> camera_to_robots;
    std::string docker_image;
    bool force{false};
    std::optional<double> max_reprojection_rms_px;
};
```

Drop `camera_id`, `topic`, `camera_to_robot`, and `has_camera_to_robot`.

In `parseDaemonOptions`:

- `--camera-id` push_back to `camera_ids` (already a vector for
  `record-kalibr-dataset` — same idiom).
- `--topic` push_back to `topics`.
- `--camera-to-robot` parse via `parsePoseCsv` (existing) and
  push_back to `camera_to_robots`.

Post-parse validation in the `CalibrateCamera` branch:

```cpp
const auto n = command.camera_ids.size();
if (n == 0 || command.bag_path.empty() ||
    (command.target_path.empty() && command.target_id.empty()) ||
    command.topics.size() != n ||
    command.camera_to_robots.size() != n ||
    command.output_dir.empty() || command.version.empty()) {
    throw std::invalid_argument(
        "calibrate-camera requires equal-count --camera-id/--topic/--camera-to-robot, "
        "plus bag, target/target-id, output-dir, version");
}
```

Update `daemonUsage()` to reflect repeated flags
(`--camera-id ID... --topic T... --camera-to-robot ...`).

### Step 7 — Multi-camera Kalibr Docker command

`buildKalibrDockerCommand` in `src/runtime/Daemon.cpp` currently emits:

```
--topics 'topic'
--models pinhole-radtan
```

Switch to space-separated lists. Each topic individually shell-quoted;
one `pinhole-radtan` per camera (Kalibr requires equal count):

```cpp
std::ostringstream topics;
std::ostringstream models;
for (std::size_t i = 0; i < options.topics.size(); ++i) {
    if (i != 0) {
        topics << " ";
        models << " ";
    }
    topics << shellQuote(options.topics[i]);
    models << "pinhole-radtan";
}
command << "--topics " << topics.str() << " "
        << "--models " << models.str();
```

### Step 8 — Wire into `runConfigCommand`

The `CalibrateCamera` branch (post-W2) does:

```
docker run → parseKalibrCameraCalibration (single) → parseKalibrCameraResults → gate → replaceCalibration → save
```

Replace with:

```
docker run
→ build topic_to_camera_id map from options.topics + options.camera_ids
→ parseKalibrAllCameras → bundle (cameras + cam_to_cam)
→ parseKalibrCameraResults → metrics map (W2)
→ for each (camera, kalibr_label) in iteration order:
       attach W2 metrics by kalibr_label ("cam0", "cam1", ...)
       attach report_path
       throwIfUnacceptableCalibration  (per-camera, --force / threshold from W2)
→ in a single save:
       for each camera, build CameraExtrinsicsConfig with the matching
         options.camera_to_robots[i], call replaceCalibration
       for each cam_to_cam row, upsert into config.camera_to_camera_extrinsics
         (de-dupe by (ref, target, version))
→ saveRuntimeConfig (single transaction; SqliteConfigStore atomicity guarantees
   all-or-nothing)
```

Add a small helper next to `replaceCalibration`:

```cpp
void replaceCameraToCameraExtrinsics(
    RuntimeConfig& config,
    CameraToCameraExtrinsicsConfig entry);
```

It removes any matching `(reference, target, version)` tuple and pushes the
new one — same pattern as `replaceCalibration`.

### Step 9 — Tests + fixtures

**New fixture** `test/fixtures/kalibr/camchain_two_cameras.yaml`:

```yaml
cam0:
  camera_model: pinhole
  intrinsics: [400.1, 400.2, 320.0, 240.0]
  distortion_model: radtan
  distortion_coeffs: [0.1, -0.01, 0.001, -0.001]
  resolution: [640, 480]
  rostopic: /posest/cam0/image_raw
cam1:
  camera_model: pinhole
  intrinsics: [402.0, 402.1, 320.5, 240.1]
  distortion_model: radtan
  distortion_coeffs: [0.09, -0.011, 0.0008, -0.0009]
  resolution: [640, 480]
  rostopic: /posest/cam1/image_raw
  T_cn_cnm1:
    - [1.0, 0.0, 0.0, 0.10]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
```

(Diagonal rotation + 0.10 m baseline along x — easy to assert against.)

**`test/test_calibration_parsers.cpp`** new cases:

- `ParsesAllCamerasReturnsBothPlusOneBaseline` — feed the two-camera
  fixture with the two-topic map, assert
  `bundle.cameras.size() == 2`, `bundle.cam_to_cam.size() == 1`,
  baseline tx == 0.10.
- `ParsesAllCamerasSingleCameraReturnsZeroBaselines` — feed
  a single-cam camchain, assert one camera, zero baselines.
- `ParsesAllCamerasThrowsOnUnknownTopic` — pass a topic→id map that
  doesn't cover one of the cam nodes' rostopic.
- `ParsesAllCamerasThrowsOnPartialResult` — pass a map for two
  cameras but feed a single-camera fixture; assert throw.

**`test/test_daemon.cpp`** — migrations + new tests:

- Migrate `ParsesCalibrationAndFieldImportCommands`: change
  `EXPECT_EQ(calibrate.calibrate_camera.camera_id, "cam0")` to
  `ASSERT_EQ(.camera_ids.size(), 1u); EXPECT_EQ(.camera_ids[0], "cam0")`.
  Same for `camera_to_robots[0].translation_m.z`.
- Migrate `BuildsKalibrDockerCommandWithExpectedMounts`: populate
  `options.topics = {"/cam/image_raw"}`. Assert the rendered command
  still contains `--topics '/cam/image_raw'` and `--models
  pinhole-radtan`.
- New `BuildsKalibrDockerCommandForTwoCameras`: two topics, assert
  `--topics 'a' 'b'` and `--models pinhole-radtan pinhole-radtan`.
- New `ParsesMultiCameraCalibrateCommand`: three pairs of
  `--camera-id`/`--topic`/`--camera-to-robot`, assert sizes and
  per-index correctness.
- New `RejectsMismatchedCalibrateFlagCounts`: 2 `--camera-id`s, 1
  `--topic` → throws.
- Migrate `ParsesForceAndMaxReprojectionRmsFlags` (W2) and
  `CalibrateCameraAcceptsTargetIdInsteadOfTargetPath` (W1) — both
  read singular fields today; switch to `.camera_ids[0]`.

**`test/test_config_schema.cpp`**:

- Extend `makeValidConfig` to push one
  `CameraToCameraExtrinsicsConfig{ "cam0", "cam1", "v1", … }`. Note:
  cam1 must already be present in `config.cameras` (it is — the
  existing fixture has cam0 + cam1).
- Round-trip assertions in `FullRuntimeConfigRoundTripsAndReopens`.
- Validator-rejection cases inside `RejectsStrictCoreFieldFailures`:
  unknown reference camera, target equal to reference, duplicate
  `(reference, target, version)` tuple.

### Step 10 — Build + ctest

```
cmake --build --preset conan-release -j
ctest --preset conan-release --output-on-failure
```

Net delta is ≈ 8–10 new tests. The two existing daemon tests that read
singular fields are migrated, not deleted. All 295 W2 tests must
continue passing.

## Reused functions / utilities

- `poseFromKalibrMatrix` (`src/config/CalibrationParsers.cpp`) — reused
  for `T_cn_cnm1` decomposition; same convention as `T_cam_imu`.
- `parsePoseCsv` (`src/config/CalibrationParsers.cpp`) — reused for each
  `--camera-to-robot` flag value.
- `replaceCalibration` (`src/runtime/Daemon.cpp`) — reused per-camera
  inside the loop.
- `Statement` / `Transaction` helpers in `SqliteConfigStore.cpp` — reused
  for the new table's load/save.
- `parseKalibrCameraResults` (W2) — reused unchanged; the camera-id key
  in its result map is Kalibr's `cam<N>` label, which `parseKalibrAllCameras`
  emits in the same iteration order.
- `throwIfUnacceptableCalibration` (W2) — reused per-camera inside the
  bundle loop.
- `findKalibrCamchain`, `findKalibrResultsCam`, `findKalibrReportCam`
  (`src/runtime/Daemon.cpp`) — unchanged.
- `chooseKalibrCameraNode` and `parseKalibrCameraCalibration` — kept
  for the camera-IMU ingestion path which still selects by topic. Once
  W6 (orchestrator) lands, the camera-IMU branch can also migrate to
  `parseKalibrAllCameras`-style multi-camera parsing — out of scope here.

## Verification

1. **Build + ctest:** all 295 W2 tests still pass + the new W3 tests.
2. **Parser fixture round-trip:**
   `./build/Release/posest_tests --gtest_filter='KalibrAllCameras*'`.
3. **Schema round-trip:**
   `./build/Release/posest_tests --gtest_filter='SqliteConfigStore.FullRuntimeConfigRoundTripsAndReopens'`
   now exercises a `camera_to_camera_extrinsics` row.
4. **CLI parse + docker-command shape:**
   `./build/Release/posest_tests --gtest_filter='DaemonOptions*'`.
5. **Manual smoke (post-merge):** run the existing single-camera flow
   to confirm no regression; then run a real two-camera Kalibr
   calibration and verify with `posest_daemon --health-once` (or by
   reading the SQLite directly) that:
   - both cameras have an `active=1` row in `calibrations` with a
     non-zero `reprojection_rms_px`,
   - both cameras have a row in `camera_extrinsics`,
   - exactly one row exists in `camera_to_camera_extrinsics` with
     `reference_camera_id=cam0`, `target_camera_id=cam1`, and a
     baseline tx that matches the physical mount.

## Out of scope for W3

- Single-shot orchestrator (W6) — that subcommand will internally call
  the same `parseKalibrAllCameras` + `runConfigCommand`-style sequence,
  but is shipped separately so the legacy three-step CLI keeps working
  during the transition.
- Camera-IMU ingestion of cam-to-cam baselines from `camchain-imucam.yaml`
  — Kalibr does not re-optimize cam-to-cam transforms during
  IMU-camera calibration, so the `camera_to_camera_extrinsics` rows
  written by `calibrate-camera` are authoritative.
- A "compose to derive cam(N)'s robot pose from cam0's" helper —
  deferred until there's a UI consumer that benefits.
- Web/UI surfacing of the new table.
