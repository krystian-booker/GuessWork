# W2 — Quality Metrics + Acceptance Gate (Detailed Plan)

## Context

Today, after `runConfigCommand` invokes Kalibr via Docker, the only failure
signal is `std::system(...) != 0`. Kalibr can return 0 with a wildly
diverged solution (numerical convergence ≠ correctness). The active
calibration row is overwritten regardless of how good the calibration
actually is, and the operator gets no rating to show in the eventual UI.

W2 closes this gap on the backend, *without* shipping any UI:

1. **Persist Kalibr's quality metrics** alongside the intrinsics:
   per-camera reprojection RMS, observation count, report-file path,
   and (for IMU-cam) gyroscope / accelerometer RMS.
2. **Refuse to persist** a Kalibr result that fails a configurable
   reprojection-RMS gate. Override with `--force`.
3. **Make the threshold tunable** via the existing
   `calibration_tool_config` singleton row (so the WebUI, when it
   lands, can edit it without a recompile) and overridable per-run via
   a CLI flag.

Files to modify trace through every layer of the calibration stack:
schema → struct → load/save → parser → daemon-orchestrator → tests. The
work is small per layer but touches a lot of layers.

## Decisions baked in

- **Default reprojection RMS gate: 1.0 px.** Matches OpenVINS guidance
  (`<0.5 px` excellent, `<1.0 px` good). Strict enough to catch a
  Kalibr run that's gone subtly wrong before it overwrites the active
  calibration row. Wide-angle / fisheye lenses can override per-run via
  `--max-reprojection-rms-px X` or `--force`.
- **Default camera-IMU reprojection RMS gate: 1.5 px.** A bit looser
  because camera-IMU calibrations have additional noise sources from
  IMU integration.
- **`coverage_score` column ships as a placeholder, always written as
  0.0.** Kalibr doesn't emit a coverage metric and computing one from
  AprilGrid detections is out of scope for W2. Keeping the column lets
  a future workstream populate it via a column-update instead of
  another migration.
- **Fail-safe on missing/unparseable RMS.** If
  `parseKalibrCameraResults` can't extract a positive reprojection RMS
  for the requested camera id, the gate throws — no row is written.
  Operator either fixes the Kalibr run or passes `--force` (which
  stores `reprojection_rms_px = 0.0` and skips the gate).

## Critical files to modify

| File | What changes |
|---|---|
| `src/config/SqliteSchema.cpp` | New `migration12Sql()`; bump `currentSchemaVersion()` to 12; chain in `applyMigrations()`. |
| `include/posest/runtime/RuntimeConfig.h` | Add four columns each to `CameraCalibrationConfig` and `CameraImuCalibrationConfig`; add two threshold fields to `CalibrationToolConfig`. |
| `src/config/SqliteConfigStore.cpp` | Extend the `calibrations`, `camera_imu_calibrations`, and `calibration_tool_config` SELECT/INSERT column lists. |
| `src/config/ConfigValidator.cpp` | Finite-and-non-negative checks on the new metric columns; positivity check on the two new thresholds. |
| `include/posest/config/CalibrationParsers.h` + `src/config/CalibrationParsers.cpp` | New `parseKalibrCameraResults` (text file) and `parseKalibrCameraImuResults` (text file). Existing parsers untouched on the calling side, but optionally fill new fields when callers pass them through. |
| `include/posest/runtime/Daemon.h` | Add `force` + `max_reprojection_rms_px` (optional double) to `CalibrateCameraOptions` and `CalibrateCameraImuOptions`. |
| `src/runtime/Daemon.cpp` | New helpers `findKalibrResultsCam`, `findKalibrResultsImuCam`, `throwIfUnacceptableCalibration`, `throwIfUnacceptableCameraImu`; wire into the `CalibrateCamera` and `CalibrateCameraImu` branches; new `--force` and `--max-reprojection-rms-px` flag parsers. |
| `test/test_config_schema.cpp` | Round-trip the new columns; check default values on a fresh DB. |
| `test/fixtures/kalibr/` (new) | `results-cam_pass.txt`, `results-cam_fail.txt`, `results-imucam_pass.txt`, `results-imucam_missing_metrics.txt`. |
| `test/test_calibration_parsers.cpp` (new) | Cover `parseKalibrCameraResults` happy path, missing-camera-id path, mixed-format path; same for camera-IMU variant. |
| `test/test_daemon.cpp` | Cover `--force` and `--max-reprojection-rms-px` parsing; cover `throwIfUnacceptableCalibration` and `throwIfUnacceptableCameraImu` (these are top-level functions in the daemon namespace). |

## Step-by-step

### Step 1 — Schema (migration 12)

Add to `src/config/SqliteSchema.cpp`:

```cpp
const char* migration12Sql() {
    // Quality metrics from Kalibr's results files. Defaults of 0.0 are
    // intentional — an unmigrated row predates the gate and should not be
    // mistaken for "0 px reprojection error".
    return R"sql(
BEGIN;

ALTER TABLE calibrations
    ADD COLUMN reprojection_rms_px REAL NOT NULL DEFAULT 0.0;
ALTER TABLE calibrations
    ADD COLUMN observation_count INTEGER NOT NULL DEFAULT 0;
ALTER TABLE calibrations
    ADD COLUMN coverage_score REAL NOT NULL DEFAULT 0.0;
ALTER TABLE calibrations
    ADD COLUMN report_path TEXT NOT NULL DEFAULT '';

ALTER TABLE camera_imu_calibrations
    ADD COLUMN reprojection_rms_px REAL NOT NULL DEFAULT 0.0;
ALTER TABLE camera_imu_calibrations
    ADD COLUMN gyro_rms_radps REAL NOT NULL DEFAULT 0.0;
ALTER TABLE camera_imu_calibrations
    ADD COLUMN accel_rms_mps2 REAL NOT NULL DEFAULT 0.0;
ALTER TABLE camera_imu_calibrations
    ADD COLUMN report_path TEXT NOT NULL DEFAULT '';

ALTER TABLE calibration_tool_config
    ADD COLUMN max_reprojection_rms_px REAL NOT NULL DEFAULT 1.0;
ALTER TABLE calibration_tool_config
    ADD COLUMN max_camera_imu_rms_px REAL NOT NULL DEFAULT 1.5;

PRAGMA user_version = 12;
COMMIT;
)sql";
}
```

Wire into `applyMigrations()` (mirror the W1 wiring) and bump
`currentSchemaVersion()` to `12`.

### Step 2 — Struct extensions

In `include/posest/runtime/RuntimeConfig.h`:

```cpp
struct CameraCalibrationConfig {
    // …existing fields…
    // Per-camera reprojection RMS in pixels, parsed from Kalibr's
    // results-cam.txt at ingestion time. 0.0 means unrated (legacy row,
    // or operator persisted with --force despite a missing metric).
    double reprojection_rms_px{0.0};
    // Number of board observations Kalibr used during optimization.
    // Surfaced primarily for the future review UI.
    std::int32_t observation_count{0};
    // Placeholder for a future coverage metric (W2 ships always 0.0).
    double coverage_score{0.0};
    // Absolute path to Kalibr's report-cam.pdf (or empty if not found).
    std::string report_path;
};

struct CameraImuCalibrationConfig {
    // …existing fields…
    double reprojection_rms_px{0.0};
    double gyro_rms_radps{0.0};
    double accel_rms_mps2{0.0};
    std::string report_path;
};

struct CalibrationToolConfig {
    std::string docker_image{"kalibr:latest"};
    // Per-run gate. A camera's reprojection RMS strictly above this
    // value rejects the calibration unless --force is passed.
    double max_reprojection_rms_px{1.0};
    double max_camera_imu_rms_px{1.5};
};
```

### Step 3 — Validator

In `src/config/ConfigValidator.cpp`, inside the existing
`for (const auto& calibration : config.calibrations)` block, append:

```cpp
require(isFinite(calibration.reprojection_rms_px) &&
            calibration.reprojection_rms_px >= 0.0,
        "calibration for camera '" + calibration.camera_id +
            "' reprojection_rms_px must be finite and >= 0");
require(calibration.observation_count >= 0,
        "calibration for camera '" + calibration.camera_id +
            "' observation_count must be >= 0");
require(isFinite(calibration.coverage_score) &&
            calibration.coverage_score >= 0.0,
        "calibration for camera '" + calibration.camera_id +
            "' coverage_score must be finite and >= 0");
```

Same pattern for the three new IMU-cam metric fields. And in the
section that already validates `calibration_tools.docker_image`:

```cpp
require(config.calibration_tools.max_reprojection_rms_px > 0.0 &&
            isFinite(config.calibration_tools.max_reprojection_rms_px),
        "calibration_tools.max_reprojection_rms_px must be finite and > 0");
require(config.calibration_tools.max_camera_imu_rms_px > 0.0 &&
            isFinite(config.calibration_tools.max_camera_imu_rms_px),
        "calibration_tools.max_camera_imu_rms_px must be finite and > 0");
```

### Step 4 — SqliteConfigStore round-trip

`loadRuntimeConfig` `calibrations` block — extend the SELECT to include
the four new columns and bind them onto `calibration.*`. Same for
`camera_imu_calibrations`. The `calibration_tool_config` SELECT goes
from a single `docker_image` column to three columns; load all three
into `config.calibration_tools.*`.

`saveRuntimeConfig` mirrors: extend each INSERT's column list and
parameter binds. The Statement helpers already in the file handle
`bindDouble` / `bindInt`. No new sqlite plumbing.

### Step 5 — Parser extensions

Add to `include/posest/config/CalibrationParsers.h`:

```cpp
struct KalibrCameraQualityMetrics {
    double reprojection_rms_px{0.0};
    std::int32_t observation_count{0};
};

// Parse Kalibr's results-cam-<bag>.txt. Returns metrics keyed by Kalibr
// camera-id label ("cam0", "cam1", ...). The caller maps those labels
// to GuessWork camera ids via the topic→id table built from CLI flags.
std::unordered_map<std::string, KalibrCameraQualityMetrics>
parseKalibrCameraResults(const std::filesystem::path& path);

struct KalibrCameraImuQualityMetrics {
    double reprojection_rms_px{0.0};
    double gyro_rms_radps{0.0};
    double accel_rms_mps2{0.0};
};

std::unordered_map<std::string, KalibrCameraImuQualityMetrics>
parseKalibrCameraImuResults(const std::filesystem::path& path);
```

Implementation (`src/config/CalibrationParsers.cpp`) is a small
line-based state machine — no YAML, no regex library required:

- Read line by line. Track the most recent `cam<N>` header (any line
  matching `^\s*cam\d+(\s|:|$)`) as the current section.
- For `parseKalibrCameraResults`, look for lines containing
  `reprojection error:`. Extract the second pair of numbers from the
  pattern `[a, b] +- [c, d]`. Compute
  `rms = std::hypot(c, d)`. Store under the current section's id.
- For `parseKalibrCameraImuResults`, additionally watch for
  `gyroscope error` and `accelerometer error` lines (case-insensitive
  match) and extract the `std:` value. If a metric is absent, leave
  its field at 0.0 — only the camera reprojection RMS gates
  persistence; gyro/accel are reporting-only.
- Trim/skip blank lines; ignore the `Calibration results\n====` header.

The fixture files (Step 9) lock the exact line forms the parser
accepts.

### Step 6 — Helpers in Daemon.cpp

Add to the anonymous namespace in `src/runtime/Daemon.cpp`:

```cpp
std::filesystem::path findKalibrResultsCam(
    const std::filesystem::path& output_dir) {
    for (const auto& entry : std::filesystem::directory_iterator(output_dir)) {
        if (!entry.is_regular_file()) continue;
        const auto name = entry.path().filename().string();
        if (name.rfind("results-cam", 0) == 0 &&
            entry.path().extension() == ".txt") {
            return entry.path();
        }
    }
    throw std::runtime_error(
        "Kalibr output did not contain a results-cam .txt file: " +
        output_dir.string());
}
// …same shape for findKalibrResultsImuCam ("results-imucam"+".txt")…

std::filesystem::path findKalibrReportCam(
    const std::filesystem::path& output_dir) {
    for (const auto& entry : std::filesystem::directory_iterator(output_dir)) {
        if (!entry.is_regular_file()) continue;
        const auto name = entry.path().filename().string();
        if (name.rfind("report-cam", 0) == 0 &&
            entry.path().extension() == ".pdf") {
            return entry.path();
        }
    }
    return {};  // PDF is best-effort; an empty path is acceptable.
}
```

```cpp
void throwIfUnacceptableCalibration(
    const CameraCalibrationConfig& cal,
    const CalibrationToolConfig& tool,
    bool force) {
    if (force) return;
    if (!(cal.reprojection_rms_px > 0.0)) {
        throw std::runtime_error(
            "Kalibr did not produce a usable reprojection RMS for camera '" +
            cal.camera_id + "' — pass --force to persist anyway");
    }
    if (cal.reprojection_rms_px > tool.max_reprojection_rms_px) {
        throw std::runtime_error(
            "Kalibr reprojection RMS for camera '" + cal.camera_id + "' (" +
            std::to_string(cal.reprojection_rms_px) +
            " px) exceeds gate " +
            std::to_string(tool.max_reprojection_rms_px) +
            " px — pass --force or rerun");
    }
}
// …throwIfUnacceptableCameraImu mirrors, using max_camera_imu_rms_px…
```

Both helpers are declared in `include/posest/runtime/Daemon.h` so
tests can call them directly.

### Step 7 — CLI flags

In `include/posest/runtime/Daemon.h`:

```cpp
struct CalibrateCameraOptions {
    // …existing fields…
    bool force{false};
    // When set, overrides calibration_tools.max_reprojection_rms_px for
    // this run only. std::nullopt → use the DB value.
    std::optional<double> max_reprojection_rms_px;
};

struct CalibrateCameraImuOptions {
    // …existing fields…
    bool force{false};
    std::optional<double> max_reprojection_rms_px;
};
```

In `parseDaemonOptions` (mirror the W1 flag parsing):

```cpp
} else if (arg == "--force") {
    options.calibrate_camera.force = true;
    options.calibrate_camera_imu.force = true;
} else if (arg == "--max-reprojection-rms-px") {
    const double value = std::stod(requireValue(i, argc, argv, arg));
    options.calibrate_camera.max_reprojection_rms_px = value;
    options.calibrate_camera_imu.max_reprojection_rms_px = value;
}
```

Update `daemonUsage()` to document both flags on the
`calibrate-camera` and `calibrate-camera-imu` lines.

### Step 8 — Wire the gate

In `runConfigCommand`'s `CalibrateCamera` branch (between the existing
`parseKalibrCameraCalibration` call and `replaceCalibration`):

```cpp
auto calibration = config::parseKalibrCameraCalibration(/* … */);
const auto results_path =
    findKalibrResultsCam(options.calibrate_camera.output_dir);
const auto metrics = config::parseKalibrCameraResults(results_path);
// Kalibr labels its cameras cam0/cam1/… in topic order. For the
// single-camera command, "cam0" is the canonical key.
const auto metric_it = metrics.find("cam0");
if (metric_it != metrics.end()) {
    calibration.reprojection_rms_px = metric_it->second.reprojection_rms_px;
    calibration.observation_count = metric_it->second.observation_count;
}
calibration.report_path = findKalibrReportCam(
    options.calibrate_camera.output_dir).string();

CalibrationToolConfig effective_tool = config.calibration_tools;
if (options.calibrate_camera.max_reprojection_rms_px) {
    effective_tool.max_reprojection_rms_px =
        *options.calibrate_camera.max_reprojection_rms_px;
}
throwIfUnacceptableCalibration(
    calibration, effective_tool, options.calibrate_camera.force);

CameraExtrinsicsConfig extrinsics;
// …unchanged from today…
```

The `CalibrateCameraImu` branch follows the same pattern, calling
`parseKalibrCameraImuResults` and `throwIfUnacceptableCameraImu`.

### Step 9 — Fixtures

`test/fixtures/kalibr/results-cam_pass.txt`:

```
Calibration results
====================

Camera-system parameters:
        cam0 (/cam/image_raw):
        type: <CameraGeometry libcalibration.geometries.types.PinholeRadialTangentialCamera>
        principal point: [319.6, 239.8]
        focal length: [400.1, 400.2]
        distortion: [-0.21, 0.04, 0.001, 0.0001]
        reprojection error: [-0.000000, -0.000000] +- [0.181675, 0.194522]
```

`results-cam_fail.txt` — same structure with `+- [1.5, 1.7]` (RMS ≈
2.27 px, above the 1.0 gate).

`results-cam_two_cameras.txt` — two `cam0` and `cam1` blocks with
distinct sigmas (used by W3 too).

`results-imucam_pass.txt` — Kalibr-style camera-IMU summary with the
three metric blocks. Conservative content the parser can extract:

```
Camera-IMU calibration

Reprojection error (cam0):
        mean: 0.32 [px]
        std:  0.41 [px]

Gyroscope error:
        mean: 0.0008 [rad/s]
        std:  0.0021 [rad/s]

Accelerometer error:
        mean: 0.04 [m/s^2]
        std:  0.07 [m/s^2]
```

`results-imucam_missing_metrics.txt` — only the cam reprojection
block; gyro/accel sections deleted. Asserts the parser leaves those
fields at 0.0.

### Step 10 — Tests

`test/test_calibration_parsers.cpp` (new file, registered in
`CMakeLists.txt`):

- `ParseKalibrCameraResultsExtractsRmsForOneCamera` — feed
  `results-cam_pass.txt`, assert `metrics["cam0"].reprojection_rms_px
  ≈ hypot(0.181675, 0.194522)` ≈ 0.2660 px.
- `ParseKalibrCameraResultsHandlesTwoCameras` — feed the two-camera
  fixture, assert `metrics.size() == 2`.
- `ParseKalibrCameraResultsThrowsOnEmptyFile` — feed an empty file,
  assert it returns an empty map (we want the gate to fire later, not
  the parser to throw on empty input).
- `ParseKalibrCameraImuResultsExtractsAllThreeMetrics`.
- `ParseKalibrCameraImuResultsLeavesMissingMetricsZero`.

`test/test_config_schema.cpp`:

- Extend `makeValidConfig()` with non-zero RMS / observation count /
  report_path on the existing calibration row, and on the camera-IMU
  row.
- Extend the round-trip assertions to cover the new fields.
- Update the empty-DB test to assert
  `calibration_tools.max_reprojection_rms_px == 1.0` and
  `max_camera_imu_rms_px == 1.5`.
- Add a validator-rejection case for a negative reprojection RMS.

`test/test_daemon.cpp`:

- `ParsesForceAndMaxReprojectionRmsFlags` — verify both flags reach
  both option structs.
- `ThrowIfUnacceptableCalibrationAcceptsBelowGate` — call the helper
  directly with `rms = 0.5`, `gate = 1.0`, expect no throw.
- `ThrowIfUnacceptableCalibrationRejectsAboveGate` — `rms = 2.0`,
  `gate = 1.0`, expect `std::runtime_error`.
- `ThrowIfUnacceptableCalibrationRejectsZeroRmsWhenNotForced` —
  `rms = 0.0`, expect throw.
- `ThrowIfUnacceptableCalibrationAcceptsZeroRmsWhenForced` —
  `rms = 0.0`, `force = true`, expect no throw.
- `ThrowIfUnacceptableCameraImuMirrorsBehaviour` — same four cases
  for the IMU-cam helper using `max_camera_imu_rms_px`.

## Reused functions / utilities

- `Statement` / `Transaction` helpers in `SqliteConfigStore.cpp` — every
  new column extension reuses them.
- `findKalibrCamchain` / `findKalibrImuCamchain` (`Daemon.cpp`) — pattern
  copy-pasted for `findKalibrResultsCam` / `findKalibrResultsImuCam` /
  `findKalibrReportCam`.
- `CalibrationRecorder::throwIfUnacceptable` — pattern mirror for the
  two new throw helpers.
- `parseKalibrCameraCalibration` / `parseKalibrCameraImuCalibration` —
  unchanged. The new metrics live in *separate* parsers so callers
  that already have a YAML path (W3 multi-cam, W6 orchestrator) don't
  need to thread an extra path through the YAML parser.

## Verification

1. **Build + ctest:**
   ```
   cmake --build --preset conan-release -j
   ctest --preset conan-release --output-on-failure
   ```
   Adds ≈ 12 new tests; all 282 existing tests must continue to pass.
2. **Migration upgrade test:** `test_config_schema.cpp` already
   contains the v4→v5 migration test pattern. Add an analogous
   v11→v12 case that pre-stages a v11 DB with one existing
   calibration row and asserts the new columns default to 0.0
   after migration.
3. **Manual smoke (post-merge):** run a real Kalibr calibration in
   Docker, then verify with `posest_daemon --health-once` (or the
   future WebUI) that the active calibration row has a non-zero
   `reprojection_rms_px` and a `report_path` pointing to the PDF.
   Re-run with a deliberately bad target.yaml to confirm the gate
   throws and the active row is *not* overwritten.

## Out of scope for W2

- Real spatial coverage metric (column ships as placeholder 0.0).
- W3's multi-camera ingestion of metrics — once W3 lands, the
  per-camera lookup in Step 8 changes from a hard-coded `"cam0"`
  to a loop over the topic→id map, but the gate plumbing here is
  unchanged.
- Web/UI surfacing of the new metrics. The fields are now in SQLite
  and the future HTTP layer will read them through `IConfigStore`.
