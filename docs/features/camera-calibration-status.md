# Camera Calibration (Intrinsics, Extrinsics, Camera-IMU)

**Status:** Backend complete end-to-end on the CLI. Headless website / live preview / quality-overlay UI are deferred until the HTTP layer exists.

This document describes the camera intrinsic, camera-to-robot extrinsic, cam-to-cam extrinsic, and camera-IMU calibration flow as it ships today, and calls out exactly which of the operator-facing requirements are still missing. The companion plan in [`camera-calibration-implementation-plan.md`](camera-calibration-implementation-plan.md) is the source of truth for the workstreams that landed; this doc is the implementation status against the user-facing brief.

---

## 1. Requirements (target behaviour)

The brief is to support a one-shot calibration flow that an operator triggers from the eventual website. Today:

1. The user kicks off intrinsic / extrinsic calibration (single or multi-camera) from the website.
2. While the calibration is running, the camera video is visible so the user can see what they are pointing at.
3. The application records video from the camera(s) and IMU data from the Teensy.
4. The recording is converted into a ROS bag and fed into Kalibr inside a Docker container.
5. Kalibr's intrinsics / extrinsics are read back and stored against the specific camera (or cameras) that were calibrated.
6. The operator sees the calibrated values and a quality rating.
7. The operator picks the calibration board (type + size) at the start of the run, defaulting to an AprilGrid.

The website/HTTP server does not exist yet, so requirements 1, 2, 6 (UI surface), and 7 (UI surface) are intentionally deferred — but the backend pieces they will need are landed below.

---

## 2. End-to-end flow today

```
┌──────────────────────┐   record-kalibr-dataset / calibrate-camera-end-to-end
│ posest_daemon (CLI)  │──────────────────────────────────────────────────────┐
└──────────────────────┘                                                      │
              │                                                               │
              ▼                                                               │
   ┌─────────────────────┐    Frame (LatestFrameSlot)                         │
   │ V4L2Producer        │──────────────────────────────────────────┐         │
   │ (or any backend)    │                                          │         │
   └─────────────────────┘                                          ▼         │
              │                                              ┌─────────────┐  │
              │ CameraTriggerEvent (via MeasurementBus)      │ Calibration │  │
              ▼                                              │ Recorder    │  │
   ┌─────────────────────┐    ImuSample (MeasurementBus)     │             │  │
   │ TeensyService       │─────────────────────────────────▶ │ frames.csv  │  │
   │ (optional, W4)      │    CameraTriggerEvent             │ imu.csv     │  │
   └─────────────────────┘                                   │ images/*.png│  │
                                                             │ session.json│  │
                                                             └─────────────┘  │
                                                                    │         │
                                  ┌─────────────────────────────────┘         │
                                  ▼                                           │
              docker run kalibr:* python3 make_rosbag.py                      │
                                  │                                           │
                                  ▼                                           │
                       kalibr.bag (or *.bag)                                  │
                                  │                                           │
                ┌─────────────────┴──────────────────┐                        │
                ▼                                    ▼                        │
   docker run kalibr_calibrate_cameras    docker run kalibr_calibrate_imu_camera
                │                                    │                        │
                ▼                                    ▼                        │
   camchain.yaml + results-cam.txt    camchain-imucam.yaml + results-imucam.txt
                │                                    │                        │
                └─────────────────┬──────────────────┘                        │
                                  ▼                                           │
   parseKalibrAllCameras / parseKalibrCameraResults / parseKalibrCameraImuCalibration
                                  │                                           │
                                  ▼                                           │
                    throwIfUnacceptableCalibration  ◀─ CalibrationToolConfig  │
                                  │   (--force overrides)                     │
                                  ▼                                           │
                          SqliteConfigStore  ◀──────────────────────────────  ┘
                                  │
                                  ▼
       calibrations / camera_extrinsics / camera_to_camera_extrinsics /
       camera_imu_calibrations  (single all-or-nothing transaction)
```

The whole pipeline is reachable two ways today:

- **Three legacy subcommands** chained by hand: `record-kalibr-dataset` → `make-kalibr-bag` → `calibrate-camera` (or `calibrate-camera-imu`).
- **One orchestrator subcommand** that does it all: `calibrate-camera-end-to-end`. This is the entrypoint the future `POST /api/calibrate` will wrap; it is implemented as `runCalibrationEndToEnd` in `src/runtime/Daemon.cpp` and is callable directly without going through CLI parsing.

---

## 3. Component-by-component status

### 3.1 Recording — `posest::calibration::CalibrationRecorder`

`include/posest/calibration/CalibrationRecorder.h`, `src/calibration/CalibrationRecorder.cpp`.

- Subscribes to the camera as an `IFrameConsumer` and to the `MeasurementBus` as an `IMeasurementSink`. The `deliver()` and `publish()` methods enqueue events under a mutex; a single worker thread drains the queue, writes PNGs, and appends to `frames.csv` / `imu.csv` / `trigger_events.csv`.
- Frame ↔ trigger matching: each `CameraConfig` may have a `CameraTriggerConfig` declaring a Teensy GPIO pin. When a `CameraTriggerEvent` arrives the recorder caches "latest trigger per pin"; each subsequent frame from the same camera is annotated with the most recent trigger that fired no later than the frame's `capture_time`. Unmatched frames are still written but flagged `matched=0`.
- `min_trigger_match_fraction` gates acceptance of a recorded session. The CLI sets it to `1.0` when a Teensy is in use and `0.0` for intrinsic-only runs (W4).
- On `stop()` the recorder writes `session.json` with `frames_seen`, `frames_recorded`, `frames_without_trigger`, `imu_samples_recorded`, `trigger_events_recorded`, and the camera ids.

What's missing here: nothing for the headless flow. **Live preview** (sending the captured frames to a websocket / MJPEG stream so the operator can see what the camera sees) is **not** implemented — the recorder just writes to disk.

### 3.2 Calibration target catalogue (W1)

`CalibrationTargetConfig` lives in `include/posest/runtime/RuntimeConfig.h`; persisted in the `calibration_targets` SQLite table (migration 11). One row is seeded by default:

```
id=default_aprilgrid_6x6  type=aprilgrid  rows=6  cols=6
tag_size_m=0.088  tag_spacing_ratio=0.3  tag_family=tag36h11
```

This matches Kalibr's bundled board so a fresh install can run a calibration without any extra setup.

Operators add boards with the `import-calibration-target` subcommand (either by passing every field, or `--from-yaml PATH` to ingest an existing Kalibr `target.yaml`). At calibration time the target is selected by `--target-id ID`; `runCalibrationEndToEnd` / `runConfigCommand`'s `CalibrateCamera` branch materializes a Kalibr-shaped YAML on disk via `posest::calibration::writeKalibrTargetYaml` (`src/calibration/CalibrationTargetWriter.cpp`) and points the Docker invocation at it. `--target PATH` is kept as a power-user override.

`parseKalibrTargetYaml` round-trips an external YAML back into the catalogue; `test_calibration_target_writer.cpp` covers AprilGrid / checkerboard / circlegrid round-trips.

### 3.3 IMU sync (W4) — `TeensyService` + `MeasurementBus`

`record-kalibr-dataset` and `calibrate-camera-end-to-end` both accept `--require-imu={auto,yes,no}`:

- `auto` (default): start a `TeensyService` and require time sync iff `config.teensy.serial_port` is non-empty. A station with no Teensy configured runs intrinsic-only without complaint.
- `yes`: always start the Teensy; throw if `time_sync_established` does not flip true within 5 s. This is the path used for camera-IMU runs.
- `no`: never construct a `TeensyService`. The recorder accepts unmatched frames (`min_trigger_match_fraction=0.0`) and `make_rosbag.py` is invoked with `--no-imu` so it keeps every frame and skips the IMU topic.

The `MakeKalibrBag` dispatch auto-detects intrinsic-only datasets by reading `imu_samples_recorded` from `session.json` and forwarding `--no-imu` automatically; the camera-IMU path always leaves it false.

### 3.4 ROS-bag emission — `scripts/kalibr/make_rosbag.py`

A standalone Python script invoked inside the Kalibr Docker container by `buildMakeKalibrBagDockerCommand`. Reads `frames.csv` + `imu.csv`, decodes each PNG, and writes:

- `/posest/<camera_id>/image_raw` — `sensor_msgs/Image` per frame, stamped with `ros_timestamp_us` (`= trigger_us + trigger_to_exposure_center_us` when matched, else `capture_us`).
- `/posest/imu/data_raw` — `sensor_msgs/Imu` per IMU sample (skipped under `--no-imu`).

Trigger-matched timestamps are the contract that ties camera and IMU streams onto a common steady-clock domain. `--no-imu` disables both the IMU topic and the `matched=1` filter so intrinsic-only runs produce a usable bag from frames-only data.

### 3.5 Kalibr Docker invocations

All three Docker shell-outs go through a single test seam:

```cpp
using SystemImpl = std::function<int(const char*)>;
SystemImpl setSystemImplForTesting(SystemImpl impl);
void resetSystemImplForTesting();
```

The default is `std::system`. `test_calibrate_end_to_end.cpp` swaps it for a lambda that pre-stages Kalibr fixture YAMLs at the path the parser expects, returns 0, and never touches the real `docker` binary.

The image is resolved in this order: explicit `--docker-image`, env `POSEST_KALIBR_DOCKER_IMAGE`, then `RuntimeConfig::calibration_tools.docker_image` (default `kalibr:latest`).

- `buildKalibrDockerCommand` — runs `kalibr_calibrate_cameras` with one model per camera (today only `pinhole-radtan`) and a space-separated `--topics` list.
- `buildMakeKalibrBagDockerCommand` — bind-mounts the dataset, output dir, and `scripts/kalibr/` into the container and shells `python3 /tools/make_rosbag.py`.
- `buildCalibrateCameraImuDockerCommand` — runs `kalibr_calibrate_imu_camera` against an `input-camchain.yaml` exported from the *active* per-camera intrinsics (`exportActiveCamchain`).

### 3.6 Multi-camera ingestion (W3) — `parseKalibrAllCameras`

`src/config/CalibrationParsers.cpp::parseKalibrAllCameras` walks the camchain.yaml in `cam<N>` sort order and, for each entry, looks up the rostopic in the operator-supplied `topic → camera_id` map. The function returns:

```cpp
struct KalibrCalibrationBundle {
    std::vector<runtime::CameraCalibrationConfig> cameras;
    std::vector<runtime::CameraToCameraExtrinsicsConfig> cam_to_cam;
};
```

- `cam_to_cam[i]` carries Kalibr's raw `T_cn_cnm1` matrix (`reference_camera_id = cam(n-1)`, `target_camera_id = cam(n)`) into the new `camera_to_camera_extrinsics` table (migration 13).
- A partial Kalibr result (fewer cam<N> than were requested) is rejected with `runtime_error` regardless of `--force`. This is a structural mismatch, not a quality issue.
- The whole bundle (intrinsics + camera-to-robot extrinsics + cam-to-cam baselines) is persisted in a single `SqliteConfigStore::saveRuntimeConfig` transaction so a failed gate or partial parse leaves the active calibration row untouched.

`replaceCalibration` deactivates any existing row for the same `camera_id` before inserting; the unique index `calibrations_one_active_per_camera` enforces "only one active calibration per camera" at the DB level.

### 3.7 Quality rating + acceptance gate (W2) — `throwIfUnacceptableCalibration`

`src/runtime/Daemon.cpp` exposes:

```cpp
void throwIfUnacceptableCalibration(
    const CameraCalibrationConfig& cal, const CalibrationToolConfig& tool, bool force);
void throwIfUnacceptableCameraImu(
    const CameraImuCalibrationConfig& cal, const CalibrationToolConfig& tool, bool force);
```

Both helpers throw a `std::runtime_error` when the parsed reprojection RMS is missing, non-positive, or above the configured threshold and `--force` is not set.

- Camera: `parseKalibrCameraResults` reads `results-cam.txt`, extracts the `+- [σ_u, σ_v]` pair, and stores `hypot(σ_u, σ_v)` as `reprojection_rms_px`. Threshold default 1.0 px (`max_reprojection_rms_px`).
- Camera-IMU: `parseKalibrCameraImuResults` reads `results-imucam.txt`, extracts per-camera reprojection error, plus IMU-side `Gyroscope error std` and `Accelerometer error std` (broadcast to every camera entry — they're global to the run). Threshold default 1.5 px (`max_camera_imu_rms_px`); gyro/accel are reporting-only.
- Both helpers also pull a best-effort report PDF path (`report-cam-*.pdf` / `report-imucam-*.pdf`) into `report_path`. Empty when Kalibr did not emit one.

Migration 12 added `reprojection_rms_px`, `observation_count`, `coverage_score`, `report_path` to `calibrations`; a parallel set to `camera_imu_calibrations`; and the two thresholds to `calibration_tool_config`. `coverage_score` is reserved for a future spatial-coverage metric and is always 0.0 today.

### 3.8 Dataset lifecycle (W5)

Two new subcommands operate on the `kalibr_datasets` table without touching the calibration outputs:

- `posest_daemon list-kalibr-datasets [--json]` — prints `id\tcreated_at\tduration\tcameras=...\ton_disk|missing` (or a JSON array, suitable for the future `GET /api/datasets`).
- `posest_daemon delete-kalibr-dataset --id ID [--remove-files]` — removes the row, then optionally `remove_all`s the directory. The disk side is idempotent (missing dir is fine).

`cleanupKalibrDataset` is a public helper used by both the CLI dispatch and the W6 `--cleanup-dataset` flag of the orchestrator.

### 3.9 End-to-end orchestrator (W6) — `runCalibrationEndToEnd`

`src/runtime/Daemon.cpp::runCalibrationEndToEnd` is the function the future HTTP layer will call. The CLI form is `calibrate-camera-end-to-end`. In sequence:

1. Resolve `--require-imu` → start TeensyService + camera producers, run for `--duration-s`, stop, write `session.json`. Persists the dataset row.
2. Look up `--target-id`, materialize `target.yaml` next to the dataset.
3. Build the bag (auto-`--no-imu` if `imu_samples_recorded == 0`).
4. Run `kalibr_calibrate_cameras` for the multi-camera topic list.
5. Parse `camchain.yaml` + `results-cam.txt`, run the W2 gate per camera.
6. Persist all intrinsics + cam-to-robot + cam-to-cam in one transaction.
7. If `--mode=intrinsic+imu`, run `kalibr_calibrate_imu_camera` on the same bag (requires `--imu PATH`), parse, gate, persist.
8. If `--cleanup-dataset`, remove the dataset row and directory.

Failure modes are tested explicitly in `test_calibrate_end_to_end.cpp` (`IntrinsicSuccessPersistsCalibrationsAndBaseline`, `FailedGateRollsBackNoCalibrations`, `IntrinsicAndImuPersistsBoth`, `IntrinsicSucceedsImuFailsKeepsIntrinsics`, `CleanupDatasetRemovesRowAndDirectory`, `KeepsDatasetByDefault`).

### 3.10 Persistence — SQLite schema today

| Table | Purpose | Migration |
|---|---|---|
| `calibrations` | Per-camera intrinsics + RMS / observation_count / coverage / report_path. Unique index enforces one active row per camera. | 1, refined in 2, extended in 12 |
| `camera_extrinsics` | Per-camera-version rigid transform from camera frame to robot frame. FK to `calibrations(camera_id, version)`. | 4 |
| `camera_to_camera_extrinsics` | Cam-to-cam baselines from Kalibr's `T_cn_cnm1`. | 13 |
| `camera_imu_calibrations` | Camera↔IMU extrinsic + time shift + RMS / gyro / accel / report_path. | 6, extended in 12 |
| `kalibr_datasets` | Recorded raw datasets (path, duration, camera ids) for traceability. | (in earlier migration) |
| `calibration_targets` | Operator-managed catalogue, default AprilGrid pre-seeded. | 11 |
| `calibration_tool_config` | Singleton row: docker image + the two RMS gate thresholds. | 5, extended in 12 |

Saves go through `SqliteConfigStore::saveRuntimeConfig` which performs full-config replacement under a single SQLite transaction.

### 3.11 Web / HTTP layer

`src/runtime/WebService.cpp` is a placeholder facade — it has `getConfig`, `stageConfig` (which calls `saveRuntimeConfig` and fires an `on_saved` callback so `FusionService` can hot-reload its tunables), `telemetry`, and `updateTelemetry`. There is **no HTTP server** wired to it. Health is exposed only via `posest_daemon --health-once` / `--health-interval-ms` (stdout JSON via `healthToJson`).

---

## 4. Status against the brief

| # | Brief requirement | Status | Where |
|---|---|---|---|
| 1 | Operator triggers calibration from the website | **Not implemented** — needs HTTP server. The orchestrator (`runCalibrationEndToEnd`) is already shaped to be the handler body once a server lands. | `src/runtime/Daemon.cpp`, `src/runtime/WebService.cpp` |
| 2 | Live camera video visible during calibration | **Not implemented** — `CalibrationRecorder` writes PNGs only. Needs an MJPEG / WebRTC stream sink hung off the camera producer. | `src/calibration/CalibrationRecorder.cpp` (no preview path); see 5.1 |
| 2a | AprilGrid corner overlay on the preview so the operator can validate detection | **Not implemented** — depends on (2) and on a detector running alongside the recorder. | — |
| 3 | Record video + IMU | **Implemented** — `CalibrationRecorder` writes PNGs + frames.csv + imu.csv + trigger_events.csv + session.json. | `src/calibration/CalibrationRecorder.cpp` |
| 4 | Build a ROS bag from the recording | **Implemented** — `make-kalibr-bag` + `scripts/kalibr/make_rosbag.py`. `--no-imu` for intrinsic-only datasets. | `src/runtime/Daemon.cpp::buildMakeKalibrBagDockerCommand`, `scripts/kalibr/make_rosbag.py` |
| 5 | Pass to Kalibr in a Docker container | **Implemented** — `buildKalibrDockerCommand`, `buildCalibrateCameraImuDockerCommand`. Test seam (`SystemImpl`) for hermetic tests. | `src/runtime/Daemon.cpp` |
| 6 | Store intrinsics / extrinsics against the specific camera | **Implemented** — `replaceCalibration` deactivates the prior row and writes the new intrinsics + extrinsics in the same transaction. Multi-camera + cam-to-cam baselines also persisted. | `calibrations`, `camera_extrinsics`, `camera_to_camera_extrinsics` tables |
| 7 | Multi-camera support | **Implemented** — repeated `--camera-id`/`--topic`/`--camera-to-robot`; `parseKalibrAllCameras` walks every cam<N>; partial Kalibr result is rejected. | `parseKalibrAllCameras`, `runConfigCommand::CalibrateCamera` |
| 8 | Display calibrated values to the user | **Backend ready, UI missing** — values are available via `RuntimeConfig`; `posest_daemon list-kalibr-datasets --json` proves the JSON-out shape. No UI surface. | — |
| 9 | Quality rating | **Partial** — Kalibr reprojection RMS parsed and stored; gate enforces a configurable threshold. No coverage score (placeholder 0.0). No human-readable rating bucket (good/ok/bad) — the website will need to compute that from `reprojection_rms_px` + `max_reprojection_rms_px`. | `parseKalibrCameraResults`, `throwIfUnacceptableCalibration`, `calibrations.reprojection_rms_px` |
| 10 | Calibration board selectable, default AprilGrid | **Backend implemented, UI missing** — `calibration_targets` table with a seeded default `default_aprilgrid_6x6`; `import-calibration-target` to add boards; `--target-id` on the calibration commands. | `migration11Sql`, `CalibrationTargetWriter`, `parseKalibrTargetYaml`, `import-calibration-target` |

---

## 5. What's missing to be feature-complete

These are the gaps that block the brief from being met. They all live above the daemon process.

### 5.1 Live preview during calibration

The recorder is a `IFrameConsumer`. The same camera producer can fan out to a *second* consumer that pushes JPEG-encoded frames to an HTTP/WebSocket endpoint. None of that exists yet. Concretely:

- A `PreviewStreamConsumer` (new) that subscribes to the same producer and exposes either an MJPEG endpoint (`GET /api/cameras/{id}/preview.mjpeg`) or a websocket carrying base64 JPEG frames.
- A simple AprilGrid detector running alongside the preview consumer to draw corner overlays / report "tags currently visible". `AprilTagPipeline` already runs detection — it can be reused, but not wired up.
- Frame budget concerns: the producer thread fans out synchronously, and the existing consumers are non-blocking via `LatestFrameSlot`. The preview consumer must follow the same pattern (drop-oldest mailbox + JPEG encode on a worker thread) so it cannot stall calibration recording.

### 5.2 HTTP server + REST API

`WebService` is in-process state only. No actual server is bound. To meet requirement 1 we need:

- An HTTP layer (e.g. cpp-httplib or Crow) embedded in `posest_daemon`.
- `POST /api/calibrate` that takes the same arguments as `calibrate-camera-end-to-end` and forwards into `runCalibrationEndToEnd`.
- `GET /api/datasets` / `DELETE /api/datasets/{id}` mapping to W5's list/delete helpers.
- `GET /api/targets` / `POST /api/targets` for the calibration-target catalogue.
- `GET /api/calibrations` / `GET /api/calibrations/{camera_id}` to surface intrinsics + RMS to the UI.
- `GET /api/health` mirroring `healthToJson`.

The `runCalibrationEndToEnd` function is already shaped to be a request handler body — it takes options + config store + camera factory and never touches global state (other than the `g_system` test seam).

### 5.3 Quality rating UX

Backend stores `reprojection_rms_px`. The website needs to bucket it into something an operator can read at a glance ("excellent / acceptable / retry") using `calibration_tool_config.max_reprojection_rms_px` as the upper bound. Coverage score (`coverage_score`) is reserved but always 0.0 today — populating it from a histogram of detected corner positions is a future workstream.

### 5.4 UI for board selection

The catalogue exists in SQLite (`calibration_targets`) and the `--target-id` flag plumbs the choice through the orchestrator. The website needs:

- A "select board" dropdown populated from `GET /api/targets`.
- An "add board" form mapping to `POST /api/targets` (= `import-calibration-target`).
- Default selection of the seeded `default_aprilgrid_6x6`.

### 5.5 Distortion model coverage

`buildKalibrDockerCommand` hardcodes `pinhole-radtan` for every camera. Wide-FOV / fisheye lenses will need `pinhole-equi` (or `omni-radtan`); the CLI needs a per-camera `--model` flag and the `CameraCalibrationConfig` already stores `camera_model` / `distortion_model` so the schema does not need to change.

### 5.6 Coverage score

`CameraCalibrationConfig::coverage_score` is reserved, always 0.0. A useful implementation would compute it from the spatial distribution of detected target corners across the image plane (e.g. fraction of a NxN grid that saw at least one detection) at recording time and stamp it onto the calibration row before the W2 gate. Out of scope today.

---

## 6. Verification

```bash
cmake --preset conan-release
cmake --build --preset conan-release -j
ctest --preset conan-release --output-on-failure -R 'Calibration|Calibrate|Kalibr'
```

Targeted GTest suites:

- `CalibrationRecorder.*` — recorder semantics (matched/unmatched frames, no-IMU acceptance).
- `CalibrationParsers.*` / `KalibrCameraResultsParser.*` / `KalibrCameraImuResultsParser.*` / `KalibrAllCamerasParser.*` — Kalibr fixture parsers (`test/fixtures/kalibr/`).
- `CalibrationTargetWriter.*` — catalogue ↔ Kalibr YAML round-trips.
- `KalibrDatasetLifecycle.*` — list/delete CLI.
- `CalibrateEndToEnd.*` — orchestrator success + failure paths via the `SystemImpl` test seam.
- `Daemon.*` (`test_daemon.cpp`) — CLI parsing for every subcommand and flag.
- `ConfigSchema.*` (`test_config_schema.cpp`) — full migration walk + round-trip of every new column / table.

Live hardware smoke (manual, one-shot):

```bash
posest_daemon import-calibration-target --config posest.db \
    --target-id apgrid_88 --type aprilgrid --rows 6 --cols 6 \
    --tag-size-m 0.088 --tag-spacing-ratio 0.3

posest_daemon calibrate-camera-end-to-end --config posest.db \
    --camera-id cam0 --camera-id cam1 \
    --topic /posest/cam0/image_raw --topic /posest/cam1/image_raw \
    --camera-to-robot 0.10,0,0.20,0,0,0 \
    --camera-to-robot -0.10,0,0.20,0,0,0 \
    --duration-s 60 --target-id apgrid_88 \
    --version v1 --output-dir ./datasets/cal-$(date +%s) \
    --mode intrinsic --require-imu no
```

`posest_daemon --config posest.db --health-once` then prints a JSON snapshot that can be eyeballed for the freshly-written rows; `posest_daemon list-kalibr-datasets --config posest.db --json` confirms the dataset entry.
