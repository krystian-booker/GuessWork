# Camera Calibration (Intrinsics + Extrinsics) — Status Review

Snapshot of where camera intrinsic/extrinsic calibration stands against the
brief. Walks the requested end-to-end flow and identifies what is implemented,
what is partial, and what still needs to be built.

The intent stated by the brief is:

1. The user starts intrinsic / extrinsic calibration from the website /
   HTTP server.
2. While calibrating, the live camera video is shown so the user can verify
   they are getting good reads of the AprilGrid.
3. The application records video and IMU data, builds a ROS bag, then runs
   Kalibr inside a Docker container.
4. Results (per-camera intrinsics and extrinsics, single- or multi-camera)
   are stored against the calibrated camera.
5. The user is shown the calibrated values plus a quality rating.
6. A calibration board (default AprilGrid) and its size are selectable from
   the WebUI before the run.

Files audited:
- `include/posest/calibration/CalibrationRecorder.h`,
  `src/calibration/CalibrationRecorder.cpp`
- `src/runtime/Daemon.cpp` (calibration subcommands, Docker command builders,
  result ingestion)
- `include/posest/runtime/Daemon.h` (`*Options` structs, `DaemonCommand`)
- `include/posest/runtime/RuntimeConfig.h`
  (`CameraCalibrationConfig`, `CameraExtrinsicsConfig`,
  `CameraImuCalibrationConfig`, `KalibrDatasetConfig`, `CalibrationToolConfig`)
- `src/config/CalibrationParsers.cpp`
  (`parseKalibrCameraCalibration`, `parseKalibrCameraImuCalibration`,
  `parsePoseCsv`)
- `src/config/SqliteSchema.cpp` (migrations 3, 4 — `calibrations`,
  `camera_extrinsics`, `kalibr_datasets`, `camera_imu_calibrations`,
  `calibration_tool_config`)
- `src/config/ConfigValidator.cpp`
- `scripts/kalibr/make_rosbag.py`
- `include/posest/runtime/WebService.h`, `src/runtime/WebService.cpp`
- `test/test_calibration_recorder.cpp`, `test/test_daemon.cpp`

---

## 1. Today's end-to-end flow

The system runs the calibration as a sequence of **CLI subcommands** of
`posest_daemon`. There is no website driving it yet — the WebService class
exists as an in-process facade only (no HTTP server, no live video, no
calibration endpoints). A human operator currently runs:

1. `posest_daemon record-kalibr-dataset`
   (`src/runtime/Daemon.cpp:693-754`) — opens the configured cameras through
   `ProductionCameraFactory`, starts `TeensyService` (waits up to 5 s for
   time sync), and feeds frames + IMU samples + camera-trigger events into
   `CalibrationRecorder` for `--duration-s`. The recorder writes
   per-frame PNGs under `output_dir/images/`, plus three CSVs
   (`frames.csv`, `imu.csv`, `trigger_events.csv`) and a `session.json`
   summary. `rememberDataset()` then registers the dataset into
   `RuntimeConfig::kalibr_datasets`.

2. `posest_daemon make-kalibr-bag` (`src/runtime/Daemon.cpp:756-765`) —
   shells out to Docker with the recorded dataset mounted read-only; the
   container runs `scripts/kalibr/make_rosbag.py`, which iterates the
   matched rows of `frames.csv` (skipping unmatched ones) and writes a
   ROS-1 bag with `/posest/<camera_id>/image_raw` plus
   `/posest/imu/data_raw`.

3. `posest_daemon calibrate-camera` (`src/runtime/Daemon.cpp:655-680`) —
   shells out to Docker again, runs `kalibr_calibrate_cameras` with
   `--models pinhole-radtan` against the bag, finds a `*camchain*.yaml`
   in the output dir, parses it via
   `config::parseKalibrCameraCalibration`, and stores the result through
   `replaceCalibration` (which deactivates prior versions, overwrites
   matching `(camera_id, version)` tuples, and writes a
   `CameraExtrinsicsConfig` from the user-supplied `--camera-to-robot
   x,y,z,roll,pitch,yaw`).

4. `posest_daemon calibrate-camera-imu` (`src/runtime/Daemon.cpp:767-801`)
   — re-bags the dataset, exports the active per-camera intrinsics into a
   Kalibr `input-camchain.yaml`, runs `kalibr_calibrate_imu_camera` in
   Docker, then parses the resulting `*camchain-imucam*.yaml` for each
   camera and stores via `replaceCameraImuCalibration` (writes
   `CameraImuCalibrationConfig`: `T_cam_imu`, `T_imu_cam`,
   `timeshift_cam_imu`).

The operator-facing surface is the usage string in `daemonUsage()`
(`Daemon.cpp:413-435`).

### What's implemented, in detail

#### `CalibrationRecorder` (`src/calibration/CalibrationRecorder.cpp`)

- `IFrameConsumer` + `IMeasurementSink` in one class, so the daemon
  subscribes the same instance to camera producers and to the
  `MeasurementBus` indirectly through `TeensyService`.
- Worker thread pulls a `std::variant<FramePtr, ImuSample,
  CameraTriggerEvent>` off a deque, so all writes happen off the
  capture thread. `enqueue()` is non-blocking (matches the
  `IFrameConsumer::deliver()` contract).
- Per-frame matching against the latest `CameraTriggerEvent` for the
  configured `teensy_pin` (`matchTrigger`,
  `CalibrationRecorder.cpp:218-231`). When a match is found, the
  emitted ROS timestamp is `trigger_timestamp + trigger_to_exposure_center_us`
  rather than the V4L2 buffer time — i.e., the recording is set up to
  hand Kalibr the optical-center timestamp, not the host arrival time.
  Frames whose latest known trigger is *after* `frame.capture_time`
  are treated as unmatched.
- `frames.csv` columns:
  `camera_id,frame_sequence,image_path,ros_timestamp_us,
   capture_timestamp_us,trigger_timestamp_us,trigger_sequence,
   trigger_pin,matched`. `matched=0` rows are filtered out by the
   bag generator.
- `imu.csv` columns:
  `timestamp_us,accel_x_mps2,accel_y_mps2,accel_z_mps2,
   gyro_x_radps,gyro_y_radps,gyro_z_radps,temperature_c,status_flags`.
- `session.json` is the manifest the next stage (`make-kalibr-bag`,
  `calibrate-camera-imu`) reads: it records `duration_s`, `camera_ids`,
  `trigger_to_exposure_center_us`, and counts of frames seen / recorded
  / without trigger / IMU samples / trigger events.
- `throwIfUnacceptable()` enforces `frames_seen > 0` and
  `frames_recorded / frames_seen >= min_trigger_match_fraction`
  (default 1.0). This is the only programmatic acceptance check
  currently in the system.
- Tested in `test/test_calibration_recorder.cpp`
  (`WritesMatchedFramesImuTriggersAndSessionMetadata`): matched-trigger
  path, image written, `frames.csv` and `session.json` contents.

#### Persistence (SQLite, `src/config/SqliteSchema.cpp`)

- `calibrations` (migration 3) holds intrinsics per
  `(camera_id, version)`: image size, `camera_model`, `distortion_model`,
  `fx fy cx cy`, distortion coefficients JSON, `active` flag with a
  partial unique index `calibrations_one_active_per_camera`.
- `camera_extrinsics` (migration 3) holds `camera_to_robot` as
  `tx ty tz roll pitch yaw`, FK to `(camera_id, version)`.
- `field_layouts` + `field_tags` (migration 3) hold the WPILib field-tag
  layout (separate ingestion via `import-field-layout`).
- `calibration_tool_config` (migration 4): single-row `docker_image`
  default `kalibr:latest`. Resolution priority is
  `--docker-image` > `POSEST_KALIBR_DOCKER_IMAGE` env > config row >
  `kalibr:latest` (`Daemon.cpp:163-177`).
- `kalibr_datasets` (migration 4): id (absolute path), path,
  `created_at`, `duration_s`, `camera_ids_json`. Used purely as a
  history list — nothing reads it back today.
- `camera_imu_calibrations` (migration 4): the full 4×4 `T_cam_imu`
  decomposed into `[t, rpy]`, the inverse `T_imu_cam`, `time_shift_s`,
  per-version with a partial unique index for `active=1`. Validation is
  in `ConfigValidator::validateCameraImuCalibrations`.

#### Result parsing (`src/config/CalibrationParsers.cpp`)

- `parseKalibrCameraCalibration` reads the chosen camera node by
  `rostopic`, requires `intrinsics` to be `[fx, fy, cx, cy]` and
  `resolution` to be `[w, h]`, copies `camera_model`,
  `distortion_model`, `distortion_coeffs`. Defaults are `pinhole` /
  `none` if absent.
- `parseKalibrCameraImuCalibration` requires `T_cam_imu` (4×4),
  decomposes to `(t, rpy)`, optionally reads `T_imu_cam` and
  `timeshift_cam_imu`.
- `poseFromKalibrMatrix` does a `[r11..r33]` → RPY decomposition with
  the gimbal-lock fallback (`Daemon.cpp` then stores both rpy vectors).

#### Validation (`src/config/ConfigValidator.cpp`)

- `kalibr_datasets` entries must have an id, a path, a positive
  duration, and only reference known cameras (no duplicates per
  dataset).
- `calibration_tools.docker_image` must be non-empty.
- Camera–IMU calibrations are validated for nonzero version and
  reasonable time-shift bounds (covered in `validateCameraImuCalibrations`).

#### Tests (`test/test_daemon.cpp`)

- `parseDaemonOptions` is covered for each subcommand.
- `buildKalibrDockerCommand`, `buildMakeKalibrBagDockerCommand`,
  `buildCalibrateCameraImuDockerCommand` produce the expected `docker
  run …` strings (mounts, image, args).
- `runConfigCommand` is **not** end-to-end tested — Docker isn't run in
  CI, so the round-trip through Kalibr is implicitly trusted.

---

## 2. How well this matches the brief

| Brief requirement | State |
|---|---|
| User triggers calibration from a website / HTTP server | **Missing.** `WebService` is an in-process façade (`getConfig`/`stageConfig`/`telemetry`) with no HTTP transport. Calibration is invoked by re-launching `posest_daemon` with a subcommand. |
| Live camera video visible to the user during calibration | **Missing.** No video preview, no MJPEG/WebRTC stream, no AprilGrid-detection overlay. The recorder writes PNGs to disk; nothing emits a live preview frame. |
| Record video + IMU during the session | **Implemented** by `CalibrationRecorder` (PNGs + CSVs + manifest, trigger-matched timestamps). |
| Produce a ROS bag from the recording | **Implemented** by `make-kalibr-bag` + `scripts/kalibr/make_rosbag.py` (ROS-1 bag, `/posest/<camera_id>/image_raw` + `/posest/imu/data_raw`). |
| Run the bag through Kalibr in Docker | **Implemented** for both `kalibr_calibrate_cameras` and `kalibr_calibrate_imu_camera`. Docker image resolution chain is in place. |
| Store intrinsics + extrinsics against the camera (single + multi) | **Partially implemented.** Single-camera intrinsics + `camera_to_robot` extrinsics + camera→IMU extrinsics all persist. Multi-camera intrinsic/extrinsic *across cameras* (the cam-to-cam transforms Kalibr produces in `camchain.yaml`) is **not** persisted — see §3.A. |
| Display calibrated values + quality rating | **Missing.** No reprojection error / pose-completeness rating is read from Kalibr's report and no UI surfaces values. The recorder's only rating is the trigger-match fraction (recording-time), not Kalibr's reprojection RMS. |
| Selectable calibration board (default AprilGrid) and size | **Missing.** Today the user passes `--target /path/to/target.yaml` on the CLI. There is no in-app target description, no default AprilGrid template, and no DB-backed selection. |
| Multi-camera support | **Recording supports multiple `--camera-id`s** in one run; **Kalibr cam-to-cam transforms are not parsed or stored**. `calibrate-camera-imu` does iterate `camera_ids` and writes a per-camera `CameraImuCalibrationConfig`. |
| AprilGrid validation feedback | **Missing.** No live tag-detection overlay during recording; no Kalibr-detection summary surfaced after the run. |

---

## 3. Gaps and missing work

### A. Multi-camera Kalibr output is only partially ingested

`exportActiveCamchain` (`Daemon.cpp:204-238`) writes a multi-cam
`input-camchain.yaml` (`cam0`, `cam1`, …) for the camera-IMU stage, so
the ingestion side knows multi-camera datasets exist. But:

- `parseKalibrCameraCalibration` reads exactly **one** camera node (the
  one whose `rostopic` matches `--topic`). When Kalibr produces a
  multi-camera `camchain.yaml`, only the requested camera's intrinsics
  are persisted on the `calibrate-camera` path. To calibrate N cameras
  in one run today, the operator must invoke `calibrate-camera` N times
  and they share one Kalibr output dir.
- `T_cn_cnm1` (the cam-to-cam baseline that Kalibr writes between
  adjacent cameras in a multi-camera rig) is **not** parsed and **not**
  persisted anywhere. There is no `camera_to_camera` extrinsics table
  or struct.
- Only `calibrate-camera-imu` parses every entry in the multi-camera
  YAML (`Daemon.cpp:790-800`), and it only reads `T_cam_imu`.

For multi-camera setups (cam0 + cam1 sharing a target), the inter-camera
extrinsic is currently lost.

### B. `--target` is a path the operator hand-curates

There is no library of calibration targets, no template/default
AprilGrid YAML in `share/`, no validator for it, and no CLI/DB way to
say "this is a 6×6 AprilGrid, 0.088 m tag size, 0.025 m gap." The
flag just gets bind-mounted into the Kalibr container.

To meet the brief ("default to AprilGrid, configurable on the WebUI"),
the system needs:
- A `CalibrationTargetConfig` struct + SQLite table with at least
  `{ id, type ∈ {aprilgrid, checkerboard, circles, ...}, rows, cols,
  size_m, spacing_m | tag_size_m + tag_spacing_m }`.
- An `IDs → target.yaml` writer that emits the YAML Kalibr expects, so
  the operator picks a target from the DB and the daemon writes it to
  a tempfile on demand.
- A default AprilGrid row in the DB (the existing migration baseline
  pattern fits — see how `calibration_tool_config` is seeded).

### C. No HTTP / web layer at all for calibration

`WebService` (`include/posest/runtime/WebService.h`) is essentially a
DAO wrapper over `IConfigStore` plus an in-memory `TelemetrySnapshot`.
There is no embedded HTTP server (no httplib/Crow/Drogon/Restbed in
`conanfile.txt` either), no REST endpoints, no event stream, no
authentication, no UI hosting. To fulfill the brief, **the entire
website + HTTP server is missing**, not just the calibration screens.

What needs to land for the calibration flow specifically:

1. **POST start / stop** for an intrinsic-only or full IMU calibration
   session, parameterized by:
   - selected camera ids
   - duration
   - calibration target id (from §3.B)
   - calibration version label
2. **Live preview** during the recording session (see §3.D).
3. **GET status** that polls dataset progress (frames recorded, IMU
   sample count, trigger-match fraction so far).
4. **GET result** that returns the parsed Kalibr output + a quality
   score (see §3.E) and a permalink to the source files.

### D. No live preview path

The capture side already publishes `Frame` shared pointers through the
producer→consumer fan-out, but no consumer turns frames into a live
encoded stream for the WebUI:

- No MJPEG/WebRTC encoder in the codebase (only V4L2 *decode* of MJPEG
  → BGR, in `V4L2Producer.cpp:114`).
- No HTTP server to host the stream.
- No AprilGrid detection running on the recording-time frames. The
  existing `AprilTagPipeline` consumes a single tag family configured
  in `RuntimeConfig` (see `docs/features/apriltag-pipeline.md`); the
  Kalibr AprilGrid uses a different tag family (`tag36h11` grouped),
  and there is no detector/overlay that draws the recognized tag
  corners onto the preview so the operator can sanity-check the read
  before recording ends.

The brief calls this the "most difficult part" — it's also the part
with the largest implementation gap.

### E. No quality rating from Kalibr

Kalibr emits a per-camera reprojection RMS in its results YAML and a
PDF report; today neither the YAML nor the report is parsed for a
rating. After `runConfigCommand` returns, the only feedback the
operator gets is whether the Docker container exited zero. There is
also no:

- Reprojection-error column in the `calibrations` schema.
- "Coverage" metric (how many of the AprilGrid corners were observed,
  in how many distinct poses).
- Surface for the IMU-camera time shift / accelerometer-walk warnings
  Kalibr prints.

A reasonable shape:
- Add `reprojection_rms_px`, `coverage_score`, `pose_count`, and a
  free-form `report_path` to `CameraCalibrationConfig` (and a parallel
  set on `CameraImuCalibrationConfig`).
- Parse them out of Kalibr's `*results-cam.yaml` (camera-only) and
  `*report-imucam.pdf`/`*results-imucam.txt` (camera-IMU) at
  ingestion time.
- Translate to a 0–100 score in a single helper so the WebUI doesn't
  need to know Kalibr's units.

### F. `record-kalibr-dataset` requires a working Teensy time-sync

`Daemon.cpp:723-732` aborts the recording if Teensy time sync isn't
established within 5 s. That's correct for VIO recordings (camera-IMU
calibration *needs* a synced IMU stream), but it makes intrinsic-only
calibration unnecessarily fragile — a user with a disconnected Teensy
cannot even record an intrinsic dataset today. There is no
`--imu=optional` / `--no-imu` flag.

If the WebUI is going to expose intrinsic-only calibration as a
separate workflow, this gate should be conditional.

### G. No cleanup of stale datasets / outputs

`rememberDataset` adds entries to `RuntimeConfig::kalibr_datasets` but
nothing ever removes them, no command lists them, nothing reads them
back, and disk space (PNGs + bag) is not garbage-collected. For a
website-driven flow, dataset lifecycle (list, view, delete, re-run
calibration on an existing dataset) is missing.

### H. Acceptance hook is recording-time, not Kalibr-time

`CalibrationRecorder::throwIfUnacceptable()` only checks
`frames_recorded / frames_seen >= min_trigger_match_fraction`. There
is no equivalent post-Kalibr gate that would refuse to write a
calibration whose reprojection RMS exceeds some threshold. Today, even
a wildly bad Kalibr run will overwrite the active calibration row
silently.

### I. No image undistortion / rectification preview after the run

Once Kalibr produces intrinsics, the natural acceptance check is
"undistort a few sample frames and show the user." Nothing in the
codebase does that — neither during recording nor after Kalibr
returns.

---

## 4. Status by brief sub-requirement

| Sub-requirement (paraphrased) | Implementation | Where |
|---|---|---|
| User starts the calibration from the website | **Missing** — no HTTP server | — |
| Live camera video during calibration | **Missing** — no preview path | — |
| Record video + IMU | **Implemented** | `CalibrationRecorder` |
| Trigger-aligned frame timestamps | **Implemented** | `matchTrigger`, `processFrame` |
| Build a ROS bag | **Implemented** | `scripts/kalibr/make_rosbag.py`, `make-kalibr-bag` |
| Run Kalibr in Docker | **Implemented** | `buildKalibrDockerCommand`, `buildCalibrateCameraImuDockerCommand` |
| Store intrinsics per camera | **Implemented** | `calibrations` table, `parseKalibrCameraCalibration`, `replaceCalibration` |
| Store camera-to-robot extrinsics | **Implemented** | `camera_extrinsics` table — but supplied **on the CLI**, not solved by Kalibr |
| Store cam-to-cam extrinsics for multi-cam rigs | **Missing** | `T_cn_cnm1` is not parsed |
| Store camera-to-IMU extrinsics | **Implemented** | `camera_imu_calibrations` table, `parseKalibrCameraImuCalibration` |
| Multi-camera support | **Partial** — recording yes; cross-camera result ingestion no | — |
| Display calibrated values | **Missing** — no UI; CLI only writes to SQLite | — |
| Quality rating | **Missing** — no Kalibr report parsing, no schema columns | — |
| Selectable target board | **Missing** — `--target` is a hand-curated YAML path | — |
| Default AprilGrid | **Missing** — no template, no default in DB | — |
| Configurable target size | **Missing** | — |
| AprilGrid live-validation overlay | **Missing** | — |

---

## 5. Suggested next steps (in dependency order)

1. **Pick an HTTP server** (httplib is the smallest dep that fits the
   existing `conanfile.txt` style) and stand up `WebService` as a real
   server with at minimum `/api/config` (already implementable from
   `IConfigStore`) and `/api/health` (already implementable from
   `DaemonHealth`). This is the foundation for every calibration UI
   below.

2. **Add a `CalibrationTargetConfig` table + struct + validator + a
   default AprilGrid row.** Add a writer that emits the Kalibr YAML on
   demand. Replace `--target PATH` with `--target-id ID` on the CLI
   (and keep PATH as a back door for power users). This unblocks the
   "selectable target" requirement.

3. **Wire a live preview consumer.** The lowest-risk shape is a new
   `posest_preview` consumer that subscribes to selected cameras,
   re-encodes to MJPEG, and exposes
   `/api/preview/<camera_id>.mjpg`. Run it only while a calibration
   session is active. Add an AprilGrid detector layer (separate
   tag-family configuration from the existing field-side `AprilTagPipeline`)
   that overlays detected corners on the preview.

4. **Make IMU sync conditional on the calibration mode.** Split
   `record-kalibr-dataset` into intrinsic-only (no Teensy gate) and
   intrinsic+IMU paths, or add `--require-imu={auto,yes,no}`.

5. **Parse Kalibr quality metrics.** Add reprojection RMS, coverage,
   pose count, and a `report_path` to `CameraCalibrationConfig` and
   `CameraImuCalibrationConfig` and persist them. Add an acceptance
   threshold (refuse-to-overwrite gate, similar to
   `throwIfUnacceptable`).

6. **Persist multi-camera cross-camera extrinsics.** Add a
   `camera_to_camera_extrinsics` table keyed by
   `(reference_camera_id, target_camera_id, version)` and parse
   `T_cn_cnm1` into it. Composing a chain back to `camera_to_robot`
   for `cam0` is enough to derive every camera's `camera_to_robot`
   pose without making the operator type each one.

7. **Wrap steps 1–3 of today's CLI flow into a single
   `POST /api/calibrate` endpoint.** It should:
   - take `{ camera_ids[], duration_s, target_id, version,
     mode ∈ {intrinsic, intrinsic+imu}, camera_to_robot? }`,
   - run record → bag → calibrate sequentially in a worker thread,
   - publish progress over an SSE/WebSocket channel,
   - return the parsed result (intrinsics, extrinsics, quality
     score) once stored.

8. **Add a "review" screen** that shows the new intrinsics, the
   reprojection RMS, an undistorted sample frame, and prompts the
   operator to either activate or discard. The undistortion preview
   needs an OpenCV `cv::initUndistortRectifyMap` + `cv::remap` helper
   — none exists today.

9. **Add dataset lifecycle management** (list, delete, re-run
   calibration on an existing dataset) on top of the
   `kalibr_datasets` table that already exists but is currently
   write-only.
