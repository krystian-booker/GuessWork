# GuessWork FRC Pose Estimation Architecture

## Summary
GuessWork is evolving into a single C++20 daemon for FRC robot pose estimation. The daemon owns camera capture, configured vision pipelines, timestamped measurement transport, placeholder fusion, Teensy/CAN output boundaries, SQLite configuration, health reporting, and eventually the web UI/API.

The main architecture is now in place as compileable scaffolding. The project can load SQLite config, build a runtime camera-to-pipeline graph, run placeholder/real vision pipelines, publish typed measurements, expose daemon health as JSON, and pass tests for the core data flow. GTSAM, real Teensy serial I/O, real CAN transport, real VIO, pose-producing AprilTag fusion, and the website remain future work.

## Implemented

### Core Runtime And Data Flow
- `posest_core`
  - Frame producer/consumer interfaces.
  - `ProducerBase`, `ConsumerBase`, `LatestFrameSlot`.
  - `MeasurementBus` for ordered bounded measurement delivery.
  - Typed measurements:
    - `AprilTagObservation`
    - `VioMeasurement`
    - `ImuSample`
    - `WheelOdometrySample`
    - `FusedPoseEstimate`
- `RuntimeGraph`
  - Builds configured cameras and pipelines through factories.
  - Wires camera producers to `IVisionPipeline` consumers before startup.
  - Starts pipelines before cameras and stops cameras before pipelines.
  - Skips disabled cameras and disabled pipelines.

### Camera Capture
- `posest_camera`
  - `CameraConfig` with `enabled`, type, device, format, and controls.
  - `CameraProducer` lifecycle abstraction.
- `posest_v4l2`
  - Linux V4L2 producer.
  - Stable device path support.
  - Format/control application.
  - MMAP streaming.
  - V4L2 monotonic timestamp conversion to `steady_clock`.
- Future camera backends are still planned as peers:
  - `posest_avfoundation`
  - `posest_spinnaker`

### SQLite Config Foundation
- SQLite is now the only persistent config backend.
- Removed XML config completely:
  - Removed XML config source interface.
  - Removed XML parser.
  - Removed sample XML config.
  - Removed camera config utility.
  - Removed pugixml dependency.
- Added `SqliteConfigStore`.
  - Opens/creates DB.
  - Applies schema migrations with `PRAGMA user_version`.
  - Enables foreign keys.
  - Loads/saves full `RuntimeConfig`.
  - Saves atomically in one transaction.
- Added strict config validation.
  - Rejects duplicate/empty IDs.
  - Rejects invalid camera dimensions/FPS.
  - Rejects duplicate camera controls.
  - Rejects invalid bindings.
  - Rejects invalid calibration refs.
  - Rejects invalid Teensy publish rate.
  - Parses pipeline `parameters_json` with `nlohmann_json`.
- Schema currently covers:
  - Cameras.
  - Camera controls.
  - Pipelines.
  - Camera-pipeline bindings.
  - Calibrations.
  - Singleton Teensy config.

### Daemon Skeleton
- Added `posest_daemon`.
- Added reusable daemon layer:
  - `DaemonOptions`
  - `DaemonHealth`
  - `DaemonController`
  - Signal helper for `SIGINT` / `SIGTERM`
- CLI support:
  - `--config PATH`
  - `--health-once`
  - `--health-interval-ms N`
  - `--help`
- Default config path:
  - `./posest.db`
- Startup order:
  - Open SQLite config.
  - Load runtime config.
  - Create `MeasurementBus`.
  - Create `FusionService`.
  - Create `TeensyService` output sink.
  - Create `WebService` facade.
  - Build `RuntimeGraph`.
  - Start fusion, then graph.
- Shutdown order:
  - Stop graph.
  - Stop fusion.
  - Fusion shutdown currently shuts down the shared `MeasurementBus`.
- Health JSON is serialized with `nlohmann_json`.

### Vision Pipelines
- Added `posest_pipelines`.
- Added `VisionPipelineBase`.
  - Implements `IVisionPipeline`.
  - Owns worker thread.
  - Uses `LatestFrameSlot`.
  - Keeps producer-side frame delivery nonblocking.
- Added placeholder pipelines:
  - `PlaceholderAprilTagPipeline`
    - Publishes timestamped `AprilTagObservation` with zero detections.
  - `PlaceholderVioPipeline`
    - Publishes `VioMeasurement` with `tracking_ok=false` and `backend_status="placeholder"`.
- Added real `AprilTagPipeline`.
  - Uses Conan `apriltag/3.4.2`.
  - Supports FRC default `tag36h11`.
  - Parses pipeline parameters with `nlohmann_json`.
  - Converts 8-bit grayscale/BGR/BGRA frames to AprilTag input.
  - Publishes tag IDs and image corners.
  - Leaves `camera_to_tag` unset until calibration/intrinsics exist.
- `ProductionPipelineFactory`
  - `apriltag` -> real AprilTag detector.
  - `vio` -> placeholder VIO.
  - `mock_apriltag` -> placeholder AprilTag.
  - Unknown pipeline types still fail fast.

### Teensy Protocol Boundary
- Added binary protocol codec.
- Frame format includes:
  - Magic.
  - Protocol version.
  - Message type.
  - Sequence.
  - Payload length.
  - Payload.
  - CRC.
- Added stream decoder.
  - Handles chunked input.
  - Tracks CRC failures.
  - Tracks sequence gaps.
- Added `TeensyService` as a fusion output sink.
  - Encodes the latest fused pose into an outbound protocol frame.
  - Does not yet own a real USB serial port.

### Fusion Boundary
- Added `FusionService`.
  - Consumes ordered measurements from `MeasurementBus`.
  - Rejects stale measurements.
  - Publishes placeholder fused pose estimates to output sinks.
  - Tracks basic stats.
- GTSAM is not integrated yet.
- Current fusion behavior is deterministic placeholder logic only.

### Web Boundary
- Added `WebService` facade.
  - Reads/stages config through `IConfigStore`.
  - Stores basic telemetry snapshot.
- No HTTP server or WebSocket server exists yet.
- No frontend exists yet.

### Dependencies Added
- `sqlite3/3.45.3`
- `nlohmann_json/3.11.3`
- `apriltag/3.4.2`

## Test Coverage Implemented
- Producer/consumer latency contract.
- Latest-frame mailbox behavior.
- V4L2 static/control/timestamp helpers.
- SQLite schema creation, migrations, load/save, reopen, validation, and atomic save behavior.
- Measurement bus FIFO and bounded overflow behavior.
- Runtime graph construction and lifecycle.
- Teensy protocol encode/decode, CRC failure, and sequence gaps.
- Fusion ordered processing and stale measurement rejection.
- Daemon option parsing, lifecycle, failure health, signal wait loop, and health JSON.
- Placeholder AprilTag and VIO pipelines.
- Real AprilTag blank-image behavior.
- Real AprilTag `tag36h11` detection using generated in-memory fixture.
- RuntimeGraph camera-to-pipeline-to-measurement flow.

Current full-suite status at the time of this update:
- `ctest --preset conan-release --output-on-failure`
- `100% tests passed, 0 tests failed out of 65`

## Still Missing

### Real Fusion / GTSAM
- Add GTSAM dependency.
- Replace placeholder `FusionService` logic with real graph/iSAM2 ownership.
- Add factors for:
  - AprilTag observations.
  - IMU preintegration.
  - Wheel odometry.
  - VIO measurements.
- Add covariance handling.
- Add timestamp alignment and out-of-order measurement policy.
- Add estimator reset/recovery behavior.

### AprilTag Pose Estimation
- Add camera calibration model.
- Store calibration/intrinsics in SQLite or calibration files referenced by SQLite.
- Add camera extrinsics relative to robot frame.
- Compute `camera_to_tag` or publish enough calibrated factors for GTSAM.
- Add field layout/tag map support.
- Add ambiguity/error metrics beyond current ID/corner output.

### Real VIO
- Choose VIO backend.
- Replace placeholder VIO pipeline.
- Define VIO config schema.
- Publish real relative motion or visual factors.
- Integrate VIO output into GTSAM.

### Real Teensy USB Serial I/O
- Implement USB serial transport.
- Decode inbound Teensy messages into:
  - `ImuSample`
  - `WheelOdometrySample`
  - CAN RX/status messages
  - time sync responses
- Encode outbound commands and CAN TX messages.
- Add reconnect handling.
- Add high-rate read loop.
- Add host/Teensy clock synchronization.

### CAN / RoboRIO Output
- Define final CAN IDs and packet layouts.
- Send fused pose, covariance/quality, and health flags through Teensy.
- Add rate limiting and stale-estimate behavior.
- Add tests with mock Teensy/CAN transport.

### Runtime Control / Hot Reload
- Current daemon supports one process-lifetime start/stop cycle.
- Hot reload is not implemented.
- Runtime topology changes still require restart.
- `FusionService::stop()` shuts down the shared `MeasurementBus`, so restart requires recreating the bus and services.

### Web API And Frontend
- Add embedded HTTP API.
- Add WebSocket telemetry.
- Add config CRUD endpoints.
- Add operational dashboard:
  - Camera status.
  - Camera controls.
  - Pipeline assignment/tuning.
  - Calibration status.
  - Teensy/CAN/IMU health.
  - Live fused pose diagnostics.

### Additional Camera Backends
- Add Spinnaker backend.
- Add AVFoundation backend.
- Add backend-specific discovery/config tooling through the web/API layer.

### Deployment / Ops
- Add structured logging.
- Add metrics beyond basic JSON health.
- Add robot deployment scripts/systemd service.
- Add configuration bootstrap tooling for first-run DB creation.
- Add hardware-gated integration tests.

## Recommended Next Implementation Order
1. Real Teensy serial transport and inbound IMU/wheel odometry decoding.
2. Camera calibration/intrinsics config model.
3. AprilTag calibrated pose/factor output.
4. Initial GTSAM fusion with wheel odometry + AprilTag.
5. IMU preintegration in GTSAM.
6. CAN output to RoboRIO through Teensy.
7. Web API and dashboard.
8. Real VIO backend.
9. Spinnaker and AVFoundation camera backends.

## Assumptions
- First production runtime target is a Linux coprocessor on an FRC robot.
- FRC AprilTag default family is `tag36h11`.
- SQLite plus calibration files is the persistent config strategy.
- Existing latest-frame behavior remains correct for camera-to-pipeline delivery.
- Ordered bounded queues remain required for IMU/odom/vision measurements into fusion.
