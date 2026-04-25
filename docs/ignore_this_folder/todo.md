I’d implement in this order:

Can you put a plan together to implement the next part:

Fusion service with GTSAM
Replace the placeholder fusion logic with real GTSAM/iSAM2 ownership, starting with wheel odometry + AprilTag.
Add GTSAM dependency.
Fuse wheel odometry + AprilTag measurements first.
Produce real FusedPoseEstimate with covariance/status.

IMU preintegration
Feed high-rate IMU samples into GTSAM.
Add bias handling, timestamp alignment, stale/out-of-order policies.

CAN output to RoboRIO
Encode fused FusedPoseEstimate into Teensy outbound messages, define CAN IDs/rates, and add health/fallback behavior for stale estimates.
Define CAN IDs and packet layouts.
Send fused pose, covariance/quality, and health.
Add rate limits and stale-estimate behavior.

Web API
Add REST endpoints for config CRUD and commands, plus WebSocket telemetry for camera/pipeline/fusion/Teensy status.
Add embedded HTTP server.
Add config CRUD endpoints.
Add command endpoints for reload/calibration/control.
Add WebSocket telemetry.


Frontend dashboard
Build the operational UI: camera setup, pipeline assignment, tuning, calibration state, Teensy/CAN health, and live pose diagnostics.
Camera status/config.
Pipeline assignment/tuning.
Calibration management.
Teensy/CAN/IMU health.
Live fused pose diagnostics.

Real VIO backend
Kimera-VIO
Replace placeholder VIO.
Integrate VIO factors/relative motion into GTSAM.
Default settings but configurable:
ParameterRecommended ValueWhy it matters for your robotfeature_detector_type1 (GFFT/Harris)Better than FAST for tracking the low-contrast, repetitive fibers of carpet.maxFeaturesPerFrame600 - 800High density ensures enough features survive the strict RANSAC outlier rejection.klt_win_size31A larger tracking window is required to "catch" the massive pixel shifts at 5 m/s.klt_max_level4You must use at least 4 pyramid levels to track motion at lower image resolutions.maxFeatureAge10 - 15At your speed, the floor flies by in fractions of a second. Don't waste CPU trying to track old features.quality_level0.001Drop this slightly so the tracker doesn't starve for points on highly uniform carpet patches.min_distance15Keeps your tracked points spread out across the lens footprint.

Hardware backends and polish
Add Spinnaker and AVFoundation after the core runtime is stable. Then harden logging, metrics, calibration workflows, startup recovery, and deploy tooling.

The key thing: don’t start with the website or GTSAM. Get config, runtime wiring, measurements, and Teensy I/O solid first. Then fusion has a clean stream of truth to consume.