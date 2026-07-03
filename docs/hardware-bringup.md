# GuessWork — Implementation Status & Hardware Bring-Up Plan

*Last updated: 2026-06-11 (Phases 1–7 complete — the full software roadmap)*

GuessWork is the onboard pose-estimation system for an FRC robot: a Mac Mini M4
runs 6 hardware-synced FLIR Chameleon3 cameras (4 AprilTag + 2 stereo VIO), a
BMI088 IMU, and a Teensy 4.1 that drives the camera triggers (the timestamp
ground truth). The robot controller (RoboRIO today, SystemCore next season)
talks to the Mac **directly over UDP on the robot LAN** — chassis speeds in,
fused pose out (`docs/ethernet-protocol.md`). AprilTag poses, OpenVINS
visual-inertial odometry, and controller chassis speeds fuse in a GTSAM
factor graph (Phase 6).

```
                  Teensy 4.1 (clock master, fw=4)
trigger pulses ──► 6× Chameleon3 ──► FrameChannel per camera
BMI088 @400 Hz ──► binary USB telemetry ──► ImuBus (+host↔Teensy ClockSync)
controller chassis speeds ──UDP :5809──► RobotLink ──► OdomBus (+RIO↔host ClockSync)
                                                  │
4× apriltag cams ──► AprilTagConsumer ──► TagPoseBus ──┤
2× VIO cams ──► StereoSyncPairer ──► OpenVINS ──► VioBus ──► (Phase 6) GTSAM
                                                  │             │
                             controller ◄──UDP :5810── fused pose┘
```

**Single time domain:** every measurement (frames, IMU samples, chassis
speeds) carries a Teensy-clock nanosecond timestamp. Frames are re-stamped
from trigger pulses; the IMU is stamped on the Teensy before transit; chassis
speeds carry the controller's FPGA sample time, mapped RIO→host→Teensy by
the two-hop `ClockSync` chain (arrival fallback while the RIO hop warms up).
No raw host clocks are ever used for estimation math.

---

## 1. What's implemented

### Phase 1 — Teensy timebase, binary telemetry, BMI088 IMU
| Piece | Status |
|---|---|
| 64-bit wrap-extended `micros()` timebase (fixes the ~71.6 min wrap bug); TRIG protocol fw=2 with u64 stamps | ✅ firmware builds; host parse unit-tested |
| Dual USB-CDC (`-DUSB_DUAL_SERIAL`): ASCII commands on `Serial`, framed binary telemetry (CRC16) on `SerialUSB1` | ✅ decoder golden-byte tested (CRC fail, truncation, resync) |
| BMI088 streamer: gyro-DRDY interrupt timestamping, 400 Hz, 4-sample batches, heartbeat with drop counters | ✅ firmware builds; **never run on real hardware** |
| Host: `TelemetryDecoder`, `MeasurementBus<ImuSample>`, dual-port `TeensyManager` (PING-probe classification), `imu_config` table, `/api/imu/*` | ✅ unit-tested + live-server verified (no Teensy attached) |

### Phase 2 — Calibration: multi-topic bags, camera-IMU extrinsics
| Piece | Status |
|---|---|
| Multi-connection `RosbagWriter` + `sensor_msgs/Imu` | ✅ validated against real `rosbag info` + genpy deserialization in the Kalibr container |
| `MultiTopicBagRecorder` (N cams + `/imu0`, Teensy-clock stamps, zero-stamp drop) | ✅ unit-tested with synthetic frames/IMU |
| Extrinsics sessions in `CalibrationSupervisor` (validation gates: hw-sync, Teensy armed, IMU healthy) + `calibrate_imu.sh` (single-cam + stereo-pair Kalibr flows) + `KalibrImuJob` per-camera result fan-out | ✅ code + failure paths live-verified; **end-to-end Kalibr run needs the IMU** |
| Typed `CalibrationStore` (camchain parse/serialize), `cameras.role`, `imu_extrinsics_json` | ✅ unit-tested round-trips |
| Single-camera intrinsics flow (Phase 0 era) | ✅ live-verified end-to-end with the real camera |

### Phase 3 — AprilTag detection pipeline
| Piece | Status |
|---|---|
| `gw_apriltag`: zero-copy tag36h11 detection on the locked IOSurface; SQPNP multi-tag / IPPE_SQUARE single-tag with ambiguity gate; numeric-Jacobian 6×6 covariance (GTSAM body-tangent) | ✅ synthetic-projection tests incl. **500-draw NEES** statistical covariance validation; rendered-real-tag end-to-end test caught (and fixed) the IPPE frame-convention bug |
| `ConsumerFactory` on `CameraSupervisor` (slot-lifecycle-safe attach/detach, rebuild-in-place on role/calibration change) | ✅ unit + live-verified (role flips rebuild consumers live) |
| WPILib field layouts (2026-rebuilt-welded seeded + auto-activated), `T_robot_imu` schema + validation, `/api/field-layouts*`, `/api/apriltag/status` | ✅ live-verified |
| Measured performance | ✅ **12.8 ms latency at 55 det/s, 2048×1536** on the M4 (4 detector threads; vendored detector forced -O2) |
| Range accuracy vs a real printed tag | ⬜ **hardware-pending** (needs a printed tag) |

### Phase 4 — OpenVINS stereo VIO
| Piece | Status |
|---|---|
| OpenVINS ROS-free build on macOS arm64 (`cmake/openvins.cmake`, pinned SHA; Boost 1.90 + cassert fixes) | ✅ spike-built + smoke-ran (VioManager constructed, IMU fed) |
| `StereoSyncPairer` (exact pulse-stamp equality), `VioFeederConsumer`s, single-threaded `OpenVinsRunner`, `epoch` reinit contract, calibration-quality gating, `/api/vio/*` | ✅ unit-tested (pairer rules, config builder goldens, covariance conversion vs hand-computed J·P·Jᵀ); gating progression live-verified |
| Actual VIO tracking on real imagery | ⬜ **hardware-pending** (needs the stereo rig + IMU) |

### Phase 5 — Robot link (controller ↔ Mac over UDP)

*(Reworked 2026-07: the Teensy CAN bridge was retired in favor of direct
UDP — lower complexity, richer payloads, no transceiver hardware. fw=4
drops CAN entirely; the Teensy keeps triggers + IMU.)*

| Piece | Status |
|---|---|
| `RobotLink` (src/net): UDP RX thread, address learning, chassis-speeds decode → OdomBus; POSE downlink (x/y/θ + quality + mode + planar covariance + rolling counter) at `output_hz` | ✅ unit-tested end-to-end over localhost UDP (`test_robot_link`) |
| Wire contract `src/net/udp_payloads.h` (32-byte CHASSIS_SPEEDS, 64-byte POSE, full 64-bit FPGA timestamps — no wrap handling anywhere) | ✅ golden-byte unit-tested (`test_udp_payloads`) |
| Two-hop clock sync `gw::ClockSync` — RIO↔host (fed by UDP arrivals, in RobotLink) and host↔Teensy (fed by IMU/TRIG arrivals, in TeensyManager); chassis speeds mapped RIO→host→Teensy | ✅ unit-tested (fit/drift/reset suite + loopback chain test) |
| `net_config` table, `/api/robot/*` routes (status incl. both sync hops, config incl. rebind, bench pose) | ✅ unit-tested (repo) + route layer |
| Chassis speeds at 100 Hz from a real controller, two-hop sync quality on real crystals, pose downlink visible in robot code, field-network behavior | ⬜ **hardware-pending** (needs a RoboRIO + robot LAN — Stage 6) |

Protocol reference: **`docs/ethernet-protocol.md`** (packet layouts,
time-sync scheme, controller-side WPILib reference class).

### Phase 6 — GTSAM fusion engine
| Piece | Status |
|---|---|
| `gw_fusion`: IncrementalFixedLagSmoother (GTSAM 4.3a1 from source, Boost-free) fusing tag priors (Mahalanobis-gated + Huber), VIO equal-epoch deltas (robot-frame conjugation + Adjoint cov transport, lazily attached to in-lag keys), chassis-speeds twist betweens (Cauchy, slip-robust, soft planarity) | ✅ **hardware-free simulation suite**: figure-8 truth, clean-run RMSE < 5 cm / < 2°, 10 % outlier tags rejected, collision detect+recover, VIO death degradation, epoch-reset safety, tag drought > lag, std-explosion reinit |
| Medoid init, collision monitor (gate-open + noise inflation), auto-reinit (pos-std / solver throw / NaN / unresolved collision), connectivity bridge factors, solve-time p95 tracking | ✅ unit-tested via the same suite |
| `FusionSupervisor` (3 bus drainers → engine thread → output thread), `TeensyNowEstimator`, planar extrapolation → `TeensyManager::send_pose` at `output_hz` | ✅ unit-tested (estimator, extrapolation) + live-server verified (boot gating, config round-trip incl. `restarted` semantics, reset, clean SIGINT joins) |
| Fused pose accuracy on a real field course; solve-time budget under real measurement rates; VIO-kill / collision behavior on hardware | ⬜ **hardware-pending** (Stage 7 — needs the full rig) |

### Phase 7 — Hardening & ops
| Piece | Status |
|---|---|
| Per-stage latency instrumentation in `/api/fusion/status` (`tag_pulse_to_fusion`, `queue_wait`, `solve`, **`pose_staleness`** — the trigger-pulse→pose-on-the-wire headline, target p95 < 50 ms) + degraded-mode `mode` string; teensy_now now also tag-fed (survives odom death) | ✅ unit-tested (`LatencyStats`, `derive_fusion_mode`) + threaded supervisor integration test + live-server verified |
| Degraded-modes matrix (docs/pose_pipeline.md §6) + new engine sim cases (odom death, tags-only) | ✅ simulation-tested; matrix rows mirror the mode unit tests 1:1 |
| Allan-variance IMU refinement, fully API-integrated: `POST /api/imu/allan/recording` → binary log in `~/.guesswork/imu_logs/` → `analyze` (overlapping ADEV, N/K fits, static-ness warnings) → `apply` into imu_config | ✅ math unit-tested (synthetic white noise + random walk recovered ±10–25%); recorder bit-exact-tested; **real overnight BMI088 recording pending (Stage 8 prep)** |
| Config snapshot export/import (`/api/config/export\|import`): cameras incl. calibration blobs, trigger groups, field layouts, all tunables; non-destructive merge with per-section error reporting | ✅ round-trip + conflict-case unit tests + live verified |
| `scripts/soak_check.sh` + threaded `test_fusion_supervisor` (the TSAN target) + `docs/pose_pipeline.md` | ✅ in repo; sanitizer soak itself is Stage 8 |

**Test totals:** 288/288 passing. The software roadmap is complete — what
remains is hardware execution (Stages 0–8) and data-driven tuning. (2026-07
addendum: the CAN bridge was replaced by the UDP robot link — Phase 5 above —
and the audit-driven fixes/test backfill landed; Stage 6 is now an Ethernet
bench.)

---

## 2. What still needs real hardware

Nothing below can be validated in software — each item exercises a physical
interface or a real-world signal path:

1. **BMI088 wiring + firmware flash** — the fw=4 firmware has never run on
   the Teensy; the IMU has never produced a real sample.
2. **IMU rate/health + timestamp soak** — 400 Hz delivery, DRDY jitter, and
   the >75-minute wrap-fix soak.
3. **Trigger-pulse regression under fw=2** — hardware-synced frames must
   still get pulse stamps after the protocol change.
4. **Camera-IMU extrinsics end-to-end** — record a real extrinsics bag, run
   `kalibr_calibrate_imu_camera`, verify the result lands in the DB. The
   reported `timeshift_cam_imu` doubles as a whole-system clock check.
5. **AprilTag range accuracy** — printed 6.5 in tag at tape-measured
   distances; the published field pose against a measured bench layout.
6. **Pipeline resilience** — USB camera yank/replug mid-run for both the
   AprilTag consumer and the VIO feeders.
7. **OpenVINS tracking quality** — initialization, the 5 m loop drift test,
   bump/shake auto-reinit, and the CPU budget.
8. **Multi-camera scaling** — all 6 cameras + IMU at once on one USB
   topology (bandwidth + CPU).
9. **Robot-link bench** — chassis speeds from a real controller at 100 Hz
   over UDP, two-hop clock-sync health/drift on real crystals, pose
   downlink visible in robot code, cable-pull recovery.
10. **Fused pose on a real course** — waypoint accuracy against a tape
    measure, solve-time budget at real measurement rates, VIO-kill
    degradation and physical collision/jostle behavior.

### Equipment checklist

Full wiring reference (every pin, with electrical notes): **`docs/teensy-pinout.md`**.

- [ ] Teensy 4.1 on USB, outputs 1–6 (pins 2–7) wired to camera opto-isolated trigger inputs (Line0)
- [ ] BMI088 breakout: SPI0 (MOSI=11, MISO=12, SCK=13), **CS accel=10, CS gyro=9, gyro INT3→pin 8**, 3V3 + GND
- [ ] At least 2 (ideally all 6) Chameleon3 cameras + lenses; the stereo pair rigidly mounted at the chosen baseline with the BMI088 hard-mounted next to the left camera
- [ ] Printed 36h11 AprilTag, **black square exactly 6.5 in / 165.1 mm — verify with a ruler, printer scaling is the classic 2 % error**
- [ ] The Kalibr AprilGrid target (printed from `data/kalibr/aprilgrid_6x6.yaml` geometry, tags 88 mm), rigid backing
- [ ] Tape measure, masking tape, good even lighting
- [ ] Docker/Colima working (`docker/kalibr/build.sh` image already built)
- [ ] **Robot-link bench (Stage 6):** the Mac and a RoboRIO on one Ethernet
      switch (or the robot radio's wired ports), Mac on a static IP
      (10.TE.AM.x convention); a RoboRIO + power + driver-station laptop
      running the `GuessWorkLink` reference class from
      `docs/ethernet-protocol.md` §5 — no transceiver hardware, no
      termination, no extra wiring

---

## 3. Hardware test plan

Run the stages in order — each builds on the previous one. Start the server
with `./build/guesswork` (default port 8080; substitute below).

### Stage 0 — Flash & smoke (≈ 30 min)

1. Wire the BMI088 per the checklist. Flash the firmware:
   `cd firmware && pio run -e teensy41 -t upload`.
2. Plug the Teensy in. The Mac should enumerate **two** `cu.usbmodem*`
   interfaces (dual-CDC).
3. `GET /api/imu/status` → expect `teensy_connected: true`,
   `telemetry_connected: true`, `fw_version: 4`, `imu_ok: true`,
   `rate_hz ≈ 400`, `crc_errors: 0`.
   - `imu_ok: false` ⇒ wiring/CS-pin problem (heartbeat distinguishes "no
     IMU" from "no Teensy").
4. Shake the board; sanity-check accel/gyro magnitudes move (watch
   `last_sample_age_ms` stay near 0).

**Pass:** 400 ±5 Hz sustained, zero CRC errors over 5 minutes.

### Stage 1 — Trigger regression + clock soak (1.5 h, mostly unattended)

1. Create a trigger group (e.g. 30 fps, the pins your cameras are wired to)
   via `POST /api/hardware-sync/groups`, then `POST /api/hardware-sync/arm`.
2. Set each camera `hardware_sync_enabled: true` + its `trigger_output_pin`.
3. `GET /api/status` → camera `fps_1s` matches the group fps (trigger-driven,
   not freerun) — confirms fw=2 TRIG parsing + pulse re-stamping.
4. **Soak:** leave everything running > 75 minutes (past the old `micros()`
   wrap), then confirm frames and IMU are still flowing and an extrinsics
   recording started after the soak doesn't error on non-monotonic stamps.

**Pass:** synced fps correct; no stamp anomalies after 75+ min.

### Stage 2 — Calibration (one camera first) (≈ 1 h)

1. **Intrinsics** (per camera): `POST /api/cameras/<id>/calibration/recording`,
   wave the AprilGrid 30–60 s covering the full field of view, `DELETE` to
   stop — the Kalibr job auto-runs (watch the SSE log at
   `…/calibration/job/log`). Target reprojection error < 0.3 px.
2. **Camera-IMU extrinsics** (the Phase 2 marquee test):
   - Prereqs auto-checked by the API: hw-sync on, Teensy armed, IMU healthy,
     stored intrinsics matching the live mode.
   - `POST /api/calibration/extrinsics/recording {"camera_ids": [<id>]}`.
   - **Motion recipe (Kalibr needs excitation):** grid fixed and well-lit,
     exposure ≤ 2 ms (no motion blur); 60–90 s; pause ~2 s still at start and
     end; excite **all 6 DOF** — 3× rotation swings (~30–45°) about each
     axis, 3× translations along each axis, then combined figure-eights;
     keep the grid mostly in frame; no impacts.
   - `DELETE …/recording` → job runs both Kalibr steps; result stored per
     camera in `imu_extrinsics_json`.
3. **Acceptance:** in the stored YAML (`GET /api/cameras/<id>/extrinsics`):
   - `timeshift_cam_imu` **≪ 1 ms** — this single number validates the whole
     shared-Teensy-clock design end-to-end. If it's large, the pulse
     re-stamping path is broken: stop and debug before anything else.
   - `T_cam_imu` translation matches the tape measure to ~1 cm.
   - `guesswork_meta.reprojection_error_std_px < 1.0` (the VIO gate).

### Stage 3 — AprilTag bench (≈ 1 h)

1. Camera with stored intrinsics, `role: "apriltag"`.
2. **Range:** printed tag at tape-measured 1 m / 2 m / 4 m (camera aperture →
   tag center). `GET /api/apriltag/status` → `last_tags[].range_m` within
   **1–2 %** at each distance; `last_latency_ms < 15`; `det_per_s` ≈ camera fps.
3. **Field pose:** set a bench `T_robot_imu` (identity is fine:
   `PUT /api/imu/config {"t_imu_robot": {"T_robot_imu": [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]}}`),
   make sure the camera has IMU extrinsics from Stage 2, and `POST
   /api/field-layouts` a single-tag bench layout placing the tag at a
   measured pose. Published `T_field_robot` (in status `last_pose`) must
   match the tape-measured camera placement within **±3 cm / ±2°**.
4. **Multi-tag:** two tags on a wall at measured spacing → reprojection
   < 1 px and visibly tighter covariance than single-tag.
5. **Resilience:** yank the camera USB mid-run, replug → status shows
   offline/online transitions, detection resumes, **no crash** (this is the
   ConsumerFactory lifecycle acceptance test).

### Stage 4 — Stereo VIO rig (≈ half a day)

Prereq: both VIO cameras rigidly mounted, BMI088 hard-mounted by the left
camera, both calibrated through Stage 2 (the **pair** flow:
`POST /api/calibration/extrinsics/recording {"camera_ids": [<left>, <right>]}`
— it solves cam-cam + cam-IMU in one session).

1. Roles `vio_left` / `vio_right`; both in **one trigger group @ 30 fps**.
2. `GET /api/vio/status` must show `reason: "ok"`, `running: true` — if not,
   the reason string says exactly which gate failed (missing role, missing
   extrinsics, calibration quality above `max_reproj_std_px`).
3. **Initialization:** pick the rig up, move gently — `initialized: true`
   within ~2 s. If it never initializes, lower
   `init_imu_thresh` territory… first check `counters.paired` is climbing and
   `dropped_unmatched ≈ 0` (pairing health), and `imu_rate_hz ≈ 400`.
4. **5 m loop drift:** tape a start line; walk the rig 5 m out and back to
   the exact start; compare `last_pose.T_odom_imu` translation at return vs
   start. **Pass: < 5 cm (< 1 %).**
5. **Bump/shake reinit:** strike the rig hard. Expect divergence detection
   (`reinits` increments, `epoch` bumps) and recovery to `tracking` within
   ~5 s. The epoch contract means downstream fusion never sees a pose jump.
6. **CPU budget:** `top` — the guesswork process should stay ≤ ~1.5 cores
   for VIO (levers if over: `downsample` is already on; drop `num_pts` to
   100 via `PUT /api/vio/config`).
7. **Resilience:** unplug one VIO camera → feeder drops out of status, the
   runner idles; replug → the >2 s frame-gap triggers a clean reinit.

### Stage 5 — Full-system soak (when all 6 cameras exist)

1. All 6 cameras + IMU, two trigger groups (apriltag @ 30 fps, VIO @ 30 fps),
   streaming + AprilTag + VIO running simultaneously.
2. Watch for USB bandwidth saturation (`frames_incomplete` in `/api/status`)
   — Chameleon3s at full res are ~95 MB/s each; spread across separate USB3
   controllers/hubs if drops appear.
3. 3-hour soak; ideally repeat once under the TSAN preset
   (`build-tsan/`, reduced rates) before competition use.

### Stage 6 — Robot-link bench with a RoboRIO (≈ 2 h; independent of Stages 2–5)

No wiring beyond Ethernet: Mac (static IP) and RoboRIO on one switch.
Controller-side test program: the `GuessWorkLink` reference class in
`docs/ethernet-protocol.md` §5, sending ChassisSpeeds at 100 Hz with
`RobotController.getFPGATime()` and logging received poses.

1. `GET /api/robot/status` → `running: true`, `bind_port: 5809`.
2. Start the RIO program. **Pass:** `odom.rate_hz ≈ 100`,
   `odom.counter_gaps ≈ 0`, `odom.rejected: 0`, `robot_addr` shows the RIO,
   `odom.last` carries the commanded speeds.
3. **Clock sync (both hops):**
   `clock_sync.rio_host.healthy: true` within ~2 s of RIO packets;
   `clock_sync.host_teensy.healthy: true` within ~2 s of the Teensy
   telemetry (IMU or armed triggers); `|drift_ppm| < 100` on both;
   `offset_us` stable to ±0.5 ms over 10 min. Record both hops' numbers —
   they are the headline metrics for this stage.
4. **Controller reboot:** restart the RIO code mid-run →
   `rio_host.resets` increments once, healthy again < 2 s, no stale
   mappings (odometry `t_ns` stays monotonic).
5. **Pose downlink:** `POST /api/robot/pose {"x":1.0,"y":2.0,"theta":0.5}`
   in a loop → RIO program sees x/y/theta with an advancing counter and the
   degraded-mode byte; `pose.sent` tracks it. With `rio_host` healthy the
   packets carry a mapped `rio_time_us` (flags bit0 set) — log
   `getFPGATime() − rio_time_us` on the RIO: it should sit at a stable few
   ms (transport + output quantization), not jump around.
6. **Cable pull:** yank the Ethernet cable 5 s, replug → rates recover on
   their own, `rio_host.resets` unchanged or +1, no restarts needed.
7. With the IMU also wired: confirm `imu.rate_hz ≈ 400` is unaffected at
   100 Hz odom (separate paths now — USB vs UDP — so this should be trivially
   true; it pins the regression anyway).

**Pass:** steps 2–6 green; the headline numbers to record are both hops'
`drift_ppm` and offset stability bands, and the RIO-side
`getFPGATime() − rio_time_us` latency figure.

### Stage 7 — Fusion field course (after Stages 3, 4, and 6 pass)

Requires: ≥2 calibrated AprilTag cameras + the stereo VIO rig + IMU + the
robot-link bench (chassis speeds flowing). Tape a small course (3×3 m is enough)
with 4–6 printed tags at surveyed positions entered as a custom field layout;
mark 5+ waypoints with tape-measured field coordinates.

1. All sources up: `GET /api/fusion/status` → `initialized: true` within a
   second of tags being visible, `sources.*` rates live, `solve_ms.p95` well
   under `min_state_dt_ms` (25 ms).
2. **Waypoint accuracy:** park the robot on each waypoint → fused
   `pose.x_m/y_m` within **±3 cm** and heading within **±2°** of the tape
   measurements. Watch `quality` sit high (> 200).
3. **Motion:** push the robot around the course at walking pace —
   `tag.rejected_gate` stays near zero, no `reinits`, pose tracks visibly in
   the status output.
4. **VIO kill:** cover the stereo cameras (or `PUT /api/vio/config
   {"enabled":false}`) mid-run → fusion continues on tags + chassis speeds;
   `sources.vio.rate_hz` drops to 0, no reinit, accuracy degrades but stays
   bounded.
5. **Collision:** physically jolt/slide the robot (wheels not rolling) →
   `collision_mode: true` within ~1 s, pose snaps to the tag solution within
   2 s, collision mode clears, `reinits` unchanged.
6. **Tag blackout:** cover all tags > 5 s → `quality` decays, no exception;
   uncover → recovery within a second.
7. **Downlink:** confirm the RIO test program sees the fused pose at
   `output_hz` with an advancing counter and a sensible mode byte
   (`output.sent` tracking `pose.sent` in /api/robot/status).

**Pass:** waypoints ±3 cm/±2°, `solve_ms.p95` < 25 ms,
`latency.pose_staleness.p95_ms` < 50 (the trigger-pulse→pose-on-the-wire
headline), all degradation scenarios recover without manual intervention.
The covariance/sigma tuning loop starts from whatever this stage measures.

### Stage 8 — 3-hour sanitizer soak + Allan refinement (overnight + 1 day)

**Prep (overnight before the soak):** with the robot powered and perfectly
still (IMU rigid, nobody touching the cart):
`POST /api/imu/allan/recording {"duration_s": 28800}` (8 h). In the morning:
`POST /api/imu/allan/analyze` → review per-axis fits + warnings (motion
heuristics, fit-quality flags) → `POST /api/imu/allan/apply` to replace the
datasheet noise values in imu_config. Less than 3 h of data earns an
explicit "random-walk fit unreliable" warning — don't apply those.

**Sanitizer builds** (one-time per dir; the first configure builds GTSAM +
OpenVINS again inside each dir — budget 30–60 min each):

```bash
cmake -S . -B build-tsan -DCMAKE_BUILD_TYPE=Debug -DGW_ENABLE_TSAN=ON && cmake --build build-tsan -j
cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=Debug -DGW_ENABLE_ASAN=ON && cmake --build build-asan -j
```

Note: the GTSAM/OpenVINS ExternalProjects are **not instrumented** (they
configure independently of our sanitizer flags). Acceptable by design — the
fusion engine thread is GTSAM's only user and OpenVINS is fed
single-threaded; our own threading is fully instrumented. The headline
software check runs without hardware:
`TSAN_OPTIONS=halt_on_error=1 build-tsan/gw_tests --gtest_filter='FusionSupervisor*'`.

**Soak runs** (all cameras + IMU attached, triggers armed, RIO test
program streaming over UDP):

```bash
TSAN_OPTIONS=halt_on_error=1:second_deadlock_stack=1 ./build-tsan/guesswork &
scripts/soak_check.sh --duration-s 10800
# then again with:
MallocNanoZone=0 ASAN_OPTIONS=halt_on_error=1:abort_on_error=1:detect_leaks=0 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 ./build-asan/guesswork &
scripts/soak_check.sh --duration-s 10800
```

(Reduce camera rates under TSAN if frame drops appear — the 5–15× slowdown
is expected; the soak is hunting races and leaks, not throughput.)

**Pass:** zero sanitizer reports across both 3-hour runs; `soak_check.sh`
prints PASS (flat crc/exception/queue-drop counters, reinits ≤ 3,
`pose_staleness` p95 < 50 ms, RSS growth < 20% from the 5-minute baseline).

### Record as you go

Append results (dates, measured numbers, any tuning changes like
`init_imu_thresh`) to this file — Stage 2's `timeshift_cam_imu`, Stage 4's
drift number, Stage 6's two-hop clock-sync stability, and Stage 7's waypoint error
are the headline metrics worth tracking over time.

---

## 4. What comes after hardware sign-off

The software roadmap (Phases 1–7) is complete. What remains is data-driven:

- **Fusion covariance/sigma tuning** from Stage 7 waypoint numbers — every
  knob is live-tunable via `PUT /api/fusion/config`.
- **Allan-refined IMU noise** (Stage 8 prep) feeding both Kalibr and VIO.
- **Config snapshot discipline:** after calibration + tuning, download
  `GET /api/config/export` and commit/back it up — it restores the robot's
  full identity (calibrations included) onto a fresh install or spare Mac.
- **Future season:** the IMU-preintegration fallback for VIO-unhealthy via
  the reserved `feed_imu` seam (`src/fusion/fusion_engine.hpp`).

---

## 5. Bench results log

### 2026-07-03 — Stage 0 + Stage 1 (first hardware contact)

Setup: 1× CM3-U3-31S4M (serial 17301963, 6 mm lens), Teensy 4.1 + BMI088,
fw=4, M4 Mac. Trigger wire: output 1 (pin 2), low-side (see below).

- **Stage 0 PASS** — fw=4 flashed via PlatformIO (`teensy.app` loader; the
  CLI loader can't open the HID device on macOS). Dual CDC enumerates.
  `imu_ok: true`, **399.7 Hz sustained, 0 CRC errors** — first real BMI088
  samples. Host↔Teensy clock sync healthy, drift fit **≈1–2 ppm**.
  Spinnaker runtime needs Homebrew `libusb` (added to install.sh).
- **Stage 1 PASS (fps-lock)** — camera slaved to the 30 fps trigger group:
  **30.02–30.04 fps** (freerun is ~55), 0 dropped / 0 incomplete. The
  75-minute stamp soak is still to run.
- **Finding 1 (wiring, fixed):** the CM3 opto input does NOT register a
  3.3 V high-side drive. Working scheme: OPTO_IN (pin 9, yellow) → Teensy
  VIN 5 V; OPTO_GND (pin 7, brown) → trigger pin (low-side switch);
  producer uses TriggerActivation=FallingEdge. docs/teensy-pinout.md
  §1 rewritten; earlier revisions also had OPTO_IN/OPTO_GND wire colors
  swapped. New firmware `TEST_PIN pin=<1..6> level=<0|1>` command holds an
  output steady for multimeter / LineStatusAll verification.
- **Finding 2 (clock sync, fixed):** feeding the host↔Teensy ClockSync from
  both IMU (telemetry CDC, up to ~15 ms batch lag) and TRIG (command CDC,
  ~1 ms) interleaved non-monotonic stamps and reset-thrashed the
  backward-jump detector (~90 resets/s). TRIG is now a fallback feed only
  while the IMU stream is stale >500 ms (teensy_manager.cpp).
- **Finding 3 (RESOLVED — root-caused + 3 fix layers):** intermittent
  "Teensy command timeout" (~7% of commands on a raw-port harness, worst
  right after STOP). Root cause: short responses ("OK\r\n") stuck in the
  Teensy USB-CDC partial-packet buffer when the core's flush timer loses
  its race — the ack only ships when the NEXT write generates traffic.
  Fixes: (1) firmware calls `Serial.send_now()` after every command
  response and TRIG flush batch (harness: 0/160 timeouts after, worst ack
  2.6 ms); (2) host `send_command` retries once (all protocol commands are
  idempotent); (3) **armed state is now persisted operator intent**
  (`sync_config` table, additive) — the arm/stop routes store it before
  pushing, main.cpp seeds the desired config at boot, and TeensyManager
  retries failed pushes every 3 s until the device converges. Verified: 5×
  server cold-starts with ZERO API calls each converged to armed + 30 fps.
  Field behavior: power the robot on and walk away.
- **Operational note:** auto-exposure in a dim room drives exposure toward
  ~90 ms and the camera silently retires only ~6 of the 30 triggers/s. Set
  manual exposure (bench: 8 ms → instant 30 fps lock); calibration wants
  ≤2 ms anyway.
