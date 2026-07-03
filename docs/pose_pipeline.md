# GuessWork pose pipeline — frames, clocks, conventions, degraded modes

Every frame, clock domain and covariance convention in one place. Frame
mix-ups are the most common bug source in systems like this; when this
document and a code comment disagree, the **normative code comments** (listed
at the bottom) win — then fix this file.

---

## 1. Clock domains

| Clock | Source | Who maps it |
|---|---|---|
| **Teensy clock** (the system time base) | wrap-extended 64-bit `micros()` on the Teensy (`firmware/src/time64.h`), carried as nanoseconds (`µs × 1000`) | Nobody — everything else maps INTO it |
| RIO FPGA clock | controller `RobotController.getFPGATime()` µs (full 64-bit — no wrap anywhere), carried in UDP chassis-speeds packets | hop A: `gw::ClockSync` inside `RobotLink`, fed by `recvfrom` host stamps (bucketed-min + drift fit; resets on controller reboot) |
| Host monotonic clock (`gw::Clock`) | `mach_absolute_time`, arrival stamps + rate windows only | hop B: `gw::ClockSync` inside `TeensyManager`, fed by IMU/TRIG telemetry arrival stamps — maps host↔Teensy |

Rules:
- **Estimation math never uses raw host clocks.** Frames are re-stamped from
  trigger pulses (`Frame::camera_ts_ns`), IMU samples are stamped on the
  Teensy before transit, and chassis speeds are mapped
  RIO → host → Teensy through the two health-gated `ClockSync` hops
  (`src/core/clock_sync.hpp`, `docs/ethernet-protocol.md` §4). Hop A
  unhealthy → arrival-stamp fallback (mapped through hop B); hop B
  unhealthy → `t_ns = 0` and fusion drops the sample.
- `TeensyNowEstimator` is fed by the fusion supervisor's **odom drainer**
  (lowest latency) and, since Phase 7, also by the **tag drainer** so it
  survives odom death. The tag feed's ~15–40 ms detect latency biases the
  estimate *early*, which only shortens output extrapolation.
- Tag measurements with `clock_source == kHost` (no pulse matched) are
  rejected by fusion (`tag_rejected_clock`).

## 2. Coordinate frames

| Frame | Definition |
|---|---|
| **field** | WPILib NWU, blue-alliance origin: +X downfield, +Y left, +Z up. Layout JSON = verbatim WPILib `AprilTagFieldLayout`. |
| **tag** | WPILib convention: +X out of the tag face, +Z up. Corners (size s): BL `(0,−s/2,−s/2)`, BR `(0,+s/2,−s/2)`, TR `(0,+s/2,+s/2)`, TL `(0,−s/2,+s/2)`. FRC 36h11 black square = 6.5 in = 0.1651 m. |
| **camera** | OpenCV: +X right, +Y down, +Z forward (into the scene). |
| ⚠ IPPE_SQUARE object frame | **OpenCV's `IPPE_SQUARE` object frame is IMAGE-ALIGNED (+X right, +Y down, +Z into the scene) — NOT the WPILib tag frame.** `det->p[0..3]` (BL,BR,TR,TL in image space) pairs DIRECTLY with object points `(−h,+h),(+h,+h),(+h,−h),(−h,−h)`; the bridge rotation is `R_tagcv_tag = [0 1 0; 0 0 −1; −1 0 0]`. Empirically verified — self-consistent synthetic tests cannot catch this class of bug; the rendered-real-tag test does. Normative: `src/apriltag/tag_pose_estimator.cpp` top comment block. |
| **IMU** | BMI088 body frame, as calibrated by Kalibr. Mounted on the left VIO camera. |
| **robot** | Chassis frame: +X forward, +Y left, +Z up; origin = chassis center (the frame WPILib ChassisSpeeds and the fused pose use). |
| **VIO odom** | OpenVINS's gravity-aligned global frame for the CURRENT epoch. Origin resets on every VIO (re)init and `epoch` increments — **never difference VIO poses across epochs, never fuse an absolute VIO pose** (`src/vio/vio_types.hpp`). |

## 3. Transform chains

- **Tag → robot pose** (Phase 3, `src/apriltag/tag_pose_estimator.cpp`):
  `T_field_robot = inv(T_cam_field) · T_cam_imu · inv(T_robot_imu)`
- **`T_robot_imu`** comes from `imu_config.t_imu_robot_json`
  (`{"T_robot_imu": [[4×4 row-major]]}`, CAD-derived, validated by
  `gw::calib::parse_t_robot_imu`). Maps IMU-frame points into the robot
  frame. Unset ⇒ AprilTag publishing gates on `no_extrinsics_chain` and
  fusion disables VIO ingestion (tags + chassis speeds still run).
- **Kalibr → OpenVINS**: stored `T_cam_imu` is Kalibr's `T_ItoC` (IMU →
  camera). OpenVINS wants `q_ItoC` + `p_IinC` — **mapped with NO inversion**
  (`src/vio/vio_config_builder.cpp`). `timeshift_cam_imu ≡ calib_camimu_dt`.
- **VIO delta → robot frame** (fusion): per-sample equal-epoch delta
  `δ = inv(T_odom_imu[k−1]) · T_odom_imu[k]` conjugated as
  `Δ_robot = T_robot_imu · δ · inv(T_robot_imu)`, covariance transported with
  the SE(3) Adjoint (`gw::apriltag::adjoint_se3`).

## 4. Covariance / tangent convention

One convention everywhere (TagPoseBus, VioBus, fusion, GTSAM):

- 6×6 row-major, tangent order **[ωx ωy ωz, tx ty tz]** — rotation first.
- **RIGHT (body-frame) perturbation: `T_true = T_est · Exp(ξ)`** — exactly
  GTSAM Pose3's retract (GTSAM 4.3a1 builds with `GTSAM_POSE3_EXPMAP=ON`
  default), so measurements flow into the factor graph **unchanged**.
- Normative: `src/apriltag/pose_math.hpp` header comment.

## 5. Latency stages (`/api/fusion/status` → `latency`)

Target: **trigger pulse → pose on the wire p95 < 50 ms** (`pose_staleness`).

| Stage | Measures | Path covered |
|---|---|---|
| `tag_pulse_to_fusion` | `teensy_now(arrival) − tag.t_ns` at the tag drainer | exposure + USB + detect + estimate + bus |
| `queue_wait` | drainer push → engine pop (host steady) | internal queue dwell |
| `solve` | one `IncrementalFixedLagSmoother::update()` wall time (last/p95 over 256) | iSAM2 solve |
| `pose_staleness` | `teensy_now − newest state t_ns` at each UDP pose send, pre-clamp (signed; slightly negative possible with the tag-biased estimator) | **the headline end-to-end number** — extrapolation covers exactly this gap |

Per-camera capture→publish latency (last + EWMA) also lives in
`/api/apriltag/status`.

## 6. Degraded-modes matrix

`mode` in `/api/fusion/status` (pure `derive_fusion_mode`,
`src/server/fusion_mode.hpp`; freshness = source seen < 500 ms ago; the unit
tests in `tests/test_fusion_mode.cpp` mirror these rows 1:1).

| Failure | `mode` | What still flows | Quality byte | How the RIO detects it |
|---|---|---|---|---|
| Nothing (all sources fresh) | `nominal` | everything | high (≈255·(1−σ/1 m)) | n/a |
| VIO dead/diverged/disabled (T_robot_imu unset) | `no_vio` | tags + chassis speeds | barely changes (tags dominate) | none needed — pose stays valid |
| UDP odom dead | `no_odom` | tags + VIO; states bridge on constant-velocity factors; teensy_now stays alive via the tag feed | barely changes | chassis-speeds path is the RIO's own — it already knows |
| VIO **and** odom dead | `tags_only` | tag priors + bridge factors | mild degradation | pose valid; counter advances |
| All tags stale (occlusion/blackout) | `dead_reckoning` | VIO+odom betweens; covariance grows until tags return or the reinit threshold trips | decays toward 1 | quality byte decays |
| Sustained tag-vs-dead-reckoning disagreement (collision/teleport) | `collision` | gate opens — tags are the truth source; VIO/odom inflated ×10 | halved | quality byte halves; pose snaps to tags within ~2 s |
| Not yet initialized (no 5-tag burst since boot/reinit) | `uninitialized` | nothing published — **deliberate**: with no field reference a pose would be fiction | 0 (not sent) | **pose counter freezes** (staleness rule: counter unchanged > 200 ms) |
| teensy_now not warmed (no odom AND no tags yet) | (any) | publishing skipped — **deliberate**: extrapolating against an unknown clock offset would mis-stamp poses | — | counter freezes |
| Teensy unplugged | (any) | no frames/IMU and no host↔Teensy clock hop; odom arrives but is dropped (t_ns=0); poses stop | — | counter freezes |
| Camera death (USB yank) | per-source rates drop; consumers detach with the slot and reattach on replug | remaining cameras | depends on remaining tag coverage | n/a |

## 7. Normative code comments (the source of truth)

- `src/apriltag/tag_pose_estimator.cpp` — ALL camera/tag frame conventions
  incl. the IPPE quirk and the corner tables.
- `src/apriltag/pose_math.hpp` — tangent ordering + right perturbation.
- `src/vio/vio_types.hpp` — the VIO epoch contract.
- `src/vio/vio_config_builder.{hpp,cpp}` — Kalibr→OpenVINS mapping +
  `cov_ov_to_body_tangent`.
- `src/core/odom_types.hpp` — chassis-speeds time domain semantics.
- `src/core/clock_sync.hpp` — the two-hop clock mapping + reset rules.
- `src/fusion/fusion_engine.cpp` — fusion design notes (lazy VIO betweens,
  why VIO delta noise comes from config sigmas, the in-lag trim margin).
- `docs/ethernet-protocol.md` ↔ `src/net/udp_payloads.h` — the robot wire
  contract.
