# GuessWork ↔ Robot Controller UDP Protocol

*Prose copy of the normative wire contract in `src/net/udp_payloads.h` —
change them together. This replaces the retired Teensy CAN bridge
(`docs/can-protocol.md`, deleted with fw=4); robot communication is now
direct UDP between the Mac Mini and the controller over the robot LAN. The
Teensy keeps its two real jobs — camera trigger master and IMU — and no
longer touches robot traffic.*

## 1. Overview

```
RoboRIO / SystemCore                          Mac Mini (guesswork)
  ChassisSpeeds @ 50–100 Hz  ──UDP──►  :5809  RobotLink ──► OdomBus ──► fusion
  pose consumer      :5810  ◄──UDP──          RobotLink ◄── fusion output @ output_hz
```

- **One datagram = one message.** Both packets are ≤ 64 bytes — no
  fragmentation, no application-level framing.
- **No application CRC.** The UDP kernel checksum covers wire corruption on
  the wired robot LAN; misdirected/foreign datagrams are rejected by magic +
  version + exact-length checks.
- **Address learning.** The host learns the controller's IP from the source
  address of the last valid CHASSIS_SPEEDS packet and sends poses back to
  that IP on the robot port. Robot code therefore only needs the Mac's
  static IP; the reverse direction is automatic. A static override
  (`robot_ip` in `PUT /api/robot/config`) exists for bench setups where the
  robot never sends speeds.
- **Ports 5809 (host) / 5810 (robot)** sit in the FRC team-use range
  (5800–5810) that the field firewall leaves open; on-robot switch traffic
  is unrestricted regardless.
- All multi-byte fields **little-endian**.

## 2. CHASSIS_SPEEDS — controller → host (32 bytes)

Robot-frame velocities out of the drivetrain's forward kinematics
(drive-type-agnostic — GuessWork never sees wheel/module math). Send at
50–100 Hz whenever robot code runs, including while disabled (zeros are
fine — the stream is also the clock-sync signal).

| off | size | field        | notes |
|----:|-----:|--------------|-------|
| 0   | 2    | magic        | `0x5747` (`"GW"` on the wire) |
| 2   | 1    | version      | 1 |
| 3   | 1    | type         | 1 |
| 4   | 4    | counter      | rolling, +1 per sample (drop detection) |
| 8   | 8    | rio_time_us  | **full 64-bit** FPGA sample time (`RobotController.getFPGATime()`). No wrap handling anywhere, by construction. |
| 16  | 4    | vx_mps f32   | robot +X (forward) |
| 20  | 4    | vy_mps f32   | robot +Y (left); nonzero only on holonomic drives |
| 24  | 4    | omega_radps f32 | yaw rate, CCW positive |
| 28  | 4    | status_flags | bit0 stale encoder data, bit1 wheel slip; bits 16+ team use |

Sample `rio_time_us` with the same call that samples the encoders, not at
send time — the timestamp is what makes the measurement fusable.

## 3. POSE — host → controller (64 bytes)

Sent at the fusion `output_hz` (default 100 Hz).

| off | size | field        | notes |
|----:|-----:|--------------|-------|
| 0   | 2    | magic        | `0x5747` |
| 2   | 1    | version      | 1 |
| 3   | 1    | type         | 2 |
| 4   | 4    | counter      | rolling. **Staleness rule: counter frozen > 200 ms ⇒ stop trusting the pose.** |
| 8   | 8    | rio_time_us  | pose validity time mapped onto the controller clock; **0 when flags bit0 is clear** |
| 16  | 4    | x_m f32      | WPILib field frame |
| 20  | 4    | y_m f32      | |
| 24  | 4    | theta_rad f32| CCW positive |
| 28  | 1    | quality      | 0–255 fusion confidence |
| 29  | 1    | mode         | 0 uninitialized, 1 nominal, 2 no_vio, 3 no_odom, 4 tags_only, 5 dead_reckoning, 6 collision |
| 30  | 2    | flags        | bit0 clock_sync_healthy (`rio_time_us` valid), bit1 extrapolation clamped |
| 32  | 24   | cov[6] f32   | planar (x, y, θ) marginal, body-tangent: xx yy tt xy xt yt (m², rad², m·rad) |
| 56  | 8    | reserved     | zero; receivers must ignore |

With flags bit0 set, `rio_time_us` lets WPILib pose estimators apply the
measurement with latency compensation
(`addVisionMeasurement(pose, Utils.fpgaToCurrentTime(rio_time_us * 1e-6))`
-style). With it clear, fall back to counter-freshness gating only.

## 4. Time synchronization (two hops, no direct RIO↔Teensy link)

Every GuessWork measurement (frames, IMU, tags) is stamped on the **Teensy
clock** — the trigger master is the time ground truth. Chassis speeds arrive
stamped on the **RIO FPGA clock**. The mapping is chained through the host:

```
RIO µs  ──(hop A: ClockSync fed by UDP recvfrom stamps)──►  host µs
host µs ──(hop B: ClockSync fed by IMU/TRIG USB stamps)──►  Teensy µs
```

Both hops are the same estimator (`src/core/clock_sync.hpp`): per-250 ms
buckets keep `min(arrival − remote)` — the minimum rejects the strictly
one-sided transit jitter — and a least-squares line through 6 s of bucket
minima tracks offset + crystal drift. Each hop's minimum transit (~150 µs
UDP on an idle LAN, ~0.5 ms batched USB telemetry) remains as a constant
bias in its offset; the pose downlink runs the same chain in reverse, so
the bias largely cancels on the wire, and the residual cross-stream skew
(≤ ~1 ms) is far below the usefulness threshold at FRC speeds.

Health and fallbacks:

- Hop A unhealthy (controller just booted, < ~1.3 s of packets): chassis
  speeds are stamped with the **arrival time** mapped through hop B.
- Hop B unhealthy (Teensy telemetry down): speeds are published with
  `t_ns = 0` and fusion drops them — without the Teensy there are no frames
  to fuse against anyway.
- Controller reboot: the backward `rio_time_us` jump resets hop A; it
  re-warms in ~1 s (`resets` increments once in
  `/api/robot/status.clock_sync.rio_host`).
- Both hops are observable at `GET /api/robot/status` → `clock_sync`
  (`healthy` is the AND of the hops).

## 5. Controller-side reference implementation (WPILib Java)

```java
// GuessWorkLink.java — chassis-speeds uplink + fused-pose downlink.
// Mac Mini has a static IP on the robot LAN (e.g. 10.TE.AM.6).
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;

import java.net.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

public class GuessWorkLink {
    public record FusedPose(
        double xMeters, double yMeters, double thetaRad,
        int quality, int mode, boolean clockSynced,
        long rioTimeUs, long receivedFpgaUs, int counter) {}

    private static final short MAGIC = 0x5747;
    private final InetAddress macAddr;
    private final DatagramSocket txSocket;              // speeds -> :5809
    private final DatagramSocket rxSocket;              // poses  <- :5810
    private final AtomicReference<FusedPose> latest = new AtomicReference<>();
    private int txCounter = 0;

    public GuessWorkLink(String macIp) throws Exception {
        macAddr  = InetAddress.getByName(macIp);
        txSocket = new DatagramSocket();
        rxSocket = new DatagramSocket(5810);
        rxSocket.setSoTimeout(250);
        Thread rx = new Thread(this::rxLoop, "guesswork-rx");
        rx.setDaemon(true);
        rx.start();
    }

    /** Call at 50–100 Hz (e.g. from robotPeriodic or a Notifier).
     *  speeds must be ROBOT-relative; sample the encoders and the FPGA
     *  time together. */
    public void sendChassisSpeeds(ChassisSpeeds speeds, long fpgaTimeUs) {
        ByteBuffer b = ByteBuffer.allocate(32).order(ByteOrder.LITTLE_ENDIAN);
        b.putShort(MAGIC).put((byte) 1).put((byte) 1);
        b.putInt(++txCounter);
        b.putLong(fpgaTimeUs);
        b.putFloat((float) speeds.vxMetersPerSecond);
        b.putFloat((float) speeds.vyMetersPerSecond);
        b.putFloat((float) speeds.omegaRadiansPerSecond);
        b.putInt(0);  // status_flags
        try {
            txSocket.send(new DatagramPacket(b.array(), 32, macAddr, 5809));
        } catch (Exception ignored) {}
    }

    /** Latest pose, empty when stale (counter frozen > 200 ms). */
    public Optional<FusedPose> getPose() {
        FusedPose p = latest.get();
        if (p == null) return Optional.empty();
        long ageUs = RobotController.getFPGATime() - p.receivedFpgaUs();
        return ageUs > 200_000 ? Optional.empty() : Optional.of(p);
    }

    private void rxLoop() {
        byte[] buf = new byte[128];
        while (true) {
            try {
                DatagramPacket pkt = new DatagramPacket(buf, buf.length);
                rxSocket.receive(pkt);
                if (pkt.getLength() != 64) continue;
                ByteBuffer b = ByteBuffer.wrap(buf, 0, 64)
                                         .order(ByteOrder.LITTLE_ENDIAN);
                if (b.getShort() != MAGIC) continue;
                if (b.get() != 1 || b.get() != 2) continue;  // version, type
                int  counter = b.getInt();
                long rioUs   = b.getLong();
                float x = b.getFloat(), y = b.getFloat(), th = b.getFloat();
                int quality = b.get() & 0xFF;
                int mode    = b.get() & 0xFF;
                int flags   = b.getShort() & 0xFFFF;
                latest.set(new FusedPose(x, y, th, quality, mode,
                                         (flags & 1) != 0, rioUs,
                                         RobotController.getFPGATime(),
                                         counter));
            } catch (SocketTimeoutException ignored) {
            } catch (Exception ignored) {}
        }
    }
}
```

Usage sketch: construct once with the Mac's IP; call
`sendChassisSpeeds(kinematics.toChassisSpeeds(...), RobotController.getFPGATime())`
every loop; feed `getPose()` into your pose estimator or use it directly.
For SystemCore the same packet layout applies — only the socket plumbing
changes with its SDK.

## 6. Host-side API

- `GET /api/robot/status` — link counters, learned robot address, odom
  rate/last sample, pose TX counters, both clock-sync hops.
- `GET/PUT /api/robot/config` — `enabled`, `bind_port`, `robot_port`,
  `robot_ip` (empty = auto-learn). PUT rebinds the socket and reports
  `restarted`/`restart_error`; the DB row is the source of truth.
- `POST /api/robot/pose {x, y, theta, quality?}` — bench downlink for
  validating the robot-side receiver before fusion is up.

## 7. Bench validation checklist

1. Robot program streaming speeds at 100 Hz → `odom.rate_hz ≈ 100`,
   `counter_gaps ≈ 0`, `robot_addr` shows the controller.
2. `clock_sync.rio_host.healthy: true` within ~2 s; `|drift_ppm| < 100`;
   `offset_us` stable to ±0.5 ms over 10 min.
3. Restart robot code mid-run → `rio_host.resets` +1, healthy again < 2 s,
   odometry `t_ns` stays monotonic.
4. `POST /api/robot/pose` in a loop → robot program sees x/y/θ with an
   advancing counter; `pose.sent` tracks it.
5. Pull the Ethernet cable 5 s and replug → rates recover, no restart
   needed (UDP is connectionless; the learned address survives).
