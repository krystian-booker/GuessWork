# Teensy 4.1 — pinout & component wiring

Every wire on the Teensy 4.1 bridge board: camera trigger outputs, the BMI088
IMU, and the TJA1051T/3 CAN transceiver. Pin numbers are Teensy 4.1
Arduino-style digital pins. The firmware constants are the source of truth —
each section cites them; change them together.

```
                          ┌──────── USB (dual CDC) ────────► Mac Mini
                          │  Serial      = ASCII commands / TRIG events
                          │  SerialUSB1  = binary telemetry (IMU/ODOM ▲, POSE ▼)
            ┌─────────────┴─────────────┐
   GND ─────┤ GND                   VIN ├───── (5 V in, or powered via USB)
            │ 0                     GND │
            │ 1                    3.3V ├──┬── BMI088 VDD/VDDIO
   cam 1 ◄──┤ 2                      23 │  └── TJA1051T/3 VIO
   cam 2 ◄──┤ 3                      22 │
   cam 3 ◄──┤ 4                      21 │
   cam 4 ◄──┤ 5                      20 │
   cam 5 ◄──┤ 6                      19 │
   cam 6 ◄──┤ 7                      18 │
  BMI088 ───┤ 8  (gyro INT3/DRDY)    17 │
  BMI088 ───┤ 9  (gyro CS)           16 │
  BMI088 ───┤ 10 (accel CS)          15 │
  BMI088 ───┤ 11 (SPI0 MOSI)         14 │
  BMI088 ───┤ 12 (SPI0 MISO)         13 ├───── BMI088 SCK (SPI0)
            └────┬────────────┬──────────┘
                 │ 30 (CRX3)  │ 31 (CTX3)        (inner/end-row pins)
                 │            │
                 ▼            ▼
              TJA1051T/3  TJA1051T/3
                 RXD          TXD     ──► CANH/CANL ──► robot CAN bus
```

---

## 1. Camera trigger outputs (→ FLIR Chameleon3 opto-isolated Line0)

Firmware: `firmware/src/trigger_engine.h` — `kOutputPins = {2,3,4,5,6,7}`.
The host API and wire labels use **output numbers 1–6**; the physical pin is
always `output + 1`.

| Output # (host label) | Teensy pin | Goes to |
|---|---|---|
| 1 | **2** | camera 1 trigger input (Line0 / OPTO_IN, brown wire on the FLIR GPIO pigtail) |
| 2 | **3** | camera 2 trigger input |
| 3 | **4** | camera 3 trigger input |
| 4 | **5** | camera 4 trigger input |
| 5 | **6** | camera 5 trigger input |
| 6 | **7** | camera 6 trigger input |
| — | GND | common ground to every camera's OPTO_GND |

Notes:
- 100 µs active-high pulses (`kPulseWidthUs`), driven push-pull at 3.3 V.
  The Chameleon3 opto input threshold is comfortably below 3.3 V; no level
  shifting needed, but each camera's opto ground must be tied to Teensy GND.
- `cameras.trigger_output_pin` in the DB stores the **output number (1–6)**,
  not the physical pin.

## 2. BMI088 IMU (SPI0)

Firmware: `firmware/src/bmi088_imu.h` — `kImuAccelCsPin=10`,
`kImuGyroCsPin=9`, `kImuGyroDrdyPin=8`; SPI0 default pins.

| Teensy pin | Signal | BMI088 breakout pin |
|---|---|---|
| **11** | SPI0 MOSI | SDI (accel + gyro shared) |
| **12** | SPI0 MISO | SDO (accel + gyro shared) |
| **13** | SPI0 SCK | SCK (accel + gyro shared) |
| **10** | accel chip select | CSB1 (accel) |
| **9** | gyro chip select | CSB2 (gyro) |
| **8** | gyro data-ready interrupt | INT3 (gyro, push-pull active-high) |
| 3.3V | power | VDD + VDDIO |
| GND | ground | GND |

Notes:
- One SPI bus, two chip selects — the BMI088 is two dies in one package.
- The **gyro's INT3** paces sampling (400 Hz): the ISR stamps `now_us64()`
  and the main loop reads both dies. Only INT3 is wired; the accel INT pins
  are left unconnected.
- The BMI088 is 3.3 V-only — never feed it 5 V.
- Mechanically: hard-mounted next to the **left VIO camera** (the camera-IMU
  extrinsics calibration assumes a rigid mount).

## 3. TJA1051T/3 CAN transceiver (CAN3)

Firmware: `firmware/src/can_bridge.h` — CAN3 is the **only FD-capable
FlexCAN controller on the Teensy 4.x**, which is why pins 30/31 are used
even though CAN1/CAN2 pins are free. Works in both bridge modes (classic
RoboRIO @ 1 Mbps and SystemCore CAN FD 1 M/4 M) on the same two pins.

| Teensy pin | Signal | TJA1051T/3 pin |
|---|---|---|
| **31** | CTX3 (CAN3 transmit) | 1 — TXD |
| **30** | CRX3 (CAN3 receive) | 4 — RXD |
| 3.3V | logic-level reference | 5 — VIO |
| 5V (VIN/VUSB) | transceiver supply | 3 — VCC |
| GND | ground | 2 — GND |
| GND | normal mode (not silent) | 8 — S |
| — | CAN bus high | 7 — CANH → bus |
| — | CAN bus low | 6 — CANL → bus |

Notes:
- The **/3 variant matters**: its VIO pin lets TXD/RXD run at the Teensy's
  3.3 V logic while VCC takes the 5 V the bus driver needs. A plain TJA1051T
  (no VIO) would drive RXD at 5 V into the Teensy — don't substitute.
- **Tie S (pin 8) to GND.** Floating/high puts the transceiver in silent
  (listen-only) mode and TX silently does nothing.
- **Termination:** 120 Ω across CANH/CANL at *both physical ends* of the bus
  (the RoboRIO end usually has its own; add one at this end if the Teensy is
  a bus endpoint). Twisted pair for CANH/CANL.
- Rated to 5 Mbps FD data phase — the SystemCore profile (4 Mbps) is within
  spec; drop `baudrateFD` in `can_bridge.cpp` to 2 Mbps if the bench shows
  errors.

## 4. Power & USB

| Connection | Notes |
|---|---|
| USB (micro-B) → Mac | Dual CDC (`-DUSB_DUAL_SERIAL`): `Serial` = ASCII command port, `SerialUSB1` = binary telemetry. Also powers the Teensy on the bench. |
| VIN | 5 V robot supply for competition use (USB data still connected to the Mac). If feeding VIN while USB is plugged, follow PJRC's guidance: cut the VUSB↔VIN pad or diode-isolate. |
| 3.3V rail budget | BMI088 (~6 mA) + TJA1051 VIO reference (µA) — trivial against the Teensy regulator's ~250 mA. |

## 5. Pin usage summary / free pins

| Pins | Used by |
|---|---|
| 2–7 | camera trigger outputs 1–6 |
| 8, 9, 10, 11, 12, 13 | BMI088 (SPI0 + CS×2 + DRDY) |
| 30, 31 | CAN3 ↔ TJA1051T/3 |
| 0, 1, 14–29, 32+ | **free** (0/1 = CAN2, 22/23 = CAN1 if a second classic-CAN bus is ever needed) |

Cross-references: `docs/hardware-bringup.md` (bring-up stages that exercise
each subsystem), `docs/can-protocol.md` (what flows over CAN),
`docs/pose_pipeline.md` (clock/frame conventions the wiring feeds).
