# Teensy 4.1 — pinout & component wiring

Every wire on the Teensy 4.1 board: camera trigger outputs and the BMI088
IMU. Pin numbers are Teensy 4.1 Arduino-style digital pins. The firmware
constants are the source of truth — each section cites them; change them
together. (The TJA1051 CAN transceiver wiring was retired with fw=4 — robot
communication is now UDP from the Mac; see `docs/ethernet-protocol.md`.)

```
                          ┌──────── USB (dual CDC) ────────► Mac Mini
                          │  Serial      = ASCII commands / TRIG events
                          │  SerialUSB1  = binary telemetry (IMU ▲, heartbeat ▲)
            ┌─────────────┴─────────────┐
   GND ─────┤ GND                   VIN ├───── (5 V in, or powered via USB)
            │ 0                     GND │
            │ 1                    3.3V ├───── BMI088 VDD/VDDIO
   cam 1 ◄──┤ 2                      23 │
   cam 2 ◄──┤ 3                      22 │
   cam 3 ◄──┤ 4                      21 │
   cam 4 ◄──┤ 5                      20 │
   cam 5 ◄──┤ 6                      19 │
   cam 6 ◄──┤ 7                      18 │
  BMI088 ───┤ 8  (gyro DRDY*)        17 │
  BMI088 ───┤ 9  (gyro CS)           16 │
  BMI088 ───┤ 10 (accel CS)          15 │
  BMI088 ───┤ 11 (SPI0 MOSI)         14 │
  BMI088 ───┤ 12 (SPI0 MISO)         13 ├───── BMI088 SCK (SPI0)
            └───────────────────────────┘
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

Board: **Bosch Sensortec BMI088 Shuttle Board 3.0** (flyer
`BST-BMI088-SF000-01`, Shuttle ID `0x66` / `BST00767`). The sensor breaks
out across two pin-strip connectors: **P2** (9-pin) carries the SPI/I2C data
lines, **P1** (7-pin) carries power and the accelerometer interrupts. Pin
names below are taken verbatim from the flyer's connector tables.

Firmware wiring (Teensy SPI0 → shuttle-board pins):

| Teensy pin | Signal | Shuttle-board pin (name — function) |
|---|---|---|
| **11** | SPI0 MOSI | **P2-4** SDI/SDA — SPI master-out / I2C data |
| **12** | SPI0 MISO | **P2-3** SDO — SPI master-in |
| **13** | SPI0 SCK | **P2-2** SCK/SCL — shared SPI/I2C clock |
| **10** | accel chip select | **P2-1** CS — SPI chip select, **accelerometer** |
| **9** | gyro chip select | **P2-5** GPIO4 — SPI chip select, **gyroscope** |
| **8** | gyro data-ready interrupt | gyro INT — **not a dedicated connector pin, see ⚠ below** |
| 3.3V | power | **P1-1** VDD **+ P1-2** VDDIO |
| GND | ground | **P1-3** GND |

⚠ **No "INT3" pin exists on this board.** The current firmware names the
gyro data-ready line `INT3`, but the Shuttle Board 3.0 connectors do **not**
expose a pin labelled INT3. The only interrupt pins on the headers are the
**accelerometer's** INT1/INT2 (P1-6 `GPIO2/INT1`, P1-7 `GPIO3/INT2`). The
gyroscope's interrupts are routed on-board: the gyro INT4 net (`INT4_G`)
lands on jumper **JP3**, which can tie it onto the INT2 connector line
(`INT2_A` ↔ `INT4_G`). To feed a 400 Hz gyro data-ready into Teensy pin 8 on
this exact board you must therefore either:
1. configure the gyro to emit data-ready on **INT4**, fit **JP3**, and wire
   Teensy 8 → **P1-7** (`GPIO3/INT2`); **or**
2. use a different BMI088 breakout that brings the gyro **INT3** pad out to
   its own pin (many third-party breakouts label it `INT3`/`INT4`/`DRDY`).

Verify which board is physically mounted before trusting the `INT3` label in
`bmi088_imu.h`.

Full connector reference (from the flyer):

**P2 — 9-pin SPI/data connector**

| Pin | Name | Function |
|---|---|---|
| 1 | CS | SPI chip select — accelerometer |
| 2 | SCK/SCL | Clock (SPI + I2C) |
| 3 | SDO | SPI master-in-slave-out |
| 4 | SDI/SDA | SPI master-out-slave-in / I2C data |
| 5 | GPIO4 | SPI chip select — gyroscope |
| 6 | GPIO5 | Protocol select — gyroscope (strap for SPI per gyro datasheet) |
| 7 | GPIO6 | NC |
| 8 | GPIO7 | NC |
| 9 | PROM_RW | on-board EEPROM (DS28E05) — leave NC for our use |

**P1 — 7-pin power/interrupt connector**

| Pin | Name | Function |
|---|---|---|
| 1 | VDD | Power supply (3.3 V) |
| 2 | VDDIO | I/O supply (3.3 V) |
| 3 | GND | Ground |
| 4 | GPIO0 | NC |
| 5 | GPIO1 | NC |
| 6 | GPIO2/INT1 | **Accelerometer** interrupt 1 |
| 7 | GPIO3/INT2 | **Accelerometer** interrupt 2 (also JP3 → gyro INT4) |

Notes:
- One SPI bus, two chip selects — the BMI088 is two dies (accel + gyro) in
  one package, so `CS` (P2-1) selects the accel and `GPIO4` (P2-5) selects
  the gyro.
- The **gyro data-ready** interrupt paces sampling (400 Hz): the ISR stamps
  `now_us64()` and the main loop reads both dies. See the ⚠ above for how that
  line actually reaches the connector on this board.
- **GPIO5** (P2-6) is the gyroscope protocol-select pin; it must be strapped
  for SPI mode. The accelerometer auto-selects SPI on its first CS falling
  edge, so it has no separate protocol pin.
- The BMI088 is 3.3 V-only — never feed it 5 V. Tie VDD and VDDIO together to
  the Teensy 3.3 V rail.
- Mechanically: hard-mounted next to the **left VIO camera** (the camera-IMU
  extrinsics calibration assumes a rigid mount).

## 3. Power & USB

| Connection | Notes |
|---|---|
| USB (micro-B) → Mac | Dual CDC (`-DUSB_DUAL_SERIAL`): `Serial` = ASCII command port, `SerialUSB1` = binary telemetry. Also powers the Teensy on the bench. |
| VIN | 5 V robot supply for competition use (USB data still connected to the Mac). If feeding VIN while USB is plugged, follow PJRC's guidance: cut the VUSB↔VIN pad or diode-isolate. |
| 3.3V rail budget | BMI088 (~6 mA) — trivial against the Teensy regulator's ~250 mA. |

## 5. Pin usage summary / free pins

| Pins | Used by |
|---|---|
| 2–7 | camera trigger outputs 1–6 |
| 8, 9, 10, 11, 12, 13 | BMI088 (SPI0 + CS×2 + DRDY) |
| 0, 1, 14–31, 32+ | **free** |

Cross-references: `docs/hardware-bringup.md` (bring-up stages that exercise
each subsystem), `docs/ethernet-protocol.md` (robot comms),
`docs/pose_pipeline.md` (clock/frame conventions the wiring feeds).
