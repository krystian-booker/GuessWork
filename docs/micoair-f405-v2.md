# MicoAir F405 V2 sync controller

This is the wiring, firmware, and acceptance reference for the board that
replaces the Teensy 4.1. The selected model must be the **MicoAir F405 V2
with BMI088 and SPL06**.

The board has everything this application needs: an STM32F405 clock/timer,
native USB, six exposed motor pads for camera triggers, and an onboard BMI088
on SPI2. The SPL06 is unused. Robot communication remains direct UDP between
the Mac and the robot controller.

## Pin map

The firmware-facing values come from MicoAir's published ArduPilot hardware
definition and are fixed in `firmware/src/board.hpp`.

| Logical output | Board pad | STM32 pin | Camera connection |
|---:|---|---|---|
| 1 | M1 | PB0 | camera 1 OPTO_GND / trigger return |
| 2 | M2 | PB1 | camera 2 OPTO_GND / trigger return |
| 3 | M3 | PA15 | camera 3 OPTO_GND / trigger return |
| 4 | M4 | PB3 | camera 4 OPTO_GND / trigger return |
| 5 | M5 | PB4 | camera 5 OPTO_GND / trigger return |
| 6 | M6 | PB5 | camera 6 OPTO_GND / trigger return |

The host and database continue to use logical output numbers 1–6, so existing
camera and trigger-group records do not need a migration.

The onboard BMI088 connection is not external wiring:

| Signal | STM32 pin |
|---|---|
| SPI2 SCK | PB13 |
| SPI2 MISO | PC2 |
| SPI2 MOSI | PC3 |
| accelerometer CS | PC13 |
| gyroscope CS | PC14 |

USB uses PA11/PA12. The BMI088 axes are rotated into the flight-controller
board frame using the board definition's `ROTATION_ROLL_180_YAW_270` before
samples are sent to the host. No BMI088 data-ready line is exposed in the
published board definition, so firmware reads both dies on a 400 Hz schedule
and timestamps the midpoint of each SPI transaction.

## Camera Line0 wiring

Keep the low-side wiring proven on the Chameleon3 bench:

```text
regulated +5 V  ─────────────► camera OPTO_IN  (JST pin 9; often yellow)
camera OPTO_GND (JST pin 7) ─► one M1..M6 pad
5 V supply GND ──────────────► flight-controller GND
```

The GPIO is LOW at idle. That conducts the camera optocoupler and leaves
Line0 high. A 100 µs GPIO-HIGH window turns the optocoupler off, creating the
Line0 falling edge used by `TriggerActivation=FallingEdge`. Firmware drives
all six pads LOW before enabling the timer and never emits triggers until the
host sends a valid atomic configuration followed by `ARM`.

Do not put 5 V directly on an STM32 GPIO. The voltage is applied to the
camera's isolated input; the GPIO is only its low-side return. Verify the
specific camera pigtail pin positions rather than trusting wire colours.

## Power decision and mandatory check

The preferred bench configuration is USB-only board power. Before connecting
any camera optocoupler, use a multimeter to check whether the board's labelled
5V pad is energized at approximately 5 V from USB.

- If it is, use that 5V pad for camera `OPTO_IN` and a nearby GND pad for the
  common reference.
- If it is not, use a separate regulated 5 V supply for `OPTO_IN` and tie its
  ground to the flight-controller ground.
- Do **not** feed VBAT merely to obtain a trigger rail, and do not connect a
  raw robot battery to a 5V pad.

Repeat the rail check on every board revision. The published pinout confirms
the available pads but does not guarantee the USB-to-5V power path.

## Build and flash

The checked-in PlatformIO environment is pinned to the STM32 platform and
builds one native USB CDC interface:

```sh
cd firmware
pio run -e micoair_f405_v2
```

To flash with the ROM DFU bootloader, hold the board's BOOT control while
connecting USB (or bridge its BOOT pads), confirm an STM32 DFU device appears,
then run:

```sh
pio run -e micoair_f405_v2 -t upload
```

The normal firmware enumerates as one `cu.usbmodem*` device. `guesswork`
probes candidates with a framed `HELLO` and only accepts the MicoAir board ID;
port numbering therefore does not need to be stable.

## Firmware contract

`firmware/include/sync_controller_protocol.h` is compiled by both firmware
and host. Every USB message has magic, protocol version, type, request ID,
payload length, and CRC16-CCITT. The one bidirectional CDC stream carries:

- `HELLO` / `DEVICE_INFO` discovery;
- atomic `SET_CONFIG`, `ARM`, `STOP`, and `TEST_OUTPUT` commands with
  correlated acknowledgements;
- 64-bit-microsecond trigger events and BMI088 records on the same TIM2
  timebase;
- one-second heartbeat state and IMU/trigger/USB drop counters.

Configuration is deliberately volatile and host-authoritative. A reset or
USB reconnect leaves every output stopped; `SyncControllerManager` remembers
the database-derived desired state and re-sends `SET_CONFIG` then `ARM`.
The complete byte layout is in `docs/sync-controller-protocol.md`.

## First-board acceptance

Do not install the board on the robot until these pass:

1. With cameras disconnected, power by USB and confirm the 5V-rail behavior.
2. Flash, start the server, and confirm `/api/imu/status` reports
   `controller_connected`, board `micoair_f405_v2`, protocol 1, `imu_ok`, and
   approximately 400 Hz with no growing error counters.
3. With the controller stopped, scope every M1–M6 pad: idle must be LOW.
   `POST /api/hardware-sync/test-output {"output_pin":1}` emits one 100 µs
   test window (repeat for outputs 1–6).
4. Connect one camera and arm a 30 Hz group. Confirm 100 µs HIGH windows at
   the assigned pad, Line0 falling-edge capture at 30 Hz, and matching frame
   timestamps. Check every logical-output mapping.
5. Run at least 90 minutes and verify monotonic trigger/IMU timestamps,
   stable clock-sync drift, no CRC errors, and no growing firmware drops.
6. Run a camera–IMU Kalibr recording. A small, stable
   `timeshift_cam_imu` is the end-to-end timestamp acceptance criterion.

The source review establishes feature compatibility, but the power-rail,
output-polarity, signal-integrity, and timing numbers above remain physical
acceptance gates.

## Primary hardware references

- [MicoAir F405 V2 specification and pinout](https://micoair.com/flightcontroller_micoair405v2/)
- [ArduPilot MicoAir405v2 hardware definition](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/MicoAir405v2/hwdef.dat)
- [Bosch BMI088 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)
