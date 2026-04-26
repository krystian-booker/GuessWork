# GuessWork Teensy 4.1 Firmware

This PlatformIO project implements the Teensy side of the GuessWork USB protocol.
The repo host protocol in `include/posest/teensy/Protocol.h` is authoritative.

## Build

```bash
pio run -d firmware/teensy41
```

Upload when a Teensy 4.1 is connected:

```bash
pio run -d firmware/teensy41 -t upload
```

Monitor:

```bash
pio device monitor -d firmware/teensy41 -b 921600
```

## Default Pinout

Camera sync outputs are active-high 3.3 V logic pulses:

| Sync | Teensy 4.1 pin |
| --- | --- |
| SYNC0 | 2 |
| SYNC1 | 3 |
| SYNC2 | 4 |
| SYNC3 | 5 |
| SYNC4 | 6 |
| SYNC5 | 7 |

## BMI088 Shuttle Board 3.0 Pinout

The firmware reads the Bosch Sensortec SHUTTLE BOARD 3.0 BMI088 over SPI1
using the vendored Bosch BMI08x SensorAPI v1.9.0 source under
`lib/BMI08x_SensorAPI`. The driver uses Bosch's synchronized accel+gyro data
mode and publishes `ImuSample` frames over the existing USB protocol.

Bosch's shuttle-board flyer lists P1 as the power/interrupt connector and P2 as
the bus connector. The BMI088 datasheet states that the `PS` protocol-select
pin must be tied to `GND` for SPI and to `VDDIO` for I2C; use SPI for this
firmware because it keeps the high-rate IMU path off the camera/control buses.

| BMI088 shuttle connector | Shuttle pin name | Teensy 4.1 connection | Notes |
| --- | --- | --- | --- |
| P1-1 | `VDD` | `3.3V` | Sensor supply, 2.4 V to 3.6 V |
| P1-2 | `VDDIO` | `3.3V` | Digital I/O supply |
| P1-3 | `GND` | `GND` | Common ground |
| P1-6 | `GPIO2/INT1` | pin 15 | Synchronized accel data-ready to Teensy |
| P1-7 | `GPIO3/INT2` | reserved pin 16 | Reserved/diagnostic |
| P2-1 | `CS` | pin 8 | Accelerometer chip select |
| P2-2 | `SCK/SCL` | pin 27 | SPI1 SCK |
| P2-3 | `SDO` | pin 1 | SPI1 MISO |
| P2-4 | `SDI/SDA` | pin 26 | SPI1 MOSI |
| P2-5 | `GPIO4` | pin 9 | Gyroscope chip select |
| P2-6 | `GPIO5` | `GND` or pin 14 driven low | Gyroscope protocol select, low means SPI |
| P2-7 | `GPIO6` | DNC | Shuttle flyer marks NC |
| P2-8 | `GPIO7` | DNC | Shuttle flyer marks NC |
| P2-9 | `PROM_RW` | DNC | Shuttle EEPROM access; not used by firmware |

If `GPIO5/PS` is wired to pin 14 rather than directly to ground, the firmware
drives pin 14 low before starting SPI. The Bosch API performs the required
accelerometer SPI-mode dummy read during `bmi08xa_init`.

For synchronized data mode, direct wiring must also provide the Bosch data-sync
connection from gyro DRDY to the accelerometer sync input. The firmware config
matches the Bosch Application Board 3.x mapping:

| BMI088 data-sync signal | BMI088 channel | Purpose |
| --- | --- | --- |
| Gyro data-ready | gyro interrupt channel 4 | Feeds accel sync input |
| Accel sync input | accel interrupt channel 2 | Receives gyro DRDY |
| Accel sync data-ready | accel interrupt channel 1 | Goes to Teensy pin 15 |

The sync connection may require a carrier-board trace or a short jumper to
accessible shuttle-board pads; do not route that signal through a 5 V device.

Default IMU configuration:

| Setting | Value |
| --- | --- |
| Data-sync rate | 1000 Hz |
| Accelerometer range | +/-12 g |
| Gyroscope range | +/-2000 dps |
| Interrupt electrical mode | active-high push-pull |
| SPI clock | 5 MHz, mode 3 |

The built-in LED on pin 13 is a heartbeat. Slow blink means no firmware error
flags are currently set; fast blink means the firmware has seen an invalid
payload, CRC failure, unsupported command, or unsupported CAN operation.

Sync pins are 3.3 V only. Use level shifting or opto-isolation for cameras that
require 5 V trigger inputs. Keep camera trigger grounds referenced correctly.

## Protocol

All multi-byte fields are little-endian. A frame is:

| Field | Size |
| --- | ---: |
| Magic `"GW"` / `0x4757` | 2 |
| Protocol version | 1 |
| Message type | 1 |
| Sequence | 4 |
| Payload length | 2 |
| Payload | variable |
| CRC32 of header and payload | 4 |

Message directions:

| Type | ID | Direction | Status |
| --- | ---: | --- | --- |
| `ImuSample` | 1 | Teensy to host | BMI088 producer enabled after successful init |
| `CanRx` | 3 | Teensy to host | Reserved |
| `TeensyHealth` | 4 | Teensy to host | Sent at 10 Hz; carries `rio_offset_us` and IMU saturation counts |
| `TimeSyncResponse` | 5 | Teensy to host | Sent for every time sync request |
| `CameraTriggerEvent` | 7 | Teensy to host | One frame per rising edge of every armed sync output |
| `ConfigAck` | 8 | Teensy to host | Reply to every `ConfigCommand`; carries effective trigger periods / IMU config |
| `ChassisSpeeds` | 9 | Teensy to host | Emitted from CAN ID `0x100` decode; carries `rio_time_us`, `vx`, `vy`, `omega` |
| `FusedPose` | 64 | Host to Teensy | Decoded and re-published on CAN ID `0x180` at `pose_publish_hz` |
| `CanTx` | 65 | Host to Teensy | Reserved; currently rejected as unsupported |
| `TimeSyncRequest` | 66 | Host to Teensy | Decoded immediately |
| `ConfigCommand` | 67 | Host to Teensy | `CameraTriggers` and `ImuConfig` kinds supported |

`ImuSample` is sent on each synchronized BMI088 DRDY event. The timestamp is
the Teensy-side DRDY time in microseconds. Acceleration is converted to `m/s^2`,
gyroscope data is converted to `rad/s`, and temperature is refreshed every 100
samples from the accelerometer temperature register.

IMU error bits are reported through `TeensyHealth.error_flags`; per-sample
saturation/overrun bits are reported through `ImuSample.status_flags`.

## Camera Trigger Config

`ConfigCommandKind::CameraTriggers` has this payload:

| Field | Size |
| --- | ---: |
| Kind, value `1` | 4 |
| Trigger count | 4 |
| Enabled | 4 |
| Teensy pin | 4 |
| Rate Hz, double | 8 |
| Pulse width us | 4 |
| Phase offset us, signed 64-bit | 8 |

The per-trigger fields repeat `count` times. The firmware accepts at most six
triggers and only pins 2 through 7. Invalid channels are disabled rather than
pulsed, and the health frame reports trigger status flags.

## IMU Config

`ConfigCommandKind::ImuConfig` payload (28 bytes after the 4-byte kind):

| Field | Size | Notes |
| --- | ---: | --- |
| Kind, value `2` | 4 | |
| Accel range, g (`3`, `6`, `12`, `24`) | 4 | Mapped to `BMI088_ACCEL_RANGE_*` |
| Accel ODR, Hz (`12`, `25`, `50`, `100`, `200`, `400`, `800`, `1600`) | 4 | |
| Accel bandwidth code (`0` OSR4, `1` OSR2, `2` Normal) | 4 | |
| Gyro range, dps (`125`, `250`, `500`, `1000`, `2000`) | 4 | |
| Gyro bandwidth code (`0`–`3`) | 4 | |
| Data-sync rate, Hz (`400`, `1000`, `2000`) | 4 | |
| Run self-test on boot | 4 | Non-zero requests boot-time self-test |

The firmware re-runs `bmi08xa_perform_selftest` / `bmi08g_perform_selftest`
when the bit is set, and gates streaming on the result via
`kErrorImuSelfTestFailure`. It then re-runs `bmi08xa_configure_data_synchronization`
with the requested mode. The resulting effective configuration is reported back
in `ConfigAck`.

## CAN-FD Bridge

The firmware drives the Teensy 4.1 CAN3 bus (RX = pin 30, TX = pin 31) via
`FlexCAN_T4FD`. A TJA1051T/3 transceiver sits between the Teensy and the
RoboRIO bus. The default rates are `1 Mbit/s` nominal and `2 Mbit/s` data phase,
configurable through `TeensyConfig::CanBusConfig` in the host SQLite store.

Message catalogue — see `firmware/teensy41/include/CanSchema.h` for the full
RoboRIO clock-domain contract (FPGA microsecond semantics, recommended ping
rate, reboot recovery behavior):

| Direction | CAN ID | Bytes | Layout |
| --- | ---: | ---: | --- |
| RIO → Teensy pose | `0x100` | 48 | `int64 rio_time_us`, `double x_m`, `double y_m`, `double theta_rad`, `uint32 status_flags`, 12 B pad |
| RIO → Teensy time sync | `0x101` | 16 | `uint64 rio_time_us`, `uint32 ping_seq`, `uint32 reserved` |
| Teensy → RIO pose | `0x180` | 48 | `int64 teensy_time_us`, `double x_m`, `double y_m`, `double theta_rad`, `uint32 status_flags`, 12 B pad |

`0x100` decodes into a `ChassisSpeeds` USB frame the host can ingest as a
`ChassisSpeedsSample`. `0x101` drives the RoboRIO time-sync filter (offset =
`teensy_recv_us − rio_time_us`, EMA `α = 1/8`, single-step rejection at 5 ms,
auto-resync after 32 consecutive rejections — see `CanSchema.h` for the full
RoboRIO clock-domain contract). `0x180` is emitted at `pose_publish_hz` from
the most recent `FusedPose` forwarded from the host.

The current RoboRIO offset is reported in `TeensyHealth.rio_offset_us`, with
`rio_status_flags` carrying `kHealthRioUnsynchronized` until at least one
ping has been accepted (and again once the offset is stale).

## Protocol Drift Check

Both `include/posest/teensy/Protocol.h` and `firmware/teensy41/include/Protocol.h`
must agree on enum and constant values. CTest runs
`scripts/check_protocol_constants.py` to fail the build if either side drifts.
