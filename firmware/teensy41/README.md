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
| `ImuSample` | 1 | Teensy to host | Encoder path present, producer disabled |
| `WheelOdometry` | 2 | Teensy to host | Encoder path present, producer disabled |
| `CanRx` | 3 | Teensy to host | Reserved |
| `TeensyHealth` | 4 | Teensy to host | Sent at 10 Hz |
| `TimeSyncResponse` | 5 | Teensy to host | Sent for every time sync request |
| `RobotOdometry` | 6 | Teensy to host | Encoder path present, producer disabled |
| `FusedPose` | 64 | Host to Teensy | Decoded and stored for future CAN output |
| `CanTx` | 65 | Host to Teensy | Counted as unsupported in this phase |
| `TimeSyncRequest` | 66 | Host to Teensy | Decoded immediately |
| `ConfigCommand` | 67 | Host to Teensy | Camera trigger config supported |

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

## CAN Boundary

The firmware includes a `CanBridge` boundary that stores the latest fused pose
and reports unsupported `CanTx` requests. It does not transmit CAN frames yet.
Before enabling real CAN output, select the Teensy 4.1 CAN port/pins, add a
3.3 V CAN transceiver, and finalize RoboRIO CAN packet layouts and IDs.
