# Sync-controller USB protocol v1

The normative implementation is
`firmware/include/sync_controller_protocol.h`. Both the MicoAir firmware and
the macOS host compile that header. This document is the operator-readable
copy; change it with the header.

## Frame

All integers are little-endian and all floats are IEEE-754 binary32.

| Offset | Size | Field |
|---:|---:|---|
| 0 | 2 | magic `A5 5A` |
| 2 | 1 | protocol version (`1`) |
| 3 | 1 | message type |
| 4 | 2 | request ID; zero for unsolicited events |
| 6 | 2 | payload length, maximum 256 |
| 8 | N | payload |
| 8+N | 2 | CRC16-CCITT-FALSE over bytes 2 through 7+N |

The decoder accepts arbitrary USB read boundaries. Bad version, length, or
CRC causes a bytewise search for the next magic sequence.

## Host commands

| Type | Name | Payload | Response |
|---:|---|---|---|
| `01` | `HELLO` | empty | `DEVICE_INFO`, same request ID |
| `03` | `SET_CONFIG` | count + complete group array | `ACK` |
| `04` | `ARM` | empty | `ACK` |
| `05` | `STOP` | empty | `ACK` |
| `06` | `TEST_OUTPUT` | logical output u8 (1–6) | `ACK` |

`SET_CONFIG` is a complete atomic replacement, not a patch. Its first four
bytes are `count u8, reserved[3]`, followed by `count` eight-byte records:
`slot u8, output_mask u8, reserved u16, rate_millihz u32`. At most four
groups and six uniquely assigned outputs are accepted. Rates are 1 mHz
through 1,000,000 mHz. A valid replacement stops existing outputs; a separate
`ARM` begins periodic pulses.

An `ACK` payload is `command_type u8, status u8, reserved u16`. Status values
cover OK, malformed message, invalid config, busy, no config, unsupported,
and internal error. The host correlates both request ID and command type and
retries a timed-out idempotent command once.

`DEVICE_INFO` is 28 bytes: board ID, firmware version, output/group counts,
capability flags, reset reason, and the three-word STM32 unique ID. Discovery
only accepts the MicoAir F405 V2 board signature.

## Controller events

| Type | Name | Payload |
|---:|---|---|
| `20` | `TRIGGER` | slot u8 + reserved[3] + index u32 + timestamp_us u64 |
| `21` | `IMU_BATCH` | count u8 + reserved[3] + 1–4 records |
| `22` | `HEARTBEAT` | 32-byte health snapshot |

Each 32-byte IMU record is `timestamp_us u64`, acceleration XYZ float32 in
m/s², then angular velocity XYZ float32 in rad/s. Values are already rotated
into the flight-controller board frame. Trigger and IMU timestamps come from
the same 1 MHz extended TIM2 counter.

The heartbeat carries `timestamp_us u64`, flags u32 (`imu_ok`, `armed`), and
u32 counters for produced IMU samples, IMU scheduling drops, trigger-event
queue drops, and USB send errors; the final four bytes are reserved.

## Recovery and safety

The controller stores no configuration in flash. Reset, invalid config,
explicit stop, or loss of the USB host leaves M1–M6 LOW and unarmed. The host
keeps desired configuration in its database, probes after reconnect, sends
one complete `SET_CONFIG`, and only then sends `ARM` when persisted operator
intent says the system should be armed.

