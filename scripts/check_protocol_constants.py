#!/usr/bin/env python3
"""Drift detector for the host and firmware copies of `Protocol.h`.

The host (`include/posest/teensy/Protocol.h`) and firmware
(`firmware/teensy41/include/Protocol.h`) carry independent copies of the same
USB protocol because they live in different build systems. Nothing at compile
time enforces that they stay in sync. This script parses both files for the
shapes that *must* agree:

  * `enum class MessageType : std::uint8_t { ... }` — every enumerator and value
  * `enum class ConfigCommandKind : std::uint32_t { ... }`
  * The framing / status / config-ack constants that appear on both sides.

Mismatches print a diff and exit 1. Run from CTest so CI fails on drift.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
HOST_PATH = REPO_ROOT / "include" / "posest" / "teensy" / "Protocol.h"
FIRMWARE_PATH = REPO_ROOT / "firmware" / "teensy41" / "include" / "Protocol.h"

# Constants that must appear in both files with identical values. The firmware
# defines these without a namespace prefix; the host uses
# `posest::teensy::...`. We normalise to bare names.
SHARED_CONSTANTS = (
    "kFrameMagic",
    "kProtocolVersion",
    "kStatusUnsynchronizedTime",
    "kStatusUnsynchronizedRioTime",
    "kStatusRobotSlipping",
    "kHealthRioUnsynchronized",
    "kHealthRioPingRejected",
    "kConfigAckUnsupportedCount",
    "kConfigAckInvalidPin",
    "kConfigAckDuplicatePin",
    "kConfigAckInvalidRate",
    "kConfigAckPulseTooWide",
    "kConfigAckInvalidImuConfig",
    "kConfigAckImuSelfTestFailure",
    "kConfigAckUnknownKind",
)

ENUM_RE = re.compile(
    r"enum\s+class\s+(\w+)[^\{]*\{([^\}]+)\}\s*;",
    re.DOTALL,
)
ENUMERATOR_RE = re.compile(r"(\w+)\s*=\s*([0-9xXa-fA-F<u\s]+),?")
CONST_RE_TEMPLATE = (
    r"constexpr\s+std::(?:uint8_t|uint16_t|uint32_t|uint64_t)\s+"
    r"{name}\s*=\s*([^;]+);"
)


def parse_int(literal: str) -> int:
    """Parse C++ integer-style literals: 0x42, 1u << 0u, 10u, 256."""
    text = literal.strip()
    text = text.replace("u", "").replace("U", "").strip()
    if "<<" in text:
        left, right = (part.strip() for part in text.split("<<", 1))
        return parse_int(left) << parse_int(right)
    if text.lower().startswith("0x"):
        return int(text, 16)
    return int(text, 10)


def parse_enums(source: str) -> dict[str, dict[str, int]]:
    enums: dict[str, dict[str, int]] = {}
    for match in ENUM_RE.finditer(source):
        name = match.group(1)
        body = match.group(2)
        members: dict[str, int] = {}
        for m in ENUMERATOR_RE.finditer(body):
            members[m.group(1)] = parse_int(m.group(2))
        enums[name] = members
    return enums


def parse_constant(source: str, name: str) -> int | None:
    pattern = re.compile(CONST_RE_TEMPLATE.format(name=name))
    match = pattern.search(source)
    if not match:
        return None
    return parse_int(match.group(1))


def main() -> int:
    host = HOST_PATH.read_text(encoding="utf-8")
    firmware = FIRMWARE_PATH.read_text(encoding="utf-8")

    failures: list[str] = []

    host_enums = parse_enums(host)
    fw_enums = parse_enums(firmware)
    for enum_name in ("MessageType", "ConfigCommandKind"):
        host_members = host_enums.get(enum_name)
        fw_members = fw_enums.get(enum_name)
        if host_members is None:
            failures.append(f"host is missing enum class {enum_name}")
            continue
        if fw_members is None:
            failures.append(f"firmware is missing enum class {enum_name}")
            continue
        if host_members != fw_members:
            host_only = sorted(set(host_members) - set(fw_members))
            fw_only = sorted(set(fw_members) - set(host_members))
            mismatched = sorted(
                key
                for key in host_members.keys() & fw_members.keys()
                if host_members[key] != fw_members[key]
            )
            failures.append(
                f"enum {enum_name} drift: "
                f"host_only={host_only}, firmware_only={fw_only}, "
                f"mismatched_values={mismatched}"
            )

    for name in SHARED_CONSTANTS:
        host_value = parse_constant(host, name)
        fw_value = parse_constant(firmware, name)
        if host_value is None:
            failures.append(f"host is missing constant {name}")
            continue
        if fw_value is None:
            failures.append(f"firmware is missing constant {name}")
            continue
        if host_value != fw_value:
            failures.append(
                f"constant {name} drift: host={host_value} firmware={fw_value}"
            )

    if failures:
        print("Protocol drift detected between host and firmware Protocol.h:",
              file=sys.stderr)
        for failure in failures:
            print(f"  - {failure}", file=sys.stderr)
        return 1

    print("Protocol.h host/firmware constants and enums agree.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
