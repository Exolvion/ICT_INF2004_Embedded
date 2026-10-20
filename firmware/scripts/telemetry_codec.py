#!/usr/bin/env python3
"""Decode and encode the robot's binary MQTT telemetry and command bytes.

This is the laptop side of the wire contract in firmware/include/telemetry.h
and firmware/include/mqttcmd.h: little-endian, least-significant-bit-first
fields after a 4-bit header (2-bit version, 2-bit frame type). Signed fields
are two's complement; their most negative code means "invalid" (None here).

  python3 telemetry_codec.py decode 51ed..          hex frames as arguments
  mosquitto_sub -t <topic> -F %x | python3 telemetry_codec.py decode -
  python3 telemetry_codec.py command STOP           print the command byte
  python3 telemetry_codec.py command STOP --raw | mosquitto_pub -t <topic> -s

No network access happens here; the broker tools carry the bytes.
"""
from __future__ import annotations

import argparse
import json
import sys

VERSION = 1
SEQUENCE_MODULO = 250

# (name, bits, kind): kind "u" unsigned, "s" signed with reserved invalid
# code, "b" one-bit flag. Order and widths are the wire contract.
STATUS = (
    ("seq", 8, "u"), ("ms", 32, "u"), ("drop", 8, "u"), ("ign", 8, "u"),
    ("age", 10, "u"), ("st", 4, "u"), ("vl", 10, "s"), ("vr", 10, "s"),
    ("pt", 12, "s"), ("pk", 9, "u"), ("rg", 12, "u"), ("stop", 1, "b"),
    ("fault", 1, "b"), ("hv", 1, "b"), ("rv", 1, "b"),
)
PERCEPTION = (
    ("seq", 8, "u"), ("ms", 32, "u"), ("run", 8, "u"), ("cal", 8, "u"),
    ("ba", 3, "u"), ("bl", 4, "u"), ("yr", 13, "s"), ("mc", 4, "u"),
    ("sw", 12, "u"), ("sn", 12, "u"), ("ss", 2, "u"), ("sv", 6, "u"),
    ("su", 6, "u"), ("tv", 1, "b"), ("ha", 1, "b"), ("sc", 1, "b"),
    ("so", 1, "b"),
)
DIAGNOSTIC = (
    ("seq", 8, "u"), ("ms", 32, "u"), ("run", 8, "u"), ("fc", 4, "u"),
    ("fm", 32, "u"), ("bu", 8, "u"), ("br", 8, "u"), ("mp", 9, "u"),
    ("mv", 1, "b"), ("mask", 16, "u"), ("src", 2, "u"),
)
FRAMES = {0: ("status", STATUS), 1: ("perception", PERCEPTION),
          2: ("diagnostic", DIAGNOSTIC)}
BYTES = {name: (4 + sum(bits for _, bits, _ in layout) + 7) // 8
         for name, layout in FRAMES.values()}

COMMAND_PATTERN = 0x15
COMMANDS = ("NONE", "STOP", "START", "RESET", "LEFT", "RIGHT", "STRAIGHT",
            "UTURN")
FAULT_BITS = (
    "OWNER_TIMEOUT", "ENCODER", "MOTION", "LEFT_STALL", "RIGHT_STALL",
    "MOTOR_OUTPUT", "OPTICAL_SAMPLE", "BARCODE_OVERRUN", "SUPERVISOR", "IMU",
    "TILT", "TILT_UNAVAILABLE", "COLLISION", "SCAN", "NO_ECHO", "WIFI_LOSS",
)
VEHICLE_STATES = ("INIT", "IDLE", "FOLLOW", "NAVIGATE", "SCAN", "BYPASS",
                  "REACQUIRE", "FAULT", "RANGE_WAIT")


class FrameError(ValueError):
    """A frame that does not match the version, type, length or padding."""


def _field(value, bits: int, kind: str) -> int:
    """Encode one field exactly as telemetry.c does (saturating)."""
    if kind == "b":
        return 1 if value else 0
    if kind == "u":
        return max(0, min(int(value), (1 << bits) - 1))
    limit = (1 << (bits - 1)) - 1
    if value is None:
        return 1 << (bits - 1)
    return max(-limit, min(int(value), limit)) & ((1 << bits) - 1)


def encode(frame: str, values: dict) -> bytes:
    """Build one frame from a dict keyed by the short field names."""
    for frame_type, (name, layout) in FRAMES.items():
        if name == frame:
            break
    else:
        raise KeyError(frame)
    word = VERSION | (frame_type << 2)
    position = 4
    for key, bits, kind in layout:
        word |= _field(values.get(key, 0), bits, kind) << position
        position += bits
    return word.to_bytes(BYTES[frame], "little")


def decode(frame: bytes) -> tuple[str, dict]:
    """Return (frame name, fields); raise FrameError for a bad frame."""
    if not frame or (frame[0] & 0x3) != VERSION:
        raise FrameError("empty frame or unknown version")
    frame_type = (frame[0] >> 2) & 0x3
    if frame_type not in FRAMES:
        raise FrameError("reserved frame type 3")
    name, layout = FRAMES[frame_type]
    if len(frame) != BYTES[name]:
        raise FrameError(f"{name} frame must be {BYTES[name]} bytes, "
                         f"got {len(frame)}")
    word = int.from_bytes(frame, "little")
    position = 4
    values: dict = {}
    for key, bits, kind in layout:
        raw = (word >> position) & ((1 << bits) - 1)
        position += bits
        if kind == "b":
            values[key] = bool(raw)
        elif kind == "s":
            values[key] = None if raw == 1 << (bits - 1) else (
                raw - (1 << bits) if raw & (1 << (bits - 1)) else raw)
        else:
            values[key] = raw
    if word >> position:
        raise FrameError("nonzero padding bits")
    return name, values


def describe(name: str, values: dict) -> dict:
    """Add readable names for enums and fault bits; keep raw fields."""
    readable = dict(values)
    if name == "status" and values["st"] < len(VEHICLE_STATES):
        readable["state"] = VEHICLE_STATES[values["st"]]
    if name == "diagnostic":
        readable["faults"] = [bit for index, bit in enumerate(FAULT_BITS)
                              if values["mask"] & (1 << index)]
        readable["stop_source"] = [label for index, label in
                                   enumerate(("LOCAL", "REMOTE"))
                                   if values["src"] & (1 << index)]
    return readable


def command_byte(name: str) -> int:
    """Command byte: fixed pattern 10101b in bits 3-7, code in bits 0-2."""
    code = COMMANDS.index(name.upper())
    if code == 0:
        raise ValueError("NONE is not a command")
    return (COMMAND_PATTERN << 3) | code


def command_name(value: int) -> str:
    """Name of a received command byte, or NONE when the pattern fails."""
    if (value >> 3) != COMMAND_PATTERN:
        return "NONE"
    return COMMANDS[value & 0x7]


def _frames(arguments: list[str]):
    """Hex frames from arguments, or one per line from stdin for '-'."""
    if arguments == ["-"]:
        for line in sys.stdin:
            line = line.strip()
            if line:
                yield line
    else:
        yield from arguments


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="action", required=True)
    decoder = commands.add_parser("decode", help="decode hex frames")
    decoder.add_argument("frames", nargs="+", help="hex frames, or - for stdin")
    sender = commands.add_parser("command", help="print a command byte")
    sender.add_argument("name", choices=[name for name in COMMANDS[1:]] +
                        [name.lower() for name in COMMANDS[1:]])
    sender.add_argument("--raw", action="store_true",
                        help="write the raw byte for mosquitto_pub -s")
    args = parser.parse_args()
    if args.action == "command":
        value = command_byte(args.name)
        if args.raw:
            sys.stdout.buffer.write(bytes([value]))
        else:
            print(f"{value:02x}")
        return 0
    status = 0
    for text in _frames(args.frames):
        try:
            name, values = decode(bytes.fromhex(text))
            print(json.dumps({"frame": name, **describe(name, values)}))
        except (ValueError, FrameError) as error:
            print(json.dumps({"error": str(error), "hex": text}))
            status = 1
    return status


if __name__ == "__main__":
    raise SystemExit(main())
