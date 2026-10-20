#!/usr/bin/env python3
"""Golden-vector and boundary tests for scripts/telemetry_codec.py.

The golden inputs and bytes are identical to tests/test_telemetry_main.c,
so the C encoder and this decoder are checked against one fixed contract.
"""
from __future__ import annotations

import random
import unittest

import telemetry_codec as codec

STATUS = dict(seq=125, ms=0x12345678, drop=3, ign=255, age=1023, st=2,
              vl=-123, vr=511, pt=None, pk=300, rg=4095, stop=True,
              fault=False, hv=True, rv=False)
STATUS_BYTES = bytes([0xD1, 0x87, 0x67, 0x45, 0x23, 0x31, 0xF0, 0xFF, 0xBF,
                      0x14, 0xFE, 0x1F, 0x00, 0xB2, 0xFC, 0xFF, 0x02])
PERCEPTION = dict(seq=125, ms=0x89ABCDEF, run=7, cal=1, ba=4, bl=15,
                  yr=-4095, mc=8, sw=1234, sn=4000, ss=2, sv=61, su=0,
                  tv=True, ha=False, sc=True, so=True)
PERCEPTION_BYTES = bytes([0xD5, 0xF7, 0xDE, 0xBC, 0x9A, 0x78, 0x10, 0xC0,
                          0x0F, 0x80, 0x28, 0x4D, 0xA0, 0x6F, 0x0F, 0x34])
DIAGNOSTIC = dict(seq=0, ms=1, run=255, fc=15, fm=0xFFFFFFFF, bu=0, br=200,
                  mp=511, mv=True, mask=0x8001, src=3)
DIAGNOSTIC_BYTES = bytes([0x09, 0x10, 0x00, 0x00, 0x00, 0xF0, 0xFF, 0xFF,
                          0xFF, 0xFF, 0xFF, 0x00, 0xC8, 0xFF, 0x07, 0x00,
                          0x0E])


class GoldenFrames(unittest.TestCase):
    def test_sizes(self):
        self.assertEqual(codec.BYTES, {"status": 17, "perception": 16,
                                       "diagnostic": 17})

    def test_encode_matches_c(self):
        self.assertEqual(codec.encode("status", STATUS), STATUS_BYTES)
        self.assertEqual(codec.encode("perception", PERCEPTION),
                         PERCEPTION_BYTES)
        self.assertEqual(codec.encode("diagnostic", DIAGNOSTIC),
                         DIAGNOSTIC_BYTES)

    def test_decode_matches_inputs(self):
        for frame, values, name in ((STATUS_BYTES, STATUS, "status"),
                                    (PERCEPTION_BYTES, PERCEPTION,
                                     "perception"),
                                    (DIAGNOSTIC_BYTES, DIAGNOSTIC,
                                     "diagnostic")):
            decoded_name, decoded = codec.decode(frame)
            self.assertEqual(decoded_name, name)
            self.assertEqual(decoded, values)

    def test_readable_fault_bits(self):
        _, values = codec.decode(DIAGNOSTIC_BYTES)
        readable = codec.describe("diagnostic", values)
        self.assertEqual(readable["faults"], ["OWNER_TIMEOUT", "WIFI_LOSS"])
        self.assertEqual(readable["stop_source"], ["LOCAL", "REMOTE"])


class Boundaries(unittest.TestCase):
    def test_saturation_and_invalid(self):
        frame = codec.encode("status", dict(STATUS, vl=600, vr=-9999,
                                            pt=-2048, age=60000, pk=700))
        _, values = codec.decode(frame)
        self.assertEqual((values["vl"], values["vr"], values["pt"]),
                         (511, -511, -2047))
        self.assertEqual((values["age"], values["pk"]), (1023, 511))

    def test_rejections(self):
        with self.assertRaises(codec.FrameError):
            codec.decode(STATUS_BYTES[:-1])
        with self.assertRaises(codec.FrameError):
            codec.decode(bytes([STATUS_BYTES[0] ^ 0x3]) + STATUS_BYTES[1:])
        with self.assertRaises(codec.FrameError):
            codec.decode(STATUS_BYTES[:-1] + bytes([STATUS_BYTES[-1] | 0x80]))
        with self.assertRaises(codec.FrameError):
            codec.decode(bytes([0x0D]))
        with self.assertRaises(codec.FrameError):
            codec.decode(b"")

    def test_random_round_trips(self):
        rng = random.Random(2004)
        for name, layout in codec.FRAMES.values():
            for _ in range(2000):
                values = {}
                for key, bits, kind in layout:
                    if kind == "b":
                        values[key] = bool(rng.getrandbits(1))
                    elif kind == "u":
                        values[key] = rng.getrandbits(bits)
                    else:
                        limit = (1 << (bits - 1)) - 1
                        values[key] = rng.choice(
                            [None, rng.randint(-limit, limit)])
                frame = codec.encode(name, values)
                self.assertEqual(codec.decode(frame), (name, values))


class Commands(unittest.TestCase):
    def test_command_bytes(self):
        self.assertEqual(codec.command_byte("STOP"), 0xA9)
        self.assertEqual([codec.command_byte(name)
                          for name in codec.COMMANDS[1:]],
                         list(range(0xA9, 0xB0)))
        valid = [value for value in range(256)
                 if codec.command_name(value) != "NONE"]
        self.assertEqual(valid, list(range(0xA9, 0xB0)))
        self.assertEqual(codec.command_name(ord("S")), "NONE")


if __name__ == "__main__":
    unittest.main()
