#!/usr/bin/env python3
"""Decode the bench image's minimum-bit telemetry frames back to its lines.

Change Requirement 1 (Tribunal 1): the mission (app_program/subsystem5/
mission.c, BENCH_FRAMES 1; the robot app's Vehicle task since 22 Nov, and
the bench image) sends no text over MQTT. Each line goes as one binary
frame:

  byte 0   0xBD: telemetry.h's 2-bit version 1, frame type 3 (the type the
           robot reserves), then the pattern 1011b. Never printable text,
           never a robot frame.
  byte 1   seq (8 bits), counted per frame sent: a gap is a lost frame.
  then     the kind (7 bits) and the line's fields, LSB first, little-endian
           (bitpack.h), zero padding to a whole byte.

The constant text is not sent: KINDS below holds each kind's template, which
mirrors the firmware's bench_text()/bench_u()/... calls for that line, so the
decoded line is the text the firmware built.

  python3 bench_codec.py decode bd00..            hex frames as arguments
  python3 bench_codec.py check sim_output.txt     every "#F" frame of a sim
                                                  or offline run against the
                                                  line printed after it
  python3 bench_codec.py table [sim_output.txt]   the frame table (markdown),
                                                  measured sizes from a run
  python3 bench_codec.py selftest                 KINDS against the C enum

No network access happens here.
"""
from __future__ import annotations

import os
import re
import sys

HEAD = 0xBD
KIND_BITS = 7
TEXT_BYTES = 160          # BENCH_TEXT_BYTES: the firmware's text is cut at
TEXT_KEEP = TEXT_BYTES - 2  # 158 characters; the frame keeps everything
SOURCE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                      "app_program", "mission_private.h")


class FrameError(ValueError):
    """Not a bench frame: wrong head, kind, length or padding."""


class Reader:
    """bitpack_read(): LSB-first, little-endian."""

    def __init__(self, data: bytes, bit: int = 0):
        self.data = data
        self.bit = bit

    def read(self, width: int) -> int:
        if self.bit + width > 8 * len(self.data):
            raise FrameError("frame too short")
        value = 0
        for index in range(width):
            byte = self.data[(self.bit + index) >> 3]
            value |= ((byte >> ((self.bit + index) & 7)) & 1) << index
        self.bit += width
        return value

    def signed(self, width: int) -> int:
        value = self.read(width)
        if value & (1 << (width - 1)):
            value -= 1 << width
        if value == -(1 << (width - 1)):
            raise FrameError("reserved signed code")
        return value

    def complete(self) -> bool:
        """Every byte used and the padding zero (bitpack_reader_complete)."""
        if (self.bit + 7) // 8 != len(self.data):
            return False
        if self.bit % 8:
            return (self.data[-1] >> (self.bit % 8)) == 0
        return True


# ---- template elements: each mirrors one firmware call ---------------------
# A template is a tuple of elements; a str is constant text (bench_text with
# a literal), never sent.

class U:
    """bench_u(value, bits): unsigned, printed as bench_dec."""

    def __init__(self, bits: int, name: str = ""):
        self.bits, self.name = bits, name

    def run(self, r: Reader, ctx: dict) -> str:
        value = r.read(self.bits)
        if self.name:
            ctx[self.name] = value
        return str(value)


class I:
    """bench_i(value, bits): signed, printed as bench_dec."""

    def __init__(self, bits: int, name: str = ""):
        self.bits, self.name = bits, name

    def run(self, r: Reader, ctx: dict) -> str:
        value = r.signed(self.bits)
        if self.name:
            ctx[self.name] = value
        return str(value)


class S(I):
    """bench_s(value, bits): signed, printed as bench_signed (+ for >= 0)."""

    def run(self, r: Reader, ctx: dict) -> str:
        text = super().run(r, ctx)
        return text if text.startswith("-") else "+" + text


def fixed(value: int, places: int) -> str:
    """bench_fixed_text: value / 10^places with that many decimals."""
    scale = 10 ** places
    magnitude = abs(value)
    return (("-" if value < 0 else "") + str(magnitude // scale) + "." +
            str(magnitude % scale).rjust(places, "0"))


class X:
    """bench_x(value, places, bits): signed fixed point."""

    def __init__(self, bits: int, places: int, name: str = ""):
        self.bits, self.places, self.name = bits, places, name

    def run(self, r: Reader, ctx: dict) -> str:
        value = r.signed(self.bits)
        if self.name:
            ctx[self.name] = value
        return fixed(value, self.places)


class XS(X):
    """'+' for >= 0 (bench_text), then bench_x: bench_ramp_deg's angles."""

    def run(self, r: Reader, ctx: dict) -> str:
        text = super().run(r, ctx)
        return text if text.startswith("-") else "+" + text


class UX:
    """bench_ux(value, places, bits): unsigned fixed point."""

    def __init__(self, bits: int, places: int, name: str = ""):
        self.bits, self.places, self.name = bits, places, name

    def run(self, r: Reader, ctx: dict) -> str:
        value = r.read(self.bits)
        if self.name:
            ctx[self.name] = value
        return fixed(value, self.places)


class CODE:
    """bench_code(value, bits): a value with no text, kept for WHEN/REP."""

    def __init__(self, bits: int, name: str):
        self.bits, self.name = bits, name

    def run(self, r: Reader, ctx: dict) -> str:
        ctx[self.name] = r.read(self.bits)
        return ""


class NAME:
    """bench_name(names, index, bits): one of a fixed list of words."""

    def __init__(self, bits: int, names, name: str = ""):
        self.bits, self.names, self.name = bits, tuple(names), name

    def run(self, r: Reader, ctx: dict) -> str:
        index = r.read(self.bits)
        if index >= len(self.names):
            raise FrameError("name index out of range")
        if self.name:
            ctx[self.name] = index
        return self.names[index]


class FLAG:
    """if (bench_flag(cond)) { then } else { otherwise }."""

    def __init__(self, then=(), otherwise=(), name: str = ""):
        self.then, self.otherwise, self.name = then, otherwise, name

    def run(self, r: Reader, ctx: dict) -> str:
        value = r.read(1)
        if self.name:
            ctx[self.name] = value
        return render(self.then if value else self.otherwise, r, ctx)


class WHEN:
    """A branch on a value read earlier (CODE/U/NAME name): cases maps the
    value to a template; default for any other value."""

    def __init__(self, name: str, cases: dict, default=()):
        self.name, self.cases, self.default = name, cases, default

    def run(self, r: Reader, ctx: dict) -> str:
        return render(self.cases.get(ctx[self.name], self.default), r, ctx)


class REP:
    """bench_code(count, bits) then the body count times (a loop)."""

    def __init__(self, bits: int, body, name: str = ""):
        self.bits, self.body, self.name = bits, body, name

    def run(self, r: Reader, ctx: dict) -> str:
        count = r.read(self.bits)
        if self.name:
            ctx[self.name] = count
        return "".join(render(self.body, r, ctx) for _ in range(count))


class CHARS:
    """count (count_bits) then count characters of char_bits each, as
    chr(base + code): text the firmware copies in (codes, map rows)."""

    def __init__(self, count_bits: int, char_bits: int, base: int):
        self.count_bits, self.char_bits, self.base = count_bits, char_bits, base

    def run(self, r: Reader, ctx: dict) -> str:
        count = r.read(self.count_bits)
        return "".join(chr(self.base + r.read(self.char_bits))
                       for _ in range(count))


CODE39 = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. *$/+%"   # g_code39_chars


class STR:
    """bench_str(text, length_bits): length, then 6 bits a character
    (index in CODE39; 45 = '?'). empty: the text printed for "" (the firmware
    prints it with bench_text after an empty string)."""

    def __init__(self, length_bits: int, empty: str = "", name: str = ""):
        self.length_bits, self.empty, self.name = length_bits, empty, name

    def run(self, r: Reader, ctx: dict) -> str:
        count = r.read(self.length_bits)
        out = []
        for _ in range(count):
            code = r.read(6)
            if code < len(CODE39):
                out.append(CODE39[code])
            elif code == len(CODE39) + 1:
                out.append("?")
            else:
                raise FrameError("bad character code")
        text = "".join(out)
        if self.name:
            ctx[self.name] = text
        return text if text else self.empty


class MARKS:
    """bench_marks(text, alphabet, bits): count characters, each its index
    in alphabet (the count is fixed by the template)."""

    def __init__(self, count: int, bits: int, alphabet: str):
        self.count, self.bits, self.alphabet = count, bits, alphabet

    def run(self, r: Reader, ctx: dict) -> str:
        out = []
        for _ in range(self.count):
            code = r.read(self.bits)
            if code >= len(self.alphabet):
                raise FrameError("bad mark")
            out.append(self.alphabet[code])
        return "".join(out)


class CH:
    """bench_char(ch, base, bits): one character, base + code."""

    def __init__(self, bits: int, base: str):
        self.bits, self.base = bits, ord(base)

    def run(self, r: Reader, ctx: dict) -> str:
        return chr(self.base + r.read(self.bits))


class HEX:
    """bench_nav_hex: a byte as two hex digits."""

    bits = 8

    def run(self, r: Reader, ctx: dict) -> str:
        return f"{r.read(8):02x}"


def render(template, r: Reader, ctx: dict) -> str:
    out = []
    for element in template:
        out.append(element if isinstance(element, str) else
                   element.run(r, ctx))
    return "".join(out)


# ---- names the firmware prints from its tables ------------------------------

LINE_STATES = ("INHIBITED", "TRACKING", "SEARCHING", "JUNCTION", "LOST",
               "FAULT")                                   # g_line_names

# ---- shared parts -----------------------------------------------------------

SPEC_NAMES = ("", "character widths", "gap between characters",
              "quiet zone")                               # g_spec_names
VOTE_NAMES = ("Decoder 1", "Decoder 2", "D0")             # g_vote_names
MODE_NAMES = (                                            # g_mode_names
    "PID steps (stand)", "DIST 1 m (floor)", "LEFT 90 (floor)",
    "RIGHT 90 (floor)", "TURN 180 (floor)", "LINE follow 80 mm/s (floor)",
    "LINE follow 120 mm/s (floor)", "LINE follow 150 mm/s (floor)",
    "LINE + stop within 10 cm of an obstacle, 120 mm/s (floor)",
    "SCAN obstacle (floor, car still)",
    "LINE centre 120 mm/s (track, runs until STOP)",
    "LINE centre 150 mm/s (track, runs until STOP)",
    "LINE smooth A 180 mm/s, soft (track, runs until STOP)",
    "LINE smooth B 150 mm/s, soft (track, runs until STOP)",
    "LINE smooth C 130 mm/s, soft (track, runs until STOP)",
    "BARCODE read, smooth A steering, 80 mm/s (track, bars on the left)",
    "AUTO-CENTRE only (track, pivots on the spot, no driving)",
    "MISSION: smooth A, barcodes and ramp read while driving (track, runs"
    " until STOP)",
    "AVOID an obstacle and return to course, 80 mm/s (floor, no line, until"
    " STOP)")
END_NAMES = (                                             # g_end_names
    "aborted", "done", "centred", "stopped: IR sensor fault",
    "stopped: obstacle ahead",
    "stopped: all three sensors dark (lifted, or off the mat)",
    "stopped: both sensors black past a junction", "stopped: line lost",
    "stopped: line sensor fault", "stopped at the distance limit",
    "stopped at the time limit", "stopped: AUTO-CENTRE failed",
    "stopped: the way along was blocked",
    "stopped: the way back to the line was blocked",
    "stopped: the obstacle's edge was out of reach",
    "stopped: no clear way round the obstacle", "stopped: lifted (IMU)",
    "OK", "INVALID", "REJECTED", "EXPIRED", "TIMED_OUT", "SENSOR_FAULT",
    "STOPPED")
AUTO_FAILS = (                                            # g_auto_fail_names
    "line sensor fault",
    "both front sensors see a line at START (a crossing?): start on a plain"
    " stretch",
    "not centred within 30 s",
    "pivot stalled: under 1 deg in 1.5 s (wheels stuck or slipping)",
    "no line within 70 deg either side of the START heading",
    "the sensors found the line over 50 deg apart: not one straight line"
    " (bend, crossing, bars?)")
MOVES = ("FORWARD", "U-TURN", "LEFT", "RIGHT")             # navmap order
LEAN_OUTS = ("at a turn", "not on a straight", "pitched", "lifted or knocked",
             "no wheel 3 mm up")                          # g_lean_out_names
DEG = XS(12, 1)                                            # bench_ramp_deg
UNIT_MM = U(21)                                            # bench_code_unit_mm
# bench_nav_reads: each decoder 0 not in, 1 no read, 2 a letter
NAV_READS = tuple(part for who in ("Decoder 1 ", ", Decoder 2 ", ", D0 ")
                  for part in (who, CODE(2, "r"), WHEN("r", {
                      0: ("?",), 1: ("-",), 2: (STR(1),)})))
# bench_nav_say_map: "MAP set: forward A, U-turn B, left C, right D"
MAP_SET = ("MAP set: forward ", CH(5, "A"), ", U-turn ", CH(5, "A"),
           ", left ", CH(5, "A"), ", right ", CH(5, "A"))
RAMP_STATUS = FLAG((" pitch ", DEG, " roll ", DEG, " h ", I(10)))
# bench_avoid_peak_say, bench_mag_say
IR_PEAK = (" (IR peak L ", U(12), " R ", U(12), ")")
MAG_SAY = FLAG(("; C", " ", S(13), " ", S(13), " ", S(13)))
# the creeps' last echo: none, or out and range
LAST_ECHO = (FLAG(("none",), (I(13), " mm out, ", U(12), " mm")),)
# AV_PATH: a side's state 0 clear, 1 edge unseen, 2 blocked
PATH_STATE = (CODE(2, "p"), WHEN("p", {0: (" mm, clear",),
                                       1: (" mm, edge unseen",),
                                       2: (" mm, blocked",)}))
# bench_ramp_side
WHEEL_HIGHER =(FLAG((" deg (left wheel ",), (" deg (right wheel ",)), U(8),
                " mm higher)")


def lean(head: str) -> tuple:
    """bench_ramp_lean_line(head, lean)."""
    return (head, FLAG(("left wheel up ",), ("right wheel up ",)), U(9),
            " mm (30 mm mean, roll ", DEG, " deg; peak ", U(9), " mm, roll ",
            DEG, " deg, pitch ", DEG, ") at ", I(21), " mm, t ", UX(16, 1),
            " s, over ", U(16), " mm")

# ---- the kinds: (C name without BK_, template), in bench_kind_t order ------

KINDS: list = [
    ("LIVE", (" t ", UX(16, 1), " s d ", I(21), " mm ", NAME(3, LINE_STATES),
              " B ", U(12), " max ", U(12), " L ", U(12), " R ", U(12), " groups ", U(10), " codes ",
              U(10), " F ", U(12), " x", U(16), RAMP_STATUS)),
    ("POSE", (" P ", I(17), " ", I(17), " ", I(9),
              FLAG((" C", " ", S(13), " ", S(13), " ", S(13))))),
    ("REC_HEAD", ("REC: last ", U(9), " samples, one per 20 ms, before the"
                  " stop: d,h,L,R,l,r,p,c,t,s,n,f,B,r,q")),
    ("REC_ROW", ("REC ", U(8), ":", REP(2, (
        " ", I(21), ",", I(9), ",", U(7), ",", U(7), ",", U(7), ",", U(7),
        ",", U(2), ",", I(8), ",", I(8), ",", I(2), ",", I(2), ",",
        U(7), ",", U(7), ",", I(12), ",", I(12))))),
    ("REC_END", ("REC end",)),
    ("CODE", (FLAG(
        (" CODE ", STR(4), FLAG((" read end to start",), (" read in order",)),
         FLAG((", PARTIAL: one * only",)), FLAG((", peaks agree",))),
        (" GROUP ", U(8), " bars, ", CODE(2, "g"), WHEN("g", {
            0: ("one swing only, not a code: ", STR(4),
                FLAG((" end to start",))),
            1: ("frame ", STR(4), " refused by the spec (",
                NAME(2, SPEC_NAMES), ")"),
            2: ("no frame: ", STR(4), " / ", STR(4)),
            3: ("not a Code 39 count",)}),
         WHEN("g", {0: ()}, (", characters ", STR(4, "none"), " / ",
                             STR(4, "none"))))),
        " at ", UNIT_MM, "-", UNIT_MM, " mm (swing ", UX(9, 1), " sd, gap ",
        U(8), " mm)")),
    ("FIT", (CODE(2, "why"), WHEN("why", {
        2: (" Decoder 2 no read: dark for ", U(16), " mm, longer than a card"),
        1: (" Decoder 2 no read: the 6 widest bars fit no character (",
            UX(10, 1), " mm off)"),
        3: (" Decoder 2 skipped: still fitting the last card",)},
        (FLAG((" Decoder 2 *",), (" Decoder 2 no read: best *",)), STR(1),
         FLAG(("* rev",), ("*",)), ", margin ", UX(14, 2), " over *", STR(1),
         FLAG(("* rev",), ("*",)), "; module ", UX(11, 2), " mm, spot ",
         UX(8, 1), " mm, dead ", UX(7, 2))),
        "; at ", U(21), "-", U(21), " mm", WHEN("why", {0: (
            "; ", U(16), " ms, ", U(16), " steps, ", U(20), "k tests",
            FLAG((", cuts joined",)))}))),
    ("D0", (" D0 ", FLAG(
        (STR(4), FLAG((" read in order",), (" read end to start",))),
        (U(10), " bars, ", CODE(2, "w"), WHEN("w", {
            0: ("too many to keep",),
            1: ("no frame: ", STR(4), " / ", STR(4)),
            2: ("not a Code 39 count",)}))),
        " at ", UNIT_MM, "-", UNIT_MM, " mm")),
    ("D0_WIDTHS", (" D0 widths u:", REP(6, (" ", U(9))))),
    ("FIT_TOTALS", (" Decoder 2: ", U(10), " read, ", U(10), " no read, ",
                    U(10), " skipped, ", U(10), " lines lost (margin needed ",
                    UX(8, 2), ")")),
    ("CAR", (" CAR ", CODE(2, "a"), WHEN("a", {
        0: ("no code",),
        1: ("*", STR(4), "*"),
        2: ("*", STR(4), "* by majority"),
        3: ("no code, the decoders disagree",)}),
        " (", *[part for index, who in enumerate(VOTE_NAMES) for part in (
            ("" if index == 0 else ", ") + who, CODE(2, "v"), WHEN("v", {
                0: (" none",), 1: (" no read",),
                2: (" *", STR(4), "*")}))],
        ") at ", U(21), "-", U(21), " mm")),
    ("CAR_TOTALS", (" CAR: ", U(11), " codes (", U(10),
                    " all readers agreed, ", U(10), " by majority), no code ",
                    U(10), " times as the decoders disagreed, ", U(10),
                    " with no read", FLAG(("; ", U(10),
                                           " results lost (no vote free)")))),
    ("RAW_HEAD", (" RAW ", U(10), ": u", U(23), " (side ", UNIT_MM, " mm), ",
                  U(13), " units, white ", U(12), " sd ", U(12),
                  "; raw = white - 10 + 2 (c - '#') below 'c', else white"
                  " + 118 + 140 (c - 'c'); '!' none")),
    ("RAW_ROW", (" R ", U(10), ": ", CHARS(7, 7, ord("!")))),
    ("MAP", MAP_SET),
    ("NAV_MAP", (" ",) + MAP_SET),
    ("NAV_RESTORED", (" NAV: map restored from its backup (restore ", U(8),
                      "); g_nav began") + (" ", HEX()) * 8),
    ("NAV_CARD", (" NAV: card ", U(21), "-", U(21), " mm (", *NAV_READS,
                  "): ", FLAG(
                      (STR(1), ", at the next junction: ",
                       FLAG((NAME(2, MOVES),),
                            (FLAG(("not in the map, straight on",),
                                  ("no map yet, straight on",)),))),
                      ("no code yet (no read, or the decoders disagree)",)))),
    ("NAV_JUNCTION", (" NAV: junction ", U(10), " at ", I(21), " mm: ",
                      FLAG(("card (", *NAV_READS, ") ")), FLAG(
        ("code ", STR(1), FLAG((" by majority",)), FLAG(
            (" -> ", NAME(2, MOVES, "m"), WHEN("m", {0: ()}, (
                ", advancing ", U(9), " mm to turn on the crossing"))),
            (FLAG((" not in the map, straight on",),
                  (", no map yet, straight on",)),))),
        (CODE(2, "n"), WHEN("n", {
            0: ("no code stored, straight on",),
            1: ("no read, straight on",),
            2: ("decoders disagree, straight on",)}))))),
    ("NAV_TURNED", (" NAV: turned ", S(10),
                    " deg on the crossing, following the line")),
    ("NAV_WAIT", (" NAV: junction at ", I(21),
                  " mm, the card is still decoding (", *NAV_READS,
                  "): stopped to wait for it")),
    ("NAV_WAITED", (" NAV: waited ", U(16), CODE(2, "w"), WHEN("w", {
        0: (" ms, two decoders agree (",),
        1: (" ms, still decoding: deciding on (",),
        2: (" ms, decoding done (",)}), *NAV_READS, ")")),
    ("RAMP_OFF", (FLAG(("RAMP off: the reference was refused",),
                       ("RAMP off: no accelerometer",)),)),
    ("RAMP_FLAT", (FLAG(("RAMP flat reference: the last still second (n ",),
                        ("RAMP flat reference: stored (car not still, n ",)),
                   U(10), "), start pitch ", DEG, " roll ", DEG, " deg")),
    ("RAMP", (" RAMP ", U(10), ": peak ", I(10), " mm at ", I(21), " mm (t ",
              UX(16, 1), " s), roll ", DEG, *WHEEL_HIGHER, "; pitch up ", DEG,
              " down ", DEG, " deg; from ", I(21), " mm")),
    ("SIDE", (FLAG((" SIDE left out (", NAME(3, LEAN_OUTS), *lean("): ")),
                   (" SIDE ", U(10), *lean(": "))),)),
    ("TERRAIN", (FLAG((" TERRAIN hump counted: peak ",),
                      (" TERRAIN hump not counted (turn, jolt or not level at"
                       " the end): peak ",)), I(10), " mm")),
    ("CAL", tuple(part for name in (" CAL L ", ", R ", ", B ") for part in (
        name, U(12), "-", U(12), " (set ", U(12), "-", U(12), ")")) +
     (" over AUTO-CENTRE and ", I(21), " mm")),
    ("RAMP_HIGHEST", (" ramp: highest ", I(10), " mm", FLAG((
        " (rise ", U(10), ") at ", I(21), " mm, t ", UX(16, 1), " s, roll ",
        DEG, *WHEEL_HIGHER)))),
    ("RAMP_OPEN", (" ramp: open at the stop, not counted: peak ", I(10),
                   " mm at ", I(21), " mm, t ", UX(16, 1), " s, roll ", DEG,
                   *WHEEL_HIGHER)),
    ("RAMP_RISES", (" ramp: rises ", U(10), " of 5 mm or more, robot module ",
                    U(10), " counted, max ", U(9),
                    " mm; car's own offset pitch ", DEG, " roll ", DEG,
                    " deg")),
    ("RAMP_LEANS", (FLAG(lean(" ramp: most side lean "),
                         (" ramp: no side lean of 5 mm",)),
                    "; leans ", U(10), ", jolts ", U(12))),
    ("RAMP_HELD", (" ramp: held straight over humps ", U(10), " times, ",
                   U(16), " mm, let go for a bend ", U(10))),
    ("RAMP_OUT", (" ramp: left out at a turn or not on a straight: rises ",
                  U(10), ", leans ", U(10), "; leans pitched over 10 deg ",
                  U(10), "; no wheel 3 mm up (level): rises ", U(10),
                  ", leans ", U(10))),
    ("RAMP_WILD", (" ramp: left out as lifted or knocked (over ", U(9),
                   " mm up): ", U(10))),
    ("LIFT", (FLAG((" lift: STOPPED the run; most height ",),
                   (" lift: not lifted; most height ",)), U(12),
              " mm (stop at ", U(12), "), most upward speed ", U(12),
              " mm/s; motor direction changes ", U(16))),
    ("RAMP_ROLL", (" ramp: most roll left ", DEG, " at ", I(21), " mm, right ",
                   DEG, " at ", I(21), " mm; samples ", U(19), ", no tilt ",
                   U(19))),
    ("DRIVE_START", (FLAG(
        ("PID start: about 15 s, wheels must be off the ground; STOP aborts",),
        ("DRIVE start: car on the floor, path clear; STOP aborts",
         FLAG(("; steering EDGE (left edge)",), ("; steering SMOOTH",)))),)),
    ("STRAIGHT", ("STRAIGHT barcode: no line following, straight on at ",
                  U(9), " mm/s until STOP")),
    ("AUTO_FIRST", ("AUTO-CENTRE first: pivots on the spot, up to 70 deg"
                    " each way, until both front sensors find the line",)),
    ("END", (FLAG(("MISSION",), (NAME(5, MODE_NAMES),)), " ",
             NAME(5, END_NAMES),
             FLAG((FLAG((" after ",), (", complete after ",)), UX(16, 1),
                   " s")),
             ": dt max ", UX(16, 1), " ms", FLAG((", ENCODER ERRORS",)))),
    ("CENTRE", (" centre: ", I(21), " mm in ", UX(16, 1), " s, mean ", I(10),
                " mm/s; time % per phase", " TRACK ", U(7), " GAP ", U(7),
                " SEARCH ", U(7), " CROSS ", U(7), ", full steer ", U(7))),
    ("CENTRE_GAPS", (" gaps ", U(16), ", crossings ", U(10), ", searches ",
                     U(12), " (longest ", U(16), " ms), darkest raw L ",
                     U(12), " R ", U(12))),
    ("AVOID_DROPPED", (" obstacle go-around: ", U(10),
                       " lines dropped (queue full)")),
    ("D0_TOTALS", (" D0: ", U(22), " changes, ", U(12), " groups (", U(12),
                   " under ", U(4), " bars), ", U(10), " read, ", U(10),
                   " dropped; high ", U(22), ", low ", U(22),
                   " readings, dark is ", FLAG(("high",), ("low",)))),
    ("BARCODES", (" barcodes: ", U(22), " readings, one per ", UX(14, 2),
                  " ms; groups ", U(10), ", codes ", U(10), "; held straight ",
                  U(10), " times, ", U(16), " mm, let go ", U(10),
                  FLAG((", the first ", U(10), " kept")),
                  FLAG(("; STORE FULL",)), "; spec refused frames: widths ",
                  U(12), ", gaps ", U(12), ", quiet ", U(12),
                  "; raw sent for ", U(10), " of ", U(10), " cuts")),
    # The go-around: queued lines, so a leading space (AVOID alone is not
    # reachable from START; its own lines are not sent).
    ("AV_OBSTACLE", (" MISSION: obstacle at ", U(12),
                     " mm, stopped to measure it")),
    ("AV_OBSTACLE_AVOID", ("AVOID: obstacle at ", U(12),
                           " mm, stopped to measure it")),
    ("AV_MEASURE", (" AVOID: measuring, turning left to its edge",)),
    ("AV_LEFT_EDGE", (" AVOID: left edge ", I(13), " mm at ", I(9), FLAG(
        (" deg (still on it at the turn limit: unseen), turning right",),
        (" deg, turning right to the other edge",)))),
    ("AV_BACKED", (" AVOID: backed up to ", I(13), FLAG(
        (" mm behind the first stop, creeping out again",),
        (" mm behind the first stop, measuring again",)))),
    ("AV_CREEP", (FLAG((" AVOID: creeping out to find its right edge, the"
                        " sonar square to its face",),
                       (" AVOID: creeping out to find its left edge, the"
                        " sonar square to its face",)),)),
    ("AV_FAR_BACK", (" AVOID: back from the far edge (", U(12),
                     " mm off the start), turning ", U(7),
                     " deg to creep out to the way round's edge")),
    ("AV_STEP_OUT", (FLAG((" AVOID: stepping out to the right",),
                          (" AVOID: stepping out to the left",)),)),
    ("AV_CLEAR", (" AVOID: clear of its edge, turning back to the line"
                  " heading",)),
    ("AV_ALONG", (" AVOID: alongside the object for ", I(13), " mm")),
    ("AV_AHEAD", (" AVOID: something ahead at ", U(12),
                  " mm alongside the object, stopped")),
    ("AV_PAST", (" AVOID: past the object, stepping back to the line",)),
    ("AV_WAY_CLEAR", (" AVOID: the way back looks clear, going on ", I(13),
                      " mm more to clear the object's unseen end")),
    ("AV_RETURN", (" AVOID: returning to the line",)),
    ("AV_ON_LINE", (" AVOID: on the line, straightening", *IR_PEAK,
                    FLAG((", line heading moved ", DEG)))),
    ("AV_ALONG_LINE", (" AVOID: came in along a line (", FLAG(("L",)),
                       FLAG(("R",)), " dark): kept as a candidate")),
    ("AV_ON_COURSE", (" AVOID: back on course, driving straight",)),
    ("AV_STILL_WAY", (" AVOID: the object is still in the way at ", U(12),
                      " mm, going on along it by ", I(13), " mm")),
    ("AV_BLOCKED", (" AVOID: the way back to the line is still blocked at ",
                    U(12), " mm, stopped")),
    ("AV_WAY_IN", (" AVOID: way in, ", NAME(1, ("L", "R")), " dark ",
                   CODE(1, "k"), I(13), " mm at ", I(13), WHEN("k", {
                       0: (" mm off the wheels' line: not tape-wide",),
                       1: (" mm off the wheels' line: that sensor only",)}))),
    ("AV_WAY_IN_BOTH", (" AVOID: way in, L and R dark ", I(13),
                        " mm apart at ", I(13),
                        " mm off the wheels' line: over 30 deg")),
    ("AV_CROSSED", (" AVOID: line crossed ", I(13),
                    " mm off the wheels' guess, car at ", DEG,
                    " deg to it (dark L ", I(13), " R ", I(13), " mm)")),
    ("AV_TURN_LINE", (" AVOID: turn check: line ", DEG,
                      " deg off the wheels' turn (dark ", DEG,
                      "), turning onto it", MAG_SAY)),
    ("AV_TURN_NONE", (" AVOID: turn check: no line within ", CODE(1, "t"),
                      U(8), WHEN("t", {0: (" deg, looking on", *IR_PEAK),
                                       1: (" deg, by the wheels", *IR_PEAK,
                                           MAG_SAY)}))),
    ("AV_TURN_BACK", (FLAG((" AVOID: turn check: back to the line it came in"
                            " along, ",),
                           (" AVOID: turn check: back to the line crossed"
                            " at ",)), DEG, " deg off the wheels' turn",
                      MAG_SAY)),
    ("AV_HOME_NONE", (" AVOID: measuring turns checked: line not seen,"
                      " heading by the wheels", MAG_SAY)),
    ("AV_HOME", (" AVOID: measuring turns checked: line ", DEG,
                 " deg off the wheels' heading (", U(8), " seen), reset",
                 MAG_SAY)),
    ("AV_IR", (" AVOID: line checks: dark from ",) + tuple(
        part for name in ("L ", " R ") for part in (
            name, U(12), FLAG((" (run ",), (" (configured, run ",)), U(12),
            "-", U(12), ")"))),
    ("AV_NO_ECHO", (" AVOID: no echo off its face creeping at ", I(9),
                    " deg (the sonar ", I(13),
                    " mm out is past its edge), creeping at ", I(9), " deg")),
    ("AV_ON_FACE", (" AVOID: still on its face ", I(13),
                    " mm out, no room to creep on: backing up ", I(13),
                    " mm (", U(4), " of ", U(4), ")")),
    ("AV_PAST_REACH", (" AVOID: its edge is past ", I(13),
                       " mm out and the reverses are spent, stopped")),
    ("AV_FAR_EDGE", (FLAG((" AVOID: far (right) edge ",),
                          (" AVOID: far (left) edge ",)), I(13),
                     FLAG((" mm out or more (last echo ",),
                          (" mm out (last echo ",)), *LAST_ECHO,
                     "), backing to the creep's start")),
    ("AV_EDGE", (" AVOID: edge ", I(13), " mm out (last echo ", *LAST_ECHO,
                 "): out ", I(13), " mm, along ", I(13), " mm")),
    ("AV_WIDTH", (" AVOID: WIDTH ", I(13), FLAG(
        (" mm or more by the creeps (left ",), (" mm by the creeps (left ",)),
        I(13), ", right ", I(13), "; raw ", I(13), ", less ", I(13),
        " an edge)")),
    ("AV_OTHER", (" AVOID: something else in the way that far out,"
                  " stopped",)),
    ("AV_FACE", (" AVOID: face ", I(13), " mm ahead of the pivot; obstacle"
                 " left ", I(13), FLAG((" mm (edge unseen), right ",),
                                       (" mm, right ",)), I(13),
                 FLAG((" mm (edge unseen); WIDTH ~",), (" mm; WIDTH ~",)),
                 I(13), FLAG((" mm or more",), (" mm",)), " (median of 3, ",
                 U(8), " spread skipped)")),
    ("AV_PAST_SWEEP", (" AVOID: an edge is past the sweep, backing up ", I(13),
                       " mm to see more (", U(4), " of ", U(4), ")")),
    ("AV_PATH", (" AVOID: path right: out ", I(13), " mm, total ", I(13),
                 *PATH_STATE, "; left: out ", I(13), " mm, total ", I(13),
                 *PATH_STATE)),
    ("AV_NO_WAY", (" AVOID: no clear way round, stopped",)),
    ("AV_GOING", (FLAG((" AVOID: going round the right",),
                       (" AVOID: going round the left",)), " (sweep out ",
                  I(13), " mm); first turning ", U(7),
                  FLAG((" deg to creep out to its left edge",),
                       (" deg to creep out to its right edge",)))),
    ("AV_OBST", (" OBST at ", I(17), " ", I(17), " h ", X(12, 1), " face ",
                 I(13), " left ", I(13), FLAG(("? right ",), (" right ",)),
                 I(13), FLAG(("? depth ",), (" depth ",)), I(13),
                 CODE(2, "s"), WHEN("s", {0: (" go none",), 1: (" go R",),
                                          2: (" go L",)}),
                 " out ", I(13), " along ", I(13), " back ", I(13))),
    ("AVOID_MAP_NONE", (" AVOID MAP: no beams",)),
    ("AVOID_MAP_HEAD", (" AVOID MAP ", U(5), " mm cells, rows with a hit as M"
                        " x cells, left at left: # obstacle, + other, . free,"
                        " - unknown, | line")),
    ("AVOID_MAP_ROW", (" M ", I(11), " ", MARKS(41, 3, "-#+.|C"))),
    ("BEAMS", (" BEAMS back ", I(12), ":", REP(4, (" ", I(9), "/", U(12))),
               FLAG((" (+", U(10), " not kept)")))),
    # Idle, boot, commands, the lift stop and AUTO-CENTRE.
    ("BOOT", (FLAG(("BENCH MOTION ready: START runs MISSION (line-follow +"
                    " barcode + obstacle go-around), STOP stops. Accel ",),
                   ("BENCH ready: START = duty sweep, STOP = abort. Accel ",)),
              FLAG(("ok",), ("FAILED",)))),
    ("ENC_ZERO", ("ENC distance zeroed",)),
    ("IR", (FLAG(tuple(part for name in ("IR L ", " R ", " B ")
                       for part in (name, U(12), " ", U(12), "-", U(12)))),)),
    ("ACC", ("ACC mg n ", U(10, "n"), WHEN("n", {0: ()}, tuple(
        part for name in (" x ", " y ", " z ")
        for part in (name, S(13), " ", S(13), "..", S(13)))))),
    ("MAG", ("MAG", " x ", S(13), " y ", S(13), " z ", S(13))),
    ("ENC", ("ENC counts L ", S(25), " R ", S(25), " mm L ", S(22), " R ",
             S(22), " err ", U(16))),
    ("LIFT_STOP", (" LIFT: raised ", U(12), " mm: motors stopped")),
    ("AUTO", (CODE(2, "o"), WHEN("o", {0: (" AUTO-CENTRE FAILED after ",),
                                       1: (" AUTO-CENTRE stopped after ",),
                                       2: (" AUTO-CENTRE done in ",)}),
              UX(10, 1), " s, sweeps ", U(8), ", corrections ", U(8),
              ", run-on after a halt ", DEG, " deg")),
    ("AUTO_WHY", ("  why: ", NAME(3, AUTO_FAILS))),
    ("AUTO_FOUND", ("  right sensor found the line turning left at ",
                    FLAG((DEG, " deg"), ("none",)), ", left turning right at ",
                    FLAG((DEG, " deg"), ("none",)))),
    ("AUTO_END", (FLAG(("  middle ", DEG, " deg, heading at the end ", DEG,
                        " deg (off the middle ", DEG, ")"),
                       ("  heading at the end ", DEG, " deg")),
                  ", front IR L ", U(12), " R ", U(12))),
    ("NAV_TURN", (" NAV: no line ", U(9), " mm past the dark at ", I(21),
                  " mm: a turn, not a junction; the card (", *NAV_READS,
                  ") waits for the next junction")),
    ("SLOW", (" slowest step at ", I(21), " mm; most in one step: Decoder 1"
              " tries ", UX(16, 1), " ms (the Decoder 2 fit ", UX(16, 1),
              " ms of it), live lines ", UX(16, 1), " ms")),
    # Robot app (22 Nov): each task's longest step (us) and skipped periods
    # since START, app_main.c's order (SYS-12 evidence).
    ("TASKS", (" tasks, longest step us/skipped periods: Safety ", U(16),
               "/", U(12), ", Motion ", U(16), "/", U(12), ", Line ", U(16),
               "/", U(12), ", Vehicle ", U(16), "/", U(12), ", IMU ", U(16),
               "/", U(12), ", Scan ", U(16), "/", U(12), ", Comm ", U(16),
               "/", U(12), ", Telemetry ", U(16), "/", U(12))),
    # Robot app (22 Nov): the IMU task's evidence since START (failed reads
    # and how long the last took, saturated reads, longest gap between fresh
    # samples, slowest read, terrain refusals).
    ("IMU", (" IMU since START: failed reads ", U(8), " (the last took ",
             U(16), " us), saturated ", U(8), ", longest gap ", U(16),
             " ms, slowest read ", U(16), " us; terrain refused ", U(8),
             " late, ", U(8), " other")),
    # Robot app (22 Nov): each owner's longest health gap since START (the
    # one over its gate is the OWNER_TIMEOUT).
    ("OWNERS", (" owner health, longest gap since START: Motion ", U(16),
                " ms, Line ", U(16), " ms, Vehicle ", U(16), " ms, IMU ",
                U(16), " ms, Scan ", U(16), " ms, Comm ", U(16), " ms")),
]


def decode(frame: bytes):
    """(seq, kind name, line) of one frame."""
    if len(frame) < 3 or frame[0] != HEAD:
        raise FrameError("not a bench frame")
    r = Reader(frame, 16)
    kind = r.read(KIND_BITS)
    if not 1 <= kind <= len(KINDS):
        raise FrameError(f"unknown kind {kind}")
    name, template = KINDS[kind - 1]
    text = render(template, r, {})
    if not r.complete():
        raise FrameError(f"{name}: length or padding wrong")
    return frame[1], name, text


def is_frame(payload: bytes) -> bool:
    return len(payload) >= 3 and payload[0] == HEAD


# ---- tools ------------------------------------------------------------------

def c_kinds(path: str = SOURCE) -> list:
    text = open(path, encoding="ascii", errors="replace").read()
    block = re.search(r"typedef enum\s*\{\s*BK_NONE = 0,(.*?)BK_KINDS\s*\}"
                      r"\s*bench_kind_t;", text, re.S)
    if not block:
        raise SystemExit("bench_kind_t not found in " + path)
    body = re.sub(r"/\*.*?\*/", "", block.group(1), flags=re.S)
    return [n[3:] for n in re.findall(r"\bBK_[A-Z0-9_]+", body)]


def selftest() -> int:
    names = [name for name, _ in KINDS]
    c_names = c_kinds()
    if names != c_names:
        print("KINDS and bench_kind_t differ:")
        print("  python:", names)
        print("  C:     ", c_names)
        return 1
    if len(names) >= (1 << KIND_BITS):
        print("too many kinds for", KIND_BITS, "bits")
        return 1
    print(f"selftest: {len(names)} kinds, same order as bench_kind_t")
    return 0


def check(path: str) -> int:
    """Each "#F" frame against the next line; "#T" lines have no frame."""
    lines = open(path, encoding="ascii", errors="replace").read().split("\n")
    good = bad = cut = 0
    text_bytes = frame_bytes = 0
    unsent = {}
    bad_shown = 0
    seqs = []
    for index, line in enumerate(lines):
        if line.startswith("#T "):
            shape = re.sub(r"[-+]?\d+(\.\d+)?", "N", line[3:])[:60]
            unsent[shape] = unsent.get(shape, 0) + 1
            continue
        if not line.startswith("#F "):
            continue
        want = lines[index + 1] if index + 1 < len(lines) else ""
        try:
            frame = bytes.fromhex(line[3:].strip())
            seq, name, got = decode(frame)
            seqs.append(seq)
        except (ValueError, FrameError) as error:
            got, name = f"<{error}>", "?"
        if got == want:
            good += 1
        elif len(want) == TEXT_KEEP and got.startswith(want):
            cut += 1
        else:
            bad += 1
            if bad_shown < 12:
                bad_shown += 1
                print(f"MISMATCH ({name}) line {index + 2}:\n  text  {want!r}"
                      f"\n  frame {got!r}")
            continue
        text_bytes += len(want) + 1
        frame_bytes += len(frame)
    seen = {}
    for index, line in enumerate(lines):
        if line.startswith("#F "):
            try:
                kind = Reader(bytes.fromhex(line[3:].strip()), 16).read(
                    KIND_BITS)
                seen[kind] = seen.get(kind, 0) + 1
            except (ValueError, FrameError):
                pass
    never = [name for number, (name, _) in enumerate(KINDS, 1)
             if number not in seen]
    print(f"kinds: {len(KINDS) - len(never)} of {len(KINDS)} seen; never: "
          + (", ".join(never) if never else "none"))
    gaps = sum(1 for a, b in zip(seqs, seqs[1:]) if (a + 1) % 256 != b)
    print(f"frames: {good} match, {cut} match a line the text buffer cut, "
          f"{bad} differ; seq gaps {gaps}")
    if good + cut:
        print(f"bytes: text {text_bytes} -> frames {frame_bytes} "
              f"({100.0 * frame_bytes / max(text_bytes, 1):.1f}%)")
    if unsent:
        print(f"lines with no frame (never sent): {sum(unsent.values())}")
        for shape, count in sorted(unsent.items(), key=lambda kv: -kv[1]):
            print(f"  {count:6d}  {shape}")
    return 1 if bad or unsent else 0


def fields(template) -> str:
    """The fields of a template in order, compactly: u21 unsigned 21 bits,
    i13 signed, x12.1 signed tenths, n3 a name (3 bits), f[...] a 1-bit
    flag and its branches (a/b), w2 a 2-bit branch code, r4*(...) a 4-bit count
    and its repeats, s4 text (4-bit length, 6 bits a character)."""
    out = []
    for element in template:
        if isinstance(element, str):
            continue
        if isinstance(element, (XS, X)):
            out.append(f"x{element.bits}.{element.places}")
        elif isinstance(element, UX):
            out.append(f"ux{element.bits}.{element.places}")
        elif isinstance(element, (S, I)):
            out.append(f"i{element.bits}")
        elif isinstance(element, U):
            out.append(f"u{element.bits}")
        elif isinstance(element, NAME):
            out.append(f"n{element.bits}")
        elif isinstance(element, CODE):
            out.append(f"w{element.bits}")
        elif isinstance(element, STR):
            out.append(f"s{element.length_bits}")
        elif isinstance(element, CH):
            out.append(f"c{element.bits}")
        elif isinstance(element, HEX):
            out.append("u8")
        elif isinstance(element, MARKS):
            out.append(f"{element.count}*m{element.bits}")
        elif isinstance(element, CHARS):
            out.append(f"r{element.count_bits}*c{element.char_bits}")
        elif isinstance(element, REP):
            out.append(f"r{element.bits}*({fields(element.body)})")
        elif isinstance(element, FLAG):
            a, b = fields(element.then), fields(element.otherwise)
            out.append(f"f[{a}/{b}]" if (a or b) else "f")
        elif isinstance(element, WHEN):
            parts = [fields(t) for t in element.cases.values()]
            parts.append(fields(element.default))
            inner = "/".join(p for p in parts if p)
            if inner:
                out.append(f"({inner})")
    return " ".join(out)


def table(path: str = "") -> None:
    """Markdown: each kind's fields, and with a sim/offline output file its
    measured lines: count, mean text bytes (with the newline) and mean frame
    bytes."""
    measured = {}
    if path:
        lines = open(path, encoding="ascii", errors="replace").read().split("\n")
        for index, line in enumerate(lines):
            if not line.startswith("#F "):
                continue
            frame = bytes.fromhex(line[3:].strip())
            kind = Reader(frame, 16).read(KIND_BITS)
            text = lines[index + 1] if index + 1 < len(lines) else ""
            entry = measured.setdefault(kind, [0, 0, 0])
            entry[0] += 1
            entry[1] += len(text) + 1
            entry[2] += len(frame)
    print("| # | Kind | Fields after the 23-bit header | Lines | Text B | Frame B |")
    print("|---|---|---|---|---|---|")
    totals = [0, 0, 0]
    for number, (name, template) in enumerate(KINDS, 1):
        count, text, frame = measured.get(number, (0, 0, 0))
        totals = [totals[0] + count, totals[1] + text, totals[2] + frame]
        sizes = (f"{count} | {text / count:.1f} | {frame / count:.1f}"
                 if count else "- | - | -")
        print(f"| {number} | {name} | {fields(template) or 'none'} | {sizes} |")
    if totals[0]:
        print(f"| | all | | {totals[0]} | {totals[1]} total | {totals[2]} total"
              f" ({100.0 * totals[2] / totals[1]:.1f}%) |")


def main(argv) -> int:
    if len(argv) >= 2 and argv[1] == "selftest":
        return selftest()
    if len(argv) == 3 and argv[1] == "check":
        return check(argv[2])
    if len(argv) >= 2 and argv[1] == "table":
        table(argv[2] if len(argv) > 2 else "")
        return 0
    if len(argv) >= 3 and argv[1] == "decode":
        for item in argv[2:]:
            seq, name, text = decode(bytes.fromhex(item))
            print(f"{seq:3d} {name}: {text}")
        return 0
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
