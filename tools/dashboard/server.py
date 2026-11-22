#!/usr/bin/env python3
"""
IS29 car dashboard: a small web server that bridges the car's MQTT topics
to a browser page.

  pip install paho-mqtt
  python server.py --log ..\\..\\testing\\system\\logs\\2026-10-29_dashboard.txt
  python server.py --broker 127.0.0.1 --port 8080

Open http://localhost:8080 on the laptop, or http://<laptop IP>:8080 from a
phone on the same hotspot. The broker is the Docker container "mqtt"
(eclipse-mosquitto) on this laptop; this server never starts one.

Topics (config/net_mqtt_config.h), base inf2004/is29/:
  telemetry  car -> laptop. The bench image (firmware/bench/bench_main.c)
             sends plain text lines cut into 20-byte messages, so the server
             joins the messages and splits on newlines. A message the car
             could not queue is dropped, which glues two lines together; the
             parser splits those again at known line starts.
             The robot image (app_program) sends binary frames instead
             (app_program/subsystem1/telemetry.h), decoded with
             firmware/scripts/telemetry_codec.py.
  status     '1' when the car connects, '0' as its last will.
  command    laptop -> car, one raw byte (app_program/subsystem1/mqttcmd.h):
             0xAA START, 0xA9 STOP, 0xAB RESET, 0xAF next test, 0xAD next
             gain set. The robot image acts on STOP only.

Standard library + paho-mqtt only; the page has no external dependencies, so
it works on a hotspot without internet.
"""
import argparse
import importlib.util
import json
import math
import os
import queue
import re
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import paho.mqtt.client as mqtt

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))


def load_codec():
    """The repo's own binary telemetry codec, loaded by path (no sys.path
    change), or None if it is missing: then binary frames are only logged."""
    path = os.path.join(REPO, "firmware", "scripts", "telemetry_codec.py")
    if not os.path.isfile(path):
        return None
    spec = importlib.util.spec_from_file_location("telemetry_codec", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CODEC = load_codec()

# Command bytes: validity pattern 10101b in bits 3-7, code in bits 0-2.
CMDS = {"START": 0xAA, "STOP": 0xA9, "RESET": 0xAB,
        "NEXT_TEST": 0xAF, "NEXT_GAINS": 0xAD}
CMD_NAMES = {0xA9: "STOP", 0xAA: "START", 0xAB: "RESET",
             0xAC: "previous gains", 0xAD: "NEXT_GAINS",
             0xAE: "configured gains", 0xAF: "NEXT_TEST"}

# ---------------------------------------------------------------------------
# Bench text lines (firmware/bench/bench_main.c, bench_text/bench_flush)
# ---------------------------------------------------------------------------
GAINS = (r"gains (?P<g>\d+)(?P<cfg> \(configured\))? P (?P<P>-?\d+\.\d+)"
         r" I (?P<I>-?\d+\.\d+) D (?P<D>-?\d+\.\d+)")
SN = r"([+-]\d+)"                                   # bench_signed: +12 / -3
RANGE = SN + r" " + SN + r"\.\." + SN               # mean min..max
LINE_RES = [
    ("ir", re.compile(r"IR L (\d+) (\d+)-(\d+) R (\d+) (\d+)-(\d+)"
                      r" B (\d+) (\d+)-(\d+)")),
    ("acc", re.compile(r"ACC mg n (\d+)(?: x " + RANGE + r" y " + RANGE +
                       r" z " + RANGE + r")?")),
    ("enc", re.compile(r"ENC counts L " + SN + r" R " + SN + r" mm L " + SN +
                       r" R " + SN + r" err (\d+)")),
    ("mode", re.compile(r"MODE (?P<name>.+), " + GAINS)),
    ("trace", re.compile(r" (end )?(\d+) d " + SN + r" L (\d+) R (\d+) B (\d+)"
                         r" s " + SN + r" ([A-Z]+) h (-?\d+\.\d)"
                         r"(?: c ([A-Z]+))? w " + SN + r" " + SN +
                         r" u " + SN + r" " + SN + r"(?: f (none|\d+))?")),
    ("start", re.compile(r"(DRIVE|PID|SWEEP|SCAN) start\b.*")),
    ("end", re.compile(r"(?!MODE )(?P<head>.+?): " + GAINS +
                       r"(?P<rest>,.*)?")),
    # MISSION ends without gains: "MISSION <reason>[ after X s]: dt max X ms".
    ("mission_end", re.compile(r"(?P<head>MISSION .+?): "
                               r"(?P<rest>dt max \d+\.\d ms.*)")),
    ("sweep_end", re.compile(r"SWEEP (done|aborted): .*")),
    ("scan_end", re.compile(r"SCAN (.+?) after (\d+\.\d) s: .*")),
    ("centre", re.compile(r" centre: (-?\d+) mm in (\d+\.\d) s, mean (-?\d+)"
                          r" mm/s; time % per phase TRACK (\d+) GAP (\d+)"
                          r" SEARCH (\d+) CROSS (\d+), full steer (\d+)")),
    ("gaps", re.compile(r" gaps (\d+), crossings (\d+), searches (\d+)"
                        r" \(longest (\d+) ms\), darkest raw L (\d+) R (\d+)")),
    ("junctions", re.compile(r" junctions crossed straight on (\d+)")),
    # Live map (bench MISSION, obstacle image): odometry pose every 0.5 s,
    # the obstacle the sonar measured and the map rows its beams hit.
    # Since 6 Nov the pose may carry the compass's raw X Y Z: " C x y z".
    ("pose", re.compile(r" P (-?\d+) (-?\d+) (-?\d+)(?: C [+-]?\d+ [+-]?\d+ [+-]?\d+)?")),
    # Sonar sweep while the car measures an obstacle: the pivot back mm behind
    # the first stop, then deg (from the line heading, + left) / range mm
    # (0: no echo) per beam (bench_beams_text).
    ("beams", re.compile(r" BEAMS back (-?\d+):((?: -?\d+/\d+)+)(?: \(\+\d+ not kept\))?")),
    ("obst", re.compile(r" OBST at (-?\d+) (-?\d+) h (-?\d+\.\d) face (-?\d+)"
                        r" left (-?\d+)(\??) right (-?\d+)(\??) depth (\d+)"
                        r" go (R|L|none) out (-?\d+) along (-?\d+)"
                        r"(?: back (-?\d+))?")),
    ("maprow", re.compile(r" M (-?\d+) ([-#+.|C]{41})")),
    # The side sensor's stored samples of a cut (bench_raw_text): a header,
    # then rows of one character per decoder unit ('!' no reading).
    ("rawhead", re.compile(r" RAW (\d+): u(\d+) \(side (-?\d+) mm\), (\d+) units,"
                           r" white (\d+) sd (\d+);.*")),
    ("rawrow", re.compile(r" R (\d+): ([!-~]+)")),
]
# Run end head: "<mode> <reason>[ after X s | , complete after X s]".
HEAD_RE = re.compile(r"(?P<mode>.*?) (?P<reason>(?:aborted|done|complete|"
                     r"stopped)(?:[: ].*?)?)"
                     r"(?:,? (?:complete )?after (?P<secs>\d+\.\d) s)?")
DT_RE = re.compile(r"dt max (\d+\.\d) ms")
# Where a line can begin; used to split two lines glued by a lost message.
SPLIT_RE = re.compile(
    r"IR L \d|ACC mg n \d|ENC counts L [+-]|MODE \S|"
    r"(?:DRIVE|PID|SWEEP|SCAN) start|AUTO-CENTRE |LINE |MISSION |DIST 1 m|"
    r"LEFT 90|"
    r"RIGHT 90|TURN 180|BARCODE|SCAN |SWEEP |BENCH | at completion:|"
    r" 1 s later:| at the stop:| centre: | gaps \d| junctions crossed|"
    r" trace t ms| w measured| f front| end \d| \d+ d [+-]|"
    r" P -?\d+ -?\d+ -?\d+(?: C [+-]?\d+ [+-]?\d+ [+-]?\d+)?$| OBST at |"
    r" M -?\d+ [-#+.|C]| BEAMS back | RAW \d+: u\d| R \d+: |"
    r" NAV: |MAP set:| RAMP \d| SIDE \d| TERRAIN | ramp: | CAR \*|"
    r"(?:AVOID|MISSION): ")
PERIODIC_RE = re.compile(r"IR L|ACC mg|ENC counts|ENC |ACC |IR |err \d|"
                         r"\s*(?:end )?\d+ d [+-]")
PERIODIC_IN_RE = re.compile(r"ACC mg n|ENC counts L|IR L \d+ \d+-\d+|"
                            r"mm L [+-]\d+ R [+-]\d+ err \d|"
                            r"\d\.\.[+-]\d|\d+-\d+ [RB] \d")


def ints(groups):
    return [None if g is None else int(g) for g in groups]


# ---------------------------------------------------------------------------
# Mission lines (bench MISSION, navvote2): barcodes, junction moves, humps,
# obstacles. They stay event lines in the log; these only feed the Mission
# card. Formats from firmware/bench/bench_main.c (bench_nav_*, bench_ramp_text,
# bench_avoid_*).
# ---------------------------------------------------------------------------
MISSION_RES = [
    # NAV: card 431-572 mm (CURRENT Z, NEW ?, D0 Z): Z, at the next junction: RIGHT
    ("card", re.compile(r"\s*NAV: card (\d+)-(\d+) mm \(([^)]*)\): (?:([A-Z]), at the"
                        r" next junction: (.+)|(no code yet.*))")),
    # NAV: junction 1 at 812 mm: card (...) code Z -> RIGHT, advancing ...
    ("junction", re.compile(r"\s*NAV: junction (\d+) at (-?\d+) mm: (.*)")),
    ("turned", re.compile(r"\s*NAV: turned ([+-]?\d+) deg.*")),
    ("restored", re.compile(r"\s*NAV: map restored.*")),
    # CAR *F* (CURRENT none, NEW *F*, D0 none) at 77-355 mm  (after STOP)
    ("car", re.compile(r"\s*CAR \*([A-Z])\* \(([^)]*)\) at (\d+)-(\d+) mm")),
    ("navmap", re.compile(r"\s*MAP set:(.*)")),
    ("ramp", re.compile(r"\s*RAMP (\d+): peak (\d+) mm at (-?\d+) mm.*")),
    ("side", re.compile(r"\s*SIDE (\d+): (left|right) wheel up (\d+) mm"
                        r"(?:.*? at (-?\d+) mm)?.*")),
    ("terrain", re.compile(r"\s*TERRAIN hump counted: peak (\d+) mm.*")),
    ("highest", re.compile(r"\s*ramp: highest (\d+) mm(?:.*? at (-?\d+) mm)?.*")),
    ("stop", re.compile(r"\s*(?:MISSION|AVOID): obstacle at.*")),
    ("around", re.compile(r"\s*AVOID: going round the (left|right).*")),
    ("rejoin", re.compile(r"\s*AVOID: back on course.*")),
]
MOVE_NAMES = {"forward": "F", "u-turn": "U", "left": "L", "right": "R"}
# Lines only a driving car prints: one arriving well after a run's end
# belongs to the next run, whose start line was lost.
MISSION_LIVE = {"card", "junction", "turned", "ramp", "side", "terrain",
                "stop", "around", "rejoin"}


def mission_line(text):
    """(kind, data) for a mission line, else None."""
    for kind, rx in MISSION_RES:
        m = rx.fullmatch(text)
        if not m:
            continue
        g = m.groups()
        if kind == "card":
            return kind, {"from": int(g[0]), "to": int(g[1]), "votes": g[2],
                          "code": g[3], "move": (g[4] or "").strip() or None}
        if kind == "junction":
            rest = g[2]
            mv = re.search(r"code ([A-Z]) -> ([A-Z-]+)", rest)
            return kind, {"n": int(g[0]), "mm": int(g[1]),
                          "code": mv.group(1) if mv else None,
                          "move": mv.group(2) if mv else "STRAIGHT",
                          "why": None if mv else rest.split(",")[0].strip()}
        if kind == "turned":
            return kind, int(g[0])
        if kind == "car":
            return kind, {"code": g[0], "votes": g[1], "from": int(g[2]),
                          "to": int(g[3])}
        if kind == "navmap":
            pairs = re.findall(r"(forward|U-turn|left|right) ([A-Z])\b", g[0])
            if not pairs:
                return None
            return kind, {MOVE_NAMES[k.lower()]: v for k, v in pairs}
        if kind == "ramp":
            return kind, {"n": int(g[0]), "peak": int(g[1]), "mm": int(g[2]),
                          "how": "pitch"}
        if kind == "side":
            return kind, {"n": int(g[0]), "peak": int(g[2]),
                          "mm": int(g[3]) if g[3] else None,
                          "how": g[1] + " wheel"}
        if kind == "terrain":
            return kind, {"peak": int(g[0])}
        if kind == "highest":
            return kind, {"peak": int(g[0]), "mm": int(g[1]) if g[1] else None}
        if kind == "around":
            return kind, g[0]
        return kind, text.strip()
    return None


def new_mission(run_id, now):
    """What the current run did against the mission requirements."""
    return {"run": run_id, "t0": now, "codes": [], "held": None,
            "junctions": [], "humps": [], "highest": None, "stops": 0,
            "profiled": 0, "around": 0, "rejoined": 0, "timeline": [],
            "cards": [], "raws": []}


# ---------------------------------------------------------------------------
# Barcode cards (bench MISSION): each decoder's result per card, D0's bar and
# space widths (decoder units), and the side sensor's raw samples, for the
# Barcodes panel. Formats from bench_d0_text, bench_code_*, bench_fit_text and
# bench_vote_text in firmware/bench/bench_main.c.
# ---------------------------------------------------------------------------
BARCODE_RES = [
    # D0 *F* read in order at 210-486 mm
    ("d0", re.compile(r"\s*D0 \*([^ ]*)\* read (in order|end to start) at (\d+)-(\d+) mm")),
    # D0 9 bars, not a Code 39 count at 1412-1578 mm (or "no frame: ...")
    ("d0none", re.compile(r"\s*D0 (\d+) bars, (.+?) at (\d+)-(\d+) mm")),
    # D0 widths u: 17 57 17 ... (bar, space, bar, ... from the first bar)
    ("d0widths", re.compile(r"\s*D0 widths u:((?: \d+)+)")),
    # CODE *Q* read end to start at 1227-1504 mm (swing 4.0 sd, gap 30 mm)
    ("cur", re.compile(r"\s*CODE \*([^ ]*)\* read (in order|end to start) at (\d+)-(\d+) mm.*")),
    # GROUP 21 bars, not a Code 39 count, characters WF4 / E0Q at 664-938 mm (...)
    ("curnone", re.compile(r"\s*GROUP (\d+) bars, (.+?) at (\d+)-(\d+) mm.*")),
    # Decoder 2 (the model fit; "NEW" before 19 Nov 2026):
    # Decoder 2 *F*, margin 1.86 over *P*; module ...; at 663-940 mm; 2932 ms, ...
    ("new", re.compile(r"\s*(?:NEW|Decoder 2) \*(.)\*( rev)?, margin (\d+\.\d+) over \*(.)\*.*?; at (\d+)-(\d+) mm.*")),
    # Decoder 2 no read: best *1*, margin 1.00 over *A*; ...; at 2067-2273 mm; ...
    # Decoder 2 skipped: still fitting the last card; at 3085-3361 mm
    ("newnone", re.compile(r"\s*(?:NEW|Decoder 2) (no read|skipped): (.*?); at (\d+)-(\d+) mm.*")),
    # CAR *A* (Decoder 1 *A*, Decoder 2 none, D0 no read) at 2067-2273 mm
    # CAR no code (Decoder 1 none, Decoder 2 no read, D0 none) at 1136-1231 mm
    # (CURRENT / NEW for Decoder 1 / Decoder 2 before 19 Nov 2026)
    ("car", re.compile(r"\s*CAR (?:\*([^ ]*)\*|no code) \(([^)]*)\) at (\d+)-(\d+) mm")),
    # NAV: card 431-572 mm (...): Z, at the next junction: RIGHT
    ("nav", re.compile(r"\s*NAV: card (\d+)-(\d+) mm \([^)]*\): ([A-Z]), at the"
                       r" next junction: (.+)")),
]
CARDS_MAX = 10
RAWS_MAX = 6
CARD_SLACK_MM = 25        # decoders place the same card a few mm apart


def barcode_line(text):
    """(kind, data) for a barcode line, else None."""
    for kind, rx in BARCODE_RES:
        m = rx.fullmatch(text)
        if not m:
            continue
        g = m.groups()
        if kind == "d0":
            return kind, {"from": int(g[2]), "to": int(g[3]),
                          "res": {"text": g[0], "rev": g[1] != "in order"}}
        if kind == "d0none":
            return "d0", {"from": int(g[2]), "to": int(g[3]),
                          "res": {"text": None, "note": f"{g[0]} bars, {g[1]}"}}
        if kind == "d0widths":
            return kind, [int(v) for v in g[0].split()]
        if kind == "cur":
            return kind, {"from": int(g[2]), "to": int(g[3]),
                          "res": {"text": g[0], "rev": g[1] != "in order"}}
        if kind == "curnone":
            return "cur", {"from": int(g[2]), "to": int(g[3]),
                           "res": {"text": None, "note": f"{g[0]} bars, {g[1]}"}}
        if kind == "new":
            return kind, {"from": int(g[4]), "to": int(g[5]),
                          "res": {"text": g[0], "rev": bool(g[1]),
                                  "margin": float(g[2]), "other": g[3]}}
        if kind == "newnone":
            note = g[1].split(";")[0]
            return "new", {"from": int(g[2]), "to": int(g[3]),
                           "res": {"text": None,
                                   "note": (g[0] + ": " + note) if g[0] == "skipped"
                                   else "no read: " + note}}
        if kind == "car":
            return kind, {"from": int(g[2]), "to": int(g[3]),
                          "res": {"text": g[0], "votes": g[1]}}
        if kind == "nav":
            return kind, {"from": int(g[0]), "to": int(g[1]),
                          "res": {"letter": g[2], "move": g[3].strip()}}
    return None


def card_letter(c):
    """The card's answer so far: the vote, else the NAV letter, else any
    decoder's read (the page's cardLetter does the same)."""
    for k, field in (("car", "text"), ("nav", "letter"), ("d0", "text"),
                     ("cur", "text"), ("new", "text")):
        if c.get(k) and c[k].get(field):
            return c[k][field]
    return None


def match_full(s):
    """(kind, data) for a complete known line, else None."""
    for kind, rx in LINE_RES:
        m = rx.fullmatch(s)
        if not m:
            continue
        g = m.groups()
        if kind == "ir":
            v = ints(g)
            return kind, {"L": v[0:3], "R": v[3:6], "B": v[6:9]}
        if kind == "acc":
            v = ints(g)
            return kind, {"n": v[0], "x": v[1:4] if v[1] is not None else None,
                          "y": v[4:7] if v[1] is not None else None,
                          "z": v[7:10] if v[1] is not None else None}
        if kind == "enc":
            v = ints(g)
            return kind, {"counts": v[0:2], "mm": v[2:4], "err": v[4]}
        if kind == "mode":
            return kind, gains_of(m, name=m.group("name"))
        if kind == "trace":
            row = [int(g[1]), int(g[2]), int(g[3]), int(g[4]), int(g[5]),
                   int(g[6]), g[7], float(g[8]), g[9], int(g[10]),
                   int(g[11]), int(g[12]), int(g[13]),
                   None if g[14] in (None, "none") else int(g[14])]
            return kind, {"end": bool(g[0]), "row": row}
        if kind == "end":
            if PERIODIC_RE.match(s) or PERIODIC_IN_RE.search(m.group("head")):
                continue                    # A cut line glued in front
            d = gains_of(m)
            head = HEAD_RE.fullmatch(m.group("head"))
            d["mode"] = head.group("mode") if head else m.group("head")
            d["reason"] = head.group("reason") if head else ""
            d["secs"] = float(head.group("secs")) if head and head.group("secs") else None
            rest = m.group("rest") or ""
            dt = DT_RE.search(rest)
            d["dt_max"] = float(dt.group(1)) if dt else None
            d["enc_errors"] = "ENCODER ERRORS" in rest
            return kind, d
        if kind == "mission_end":
            head = HEAD_RE.fullmatch(m.group("head"))
            rest = m.group("rest")
            dt = DT_RE.search(rest)
            return "end", {
                "mode": head.group("mode") if head else "MISSION",
                "reason": (head.group("reason") if head
                           else m.group("head")[len("MISSION "):]),
                "secs": (float(head.group("secs"))
                         if head and head.group("secs") else None),
                "dt_max": float(dt.group(1)) if dt else None,
                "enc_errors": "ENCODER ERRORS" in rest}
        if kind == "sweep_end":
            return "end", {"mode": "SWEEP", "reason": g[0], "secs": None}
        if kind == "scan_end":
            return "end", {"mode": "SCAN", "reason": g[0], "secs": float(g[1])}
        if kind == "centre":
            return kind, {"mm": int(g[0]), "secs": float(g[1]),
                          "mean": int(g[2]),
                          "phase": {"TRACK": int(g[3]), "GAP": int(g[4]),
                                    "SEARCH": int(g[5]), "CROSS": int(g[6])},
                          "full_steer": int(g[7])}
        if kind == "gaps":
            v = ints(g)
            return kind, {"gaps": v[0], "crossings": v[1], "searches": v[2],
                          "longest_ms": v[3], "dark_L": v[4], "dark_R": v[5]}
        if kind == "junctions":
            return kind, int(g[0])
        if kind == "pose":
            return kind, [int(g[0]), int(g[1]), int(g[2])]
        if kind == "obst":
            return kind, {"x0": int(g[0]), "y0": int(g[1]), "h0": float(g[2]),
                          "face": int(g[3]), "left": int(g[4]),
                          "left_open": g[5] == "?", "right": int(g[6]),
                          "right_open": g[7] == "?", "depth": int(g[8]),
                          "go": g[9], "out": int(g[10]), "along": int(g[11]),
                          "back": int(g[12] or 0)}
        if kind == "maprow":
            return kind, [int(g[0]), g[1]]
        if kind == "beams":
            back = int(g[0])
            return kind, [[back, int(a), int(r)] for a, r in
                          re.findall(r"(-?\d+)/(\d+)", g[1])]
        if kind == "rawhead":
            return kind, {"cut": int(g[0]), "u": int(g[1]), "start": int(g[2]),
                          "units": int(g[3]), "white": int(g[4]),
                          "sd": int(g[5]), "text": s}
        if kind == "rawrow":
            return kind, {"cut": int(g[0]), "data": g[1]}
        return kind, s                              # start
    return None


def gains_of(m, **extra):
    return dict(extra, g=int(m.group("g")), cfg=bool(m.group("cfg")),
                P=m.group("P"), I=m.group("I"), D=m.group("D"))


def parse_line(s):
    """Split one received text line into [(kind, data)]. Kinds: ir, acc,
    enc, mode, trace, start, end, centre, gaps, junctions, event (any other
    line, kept verbatim) and garbled (cut by a lost message)."""
    s = s.rstrip("\r")
    if not s.strip():
        return []
    r = match_full(s)
    if r:
        return [r]
    # Two lines glued: a known line start inside, with a complete line after.
    for m in SPLIT_RE.finditer(s, 1):
        if m.start() == 0:
            continue
        r = match_full(s[m.start():])
        if r:
            return [("garbled", s[:m.start()]), r]
    if not re.search(r"[A-Za-z]{2}", s):
        return [("garbled", s)]            # A bare fragment such as "-180"
    if PERIODIC_RE.match(s) or PERIODIC_IN_RE.search(s):
        # A periodic line cut short, maybe with an event line after it.
        for m in SPLIT_RE.finditer(s, 1):
            tail = s[m.start():]
            if m.start() > 0 and not PERIODIC_RE.match(tail):
                return [("garbled", s[:m.start()]), ("event", tail)]
        return [("garbled", s)]
    return [("event", s)]


def is_text(payload):
    return all(b in (9, 10, 13) or 32 <= b < 127 for b in payload)


def event_class(text):
    up = text.upper()
    if "FAILED" in up or "FAULT" in up or "ERROR" in up or "REFUSED" in up:
        return "bad"
    return ""


class Hub:
    """Latest state + history, and the list of browsers to push to."""

    def __init__(self, base, log_path=None):
        self.base = base.rstrip("/") + "/"
        self.lock = threading.RLock()
        self.parse_lock = threading.Lock()
        self.clients = []                  # one queue per open page
        self.mqtt_ok = False
        self.last_tel = 0.0                # any message on the telemetry topic
        self.presence = None               # True '1', False '0' (last will)
        self.live = {}                     # ir / acc / enc / robot frames
        self.ir_hist = deque(maxlen=1200)  # [t, L, R, B]
        self.mode = None                   # last MODE line
        self.runs = deque(maxlen=12)
        self.run_seq = 0
        self.events = deque(maxlen=600)
        self.stats = {"messages": 0, "lines": 0, "garbled": 0, "frames": 0}
        self.buf = ""
        self.buf_t = 0.0
        self.robot_state = None
        self.robot_faults = None
        self.map = new_map(None, 0.0)      # live map of the current run
        self.mission = new_mission(None, 0.0)
        self.navmap = None                 # the car's last "MAP set:" answer
        self.log = None
        if log_path:
            os.makedirs(os.path.dirname(os.path.abspath(log_path)), exist_ok=True)
            self.log = open(log_path, "a", encoding="utf-8")
            self.write_log(f"# dashboard start {time.strftime('%Y-%m-%d')}, "
                           f"topics {self.base}#")

    # -- page side -----------------------------------------------------------
    def snapshot(self):
        with self.lock:
            return {"type": "snapshot", "now": time.time(),
                    "mqtt_ok": self.mqtt_ok, "base": self.base,
                    "last_tel": self.last_tel, "presence": self.presence,
                    "live": self.live,
                    "ir_hist": [h for h in self.ir_hist
                                if h[0] > time.time() - 90],
                    "mode": self.mode,
                    # Trace rows only for the latest run; /api/run?id=N
                    # gives an older one.
                    "runs": [run_meta(r) for r in self.runs],
                    "last_run": self.runs[-1] if self.runs else None,
                    "events": list(self.events), "stats": self.stats,
                    "map": self.map,
                    "mission": self.mission_payload(),
                    "codec": CODEC is not None}

    def get_run(self, run_id):
        with self.lock:
            for r in self.runs:
                if r["id"] == run_id:
                    return json.loads(json.dumps(r))
        return None

    def push(self, item):
        item["now"] = time.time()
        item["last_tel"] = self.last_tel
        with self.lock:
            item["stats"] = dict(self.stats)
        data = json.dumps(item)
        with self.lock:
            for q in list(self.clients):
                try:
                    q.put_nowait(data)
                except queue.Full:
                    pass                   # Slow page: it gets the next one

    def write_log(self, line):
        if self.log:
            t = time.time()
            self.log.write(time.strftime("%H:%M:%S", time.localtime(t)) +
                           f".{int(t * 1000) % 1000:03d} {line}\n")
            self.log.flush()

    def event(self, now, text, cls=""):
        item = {"host_t": now, "text": text, "cls": cls}
        with self.lock:
            self.events.append(item)
        self.push({"type": "event", **item})

    # -- MQTT ----------------------------------------------------------------
    def on_message(self, topic, payload, retained=False):
        now = time.time()
        sub = topic[len(self.base):] if topic.startswith(self.base) else topic
        with self.lock:
            self.stats["messages"] += 1
        if sub == "telemetry":
            with self.lock:
                self.last_tel = now
            self.on_telemetry(payload, now)
        elif sub == "status":
            text = payload.decode(errors="replace").strip()
            online = text == "1"
            with self.lock:
                self.presence = online
            self.write_log(f"[status] {text}")
            self.push({"type": "presence", "online": online})
            self.event(now, "car connected to the broker" if online else
                       "car offline (its last will)", "ok" if online else "bad")
        elif sub == "command":
            name = (CMD_NAMES.get(payload[0], "unknown") if len(payload) == 1
                    else f"{len(payload)} bytes, not a command")
            text = (f"command {name} (0x{payload.hex().upper()}) on the broker"
                    + (", retained: the car ignores it" if retained else ""))
            self.write_log(f"[command] {text}")
            self.event(now, text, "sent")

    def on_telemetry(self, payload, now):
        if not is_text(payload) and CODEC is not None:
            try:
                name, values = CODEC.decode(bytes(payload))
                self.on_frame(name, CODEC.describe(name, values), now)
                return
            except ValueError:
                pass                       # Not a frame: treat it as text
        text = payload.decode("ascii", errors="replace")
        with self.parse_lock:
            self.buf += text
            self.buf_t = now
            *lines, self.buf = self.buf.split("\n")
            if len(self.buf) > 600:        # No newline for far too long
                lines.append(self.buf)
                self.buf = ""
            for line in lines:
                self.on_line(line, now)

    def flush_idle(self):
        """A line whose last message (with the newline) was lost would sit
        in the buffer until the car sends again; let it out after 3 s."""
        while True:
            time.sleep(0.5)
            with self.parse_lock:
                if self.buf and time.time() - self.buf_t > 3.0:
                    line, self.buf = self.buf, ""
                    self.on_line(line, time.time())

    def on_frame(self, name, d, now):
        with self.lock:
            self.stats["frames"] += 1
            robot = self.live.setdefault("robot", {})
            robot[name] = dict(d, t=now)
        self.write_log(f"[frame {name}] {json.dumps(d)}")
        self.push({"type": "frame", "name": name, "data": dict(d, t=now)})
        if name == "status" and d.get("state") != self.robot_state:
            self.event(now, f"robot state {self.robot_state or '?'} -> "
                       f"{d.get('state')}", "bad" if d.get("state") == "FAULT" else "")
            self.robot_state = d.get("state")
        if name == "diagnostic" and d.get("faults") != self.robot_faults:
            self.robot_faults = d.get("faults")
            self.event(now, "robot faults: " + (", ".join(self.robot_faults)
                                                or "none"),
                       "bad" if self.robot_faults else "ok")

    # -- one text line -------------------------------------------------------
    def on_line(self, line, now):
        if not line.strip():
            return
        self.write_log(line.rstrip("\r"))
        with self.lock:
            self.stats["lines"] += 1
        for kind, data in parse_line(line):
            getattr(self, "on_" + kind)(data, now)

    def on_ir(self, d, now):
        self.periodic("ir", d, now)
        with self.lock:
            self.ir_hist.append([round(now, 2), d["L"][0], d["R"][0], d["B"][0]])

    def on_acc(self, d, now):
        self.periodic("acc", d, now)

    def on_enc(self, d, now):
        self.periodic("enc", d, now)

    def periodic(self, kind, d, now):
        d = dict(d, t=now)
        cleared = None
        with self.lock:
            self.live[kind] = d
            run = self.runs[-1] if self.runs else None
            # The bench prints these only while idle: a run is over.
            if run and run["state"] == "running" and now - run["t0"] > 2.0:
                run.update(state="ended", reason="end line not received",
                           t_end=now)
                self.push({"type": "run", "data": run_meta(run)})
            # Idle, so the run is over: mark its map ended, once. It stays on
            # show until the next START (it used to be cleared here, so the
            # map was blank just when the run could be looked at).
            m = self.map
            if (not m.get("ended") and (m["run"] is not None or m["path"]
                                        or m["obst"] or m["marks"])
                    and now - m["t0"] > 2.0):
                m["ended"] = {"run": m["run"],
                              "reason": run["reason"] if run else None}
                m["sweep"] = None
                cleared = json.loads(json.dumps(m))
        if cleared:
            self.push({"type": "map", "data": cleared})
        self.push({"type": kind, "data": d})

    def on_mode(self, d, now):
        d = dict(d, t=now)
        with self.lock:
            self.mode = d
        self.push({"type": "mode", "data": d})
        self.event(now, "MODE " + d["name"] + ", gains " + str(d["g"]) +
                   (" (configured)" if d["cfg"] else "") +
                   f" P {d['P']} I {d['I']} D {d['D']}", "mode")

    def new_run(self, now, mode=None, gains=None):
        with self.lock:
            self.run_seq += 1
            m = self.mode or {}
            run = {"id": self.run_seq, "t0": now, "t_end": None,
                   "mode": mode or m.get("name"), "state": "running",
                   "gains": gains or ({k: m[k] for k in ("g", "cfg", "P", "I", "D")}
                                      if m else None),
                   "reason": None, "secs": None, "dt_max": None,
                   "enc_errors": False, "lines": [], "centre": None,
                   "gaps": None, "junctions": None, "trace": [],
                   "trace_end": None}
            self.runs.append(run)
            return run

    def on_start(self, text, now):
        run = self.new_run(now)
        run["lines"].append(text)
        with self.lock:
            self.map = new_map(run["id"], now)
            snap = json.loads(json.dumps(self.map))
            self.mission = new_mission(run["id"], now)
            self.mission_note(now, "start", "Run started", "ok")
        self.push({"type": "map", "data": snap})
        self.push({"type": "run", "data": run_meta(run)})
        self.push_mission()
        self.event(now, text, "ok")

    # -- mission -------------------------------------------------------------
    def mission_payload(self):
        with self.lock:
            return json.loads(json.dumps(dict(self.mission, navmap=self.navmap)))

    def push_mission(self):
        self.push({"type": "mission", "data": self.mission_payload()})

    def mission_note(self, now, kind, text, tone=""):
        tl = self.mission["timeline"]
        tl.append({"t": now, "kind": kind, "text": text, "tone": tone})
        del tl[:-80]

    def mission_code(self, d, now):
        """A card's letter: NAV card lines repeat as more decoders report,
        and the CAR line after STOP repeats it again: one entry per card."""
        codes = self.mission["codes"]
        for c in codes:
            if abs(c["from"] - d["from"]) <= 60:
                c.update(code=d["code"], to=d["to"])
                if d.get("move"):
                    c["move"] = d["move"]
                return
        codes.append({"code": d["code"], "from": d["from"], "to": d["to"],
                      "move": d.get("move"), "t": now})
        then = f" → {d['move']} at the next junction" if d.get("move") else ""
        self.mission_note(now, "card", f"Card {d['code']} read at "
                          f"{d['from']}–{d['to']} mm{then}", "info")

    def on_mission_text(self, text, now):
        r = mission_line(text)
        if not r:
            return
        kind, d = r
        with self.lock:
            if (kind in MISSION_LIVE and "end" in self.mission
                    and now - self.mission["end_t"] > 3.0):
                self.mission = new_mission(None, now)
                self.mission_note(now, "start", "Run started (its start line was lost)", "ok")
            ms = self.mission
            if kind == "navmap":
                self.navmap = dict(d, t=now)
            elif kind in ("card", "car"):
                if d["code"]:
                    self.mission_code(d, now)
                    if kind == "card":
                        ms["held"] = {"code": d["code"], "move": d["move"]}
            elif kind == "junction":
                ms["junctions"].append(dict(d, turned=None, t=now))
                ms["held"] = None
                if d["code"]:
                    self.mission_note(now, "junction", f"Junction {d['n']} at "
                                      f"{d['mm']} mm: {d['code']} → {d['move']}", "ok")
                else:
                    self.mission_note(now, "junction", f"Junction {d['n']} at "
                                      f"{d['mm']} mm: straight on ({d['why']})")
            elif kind == "turned":
                if ms["junctions"]:
                    ms["junctions"][-1]["turned"] = d
                self.mission_note(now, "turn", f"Turned {d:+d}° on the crossing", "ok")
            elif kind == "restored":
                self.mission_note(now, "warn", "Junction map was wiped and restored "
                                  "from its backup", "warn")
            elif kind in ("ramp", "side", "terrain"):
                recent = [h for h in ms["humps"] if now - h["t"] < 1.5]
                if kind == "terrain" and recent:
                    return                 # The module's copy of the same hump
                ms["humps"].append(dict(d, t=now))
                ms["highest"] = max(ms["highest"] or 0, d["peak"])
                where = f" at {d['mm']} mm" if d.get("mm") is not None else ""
                self.mission_note(now, "hump", f"Hump {d['peak']} mm{where}"
                                  f" ({d.get('how', 'IMU module')})", "info")
            elif kind == "highest":
                ms["ramp_report"] = d["peak"]
                if d["peak"] > 0:
                    ms["highest"] = max(ms["highest"] or 0, d["peak"])
            elif kind == "stop":
                last = ms.get("stop_t", 0.0)
                ms["stop_t"] = now
                if now - last > 1.5:
                    ms["stops"] += 1
                    self.mission_note(now, "obstacle", "Obstacle ahead: stopped", "warn")
            elif kind == "around":
                ms["around"] += 1
                self.mission_note(now, "obstacle", f"Going round the {d}", "info")
            elif kind == "rejoin":
                ms["rejoined"] += 1
                self.mission_note(now, "line", "Back on the line after the obstacle", "ok")
        self.push_mission()
        # On the map, where along the path they were.
        if kind == "junction":
            self.pin(d["mm"], "junction",
                     f"J{d['n']} " + (f"{d['code']} → {d['move']}" if d["code"]
                                       else "straight on"), f"j{d['n']}")
        elif kind in ("ramp", "side") and d.get("mm") is not None:
            self.pin(d["mm"], "hump", f"hump {d['peak']} mm", f"h{kind}{d['n']}")

    # -- barcode cards ---------------------------------------------------------
    def card_at(self, lo, hi, now):
        """The card a decoder's report for lo..hi mm belongs to: the newest
        one it overlaps (within CARD_SLACK_MM), else a new one."""
        cards = self.mission.setdefault("cards", [])
        for c in reversed(cards):
            if lo <= c["to"] + CARD_SLACK_MM and c["from"] <= hi + CARD_SLACK_MM:
                c["from"], c["to"] = min(c["from"], lo), max(c["to"], hi)
                return c
        c = {"from": lo, "to": hi, "t": now, "d0": None, "cur": None,
             "new": None, "car": None, "nav": None}
        cards.append(c)
        del cards[:-CARDS_MAX]
        return c

    def on_barcode_text(self, text, now):
        r = barcode_line(text)
        if not r:
            return
        kind, d = r
        letter = None
        with self.lock:
            if kind == "d0widths":
                c = getattr(self, "d0_card", None)
                if c is None or c.get("d0") is None:
                    return                 # Its D0 line was lost
                c["d0"]["widths"] = d
                self.d0_card = None
            else:
                c = self.card_at(d["from"], d["to"], now)
                c[kind] = dict(d["res"], at=[d["from"], d["to"]])
                if kind == "d0":
                    self.d0_card = c
                letter = card_letter(c)
                if letter:
                    c["key"] = c.get("key") or f"card{len(self.mission['cards'])}-{c['from']}"
                    rev = any(c.get(k) and c[k].get("text") == letter and c[k].get("rev")
                              for k in ("d0", "cur", "new"))
                    pin = ((c["from"] + c["to"]) / 2, letter, c["key"],
                           {"len": c["to"] - c["from"], "rev": rev})
        self.push_mission()
        if kind != "d0widths" and letter:
            self.pin(pin[0], "card", pin[1], pin[2], pin[3])

    def on_rawhead(self, d, now):
        with self.lock:
            raws = self.mission.setdefault("raws", [])
            raws.append({"start": d["start"], "u": d["u"], "units": d["units"],
                         "white": d["white"], "sd": d["sd"], "data": ""})
            del raws[:-RAWS_MAX]
        self.event(now, d["text"], "per")

    def on_rawrow(self, d, now):
        with self.lock:
            raws = self.mission.get("raws") or []
            if not raws or len(raws[-1]["data"]) >= raws[-1]["units"]:
                return                     # Its header was lost
            raw = raws[-1]
            raw["data"] = (raw["data"] + d["data"])[:raw["units"]]
            raw["rows"] = raw.get("rows", 0) + 1
            push = len(raw["data"]) >= raw["units"] or raw["rows"] % 4 == 0
        if push:
            self.push_mission()            # Every 4th row, and at the end

    # -- live map --------------------------------------------------------------
    def on_pose(self, p, now):
        fresh = None
        with self.lock:
            if self.map.get("ended"):
                # A pose after the run ended: the next run, whose start line
                # was lost. Start its own map, not a line back to the origin.
                self.map = new_map(None, now)
                fresh = json.loads(json.dumps(self.map))
        if fresh:
            self.push({"type": "map", "data": fresh})
        with self.lock:
            path = self.map["path"]
            if len(path) >= MAP_PATH_MAX:
                del path[: MAP_PATH_MAX // 10]
            # [x, y, deg, 1 while measuring or going round an obstacle]:
            # off the line there, so the 3D map draws no tape for it.
            p = list(p[:3]) + [1 if self.map.get("enc") in ("measuring", "around", "none") else 0]
            path.append(p)
            sw = self.map["sweep"]
            # The stop line comes before the car's pose at the stop: the
            # sweep's frame follows the poses until its first beams.
            moved = sw is not None and not sw["fixed"]
            if moved:
                sw.update(x0=p[0], y0=p[1], h0=float(p[2]))
        self.push({"type": "pose", "data": p})
        if moved:
            self.push_sweep()

    # -- obstacle encounter: the sweep while measuring, then the go-around --
    def push_sweep(self):
        with self.lock:
            data = json.loads(json.dumps({"sweep": self.map["sweep"],
                                          "enc": self.map["enc"]}))
        self.push({"type": "sweep", "data": data})

    def new_sweep(self, now):
        x, y, h = self.map["path"][-1][:3] if self.map["path"] else (0, 0, 0)
        return {"x0": x, "y0": y, "h0": float(h), "fixed": False, "beams": [],
                "est": None, "t": now}

    def on_beams(self, beams, now):
        with self.lock:
            # Beams after OBST (the creep and the step out) are in the
            # mapped obstacle's frame: they go with it.
            if (self.map["sweep"] is None and self.map["obst"]
                    and self.map["enc"] in ("around", "none")):
                o = self.map["obst"][-1]
                o["beams"].extend(beams)
                del o["beams"][:-400]
                self.push({"type": "obstbeams", "data": beams})
                return
            if self.map["sweep"] is None:
                self.map["sweep"] = self.new_sweep(now)
                self.map["enc"] = "measuring"
            sw = self.map["sweep"]
            sw["fixed"] = True
            sw["beams"].extend(beams)
            del sw["beams"][:-400]
        self.push_sweep()

    def encounter_line(self, text, now):
        """AVOID / MISSION lines that move the encounter on."""
        changed = False
        with self.lock:
            if re.match(r"\s*(?:MISSION|AVOID): obstacle at", text):
                sw = self.map["sweep"]
                if sw is None or sw["fixed"]:      # Not the same stop twice
                    self.map["sweep"] = self.new_sweep(now)
                self.map["enc"] = "measuring"
                changed = True
            elif EST_RE.fullmatch(text):
                g = EST_RE.fullmatch(text).groups()
                if self.map["sweep"] is None:
                    self.map["sweep"] = self.new_sweep(now)
                self.map["sweep"]["est"] = {
                    "face": int(g[0]), "left": int(g[1]), "left_open": g[2] == "?",
                    "right": int(g[3]), "right_open": g[4] == "?", "width": int(g[5])}
                changed = True
            elif re.match(r"\s*AVOID: back on course", text):
                self.map["enc"] = "rejoined"
                changed = True
        if changed:
            self.push_sweep()

    def on_obst(self, o, now):
        with self.lock:
            sw = self.map["sweep"]
            o = dict(o, rows=[], t=now, beams=sw["beams"] if sw else [])
            self.map["obst"].append(o)
            self.map["sweep"] = None           # Its beams went with o
            self.map["enc"] = "none" if o["go"] == "none" else "around"
        self.push({"type": "obst", "data": o})
        self.push_sweep()
        width = o["left"] + o["right"]
        more = " or more" if o["left_open"] or o["right_open"] else ""
        with self.lock:
            self.mission["profiled"] += 1
            self.mission["obst"] = {"width": width, "open": bool(more),
                                    "face": o["face"], "depth": o["depth"],
                                    "go": o["go"]}
            self.mission_note(now, "obstacle", f"Obstacle profiled: {width}{'+' if more else ''}"
                              f" mm wide, face {o['face']} mm ahead", "warn")
        self.push_mission()
        side = {"R": "going round the right", "L": "going round the left",
                "none": "no clear way round"}[o["go"]]
        if o["back"] > 0:              # - once the creep took it ahead
            side += f" (after backing up {o['back']} mm to see its edges)"
        self.event(now, f"obstacle mapped: {width} mm wide{more} (left {o['left']},"
                   f" right {o['right']}), depth up to {o['depth']} mm, face"
                   f" {o['face']} mm ahead; {side}", "warn")

    def on_maprow(self, r, now):
        with self.lock:
            if not self.map["obst"]:
                return                     # Rows without their obstacle line
            self.map["obst"][-1]["rows"].append(r)
        self.push({"type": "maprow", "data": r})

    def map_mark(self, text, now):
        """An obstacle stop, pinned at the latest pose."""
        with self.lock:
            if not self.map["path"]:
                return
            x, y, h = self.map["path"][-1][:3]
            mark = {"x": x, "y": y, "h": h, "text": text, "kind": "stop"}
            self.map["marks"].append(mark)
        self.push({"type": "mapmark", "data": mark})

    def mark_at(self, mm, kind, text, key, extra=None):
        """A card, junction or hump pinned where the car was mm along its
        path (the cumulative length of the poses; the car's own distance
        lines and these share the wheels), not where it is when the line
        arrives: card votes come after the stop. key: one mark per thing,
        updated in place. Call with the lock held; returns the mark."""
        path = self.map["path"]
        if not path:
            return None
        x, y, h = path[-1][:3]
        run = 0.0
        prev = (0.0, 0.0, path[0][2])
        for p in path:
            step = math.hypot(p[0] - prev[0], p[1] - prev[1])
            if run + step >= mm and step > 0:
                f = (mm - run) / step
                x = round(prev[0] + f * (p[0] - prev[0]))
                y = round(prev[1] + f * (p[1] - prev[1]))
                h = p[2]
                break
            run += step
            prev = p
        for mark in self.map["marks"]:
            if mark.get("key") == key:
                mark.update(x=x, y=y, h=h, text=text, mm=mm, **(extra or {}))
                return mark
        mark = dict({"x": x, "y": y, "h": h, "text": text, "kind": kind,
                     "mm": mm, "key": key}, **(extra or {}))
        self.map["marks"].append(mark)
        return mark

    def pin(self, mm, kind, text, key, extra=None):
        with self.lock:
            mark = self.mark_at(mm, kind, text, key, extra)
            data = dict(mark) if mark else None
        if data:
            self.push({"type": "mapmark", "data": data})

    def on_end(self, d, now):
        with self.lock:
            run = self.runs[-1] if self.runs else None
        gains = {k: d[k] for k in ("g", "cfg", "P", "I", "D")} if "g" in d else None
        if not run or run["state"] != "running":
            run = self.new_run(now, d["mode"], gains)   # Start was missed
        with self.lock:
            run.update(state="ended", t_end=now, reason=d["reason"],
                       secs=d["secs"], dt_max=d.get("dt_max"),
                       enc_errors=d.get("enc_errors", False))
            if d.get("mode"):
                run["mode"] = d["mode"]
            if gains:
                run["gains"] = gains
            text = self.describe_end(d)
            run["lines"].append(text)
        self.push({"type": "run", "data": run_meta(run)})
        reason = d["reason"] or ""
        cls = ("bad" if "FAIL" in reason.upper() or "fault" in reason
               else "warn" if reason.startswith("stopped:") or reason == "aborted"
               else "ok")
        with self.lock:
            if self.mission["run"] is None and "end" not in self.mission:
                self.mission["run"] = run["id"]     # Its start line was lost
            if self.mission["run"] == run["id"]:
                self.mission["end"] = reason
                self.mission["end_t"] = now
                self.mission_note(now, "end", "Run " + (reason or "ended")
                                  + (f" after {d['secs']:.1f} s" if d.get("secs") else ""),
                                  "ok" if cls == "ok" else "warn")
        self.push_mission()
        self.event(now, text, cls)

    @staticmethod
    def describe_end(d):
        out = f"{d['mode']} {d['reason']}"
        if d.get("secs") is not None:
            out += f" after {d['secs']:.1f} s"
        if "g" in d:
            out += (f": gains {d['g']}{' (configured)' if d['cfg'] else ''}"
                    f" P {d['P']} I {d['I']} D {d['D']}")
        if d.get("dt_max") is not None:
            out += f", dt max {d['dt_max']:.1f} ms"
        if d.get("enc_errors"):
            out += ", ENCODER ERRORS"
        return out

    def ended_run(self):
        with self.lock:
            run = self.runs[-1] if self.runs else None
            return run if run and run["state"] == "ended" else None

    def summary(self, key, value, text, now):
        run = self.ended_run()
        if run:
            with self.lock:
                if key:
                    run[key] = value
                run["lines"].append(text)
            self.push({"type": "run", "data": run_meta(run)})
        self.event(now, text, "")

    def on_centre(self, d, now):
        self.summary("centre", d, f" centre: {d['mm']} mm in {d['secs']:.1f} s,"
                     f" mean {d['mean']} mm/s; time % per phase " +
                     " ".join(f"{k} {v}" for k, v in d["phase"].items()) +
                     f", full steer {d['full_steer']}", now)

    def on_gaps(self, d, now):
        self.summary("gaps", d, f" gaps {d['gaps']}, crossings {d['crossings']},"
                     f" searches {d['searches']} (longest {d['longest_ms']} ms),"
                     f" darkest raw L {d['dark_L']} R {d['dark_R']}", now)

    def on_junctions(self, n, now):
        self.summary("junctions", n, f" junctions crossed straight on {n}", now)

    def on_trace(self, d, now):
        run = self.ended_run()
        if not run:
            return                         # Rows with no run end seen
        with self.lock:
            if d["end"]:
                run["trace_end"] = d["row"]
            else:
                run["trace"].append(d["row"])
        self.push({"type": "trace", "id": run["id"], "end": d["end"],
                   "row": d["row"]})

    def on_event(self, text, now):
        self.on_barcode_text(text, now)
        self.on_mission_text(text, now)
        self.encounter_line(text, now)
        # Cards are pinned by their distance (on_barcode_text); an obstacle
        # stop where the car is now.
        if re.match(r" (?:MISSION|AVOID): obstacle at", text):
            self.map_mark("stop", now)
        if re.match(r" AVOID", text):
            with self.lock:
                self.map["avoid"] = text.strip()
            self.push({"type": "avoid", "data": text.strip()})
        run = self.ended_run()
        if run and text.startswith(" "):   # Run report lines start with a space
            self.summary(None, None, text, now)
            return
        self.event(now, text, event_class(text))

    def on_garbled(self, text, now):
        with self.lock:
            self.stats["garbled"] += 1
        if (re.search(r"[A-Za-z]{2}", text) and not PERIODIC_RE.match(text)
                and not PERIODIC_IN_RE.search(text)):
            self.event(now, "(cut short) " + text, "garbled")


MAP_PATH_MAX = 6000                        # 50 min of 0.5 s poses


def new_map(run_id, now, ended=None):
    """The live map of one run: odometry path [x, y, deg] (mm, from START,
    + left), obstacles with their sonar rows, pinned marks, last AVOID line.
    An empty map cleared after a run carries ended: {run, reason}."""
    return {"run": run_id, "t0": now, "path": [], "obst": [], "marks": [],
            "avoid": "", "ended": ended, "sweep": None, "enc": None}


# AVOID: face 189 mm ahead of the pivot; obstacle left 78 mm, right 52 mm; WIDTH ~129 mm
EST_RE = re.compile(r"\s*AVOID: face (-?\d+) mm ahead of the pivot; obstacle left"
                    r" (-?\d+) mm(\??), right (-?\d+) mm(\??); WIDTH ~?(\d+).*")


def run_meta(run):
    """A run without its trace rows (they go to the page one by one)."""
    return {k: v for k, v in run.items() if k not in ("trace", "trace_end")} | {
        "rows": len(run["trace"]), "has_end_row": run["trace_end"] is not None,
        "end_d": run["trace_end"][1] if run["trace_end"] else None}


def start_mqtt(hub, host, port):
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                         client_id=f"is29-dashboard-{os.getpid()}")

    def on_connect(c, _u, _f, rc, _p=None):
        ok = (rc == 0) if isinstance(rc, int) else not rc.is_failure
        with hub.lock:
            hub.mqtt_ok = ok
        if ok:
            c.subscribe([(hub.base + "telemetry", 0), (hub.base + "status", 0),
                         (hub.base + "command", 0)])
        hub.push({"type": "mqtt", "ok": ok})
        print(f"MQTT {'connected to' if ok else 'refused by'} {host}:{port}",
              flush=True)

    def on_disconnect(_c, _u, _f, _rc, _p=None):
        with hub.lock:
            hub.mqtt_ok = False
        hub.push({"type": "mqtt", "ok": False})
        print("MQTT disconnected - retrying", flush=True)

    def on_message(_c, _u, msg):
        try:
            hub.on_message(msg.topic, msg.payload, msg.retain)
        except Exception as exc:           # One bad message must not kill it
            print(f"message error on {msg.topic}: {exc!r}", flush=True)

    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.reconnect_delay_set(1, 5)
    client.connect_async(host, port, 30)
    client.loop_start()
    return client


STATIC_TYPES = {".js": "text/javascript; charset=utf-8",
                ".txt": "text/plain; charset=utf-8"}


def make_handler(hub, client):
    page = os.path.join(HERE, "static", "index.html")

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *_a):        # Keep the console readable
            pass

        def send_static(self, rel):
            """A script beside the page (static/, static/vendor/: the 3D
            map and its three.js copy). Nothing outside static/."""
            root = os.path.join(HERE, "static")
            full = os.path.normpath(os.path.join(root, rel))
            kind = STATIC_TYPES.get(os.path.splitext(full)[1])
            if (not full.startswith(root + os.sep) or kind is None
                    or not os.path.isfile(full)):
                self.send_json(404, {"error": "not found"})
                return
            with open(full, "rb") as fh:
                body = fh.read()
            self.send_response(200)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(body)

        def send_json(self, code, obj):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                with open(page, "rb") as fh:
                    body = fh.read()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
            elif path == "/api/state":
                self.send_json(200, hub.snapshot())
            elif path == "/api/run":
                m = re.search(r"[?&]id=(\d+)", self.path)
                run = hub.get_run(int(m.group(1))) if m else None
                if run:
                    self.send_json(200, run)
                else:
                    self.send_json(404, {"error": "no such run"})
            elif path == "/events":
                self.stream()
            elif path.startswith("/static/"):
                self.send_static(path[len("/static/"):])
            else:
                self.send_json(404, {"error": "not found"})

        def stream(self):
            q = queue.Queue(maxsize=2000)
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            with hub.lock:
                hub.clients.append(q)
            try:
                self.wfile.write(
                    f"data: {json.dumps(hub.snapshot())}\n\n".encode())
                self.wfile.flush()
                while True:
                    try:
                        data = q.get(timeout=10)
                        self.wfile.write(f"data: {data}\n\n".encode())
                    except queue.Empty:
                        self.wfile.write(b": keep-alive\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError,
                    ConnectionAbortedError, OSError):
                pass
            finally:
                with hub.lock:
                    if q in hub.clients:
                        hub.clients.remove(q)

        def do_POST(self):
            if self.path == "/api/map":
                self.post_map()
                return
            if self.path != "/api/cmd":
                self.send_json(404, {"error": "not found"})
                return
            try:
                n = int(self.headers.get("Content-Length", "0"))
                req = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                self.send_json(400, {"error": "bad JSON"})
                return
            cmd = str(req.get("cmd", "")).upper()
            if cmd not in CMDS:
                self.send_json(400, {"error": f"unknown command {cmd}"})
                return
            if not hub.mqtt_ok:
                self.send_json(503, {"error": "not connected to the MQTT broker"})
                return
            byte = CMDS[cmd]
            client.publish(hub.base + "command", bytes([byte]), qos=1,
                           retain=False)
            hub.write_log(f"[sent] {cmd} 0x{byte:02X}")
            self.send_json(200, {"sent": cmd, "byte": f"0x{byte:02X}"})

        def post_map(self):
            """Letter -> junction move map: {"F": "A", "U": "Z", "L": "B",
            "R": "C"}, sent as the 3-byte message of navmap.h."""
            try:
                n = int(self.headers.get("Content-Length", "0"))
                req = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                self.send_json(400, {"error": "bad JSON"})
                return
            try:
                payload, text = pack_map(req)
            except ValueError as e:
                self.send_json(400, {"error": str(e)})
                return
            if not hub.mqtt_ok:
                self.send_json(503, {"error": "not connected to the MQTT broker"})
                return
            client.publish(hub.base + "command", payload, qos=1, retain=False)
            hub.write_log(f"[sent] MAP {text} ({payload.hex()})")
            self.send_json(200, {"sent": text, "bytes": payload.hex()})

    return Handler


# Letter -> junction move map (app_program/subsystem1/navmap.h, Change
# Requirement 1): bits 0-4 FORWARD, 5-9 U-TURN, 10-14 LEFT, 15-19 RIGHT
# (0 = A .. 25 = Z), bits 20-23 the pattern 1011b, least significant byte
# first. Same bytes as tools/broker/send_cmd.js.
MAP_MOVES = (("F", "forward"), ("U", "U-turn"), ("L", "left"), ("R", "right"))
MAP_PATTERN = 0xB


def pack_map(req):
    letters = []
    for key, name in MAP_MOVES:
        value = str(req.get(key, "")).strip().upper()
        if len(value) != 1 or not ("A" <= value <= "Z"):
            raise ValueError(f"{name} needs one letter A-Z")
        letters.append(value)
    if len(set(letters)) != len(letters):
        raise ValueError("the four letters must all be different")
    word = MAP_PATTERN << 20
    for index, letter in enumerate(letters):
        word |= (ord(letter) - ord("A")) << (5 * index)
    text = ", ".join(f"{name} {letter}" for (_, name), letter
                     in zip(MAP_MOVES, letters))
    return bytes([word & 0xFF, (word >> 8) & 0xFF, (word >> 16) & 0xFF]), text


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--broker", default="127.0.0.1", help="MQTT broker host")
    ap.add_argument("--mqtt-port", type=int, default=1883)
    ap.add_argument("--topic-base", default="inf2004/is29",
                    help="topics are <base>/telemetry, /status, /command")
    ap.add_argument("--port", type=int, default=8080, help="web page port")
    ap.add_argument("--log", help="append every received line to this file")
    args = ap.parse_args()

    hub = Hub(args.topic_base, args.log)
    threading.Thread(target=hub.flush_idle, daemon=True).start()
    client = start_mqtt(hub, args.broker, args.mqtt_port)
    server = ThreadingHTTPServer(("0.0.0.0", args.port),
                                 make_handler(hub, client))
    server.daemon_threads = True
    print(f"IS29 dashboard on http://localhost:{args.port}  "
          f"(other devices: http://<this laptop's IP>:{args.port}); "
          f"binary frame decoder {'loaded' if CODEC else 'MISSING'}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        server.server_close()


if __name__ == "__main__":
    main()
