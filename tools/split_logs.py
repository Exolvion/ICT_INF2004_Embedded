#!/usr/bin/env python3
"""Split the whole-day system logs into the five subsystems' logs.

    python3 tools/split_logs.py            # every day log
    python3 tools/split_logs.py 2026-11-22 # one day

A day's dashboard log (testing/system/logs/<date>_dashboard.txt) has every
subsystem's lines mixed. Each line goes to the subsystem it is about, by its
tag (the text after the time), into testing/subsystemN/logs/<date>_dashboard
.txt; a line with no time (a continuation) follows the line before it. The
broker log (<date>_broker.txt) is all subsystem 1. Run-level lines (MISSION,
DRIVE start, the task timing and owner health of the end report) belong to
no one subsystem and stay only in the day log, which is never changed: it is
the full record. Re-run after new logs; the split files are rewritten.
"""
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SYSTEM = REPO / "testing" / "system" / "logs"
TIME = re.compile(r"^(\d\d:\d\d:\d\d(?:\.\d+)?)\s(.*)$")

# First match wins; tested on the line's text after the time, leading
# blanks removed.
RULES = (
    # 1: Wi-Fi, MQTT, commands, the robot's telemetry frames
    (1, r"^\[(frame (status|perception|diagnostic)|frame\]|sent|command|"
        r"status)\b|^\[frame\]|^MAP\b|^MQTT|^BENCH MOTION ready|^WIFI|"
        r"^undecodable|^net "),
    # 5: the front sonar and the obstacle go-around (before 3's "M"/"mm")
    (5, r"^(AVOID|OBST|BEAMS|SCAN|MISSION: obstacle|FRONT|front sonar)\b|"
        r"^M -?\d|^M \+|^range\b"),
    # 4: accelerometer, compass, hump, side leans, lift
    (4, r"^(ACC|MAG|RAMP|SIDE|LIFT|IMU|RISE|LEAN|TERRAIN)\b|"
        r"^(ramp|lift):|^C [+-]\d"),
    # 3: IR, line following, AUTO-CENTRE, calibration, barcodes, junctions
    (3, r"^(IR|NAV|CODE|CAR|Decoder|D0|D|RAW|REC|centre|gaps|barcodes|"
        r"GROUP|NEW|CAL|AUTO-CENTRE|LINE|right sensor|heading at the end|"
        r"middle|widths|Code 39|card group|junctions crossed|trace)\b|"
        r"^R \d+:|^D:|^t \d+\.\d s d |^\d+ d |^ ?(TRACKING|SEARCHING|"
        r"JUNCTION)"),
    # 2: wheels, encoders, pose, the drive's own lines
    (2, r"^(ENC|PID|PIVOT|SWEEP|MOTION|STALL|wheel|duty)\b|^P -?\d|"
        r"^mm L\b"),
)
COMPILED = tuple((n, re.compile(pattern)) for n, pattern in RULES)


def subsystem(text: str) -> int:
    """1-5, or 0 for a run-level line (it stays in the day log only)."""
    body = text.lstrip()
    for number, pattern in COMPILED:
        if pattern.search(body):
            return number
    return 0


def split_day(log: Path) -> dict:
    date = log.name.split("_")[0]
    kind = "broker" if log.name.endswith("_broker.txt") else "dashboard"
    out = {n: [] for n in range(6)}
    current = 0
    for line in log.read_text(encoding="utf-8", errors="replace").split("\n"):
        if not line:
            continue
        match = TIME.match(line)
        if kind == "broker":
            current = 1
        elif match:
            current = subsystem(match.group(2))
        out[current].append(line)
    written = {}
    for number in range(1, 6):
        if not out[number]:
            continue
        target = (REPO / "testing" / f"subsystem{number}" / "logs" /
                  f"{date}_{kind}.txt")
        header = (f"# subsystem {number} lines of testing/system/logs/"
                  f"{log.name} (tools/split_logs.py); the day log has them "
                  f"all.\n")
        if target.exists() and not target.read_text(
                encoding="utf-8", errors="replace").startswith("# subsystem"):
            print(f"skipped {target.relative_to(REPO)}: not a split file")
            continue
        target.write_text(header + "\n".join(out[number]) + "\n",
                          encoding="utf-8")
        written[number] = len(out[number])
    total = sum(len(lines) for lines in out.values())
    print(f"{log.name}: {total} lines -> " +
          ", ".join(f"S{n} {written.get(n, 0)}" for n in range(1, 6)) +
          f", run-level {len(out[0])}")
    return written


def main(argv) -> int:
    days = argv[1:]
    logs = sorted(SYSTEM.glob("*_dashboard.txt")) + \
        sorted(SYSTEM.glob("*_broker.txt"))
    if days:
        logs = [log for log in logs if log.name.split("_")[0] in days]
    for log in logs:
        split_day(log)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
