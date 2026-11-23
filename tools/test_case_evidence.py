#!/usr/bin/env python3
"""Write one evidence file per subsystem test case, from the logs.

    python3 tools/test_case_evidence.py

For each test case in testing/subsystemN/TEST_LOG.md this writes
testing/subsystemN/cases/<ID>.md: the case as defined, its result rows, and
the lines of the logs that bear on it, quoted with the file they come from:

- the test's own logs (testing/subsystemN/logs/ files named with its ID),
- for the host tests, their block of the newest SYS-01 or SYS-04 log,
- for cases the car runs showed, the matching lines of the day logs
  (testing/system/logs/*_dashboard.txt, or the broker logs for S1-U04).

Nothing is judged here: a case with no log says so. Re-run after new logs.
"""
import re
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
TESTING = REPO / "testing"
SYSTEM = TESTING / "system" / "logs"
WIDTH = 200          # longest quoted line
FIRST = 3            # matching lines quoted per day log
HEAD = 10            # opening lines quoted per test log

# Host tests: the executable whose block of the SYS-01 log is quoted.
HOST = {"S1-U01": "test_telemetry_main", "S1-U02": "test_mqtt_main",
        "S2-U01": "test_motion_main", "S2-U02": "test_core_main",
        "S3-U01": "test_line_main", "S4-U01": "test_sensor_main",
        "S5-U01": "test_sensor_main", "S5-U02": "test_motion_main"}
# Integration fixtures: lines of the SYS-04 log.
FIXTURE = {"S1-U03": r"mqttnet|radio/kernel|mock adapter",
           "S2-U03": r"board ISR"}
# Car runs: (day logs, pattern, note).
DAY = {
    "S1-U04": ("broker", r"\+ connect +inf2004-is29-car",
               "The broker logged each time the car connected."),
    "S1-U10": ("dashboard", r"\[status\] 0",
               "`[status] 0` is the car's last will, published by the broker "
               "when the car dropped off; the logs do not say which were "
               "power-offs."),
    "S2-U10": ("dashboard", r"NAV: turned [+-]\d+ deg",
               "Each junction turn as the car measured it (its own heading), "
               "not a protractor reading; the planned turns are +-90 and "
               "180 deg."),
    "S3-U06": ("dashboard", r"advancing \d+ mm to turn on the crossing",
               "The travel the car is set to drive from the junction's "
               "detection to the pivot (BENCH_NAV_ADVANCE_MM), not a measured "
               "travel."),
    "S4-U02": ("dashboard", r"IMU since START: failed reads",
               "Each run's end report counts the accelerometer reads that "
               "failed on the I2C bus."),
    "S5-U07": ("dashboard", r"AVOID: (face \d+ mm ahead of the pivot|WIDTH)",
               "Measured while the car followed the line (the mission's "
               "go-around), not from a stationary scan; the obstacles' true "
               "widths were not recorded."),
}
TEXT = {".txt", ".md", ".log"}


def cells(line):
    return [c.strip() for c in line.strip().strip("|").split("|")]


def tables(path):
    """The test-case rows by ID, and the result rows by ID."""
    cases, results, part = {}, {}, None
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("## Test cases"):
            part = cases
        elif line.startswith("## Results"):
            part = results
        elif part is not None and line.startswith("| "):
            row = cells(line)
            key = row[0] if part is cases else (row[1] if len(row) > 1 else "")
            if re.fullmatch(r"S\d-U\d\d", key):
                part.setdefault(key, []).append(row)
    return cases, results


def quote(lines):
    out = ["```text"]
    out += [l[:WIDTH] + (" ..." if len(l) > WIDTH else "") for l in lines]
    return out + ["```"]


def read(path):
    return path.read_text(encoding="utf-8", errors="replace").splitlines()


def own_logs(folder, test_id):
    out = []
    for path in sorted((folder / "logs").glob(f"*{test_id}*")):
        lines = read(path)
        out += ["", f"### [`logs/{path.name}`](../logs/{path.name})", ""]
        if path.suffix in TEXT:
            body = [l for l in lines if l.strip()]
            picked = body[:HEAD]
            picked += [l for l in body[HEAD:]
                       if re.search(r"(?i)\bresult\b|\bpass\b|\bfail\b", l)][:5]
            out += [f"{len(lines)} lines; the opening lines and any result lines:", ""]
            out += quote(picked)
        else:
            out += [f"{len(lines)} records; the first:", ""]
            out += quote(lines[:1])
    return out


def newest(pattern):
    found = sorted(SYSTEM.glob(pattern))
    return found[-1] if found else None


def host_block(executable):
    path = newest("*_SYS-01.txt")
    if path is None:
        return []
    picked, inside = [], False
    for line in read(path):
        if re.match(r"test_\w+_main: ", line):
            inside = line.startswith(executable + ":")
        if inside and line.strip():
            picked.append(line)
    return ["", f"### [`system/logs/{path.name}`](../../system/logs/{path.name})",
            "", f"The `{executable}` block of the host-test run:", ""] + quote(picked)


def fixture_lines(pattern):
    path = newest("*_SYS-04.txt")
    if path is None:
        return []
    picked = [l for l in read(path) if re.search(pattern, l)]
    return ["", f"### [`system/logs/{path.name}`](../../system/logs/{path.name})",
            "", "The integration fixtures' lines for it:", ""] + quote(picked)


def day_lines(kind, pattern, note):
    out = ["", f"### Day logs: `{pattern}`", "", note]
    if kind == "broker":
        paths = sorted((TESTING / "subsystem1" / "logs").glob("*_broker.txt"))
        base = "../../subsystem1/logs/"
    else:
        paths = sorted(SYSTEM.glob("*_dashboard.txt"))
        base = "../../system/logs/"
    for path in paths:
        hits = [l for l in read(path)
                if "[frame" not in l and re.search(pattern, l)]
        if hits:
            out += ["", f"- [`{path.name}`]({base}{path.name}): {len(hits)} "
                    f"lines; the first {min(FIRST, len(hits))}:", ""]
            out += ["  " + l for l in quote(hits[:FIRST])]
    return out


def write_case(folder, row, results):
    test_id, what = row[0], row[1]
    req, method, steps, expected = (row + [""] * 6)[2:6]
    out = [f"# {test_id} {what}", "",
           "Written by `tools/test_case_evidence.py` from `TEST_LOG.md` and "
           "the logs; re-run it after new results.", "",
           "| Requirement / interface | Method | Steps / how | Expected result |",
           "|---|---|---|---|",
           f"| {req} | {method} | {steps} | {expected} |", "",
           "## Results", ""]
    if results:
        out += ["| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |",
                "|---|---|---|---|---|---|---|"]
        out += ["| " + " | ".join(r) + " |" for r in results]
    else:
        out += ["No result recorded in `TEST_LOG.md`."]
    out += ["", "## Evidence in the logs"]
    evidence = own_logs(folder, test_id)
    if test_id in HOST:
        evidence += host_block(HOST[test_id])
    if test_id in FIXTURE:
        evidence += fixture_lines(FIXTURE[test_id])
    if test_id in DAY:
        evidence += day_lines(*DAY[test_id])
    out += evidence if evidence else ["", "No log of this test case."]
    target = folder / "cases" / f"{test_id}.md"
    target.parent.mkdir(exist_ok=True)
    target.write_text("\n".join(out) + "\n", encoding="utf-8", newline="\n")
    return target, bool(evidence)


def main():
    for n in range(1, 6):
        folder = TESTING / f"subsystem{n}"
        cases, results = tables(folder / "TEST_LOG.md")
        for test_id, rows in sorted(cases.items()):
            target, b_evidence = write_case(
                folder, rows[0],
                [r for r in results.get(test_id, []) if r[0] != "YYYY-MM-DD"])
            print(f"{target.relative_to(REPO)}: "
                  f"{'evidence' if b_evidence else 'no log'}")


if __name__ == "__main__":
    main()
