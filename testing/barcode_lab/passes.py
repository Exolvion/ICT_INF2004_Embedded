"""Load every recorded barcode-sensor pass (raw samples) into one list.

Sources:
  live RAW dumps (both encodings) in the 30 and 31 Oct live logs,
  the 30 Oct mission/static cut files, the 1 mm traces.
Each pass: dict(src, key, du (mm per sample), v (np.array, nan = none),
white, sd, line, logged (CODE/GROUP lines of that cut), drive (index of
the DRIVE start it belongs to)).
"""
import os
import re
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_UP = os.path.normpath(os.path.join(_HERE, "..", ".."))
REPO = os.environ.get("REPO") or _UP
LOGS = os.path.join(REPO, "testing", "subsystem3", "logs")
OFF = os.path.join(REPO, "firmware", "bench", "offline")
UNIT_MM = 0.3215


def _value(ch, white, two_part):
    k = ord(ch) - ord("#")
    if two_part and k >= 64:
        return white + 118 + 140 * (k - 64) + 70
    return max(0, white - 10 + 2 * k + 1)


def live(path, tag):
    lines = open(path, encoding="latin-1").read().split("\n")
    out = []
    found = []
    drive = 0
    i = 0
    while i < len(lines):
        l = lines[i]
        if "DRIVE start" in l:
            drive += 1
            found = []
        s = l.strip()
        for m in re.finditer(r"(CODE \*.*?\)|GROUP .*?\))", s):
            found.append(m.group(1))
        m = re.search(r"RAW (\d+): u(\d+) .*?(\d+) units, white (\d+) sd (\d+)", s)
        if m:
            num = m.group(1)
            first = int(m.group(2))
            n_units = int(m.group(3))
            white = int(m.group(4))
            sd = int(m.group(5))
            two_part = "below 'c'" in s
            chars = ""
            j = i + 1
            while j < len(lines):
                t = lines[j].strip("\r")
                if t.strip().startswith("R %s: " % num):
                    chars += t[t.index(":") + 2:]
                    j += 1
                elif t.strip() == "":
                    j += 1
                elif t.lstrip().startswith(("t ", "ACC", "IR ", "ENC")) and len(chars) < n_units:
                    # status lines interleave with the R lines
                    j += 1
                else:
                    break
            v = np.array([np.nan if c == "!" else _value(c, white, two_part)
                          for c in chars], float)
            out.append(dict(src=tag, key="%s:%d" % (tag, i + 1), du=UNIT_MM,
                            v=v, white=white, sd=sd, line=i + 1,
                            n_units=n_units, logged=list(found), drive=drive,
                            first=first, two_part=two_part))
            found = []
            i = j
            continue
        i += 1
    return out


def cutfile(path, tag):
    rows = []
    white = None
    for l in open(path, encoding="latin-1"):
        if not re.match(r"\s*u\d+:", l):
            continue
        for tok in l.split():
            m = re.match(r"u(\d+):$", tok)
            if m:
                rows.append((int(m.group(1)), []))
            else:
                try:
                    rows[-1][1].append(float(tok))
                except ValueError:
                    pass
    if not rows:
        return None
    u0 = rows[0][0]
    u1 = max(u + len(r) for u, r in rows)
    v = np.full(u1 - u0, np.nan)
    for u, r in rows:
        v[u - u0:u - u0 + len(r)] = r
    return dict(src=tag, key="%s:%s" % (tag, os.path.basename(path)),
                du=UNIT_MM, v=v, white=None, sd=None, line=0, logged=[],
                drive=0, first=u0)


def trace(path, tag):
    vals = []
    for l in open(path, encoding="latin-1"):
        m = re.match(r"\s*\d+ c\d+: (.*)", l)
        if m:
            for tok in m.group(1).split():
                if re.match(r"c\d+:$", tok):
                    vals.pop()          # the glued row's index
                else:
                    vals.append(float(tok))
    return dict(src=tag, key="%s:%s" % (tag, os.path.basename(path)), du=1.0,
                v=np.array(vals), white=None, sd=None, line=0, logged=[],
                drive=0, first=0)


def load_all():
    out = []
    out += live(os.path.join(LOGS, "2026-10-30_S3-U07_live.txt"), "L30")
    out += live(os.path.join(LOGS, "2026-10-31_S3-U07_live.txt"), "L31")
    d = os.path.join(OFF, "recordings", "2026-10-30_cuts")
    for f in sorted(os.listdir(d)):
        if f.endswith(".txt"):
            p = cutfile(os.path.join(d, f), "C30")
            if p is not None:
                out.append(p)
    d = os.path.join(OFF, "traces")
    for f in sorted(os.listdir(d)):
        out.append(trace(os.path.join(d, f), "T"))
    return out


if __name__ == "__main__":
    ps = load_all()
    for p in ps:
        v = p["v"]
        ok = np.isfinite(v)
        print("%-34s n %5d of %5s ok %5d white %4s sd %3s max %6.0f drive %3d | %s" % (
            p["key"], len(v), p.get("n_units", "-"), ok.sum(), p["white"], p["sd"],
            np.nanmax(v) if ok.any() else -1, p["drive"],
            " || ".join(p["logged"])[:120]))
