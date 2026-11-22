"""Live MISSION RAW cuts from a run log -> one sim dump file per cut.

Usage: python3 raw2dump.py <log> <out dir>
Writes <out dir>/cut_<line>.txt in the sim's dump format (" raw 1 of 1:
N units of ..." then "  u<unit>: v v ..." rows, a new row after each gap)
and <out dir>/index.tsv: file, log line, units, the live CODE/GROUP lines
of that cut. Values are the band middles of the two-part encoding.
"""
import os
import sys

log, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
lines = open(log, encoding="latin-1").read().split("\n")


def value(ch, white):
    k = ord(ch) - ord("#")
    if k >= 64:
        return white + 118 + 140 * (k - 64) + 70
    return max(0, white - 10 + 2 * k + 1)


index = []
found = []
i = 0
while i < len(lines):
    l = lines[i]
    if "DRIVE start" in l:
        found = []
    s = l.strip()
    if s.startswith("CODE ") or s.startswith("GROUP "):
        found.append(s)
    if s.startswith("RAW ") and ": u" in s and "below 'c'" in s:
        num = s.split(":")[0].split()[1]
        first = int(s.split(": u")[1].split(" ")[0])
        white = int(s.split("white ")[1].split(" ")[0])
        chars = ""
        j = i + 1
        while j < len(lines):
            t = lines[j].strip("\r")
            # Status and CODE/GROUP lines come between the R lines while
            # the cut goes out: skip them; stop at the next cut or run.
            if t.startswith(" R %s: " % num):
                chars += t[t.index(":") + 2:]
                j += 1
            elif t.strip().startswith("RAW ") or "DRIVE start" in t:
                break
            else:
                j += 1
        name = "cut_%05d.txt" % (i + 1)
        rows = []
        unit = first
        row = []
        row_unit = unit
        for ch in chars:
            if ch == "!":
                if row:
                    rows.append((row_unit, row))
                row = []
                unit += 1
                row_unit = unit
                continue
            if not row:
                row_unit = unit
            row.append(value(ch, white))
            unit += 1
        if row:
            rows.append((row_unit, row))
        with open(os.path.join(out, name), "w") as f:
            f.write(" raw 1 of 1: %d units of cut\n" % len(chars))
            for u, r in rows:
                f.write("  u%d: %s\n" % (u, " ".join(str(v) for v in r)))
        index.append((name, i + 1, len(chars), " || ".join(found)))
        found = []
        i = j
        continue
    i += 1
with open(os.path.join(out, "index.tsv"), "w") as f:
    for row in index:
        f.write("%s\t%d\t%d\t%s\n" % row)
print(len(index), "cuts")
