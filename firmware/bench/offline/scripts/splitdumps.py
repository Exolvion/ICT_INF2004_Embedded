"""End-of-run MISSION dumps (" raw k of n: ..." + "  u<unit>: v ...") from
run logs -> one sim dump file per cut, with the live CODE/GROUP lines of
the run that dumped it in index.tsv.

Usage: python3 splitdumps.py <out dir> <log> [<log> ...]
"""
import os
import sys

out = sys.argv[1]
os.makedirs(out, exist_ok=True)
index = []
for log in sys.argv[2:]:
    tag = os.path.basename(log).replace("2026-", "").replace(".txt", "")
    lines = open(log, encoding="latin-1").read().split("\n")
    found = []
    cur = None
    rows = []
    for i, l in enumerate(lines):
        s = l.strip()
        if "DRIVE start" in l:
            found = []
        if s.startswith("CODE ") or s.startswith("GROUP "):
            found.append(s)
        if " raw " in l and " of " in l and "units of" in l:
            if cur is not None:
                index.append((cur, rows_count, " || ".join(found)))
            cur = "%s_%05d.txt" % (tag, i + 1)
            rows_count = 0
            f = open(os.path.join(out, cur), "w")
            f.write(" raw 1 of 1: " + l.split(":", 1)[1].strip() + "\n")
            f.close()
            continue
        if cur is not None and l.startswith("  u") and ":" in l:
            with open(os.path.join(out, cur), "a") as f:
                f.write(l.rstrip("\r") + "\n")
            rows_count += 1
            continue
        if cur is not None and rows_count and s and not l.startswith("  u"):
            index.append((cur, rows_count, " || ".join(found)))
            cur = None
    if cur is not None:
        index.append((cur, rows_count, " || ".join(found)))
with open(os.path.join(out, "index.tsv"), "w") as f:
    for row in index:
        f.write("%s\t%d\t%s\n" % row)
print(len(index), "cuts")
