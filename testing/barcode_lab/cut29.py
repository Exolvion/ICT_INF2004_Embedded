"""Cut the two 29 Oct 2 cm side-sensor traces (1 mm) out of the S3-U07 log
into t29_1_trace.txt and t29_2_trace.txt (used by realrun.py)."""
import os
import re
from passes import LOGS

src = os.path.join(LOGS, "2026-10-29_S3-U07_barcode.txt")
lines = open(src, encoding="latin-1").read().split("\n")
n = 0
for i, l in enumerate(lines):
    if "side sensor raw every 1 mm of its own path" in l:
        n += 1
        with open("t29_%d_trace.txt" % n, "w") as out:
            j = i + 1
            while j < len(lines) and re.match(r"\s*\d+ c\d+:", lines[j]):
                out.write(lines[j] + "\n")
                j += 1
        print("t29_%d" % n, "from log line", i + 1, j - i - 1, "rows")
