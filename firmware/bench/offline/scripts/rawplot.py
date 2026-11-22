"""Plot of one live RAW cut: darkness above the cut's white (raw counts).
Reads both encodings: the old linear one (ceiling white + 172) and the
two-part one (header "below 'c'": 2 counts per char, then 140 from 'c').
Usage: python3 rawplot.py <log> <raw no> [run from end, 1 = last] [mm per row]
"""
import sys

log, want = sys.argv[1], int(sys.argv[2])
back = int(sys.argv[3]) if len(sys.argv) > 3 else 1
step = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
lines = open(log, encoding="latin-1").read().split("\n")
starts = [i for i, l in enumerate(lines) if "DRIVE start" in l]
start = starts[-back]
end = starts[-back + 1] if back > 1 else len(lines)
chars, first, white, head = "", None, None, ""
for l in lines[start:end]:
    if l.startswith(" RAW %d:" % want):
        head = l
        first = int(l.split(": u")[1].split(" ")[0])
        white = int(l.split("white ")[1].split(" ")[0])
    elif l.startswith(" R %d: " % want):
        chars += l[l.index(":") + 2:].rstrip("\r")
two = "below 'c'" in head


def above(ch):
    """Darkness above white of one char (lower bound of its band, +half)."""
    k = ord(ch) - ord("#")
    if two and k >= 64:
        return 118 + 140 * (k - 64) + 70
    return 2 * k - 10 + 1


print(head.strip())
print("chars", len(chars), "gaps", chars.count("!"), "at the top",
      chars.count("~"), "two-part" if two else "linear")
units = max(1, int(round(step / 0.32)))
scale = 60.0 / (4000.0 if two else 180.0)
for i in range(0, len(chars), units):
    seg = [above(c) for c in chars[i:i + units] if c != "!"]
    if not seg:
        continue
    d = sum(seg) / len(seg)
    mm = (first + i) * 0.32
    print("  %7.1f %5d |%s" % (mm, d, "#" * max(0, min(60, int(d * scale)))))
