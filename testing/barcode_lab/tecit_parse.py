"""Read the tec-it Code 39 GIFs (one per letter): bar/space runs along a
row through the bars, in pixels; compare with model.py's Code 39 table.
usage: python -I tecit_parse.py <gif dir> <out json>"""
import json
import os
import sys
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from model import elements

src, out = sys.argv[1], sys.argv[2]
res = {}
bad = 0
for f in sorted(os.listdir(src)):
    if not f.endswith(".gif"):
        continue
    ch = f[:-4]
    im = Image.open(os.path.join(src, f)).convert("L")
    w, h = im.size
    row = [im.getpixel((x, int(h * 0.25))) < 128 for x in range(w)]
    runs = []
    for x, d in enumerate(row):
        if runs and runs[-1][0] == d:
            runs[-1][1] += 1
        else:
            runs.append([d, 1])
    # trim white margins
    while runs and not runs[0][0]:
        runs.pop(0)
    while runs and not runs[-1][0]:
        runs.pop()
    px = [n for _, n in runs]
    narrow = min(px)
    mods = [round(n / narrow, 2) for n in px]
    ours = [wd for wd, _ in elements("*%s*" % ch)]
    same = len(mods) == len(ours) and all(abs(a - b) < 0.01 for a, b in zip(mods, ours))
    bad += 0 if same else 1
    res[ch] = dict(px=px, image=[w, h], narrow_px=narrow, modules=mods, matches_table=same)
    print("%s %dx%d px, %d elements, narrow %d px, wide %s px, total %d modules, matches our table: %s" % (
        ch, w, h, len(px), narrow, sorted(set(n for n in px if n > narrow)), sum(mods), same))
json.dump(res, open(out, "w"), indent=1)
print("letters that differ from our table:", bad)
