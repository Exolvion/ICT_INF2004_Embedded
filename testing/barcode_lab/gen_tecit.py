"""Test passes from tec-it's bars and the second (disc + logistic) sensor
model fitted to the recorded big-card passes.  Same file layout as
synth_bench.py (truth.tsv, syn_*.txt for the car sim, syn_*.npy).

usage: gen_tecit.py OUT SEED REPS CARD OPTICS
  CARD   small (measured 142 mm) | big (measured 281 mm)
  OPTICS 1cm   = the fit to 30 Oct (disc 11-13.5 mm, s 0.5-2, c0 0.72-0.79, k 9.5-12.6)
         sharp = the same with a 7-9 mm disc (a better day)
"""
import os
import sys
import numpy as np
from altmodel import tecit_elements, render

UNIT = 0.3215
LET = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
out, seed, reps, card, optics = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), sys.argv[4], sys.argv[5]
rng = np.random.default_rng(seed)
os.makedirs(out, exist_ok=True)
length = 142.0 if card == "small" else 281.0
rows = []
k = 0
for rep in range(reps):
    for ch in LET:
        els = tecit_elements(ch)
        rev = bool(rng.integers(0, 2))
        m = length / 47.0 * rng.uniform(0.98, 1.02) * rng.uniform(1.0, 1.12)   # print, crossing angle
        ratio = rng.uniform(2.95, 3.04)
        gain = rng.uniform(0.0, 0.5)
        D = rng.uniform(11.0, 13.5) if optics == "1cm" else rng.uniform(7.0, 9.0)
        s = rng.uniform(0.5, 2.0)
        c0 = rng.uniform(0.72, 0.79)
        kk = rng.uniform(9.5, 12.6)
        white = rng.uniform(165, 195)
        black = rng.uniform(2900, 3200)
        warp = rng.uniform(-0.06, 0.06)
        dark_noise = rng.choice([0.003, 0.012])
        xr, t = render(els, m / 1.0, ratio, gain, D, s, c0, kk, reverse=rev)
        span = xr[-1] - 80.0                      # symbol length (render pads 40 mm each side)
        pre, post = 150.0, 100.0
        xs = np.arange(0.0, pre + span + post, UNIT)
        u = (xs - pre) / span
        xc = (xs - pre) * (1.0 + warp * (u - 0.5))
        v = white + (black - white) * np.interp(xc, xr, t, left=0.0, right=0.0)
        sd = rng.uniform(6, 12) + dark_noise * (v - white)
        v = np.round(np.clip(v + rng.normal(0, 1, len(v)) * sd, 0, 4095))
        v[rng.random(len(v)) < rng.uniform(0.0, 0.12)] = np.nan
        name = "syn_%04d.txt" % k
        with open(os.path.join(out, name), "w") as f:
            f.write(" raw 1 of 1: %d units of cut\n" % len(v))
            row, start = [], None
            for i, val in enumerate(v):
                if np.isnan(val):
                    if row:
                        f.write("  u%d: %s\n" % (start, " ".join("%d" % q for q in row)))
                    row, start = [], None
                    continue
                if start is None:
                    start = i
                row.append(val)
                if len(row) == 25:
                    f.write("  u%d: %s\n" % (start, " ".join("%d" % q for q in row)))
                    row, start = [], None
            if row:
                f.write("  u%d: %s\n" % (start, " ".join("%d" % q for q in row)))
        np.save(os.path.join(out, name[:-4] + ".npy"), np.vstack([xs, v]))
        rows.append((name, ch, int(rev), m, D, c0, kk, white, black, warp, ratio, gain))
        k += 1
with open(os.path.join(out, "truth.tsv"), "w") as f:
    for r in rows:
        f.write("\t".join(str(q) for q in r) + "\n")
print(out, k, "passes")
