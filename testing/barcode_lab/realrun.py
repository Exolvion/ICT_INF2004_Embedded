"""Template-match every recorded card group against *A*..*Z* (both ways).
usage: realrun.py [key-substring ...]"""
import sys
import time
import pickle
import numpy as np
from fast import match2, candidates
from groups import find_groups
from passes import trace

ps, rows = pickle.load(open("inv.pkl", "rb"))
ps = ps + [trace("t29_1_trace.txt", "T29"), trace("t29_2_trace.txt", "T29")]
sel = [a for a in sys.argv[1:] if not a.startswith("--gap=")]
GAP = float(([a[6:] for a in sys.argv[1:] if a.startswith("--gap=")] or ["40"])[0])
SIG = [1.0, 1.75, 2.5, 3.25, 4.0, 5.0]
C0 = [0.0, 0.2, 0.35, 0.5, 0.6]
res_all = []
for p in ps:
    wins = [s.split("@")[1] for s in sel if "@" in s and s.split("@")[0] == p["key"]]
    if sel and not wins and not any(("@" not in s) and s in p["key"] for s in sel):
        continue
    if np.isfinite(p["v"]).sum() < 100:
        continue
    v = p["v"]
    x = np.arange(len(v)) * p["du"]
    white = p["white"]
    clip = (white + 172) if (p.get("two_part") is False) else None
    gl = find_groups(p, k_sd=5.0, gap_mm=GAP)
    if wins:
        w = [float(t) for t in wins[0].split(",")]
        gl = [dict(a=int(w[0] / p["du"]), b=int(w[1] / p["du"]), len_mm=w[1] - w[0])]
    for g in gl:
        L = g["len_mm"]
        if not (90 <= L <= 200 or 200 < L <= 330):
            continue
        big = L > 200
        a_mm, b_mm = g["a"] * p["du"], g["b"] * p["du"]
        keep = (x >= a_mm - 30) & (x <= b_mm + 30) & np.isfinite(v)
        if clip is not None:
            keep &= v < clip - 1          # censored samples dropped
        xs, vs = x[keep], v[keep]
        t0 = time.time()
        fine, coarse = match2(xs, vs, a_mm, L, candidates(),
                              sig_grid=[s * (2 if big else 1) for s in SIG], c0_grid=C0)
        N = len(vs)
        best = fine[0]
        other = next(r for r in fine if r[1] != best[1])
        ratio = other[0] / max(best[0], 1e-9)
        print("%-30s %5.0f-%5.0f len %5.1f | %s%s rms %5.1f | next %s%s x%.2f | m %.2f sig %.2f c0 %.2f | %4.0fs | %s" % (
            p["key"], a_mm, b_mm, L, best[1], "<" if best[2] else " ", np.sqrt(best[0] / N),
            other[1], "<" if other[2] else " ", ratio,
            best[3]["m"], best[3]["sigma"], best[3]["c0"], time.time() - t0,
            " || ".join(p["logged"])[:80]), flush=True)
        res_all.append(dict(key=p["key"], a=a_mm, b=b_mm, L=L, fine=fine, N=N,
                            logged=p["logged"]))
tag = "_".join(sel)[:24].replace(":", "") or "all"
pickle.dump(res_all, open("rr_%s.pkl" % tag, "wb"))
