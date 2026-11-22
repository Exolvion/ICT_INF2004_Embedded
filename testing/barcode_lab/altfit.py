"""Fit the second (disc + logistic) model to the recorded big *A* passes."""
import pickle
import numpy as np
from scipy.optimize import least_squares
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from altmodel import tecit_elements, render

ps, rows = pickle.load(open("inv.pkl", "rb"))
by = {p["key"]: p for p in ps}
SPECS = [("L30:13573", 80, 420), ("L30:13522", 30, 360), ("L30:12540", 60, 400), ("L30:12480", 160, 500)]
els = tecit_elements("A")
fig, axes = plt.subplots(len(SPECS), 1, figsize=(15, 2.4 * len(SPECS)))
for ax, (key, lo, hi) in zip(axes, SPECS):
    p = by[key]
    v = p["v"]
    x = np.arange(len(v)) * p["du"]
    mk = (x >= lo) & (x <= hi) & np.isfinite(v)
    x, v = x[mk], v[mk]
    # start from the decoder-model fit's alignment (calib.py results)
    x0g = {"L30:13573": 105.3, "L30:13522": 51.8, "L30:12540": 76.2, "L30:12480": 178.1}[key]

    def res(q):
        x0, m, ratio, gain, D, s, c0, k, w, b = q
        xr, t = render(els, m, ratio, gain, D, s, c0, k)
        return w + (b - w) * np.interp(x - x0, xr, t, left=0, right=0) - v
    best = None
    for D in (6.0, 8.0, 10.0, 12.0):
        for c0 in (0.4, 0.5, 0.6, 0.7):
            for k in (6.0, 12.0):
                e = np.mean(res([x0g, 6.1, 3.0, 0.0, D, 1.0, c0, k, np.percentile(v, 20), np.percentile(v, 99)]) ** 2)
                if best is None or e < best[0]:
                    best = (e, D, c0, k)
    q0 = [x0g, 6.1, 3.0, 0.0, best[1], 1.0, best[2], best[3], np.percentile(v, 20), np.percentile(v, 99)]
    lb = [x0g - 5, 5.5, 2.5, -1.0, 3.0, 0.05, 0.2, 2.0, 100, 2000]
    ub = [x0g + 5, 6.7, 3.3, 2.0, 20.0, 5.0, 0.95, 40.0, 400, 4500]
    r = least_squares(res, q0, bounds=(lb, ub), diff_step=0.01)
    q = r.x
    rms = np.sqrt(np.mean(r.fun ** 2))
    print("%-10s rms %5.1f  m %.2f ratio %.2f gain %+.2f mm  disc D %.1f mm s %.2f  c0 %.2f k %.1f  white %.0f black %.0f" % (
        key, rms, *q[1:]))
    ax.plot(x, v, ".", ms=1.5)
    ax.plot(x, v + r.fun, "r-", lw=0.8)
    ax.set_title("%s disc+logistic rms %.0f D %.1f c0 %.2f k %.1f" % (key, rms, q[4], q[6], q[7]), fontsize=8)
plt.tight_layout()
plt.savefig("altfit.png", dpi=70)
