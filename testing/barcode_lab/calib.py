"""Fit the sensor model to passes whose card is known.
usage: calib.py key:text:dir[:from_mm:to_mm] ..."""
import sys
import pickle
import numpy as np
from scipy.optimize import least_squares
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from model import synth, modules
from groups import find_groups

ps, rows = pickle.load(open("inv.pkl", "rb"))
by = {p["key"]: p for p in ps}


def segment(p, lo=None, hi=None):
    v = p["v"]
    x = np.arange(len(v)) * p["du"]
    if lo is None:
        gs = [g for g in find_groups(p) if g["len_mm"] > 60]
        g = max(gs, key=lambda g: g["len_mm"])
        lo = g["a"] * p["du"] - 25
        hi = g["b"] * p["du"] + 25
    m = (x >= lo) & (x <= hi) & np.isfinite(v)
    return x[m], v[m]


def fit(p, text, rev, lo=None, hi=None, fixed=None):
    x, v = segment(p, lo, hi)
    white = p["white"] or np.percentile(v, 20)
    clip = None if p.get("two_part", True) is not False else white + 173
    black = np.percentile(v, 99)
    span = x[-1] - x[0] - 40
    L = modules(text)
    best = None
    # coarse search of start and module width with a default blur
    for m in np.linspace(span / L * 0.8, span / L * 1.1, 13):
        for x0 in np.arange(x[0], x[0] + 45, 1.0):
            s = synth(x, text, x0, m, white, black, 0.8 * m * 1.5, 1.0, 0.3, 1.0,
                      reverse=rev, clip=clip)
            e = np.mean((s - v) ** 2)
            if best is None or e < best[0]:
                best = (e, x0, m)
    _, x0, m = best
    names = ["x0", "m", "white", "black", "aper", "sigma", "c0", "gamma", "ratio"]
    p0 = [x0, m, white, black, 1.2 * m, 1.0, 0.3, 1.0, 3.0]
    lb = [x0 - 10, m * 0.8, white - 100, white + 50, 0.5, 0.05, 0.0, 0.3, 2.0]
    ub = [x0 + 10, m * 1.2, white + 100, 5000, 30, 10, 0.9, 4.0, 3.5]
    if fixed:
        for k, val in fixed.items():
            i = names.index(k)
            p0[i] = val
            lb[i] = val - 1e-6
            ub[i] = val + 1e-6

    def res(q):
        s = synth(x, text, q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7],
                  ratio=q[8], reverse=rev, clip=clip)
        return s - v
    r = least_squares(res, p0, bounds=(lb, ub), x_scale="jac")
    q = r.x
    rms = np.sqrt(np.mean(r.fun ** 2))
    return dict(zip(names, q)), rms, x, v, q, clip


if __name__ == "__main__":
    out = []
    specs = sys.argv[1:]
    fig, axes = plt.subplots(len(specs), 1, figsize=(15, 2.4 * len(specs)))
    axes = np.atleast_1d(axes)
    for ax, s in zip(axes, specs):
        f = s.split(":")
        key = f[0] + ":" + f[1]
        text, d = f[2], f[3]
        lo = float(f[4]) if len(f) > 4 else None
        hi = float(f[5]) if len(f) > 5 else None
        p = by[key]
        prm, rms, x, v, q, clip = fit(p, text, d == "r", lo, hi)
        print("%-12s %s %s rms %6.1f  " % (key, text, d, rms) +
              " ".join("%s %.2f" % (k, val) for k, val in prm.items()))
        ax.plot(x, v, ".", ms=1.5)
        ax.plot(x, synth(x, text, *q[:8], ratio=q[8], reverse=(d == "r"), clip=clip), "r-", lw=0.8)
        ax.set_title("%s %s %s rms %.0f aper %.1f sigma %.1f c0 %.2f gamma %.2f m %.2f" % (
            key, text, d, rms, prm["aper"], prm["sigma"], prm["c0"], prm["gamma"], prm["m"]), fontsize=8)
    plt.tight_layout()
    plt.savefig("calib.png", dpi=80)
