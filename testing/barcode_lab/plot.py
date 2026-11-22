"""plot.py out.png key[:from_mm:to_mm] ... -> one panel per pass."""
import sys
import pickle
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ps, rows = pickle.load(open("inv.pkl", "rb"))
by = {p["key"]: p for p in ps}
out = sys.argv[1]
specs = sys.argv[2:]
fig, axes = plt.subplots(len(specs), 1, figsize=(15, 2.3 * len(specs)))
if len(specs) == 1:
    axes = [axes]
for ax, s in zip(axes, specs):
    parts = s.split("@")
    key = parts[0]
    p = by[key]
    v = p["v"]
    x = np.arange(len(v)) * p["du"]
    if len(parts) > 1:
        a, b = [float(t) for t in parts[1].split(",")]
        m = (x >= a) & (x <= b)
        x, v = x[m], v[m]
    ax.plot(x, v, "-", lw=0.8)
    if p["white"]:
        ax.axhline(p["white"], color="g", lw=0.5)
    ax.set_title("%s  %s" % (key, " || ".join(p["logged"])[:150]), fontsize=8)
    ax.grid(alpha=0.3)
    ax.set_xlim(x[0], x[-1])
plt.tight_layout()
plt.savefig(out, dpi=80)
