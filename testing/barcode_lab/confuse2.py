"""Letter confusability on the normal card for a given blur, noise-free:
each letter's template vs every other candidate's best fit (RMS counts)."""
import sys
import time
import numpy as np
from fast import cover_fine, match, candidates, PAD, DX

sigma = float(sys.argv[1])
c0 = float(sys.argv[2])
m = 3.05
W, B = 180.0, 3150.0
x = np.arange(-25, 47 * m + 25, 0.3215)
LET = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
t0 = time.time()
worst = []
for ch in LET:
    cf = cover_fine("*%s*" % ch, False, m, sigma)
    idx = np.rint((x + PAD) / DX).astype(int)
    c = np.where((idx >= 0) & (idx < len(cf)), cf[np.clip(idx, 0, len(cf) - 1)], 0)
    v = W + (B - W) * np.clip((c - c0) / (1 - c0), 0, 1)
    res = match(x, v, candidates(LET),
                m_grid=m * np.array([0.98, 0.99, 1.0, 1.01, 1.02]),
                x0_grid=np.arange(-3, 3.01, 0.5),
                sigma_grid=[sigma * 0.85, sigma, sigma * 1.15],
                c0_grid=[c0 - 0.1, c0, c0 + 0.1])
    N = len(v)
    others = [(np.sqrt(s / N), t, r) for s, t, r, _ in res if not (t == "*%s*" % ch and not r)]
    self_rms = [np.sqrt(s / N) for s, t, r, _ in res if t == "*%s*" % ch and not r][0]
    worst.append((others[0][0], ch, others[0][1][1] + ("<" if others[0][2] else "")))
    print("%s self %.0f | nearest %s" % (ch, self_rms, "  ".join(
        "%s%s %.0f" % (t[1], "<" if r else "", d) for d, t, r in others[:3])), flush=True)
worst.sort()
print("closest pairs:", ", ".join("%s~%s %.0f" % (a, b, d) for d, a, b in worst[:8]))
print("time %.0f s" % (time.time() - t0))
