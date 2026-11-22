import numpy as np
from model import elements
LET = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
def wide_centres(text, rev=False):
    el = elements(text)
    if rev: el = el[::-1]
    x = 0; out = []
    for w, b in el:
        if b and w > 1: out.append(x + w / 2)
        x += w
    return np.array(out)
assert all(len(wide_centres("*%s*" % c)) == 6 for c in LET)
groups = {}
for c in LET:
    for rev in (False, True):
        groups.setdefault(tuple(wide_centres("*%s*" % c, rev)), []).append(c + ("<" if rev else ""))
print("letters sharing all 6 wide-bar positions:", [g for g in groups.values() if len(g) > 1])
def aff(a, b):
    A = np.vstack([np.ones(6), b]).T
    coef, *_ = np.linalg.lstsq(A, a, rcond=None)
    return np.sqrt(np.mean((a - A @ coef) ** 2))
pairs = []
for c in LET:
    a = wide_centres("*%s*" % c)
    for d in LET:
        for rev in (False, True):
            if d == c and not rev: continue
            pairs.append((aff(a, wide_centres("*%s*" % d, rev)), c, d + ("<" if rev else "")))
pairs.sort()
print("closest distinct pairs (RMS modules):", ", ".join("%s~%s %.2f" % (b, c, a) for a, b, c in [p for p in pairs if p[0] > 1e-9][:14]))
