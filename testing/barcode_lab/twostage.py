"""Two-stage decoder, cheap enough for the car.

Stage 1: the six wide bars.  Every A-Z letter and '*' has exactly two wide
bars, so *X* has six.  Humps above the pass's white are found; the six with
the most darkness area are the wide bars; their centres are fitted
(least squares, start x0 + module m) to each candidate's six wide-bar
centres.  Candidates sharing the six positions form one class.
Stage 2: inside the best class (and the next class if close), the waveform
is template-matched with x0 and m from stage 1 refined locally.
usage: twostage.py DIR  (synthetic set)  |  twostage.py real KEY@lo,hi ...
"""
import os
import sys
import numpy as np
from model import elements
from fast import match

from model import CHARS
# Every Code 39 character with two wide bars (all but $ / + %), straight
# from the pattern table; CHARSET=letters keeps A-Z only (the course rule).
ALL40 = "".join(c for c in CHARS if c not in "*$/+%")
LET = "ABCDEFGHIJKLMNOPQRSTUVWXYZ" if os.environ.get("CHARSET") == "letters" else ALL40


def wide_centres(text, rev=False):
    el = elements(text)
    if rev:
        el = el[::-1]
    x = 0.0
    out = []
    for w, b in el:
        if b and w > 1:
            out.append(x + w / 2)
        x += w
    return np.array(out)


CANDS = [("*%s*" % c, rev) for c in LET for rev in (False, True)]
WC = {k: wide_centres(*k) for k in CANDS}


def humps(x, v, white, sd, rel=0.15):
    """Humps between valley turning points: (centre of the part above half
    height, darkness area, peak).  A turning point needs a swing of 6 white
    noise sds or rel of the level above white, whichever is larger: dark
    readings are noisier than white ones, so a fixed swing splits the flat
    top of a wide bar into several humps."""
    good = np.isfinite(v)
    x, v = x[good], v[good]
    dxm = float(np.median(np.diff(x)))
    k = max(1, int(round(1.0 / dxm)))                       # 1 mm box
    vs = np.convolve(v, np.ones(k) / k, mode="same")
    n = len(vs)
    sw = 6 * sd
    need = lambda lvl: max(sw, rel * (lvl - white))
    ext = []                                              # ("p"|"v", index)
    hi_i = lo_i = 0
    mode = 0                                              # 0 start, 1 rising, -1 falling
    for j in range(1, n):
        if mode == 0:
            if vs[j] < vs[lo_i]:
                lo_i = j
            if vs[j] - vs[lo_i] >= need(vs[j]):
                ext.append(("v", lo_i))
                mode, hi_i = 1, j
        elif mode == 1:
            if vs[j] > vs[hi_i]:
                hi_i = j
            elif vs[hi_i] - vs[j] >= need(vs[hi_i]):
                ext.append(("p", hi_i))
                mode, lo_i = -1, j
        else:
            if vs[j] < vs[lo_i]:
                lo_i = j
            elif vs[j] - vs[lo_i] >= need(vs[j]):
                ext.append(("v", lo_i))
                mode, hi_i = 1, j
    if mode == 1:
        ext.append(("p", hi_i))
    ext.append(("v", n - 1))
    out = []
    for i, (t, p) in enumerate(ext):
        if t != "p" or vs[p] < white + 6 * sd:
            continue
        a = ext[i - 1][1] if i > 0 else 0
        b = ext[i + 1][1] if i + 1 < len(ext) else n - 1
        seg = slice(a, b + 1)
        base = max(vs[a], vs[b])
        half = base + 0.5 * (vs[p] - base)
        w = np.clip(vs[seg] - half, 0, None)
        if w.sum() <= 0:
            continue
        c = float((x[seg] * w).sum() / w.sum())
        area = float(np.clip(vs[seg] - white, 0, None).sum() * dxm)
        out.append((c, area, float(vs[p])))
    return out


def stage1(x, v, white, sd):
    hs = humps(x, v, white, sd)
    if len(hs) < 6:
        return None, hs
    top = sorted(hs, key=lambda h: -h[1])[:6]
    pos = np.sort([h[0] for h in top])
    res = []
    for k in CANDS:
        c = WC[k]
        A = np.vstack([np.ones(6), c]).T
        coef, *_ = np.linalg.lstsq(A, pos, rcond=None)
        r = pos - A @ coef
        res.append((float(np.sqrt(np.mean(r * r))), k, float(coef[0]), float(coef[1])))
    res.sort(key=lambda t: t[0])
    return res, hs


def decode(x, v, white, sd, margin=1.2):
    s1, hs = stage1(x, v, white, sd)
    if s1 is None:
        return dict(letter=None, why="%d humps" % len(hs))
    best_rms = s1[0][0]
    if best_rms > 0.35 * s1[0][3]:
        # the six widest humps fit no character's wide-bar layout
        return dict(letter=None, why="stage 1 misfit %.2f mm" % best_rms)
    # the 8 best layouts, plus any within 1.5x the best rms + 0.4 mm
    lim = max(best_rms * 1.5, best_rms + 0.4)
    near = [t for i, t in enumerate(s1) if i < 8 or t[0] <= lim]
    from fast import bin1
    xb, vb = bin1(x, v, 1.0)
    out = []
    for rms, k, x0c, m in near:
        # coarse on 1 mm bins around the stage-1 alignment, then fine
        r = match(xb, vb, [k],
                  m_grid=m * np.arange(0.95, 1.0501, 0.01),
                  x0_grid=x0c + np.arange(-4.0, 4.01, 0.5),
                  sigma_grid=[1.5, 2.25, 3.0, 3.75, 4.5],
                  c0_grid=[0.2, 0.35, 0.45, 0.55, 0.65])
        p = r[0][3]
        r = match(x, v, [k],
                  m_grid=p["m"] * np.arange(0.99, 1.0101, 0.005),
                  x0_grid=p["x0"] + np.arange(-0.75, 0.76, 0.25),
                  sigma_grid=[p["sigma"] * f for f in (0.9, 1.0, 1.1)],
                  c0_grid=[max(0.0, p["c0"] + d) for d in (-0.05, 0.0, 0.05)])
        out.append((r[0][0], k, rms, m))
    out.sort(key=lambda t: t[0])
    best = out[0]
    other = next((o for o in out if o[1][0] != best[1][0]), None)
    ratio = (other[0] / max(best[0], 1e-9)) if other else 99.0
    return dict(letter=best[1][0][1], rev=best[1][1], ratio=ratio, s1rms=best[2],
                m=best[3], n_near=len(near),
                other=other[1][0][1] if other else "-", ok=ratio >= margin)


if __name__ == "__main__":
    d = sys.argv[1]
    truth = {}
    for l in open(os.path.join(d, "truth.tsv")):
        f = l.rstrip("\n").split("\t")
        truth[f[0]] = f[1]
    right = wrong = none = 0
    rows = []
    for name in sorted(truth):
        xs, v = np.load(os.path.join(d, name[:-4] + ".npy"))
        ok = np.isfinite(v)
        white = float(np.median(v[ok][:300]))
        sd = float(np.std(v[ok][:300]))
        # crop to the card: first/last sample above white + 6 sd, +-30 mm
        dark = np.where(ok & (np.nan_to_num(v) > white + 6 * sd))[0]
        a, b = xs[dark[0]] - 30, xs[dark[-1]] + 30
        keep = ok & (xs >= a) & (xs <= b)
        r = decode(xs[keep], v[keep], white, sd)
        t = truth[name]
        if r["letter"] is None or not r["ok"]:
            none += 1
            tag = "none"
        elif r["letter"] == t:
            right += 1
            tag = "right"
        else:
            wrong += 1
            tag = "WRONG"
        rows.append((name, t, r.get("letter"), r.get("ratio", 0), r.get("s1rms", 0), r.get("other"), tag))
    for row in rows:
        if row[-1] != "right":
            print("%s truth %s got %s x%.2f s1 %.2f next %s %s" % row)
    print("%s: right %d, WRONG %d, none %d of %d" % (d, right, wrong, none, len(rows)))
