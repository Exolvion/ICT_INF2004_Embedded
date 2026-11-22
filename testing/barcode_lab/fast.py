"""Fast template matching of a card pass against Code 39 symbols.

A candidate is (text, reversed).  For each candidate, module width m,
aperture sigma, start x0 and dead zone c0 on grids, the pass is fitted as
v = a + b * S, S = clip((coverage - c0) / (1 - c0), 0, 1); the score is the
residual sum of squares.  Returns candidates sorted by their best SSE.
"""
import numpy as np
from model import bars

DX = 0.1
PAD = 40.0
_cache = {}


def cover_fine(text, rev, m, sigma, ratio=3.0, gap=1.0):
    key = (text, rev, round(m, 4), round(sigma, 3), ratio, gap)
    c = _cache.get(key)
    if c is not None:
        return c
    iv, L = bars(text, ratio, gap, rev)
    n = int((L * m + 2 * PAD) / DX) + 1
    r = np.zeros(n)
    for lo, hi in iv:
        a = int(round((lo * m + PAD) / DX))
        b = int(round((hi * m + PAD) / DX))
        r[a:b] = 1.0
    k = np.arange(-int(4 * sigma / DX) - 1, int(4 * sigma / DX) + 2) * DX
    g = np.exp(-0.5 * (k / sigma) ** 2)
    g /= g.sum()
    c = np.convolve(r, g, mode="same")
    if len(_cache) > 20000:
        _cache.clear()
    _cache[key] = c
    return c


def candidates(letters="ABCDEFGHIJKLMNOPQRSTUVWXYZ"):
    return [("*%s*" % ch, rev) for ch in letters for rev in (False, True)]


def match(x, v, cands, m_grid, x0_grid, sigma_grid, c0_grid, clip=None,
          ratio=3.0, gap=1.0, keep=None):
    """x: sample positions (mm), v: values (finite).  x0 is the symbol's
    first edge.  Returns sorted list of (sse, text, rev, params)."""
    v = np.asarray(v, float)
    N = len(v)
    vm = v - v.mean()
    Svv = float(vm @ vm)
    out = []
    x0g = np.asarray(x0_grid, float)
    c0g = np.asarray(c0_grid, float)
    for text, rev in cands:
        best = (np.inf, None)
        for m in m_grid:
            for sg in sigma_grid:
                cf = cover_fine(text, rev, m, sg, ratio, gap)
                idx = np.rint((x[None, :] - x0g[:, None] + PAD) / DX).astype(int)
                inside = (idx >= 0) & (idx < len(cf))
                C = np.where(inside, cf[np.clip(idx, 0, len(cf) - 1)], 0.0)  # K x N
                S = np.clip((C[None, :, :] - c0g[:, None, None]) /
                            (1.0 - c0g[:, None, None]), 0.0, 1.0)       # J x K x N
                Sm = S - S.mean(axis=2, keepdims=True)
                Sss = np.einsum("jkn,jkn->jk", Sm, Sm)
                Ssv = np.einsum("jkn,n->jk", Sm, vm)
                with np.errstate(divide="ignore", invalid="ignore"):
                    sse = np.where((Sss > 1e-9) & (Ssv > 0), Svv - Ssv ** 2 / Sss, Svv)
                j, k = np.unravel_index(np.argmin(sse), sse.shape)
                if sse[j, k] < best[0]:
                    best = (float(sse[j, k]), dict(m=m, sigma=sg, x0=x0g[k], c0=c0g[j],
                                                    b=float(Ssv[j, k] / max(Sss[j, k], 1e-9))))
        out.append((best[0], text, rev, best[1]))
    out.sort(key=lambda t: t[0])
    return out


def bin1(x, v, step=1.0):
    """Average samples into step-mm bins (skips empty bins)."""
    k = np.floor((x - x[0]) / step).astype(int)
    n = k.max() + 1
    s = np.bincount(k, v, n)
    c = np.bincount(k, None, n)
    ok = c > 0
    return x[0] + (np.arange(n)[ok] + 0.5) * step, s[ok] / c[ok]


def match2(x, v, a_mm, L_mm, cands, sig_grid, c0_grid, m_lo=0.92, m_hi=1.35,
           x0_lo=-25.0, x0_hi=12.0, top=6, letters_modules=47.0):
    """Coarse (1 mm bins) then fine search; returns sorted results."""
    xb, vb = bin1(x, v, 1.0)
    m0 = L_mm / letters_modules
    coarse = match(xb, vb, cands,
                   m_grid=m0 * np.arange(m_lo, m_hi + 1e-9, 0.02),
                   x0_grid=np.arange(a_mm + x0_lo, a_mm + x0_hi + 1e-9, 1.0),
                   sigma_grid=sig_grid, c0_grid=c0_grid)
    fine = []
    for sse, text, rev, prm in coarse[:top]:
        r = match(x, v, [(text, rev)],
                  m_grid=prm["m"] * np.arange(0.98, 1.0201, 0.005),
                  x0_grid=prm["x0"] + np.arange(-1.5, 1.51, 0.25),
                  sigma_grid=[prm["sigma"] * f for f in (0.85, 1.0, 1.15)],
                  c0_grid=[max(0.0, prm["c0"] + d) for d in (-0.05, 0.0, 0.05)])
        fine.append(r[0])
    fine.sort(key=lambda t: t[0])
    return fine, coarse
