"""Card groups in a pass: dark stretches above the pass's own white,
merged across white gaps shorter than GAP_MM; plus helpers."""
import numpy as np

GAP_MM = 25.0


def fill(v):
    v = v.copy()
    ok = np.isfinite(v)
    if ok.sum() < 2:
        return v
    idx = np.arange(len(v))
    v[~ok] = np.interp(idx[~ok], idx[ok], v[ok])
    return v


def white_sd(v):
    """Commonest level and the noise below it, as the car does."""
    x = v[np.isfinite(v)]
    if len(x) == 0:
        return 0.0, 1.0
    lo = np.percentile(x, 50)
    w = np.median(x[x <= np.percentile(x, 70)])
    below = x[x <= w]
    sd = max(1.0, (w - np.percentile(below, 16)) if len(below) > 5 else 1.0)
    return float(w), float(sd)


def find_groups(p, k_sd=6.0, gap_mm=GAP_MM, min_len_mm=20.0):
    v = fill(p["v"])
    if len(v) < 10:
        return []
    w, sd = white_sd(p["v"])
    thr = w + max(k_sd * sd, 25.0)
    dark = v > thr
    du = p["du"]
    gap = int(round(gap_mm / du))
    out = []
    i = 0
    n = len(v)
    while i < n:
        if not dark[i]:
            i += 1
            continue
        j = i
        last = i
        while j < n:
            if dark[j]:
                last = j
                j += 1
            elif j - last > gap:
                break
            else:
                j += 1
        if (last - i) * du >= min_len_mm:
            out.append((i, last + 1))
        i = last + 1
    return [dict(a=a, b=b, w=w, sd=sd, len_mm=(b - a) * du) for a, b in out]
