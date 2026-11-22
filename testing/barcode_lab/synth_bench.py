"""Synthetic normal-card passes at the 1 cm blur, both decoders.

usage: synth_bench.py gen N_PER_LETTER SEED   -> writes syn/*.txt + syn/truth.tsv
       synth_bench.py match                    -> template matcher on syn/, syn/match.tsv
The optics are drawn around the fit to the 30 Oct big-card passes
(sigma 3.3-3.8 mm, c0 0.45-0.55, gamma ~1, black ~3150, white ~250 then;
31 Oct white 165-190)."""
import os
import sys
import numpy as np
from model import bars

UNIT = 0.3215
LET = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"


def render(text, rev, m, sigma, c0, gamma, white, black, stretch_warp, rng,
           pre=150.0, post=100.0):
    iv, L = bars(text, 3.0, 1.0, rev)
    span = L * m
    xs = np.arange(0.0, pre + span + post, UNIT)       # car path, mm
    # card coordinate along the path: linear stretch plus a slow warp
    u = (xs - pre) / span
    xc = (xs - pre) * (1.0 + stretch_warp * (u - 0.5))
    fine = np.arange(-30, span + 30, 0.05)
    r = np.zeros_like(fine)
    for lo, hi in iv:
        r[(fine >= lo * m) & (fine < hi * m)] = 1.0
    k = np.arange(-int(4 * sigma / 0.05) - 1, int(4 * sigma / 0.05) + 2) * 0.05
    g = np.exp(-0.5 * (k / sigma) ** 2)
    g /= g.sum()
    c = np.convolve(r, g, mode="same")
    cov = np.interp(xc, fine, c, left=0.0, right=0.0)
    t = np.clip((cov - c0) / (1 - c0), 0, 1) ** gamma
    v = white + (black - white) * t
    sd = rng.uniform(6, 12) + float(os.environ.get("DARKNOISE", "0.012")) * (v - white)
    v = v + rng.normal(0, 1, len(v)) * sd
    drop = rng.random(len(v)) < rng.uniform(0.0, 0.12)
    v = np.round(np.clip(v, 0, 4095))
    v[drop] = np.nan
    return xs, v


def gen(n_per, seed, out="syn", sig=(3.0, 4.0), c0r=(0.42, 0.58)):
    rng = np.random.default_rng(seed)
    os.makedirs(out, exist_ok=True)
    rows = []
    k = 0
    for rep in range(n_per):
        for ch in LET:
            rev = bool(rng.integers(0, 2))
            m = 3.05 * rng.uniform(1.0, 1.12)        # print + crossing angle
            prm = dict(sigma=rng.uniform(*sig), c0=rng.uniform(*c0r),
                       gamma=rng.uniform(0.9, 1.15), white=rng.uniform(165, 195),
                       black=rng.uniform(2900, 3300), warp=rng.uniform(-0.06, 0.06))
            xs, v = render("*%s*" % ch, rev, m, prm["sigma"], prm["c0"], prm["gamma"],
                           prm["white"], prm["black"], prm["warp"], rng)
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
            rows.append((name, ch, int(rev), m, prm["sigma"], prm["c0"], prm["gamma"],
                         prm["white"], prm["black"], prm["warp"]))
            k += 1
    with open(os.path.join(out, "truth.tsv"), "w") as f:
        for r in rows:
            f.write("\t".join(str(q) for q in r) + "\n")
    print(k, "passes")


DIR = os.environ.get("SYN", "syn")


def one(name):
    from fast import match2, candidates
    from groups import find_groups
    xs, v = np.load(os.path.join(DIR, name[:-4] + ".npy"))
    p = dict(v=v, du=UNIT, white=None)
    gl = [g for g in find_groups(p, k_sd=5.0, gap_mm=55.0) if g["len_mm"] >= 60]
    if not gl:
        return name, "-", 0, 0.0, 0.0, "-"
    g = max(gl, key=lambda g: g["len_mm"])
    a_mm, b_mm = xs[g["a"]], xs[min(g["b"], len(xs) - 1)]
    keep = (xs >= a_mm - 30) & (xs <= b_mm + 30) & np.isfinite(v)
    fine, _ = match2(xs[keep], v[keep], a_mm, b_mm - a_mm,
                     candidates(os.environ.get("CHARS", "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. ")),
                     sig_grid=[1.75, 2.5, 3.25, 4.0, 4.75], c0_grid=[0.2, 0.32, 0.42, 0.52, 0.62],
                     m_lo=0.95, m_hi=1.32)
    best = fine[0]
    other = next(r for r in fine if r[1] != best[1])
    N = keep.sum()
    return (name, best[1][1], int(best[2]), other[0] / max(best[0], 1e-9),
            np.sqrt(best[0] / N), other[1][1])


if __name__ == "__main__":
    if sys.argv[1] == "gen":
        gen(int(sys.argv[2]), int(sys.argv[3]), sys.argv[4],
            tuple(float(q) for q in sys.argv[5].split(",")), tuple(float(q) for q in sys.argv[6].split(",")))
    elif sys.argv[1] == "match":
        from multiprocessing import Pool
        names = sorted(f for f in os.listdir(DIR) if f.startswith("syn_") and f.endswith(".txt"))
        with Pool(int(sys.argv[2]) if len(sys.argv) > 2 else 8) as pool:
            out = pool.map(one, names, chunksize=2)
        with open(os.path.join(DIR, "match.tsv"), "w") as f:
            for r in out:
                f.write("%s\t%s\t%d\t%.3f\t%.1f\t%s\n" % r)
        print("done", len(out))
