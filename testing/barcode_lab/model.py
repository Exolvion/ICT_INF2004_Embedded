"""Code 39 layout and a sensor model: reflectance -> aperture blur ->
non-linear response.  Positions in mm."""
import numpy as np

CHARS = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. *$/+%"
PATS = [0x034, 0x121, 0x061, 0x160, 0x031, 0x130, 0x070, 0x025,
        0x124, 0x064, 0x109, 0x049, 0x148, 0x019, 0x118, 0x058,
        0x00D, 0x10C, 0x04C, 0x01C, 0x103, 0x043, 0x142, 0x013,
        0x112, 0x052, 0x007, 0x106, 0x046, 0x016, 0x181, 0x0C1,
        0x1C0, 0x091, 0x190, 0x0D0, 0x085, 0x184, 0x0C4, 0x094,
        0x0A8, 0x0A2, 0x08A, 0x02A]
PAT = dict(zip(CHARS, PATS))


def elements(text, ratio=3.0, gap=1.0):
    """[(width in modules, is_bar)] for text, gaps between characters."""
    out = []
    for k, ch in enumerate(text):
        p = PAT[ch]
        for e in range(9):
            wide = (p >> (8 - e)) & 1
            out.append((ratio if wide else 1.0, e % 2 == 0))
        if k + 1 < len(text):
            out.append((gap, False))
    return out


def modules(text, ratio=3.0, gap=1.0):
    return sum(w for w, _ in elements(text, ratio, gap))


def bars(text, ratio=3.0, gap=1.0, reverse=False):
    """Bar intervals in modules from the symbol's first edge."""
    el = elements(text, ratio, gap)
    if reverse:
        el = el[::-1]
    out = []
    x = 0.0
    for w, b in el:
        if b:
            out.append((x, x + w))
        x += w
    return out, x


def coverage(x, bar_iv, aperture, sigma):
    """Fraction of a box aperture (width aperture mm, edges softened by a
    Gaussian sigma mm) covered by bars, at positions x (mm)."""
    from scipy.special import erf
    c = np.zeros_like(x, dtype=float)
    a = max(aperture, 1e-3)
    s = max(sigma, 1e-3)

    def box_cdf(u):
        # integral of (box a convolved with gauss s), normalised, from -inf to u
        def F(t):  # integral of Phi(t/s)
            return t * 0.5 * (1 + erf(t / (s * np.sqrt(2)))) + \
                s / np.sqrt(2 * np.pi) * np.exp(-t * t / (2 * s * s))
        return (F(u + a / 2) - F(u - a / 2)) / a
    for lo, hi in bar_iv:
        c += box_cdf(hi - x) - box_cdf(lo - x)
    return np.clip(c, 0.0, 1.0)


def response(c, c0, gamma):
    t = np.clip((c - c0) / max(1e-3, 1.0 - c0), 0.0, 1.0)
    return t ** gamma


def synth(x, text, x0, m, white, black, aperture, sigma, c0, gamma,
          ratio=3.0, gap=1.0, reverse=False, clip=None):
    iv, _ = bars(text, ratio, gap, reverse)
    iv = [(x0 + lo * m, x0 + hi * m) for lo, hi in iv]
    v = white + (black - white) * response(coverage(x, iv, aperture, sigma), c0, gamma)
    if clip is not None:
        v = np.minimum(v, clip)
    return v
