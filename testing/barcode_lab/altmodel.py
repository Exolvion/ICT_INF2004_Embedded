"""A second sensor model, built differently from the decoder's on purpose:
a round spot (uniform disc of diameter D, edges softened by a Gaussian s)
and a smooth S-shaped response (logistic, steepness k, middle c0), instead
of the decoder's Gaussian spot and hinge.  Fitted to the same recorded
big-card passes, then used to make test data the decoder has never seen
the formula of.  Bars come from tec-it's images (tecit_bars.json)."""
import json
import os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DX = 0.05


def tecit_elements(ch, path=os.path.join(HERE, "tecit_bars.json")):
    """[(modules, is_bar)] for *ch* as tec-it draws it."""
    t = json.load(open(path))[ch]
    return [(m, i % 2 == 0) for i, m in enumerate(t["modules"])]


def disc_lsf(D, s):
    half = D / 2 + 4 * s
    k = np.arange(-half, half + DX, DX)
    lsf = np.sqrt(np.clip(1 - (2 * k / D) ** 2, 0, None))
    if s > 0.01:
        g = np.exp(-0.5 * (k / s) ** 2)
        lsf = np.convolve(lsf, g / g.sum(), mode="same")
    return lsf / lsf.sum()


def respond(c, c0, k):
    L = lambda z: 1 / (1 + np.exp(-k * (z - c0)))
    return (L(c) - L(0.0)) / (L(1.0) - L(0.0))


def render(els, m, ratio, gain, D, s, c0, k, reverse=False, pad=40.0):
    """Coverage-and-response profile on a 0.05 mm grid; returns (x, t)
    with x from the symbol's first edge.  ratio rescales wide elements,
    gain widens every bar (and narrows its neighbours) by gain mm."""
    if reverse:
        els = els[::-1]
    widths = [(ratio if w > 1.5 else 1.0) * m for w, _ in els]
    total = sum(widths)
    x = np.arange(-pad, total + pad, DX)
    r = np.zeros_like(x)
    pos = 0.0
    for wd, (w, bar) in zip(widths, els):
        if bar:
            r[(x >= pos - gain / 2) & (x < pos + wd + gain / 2)] = 1.0
        pos += wd
    c = np.convolve(r, disc_lsf(D, s), mode="same")
    return x, respond(np.clip(c, 0, 1), c0, k)
