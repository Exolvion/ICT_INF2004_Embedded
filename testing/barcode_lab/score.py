"""score.py DIR [threshold ...]: current decoder (sim.tsv) and template
matcher (match.tsv) against truth.tsv."""
import os
import sys
import collections

d = sys.argv[1]
ths = [float(t) for t in sys.argv[2:]] or [1.0, 1.1, 1.2, 1.3, 1.5]
truth = {}
for l in open(os.path.join(d, "truth.tsv")):
    f = l.rstrip("\n").split("\t")
    truth[f[0]] = (f[1], f[2] == "1", float(f[4]), float(f[5]))
n = len(truth)
if os.path.exists(os.path.join(d, "sim.tsv")):
    right = wrong = single_r = single_w = 0
    nb = collections.Counter()
    for l in open(os.path.join(d, "sim.tsv")):
        name, codes, single, bars = l.rstrip("\n").split("\t")
        ch = truth[name][0]
        cs = [c for c in codes.split() if c.startswith("*")]
        if cs:
            if all(c.strip("<") == "*%s*" % ch for c in cs):
                right += 1
            else:
                wrong += 1
        ss = [c for c in single.split() if c.startswith("*")]
        if ss and not cs:
            if ss[0].strip("<") == "*%s*" % ch:
                single_r += 1
            else:
                single_w += 1
        b = [int(q) for q in bars.strip(",").split(",") if q and q != "-"]
        nb[max(b) if b else 0] += 1
    print("current decoder: %d passes: right %d, WRONG %d, none %d  (one-swing only: right %d wrong %d)" % (
        n, right, wrong, n - right - wrong, single_r, single_w))
    print("  bars in the biggest group:", " ".join("%d:%d" % kv for kv in sorted(nb.items())))
if os.path.exists(os.path.join(d, "match.tsv")):
    rows = [l.rstrip("\n").split("\t") for l in open(os.path.join(d, "match.tsv"))]
    conf = collections.Counter()
    for th in ths:
        r = w = 0
        for name, ch, rev, ratio, rms, nxt in rows:
            if float(ratio) < th:
                continue
            if ch == truth[name][0]:
                r += 1
            else:
                w += 1
        print("template matcher, margin >= %.2f: right %d, WRONG %d, none %d" % (th, r, w, n - r - w))
    for name, ch, rev, ratio, rms, nxt in rows:
        if ch != truth[name][0]:
            conf["%s->%s" % (truth[name][0], ch)] += 1
    print("  top-choice confusions:", ", ".join("%s x%d" % kv for kv in conf.most_common()))
    wr = sorted(float(ratio) for name, ch, rev, ratio, rms, nxt in rows if ch != truth[name][0])
    rr = sorted(float(ratio) for name, ch, rev, ratio, rms, nxt in rows if ch == truth[name][0])
    print("  margins of wrong top choices:", " ".join("%.2f" % q for q in wr))
    if rr:
        print("  margins of right top choices: min %.2f median %.2f" % (rr[0], rr[len(rr) // 2]))
