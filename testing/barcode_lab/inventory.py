"""Every card group in every pass: source, length, peak, logged decode."""
import pickle
import numpy as np
from passes import load_all
from groups import find_groups, fill

ps = load_all()
rows = []
for p in ps:
    if np.isfinite(p["v"]).sum() < 50:
        continue
    for g in find_groups(p):
        v = fill(p["v"])[g["a"]:g["b"]]
        rows.append(dict(key=p["key"], src=p["src"], a=g["a"], b=g["b"],
                         len_mm=g["len_mm"], w=g["w"], sd=g["sd"],
                         peak=float(v.max()), two=p.get("two_part", None),
                         logged=" || ".join(p["logged"])))
pickle.dump((ps, rows), open("inv.pkl", "wb"))
for r in rows:
    print("%-34s %6d-%6d len %6.1f mm white %5.0f sd %4.1f peak %5.0f two %s | %s" % (
        r["key"], r["a"], r["b"], r["len_mm"], r["w"], r["sd"], r["peak"], r["two"],
        r["logged"][:110]))
