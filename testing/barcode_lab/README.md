# Barcode lab: offline decoder experiments (1 Nov 2026)

Offline only. Nothing here is built into the car and nothing here touches `firmware/`, `app_program/` or any
`.uf2`. The scripts read the recorded logs and recordings without changing them.

## What the recorded data says

**The sensor sees a blurred spot with a dead zone.** One model fits every blurred big-card pass from 30 Oct
evening (L30:12480, 12540, 13522, 13573, all `*A*`):

- the spot is a Gaussian with **σ ≈ 3.3–3.8 mm** (about 8 mm across at half height);
- the reading does not move until **about half the spot (c0 ≈ 0.45–0.55)** is dark, then rises linearly;
- the cards are Code 39 with wide/narrow **2.9–3.0** and a 6.0–6.2 mm module on the big card, so about 3 mm
  on the small card.

See `results/figures/calib.png`. Scaled to the small card, the same spot predicts:

- **Narrow 3 mm bars:** they cover about 33% of the spot, under the 50% dead zone, so a lone narrow bar gives
  **no signal at all**.
- **Wide 9 mm bars:** they peak about +1800 over white. That matches the 31 Oct summary (bar peaks 1500–1900).
- **Narrow bars beside a wide bar:** they show up only as a shoulder on that wide bar.

**Why the current decoder misses.** It counts bars and judges each bar's darkness and each space's width.
At 1 cm the narrow bars merge or vanish (31 Oct: groups of 5–14 bars instead of 15), or plateau noise
splits wide bars (16–33). Either way there is no Code 39 count, so no code. The older image read a small
`*Z*` as `*P*` (31 Oct). Z and P have identical bars and differ only in which space is wide, so a
space-width rule fails once the narrow bar next to that space is invisible.

**What survives the blur.** Every Code 39 character except `$ / + %` has exactly 2 wide bars, so a card
`*X*` always has 6. Their positions alone split the 26 letters into 10 groups (`widepos.py`). The groups are
at least 0.72 module (≈2 mm) apart:

| Unique from the wide bars | Groups that need the narrow-bar shoulders |
|---|---|
| G H I J S | A/K/U · B/L · C/M · D/N/V · E/O/W · F/P · Q/X · R/Y · T/Z |

## The decoders tried

1. `fast.py` / `realrun.py`, the **template matcher**. It renders every candidate `*X*` (both directions)
   through the blur model and fits the whole waveform: start, module, blur, dead zone, white and black. The
   smallest residual wins. The read is accepted only if the next different letter's residual is at least
   `margin` times larger.
2. `twostage.py`, the **two-stage** version, about 20× cheaper. First it finds the 6 humps with the most
   darkness area and fits their centres to each candidate's 6 wide-bar centres. Then it template-matches only
   the candidates close to that fit.

Nothing is hardcoded about the cards or the track:

- **Characters:** candidates come from the Code 39 pattern table at run time. That is every character with
  two wide bars (`0-9 A-Z - . space`), or A–Z only with `CHARSET=letters`.
- **Measured per pass:** module width, start, direction, blur and dead zone.
- **The one fixed print constant:** the wide/narrow ratio is 3.0, taken from the cards' measured 2.9–3.0
  (Code 39 allows 2–3, and a ratio search can be added).

## Results

**Synthetic small cards** (`synth_bench.py`): 104 passes per set, every letter A–Z 4×, random direction,
angle stretch up to 12%, speed warp ±6%, up to 12% missing samples. Each cell is right / wrong / none. The
current decoder is the real `bench_main.c` run through the offline sim (`runsim.sh`).

| Optics | Noise on dark readings | Current decoder | Template matcher, margin ≥ 1.2 | Two-stage, all 39 chars, margin ≥ 1.2 |
|---|---|---|---|---|
| sharper (σ 2–3 mm, c0 0.30–0.45) | high | 8 / 0 / 96 | 101 / 0 / 3 | 97 / 0 / 7 |
| sharper | low | 64 / 0 / 40 | 101 / 0 / 3 | 100 / 0 / 4 |
| **1 cm fit (σ 3–4 mm, c0 0.42–0.58)** | high | **0 / 0 / 104** | **79 / 0 / 25** | 78 / 0 / 26 |
| **1 cm fit** | low | **0 / 0 / 104** | **79 / 0 / 25** | 78 / 1 / 25 |

The one two-stage wrong read was O→E at a margin of 1.21. A margin of 1.3 refuses it. Wrong top choices
always had margins ≤ 1.12 for the matcher and ≤ 1.21 for the two-stage.

**Recorded passes** (`realrun.py`, results in `results/real_passes.txt`), margin ≥ 1.2:

| Passes | Truth | Matcher | Car on the day |
|---|---|---|---|
| big card, 30 Oct evening, 5 passes | `*A*` | 5 right (1.33–1.58) | 2 read |
| big card, 30 Oct, 10 passes | `*Z*` | 8 right (1.21–1.72), 2 none | 4 read |
| big card at 2 cm, 29 Oct, 2 traces (looks like the small card at 1 cm) | `*Z*` | 2 right but margins 1.14 / 1.08, so none at 1.2 | 0 read |
| 1 mm traces 30 Oct morning (cards confirmed on the day) | big `*Z*`, small `*Y*` | Z 2.20, Y 1.58 / 1.05 | |
| split or partial groups (the card finder cut big cards up) | | all ≤ 1.08, so none | |

There were **no wrong reads** at margin ≥ 1.2 on any recorded pass with a known card.

**Open question.** On 30 Oct around 20:05–20:37 the small card's `*Y*` was marked wrong. Those passes were
sharp: all 15 bars are visible, they line up with `*Y*` bar for bar, and `*R*` doesn't fit
(`results/figures/calib_RY.png`). If that card wasn't Y, it's a wrong read for both decoders.

## Limits, and what would help most

- **No small-card raw data from the current 1 cm side mount exists.** On 31 Oct the car announced 73 RAW
  dumps (53,495 units, about 17 m of sensor path), but only 2,035 units (3.8%) arrived before STOP. They came
  in 18 pieces of 27–67 mm, each the start of a dump, and none holds a whole card (a small card is ~142 mm).
  The new decoder cannot be scored on 31 Oct. The 1 cm results above rest on the model fitted to the big card (validated by the 31 Oct bar
  peaks) and on the 2 cm big-card stand-in. The most useful next car data is a few small-card passes with
  the car left still for ~15 s after the card, so the raw samples arrive.
- **The hardest pairs at 1 cm are exactly the test cards' groups:** A/K/U and T/Z, which depend on faint
  shoulders. A narrower spot (slit or tube, discussed 31 Oct) would help more than any decoder. The sensor
  height is a team decision.
- **Card grouping:** with narrow bars invisible, a small card can contain ~15 modules (~45 mm) of white,
  so the car's 20/30 mm group gaps split it. A port should derive the gap from the measured wide-bar width
  and not use a fixed distance.
- **Cost on the car:** the two-stage version is small apart from the template grid. Fixing blur and dead
  zone once per pass from the two `*` characters would cut it to a few hundred thousand operations.

## How to run (Windows, from this folder)

```bash
python -m venv venv && venv/Scripts/python -m pip install numpy scipy matplotlib
venv/Scripts/python inventory.py          # parse every recorded pass -> inv.pkl, inv list on stdout
venv/Scripts/python cut29.py              # the two 29 Oct 2 cm traces
venv/Scripts/python calib.py "L30:13573:*A*:f:80:420" "L30:12480:*A*:f:160:500"   # model fit -> calib.png
venv/Scripts/python realrun.py --gap=55 "L30:" "C30:" "T:" "T29"                 # match recorded passes
venv/Scripts/python widepos.py            # wide-bar position groups
venv/Scripts/python synth_bench.py gen 4 7 synB 3.0,4.0 0.42,0.58               # synthetic set (1 cm fit)
SYN=synB venv/Scripts/python synth_bench.py match 8 && venv/Scripts/python score.py synB 1.2 1.3
venv/Scripts/python twostage.py synB      # two-stage decoder on the set
```

The current-decoder baseline needs the offline sim built **outside the firmware tree**. Copy
`firmware/bench/offline/sim/{sim.c,make.sh,inc}` to `simcur/` here, then in WSL run
`bash simcur/make.sh <repo path>` and `bash runsim.sh synB`, then `score.py synB`.
