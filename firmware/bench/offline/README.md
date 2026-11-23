# Offline barcode decoder tools

Replay recorded barcode readings through the bench decoder on a PC, without the car. Use them to
check a decoder change before building and flashing an image.

| Folder | What |
|---|---|
| `sim/` | Host harness: `../../bench_main.c` in one translation unit with a simulated car (`sim.c`). |
| `recordings/2026-10-30_cuts/` | Card readings cut from the 30 Oct end-of-run dumps (one file per card pass). |
| `recordings/2026-10-31_cuts/` | Cuts from the 31 Oct live RAW lines; only 2 have samples (the car was stopped before the rest were sent). |
| `traces/` | 1 mm traces of whole runs from 29-30 Oct. |
| `scripts/` | `evalcuts.sh` (replay every cut), `raw2dump.py` (live RAW lines to a cut), `splitdumps.py` (an end-of-run dump to one file per cut), `rawplot.py`, `uf2grep.py`, `fullcheck.sh`. |
| `barcode_research_2026-10-31.md` | How other projects and patents read barcodes with blurred sensors. |

## Build and run (WSL, from this folder)

```bash
cd sim && bash make.sh                      # builds ./sim from the repo's current source
./sim stream ../traces/run4_trace.txt ../traces/run6_trace.txt | grep -a "CODE\|barcodes:"
bash ../scripts/evalcuts.sh ./sim ../recordings/2026-10-30_cuts ../recordings/2026-10-31_cuts
./sim batch | tail -n 1                     # 384 synthetic MISSION runs: right / wrong / one-swing counts
```

Expected with source `f63947b6` (image `3414a534`): the traces read `*Z*` end to start and `*Y*` in
order; the cuts give 6 `*Z*` and 2 `*F*` (all end to start); the batch gives 28 right, 0 wrong.

`make.sh` copies the repo into `sim/proj/` and writes `sim/sim`; both are build products (ignored).
`fullcheck.sh <broker IPv4>` is the full pre-flash check set; it copies this repository to a WSL-native tree first.
