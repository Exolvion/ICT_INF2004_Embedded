# Trial barcode decoder: what changed and how to test it

This is a **trial** bench image for the MISSION run (S3-U07). It is the bench image now on the car
(source `f63947b6`, image `3414a534`) with a second barcode decoder added. The second decoder runs
alongside the first and prints its own `NEW` lines.

Nothing else changes:

- the darkness-rule decoder's `CODE` and `GROUP` lines;
- the driving, the straight hold over cards and the line following;
- every other line in the log.

The trial lives in `testing/barcode_lab/trial_firmware/`. The repo's `firmware/bench/bench_main.c`
is untouched.

## Why

At 1 cm the side sensor's spot is about as wide as a small card's narrow bar (3 mm), so the narrow
bars blur away. The current decoder must see every bar, one by one, and on small cards it can't.
The new decoder doesn't count bars. For each Code 39 character it asks: "if this card said X, what
would the sensor read, given its blur?" It then picks the character whose prediction fits the
reading best.

## How the new decoder works

1. **Wide bars first.** Every `*X*` card has six wide bars, and their positions narrow the 39
   characters down to a few with the same wide-bar layout. The six largest dark humps are taken as
   the wide bars, and their centres are fitted to each candidate's wide-bar centres (both reading
   directions). The 10 closest candidates go on.
2. **Sensor model.** For each candidate the bars are blurred by a round spot (Gaussian, size
   *spot*), and a dead zone (*dead*) is applied: the reading only moves once that share of the spot
   is dark. The result is then scaled to the pass's white and black. The spot and dead zone belong
   to the sensor, not the card, so they are fitted once per pass, over the closest group of
   candidates. Then module width and start position are fitted for every candidate.
3. **Decision.** The best fit wins only if the best *other* character fits at least **1.3 times**
   worse (the *margin*). Otherwise the line says `no read` and names both.

Nothing is hard-coded:

- The candidates come from the firmware's own Code 39 table: all 39 characters with two wide bars
  (`0-9`, `A-Z`, `-`, `.`, space), so digits compete with letters.
- Module width, start, reading direction, spot and dead zone are measured on every pass.
- There is no letter list, card size or track position anywhere.
- The constants come from the Code 39 spec (1:3 ratio, 47 modules, 3 mm narrow bar) or are search
  ranges wide enough for the sensor.

It runs on its own copy of each card's readings (1 mm bins), sliced into the line steps: at most
1.5 ms of each step.

- A card cut in two by the current decoder's 30 mm white gap is joined back together. The longest
  white stretch inside a blurred small card is 15 modules, so the join distance is 54 mm (18 modules
  at 3 mm).
- A second card can be collected while the first is still being fitted.

## What the NEW lines say

| Line | Meaning |
|---|---|
| `NEW *Z* rev, margin 1.86 over *T* rev; module 6.05 mm, spot 3.3 mm, dead 0.35; at 2558-2843 mm; 412 ms, 290 steps, 4188k tests` | Read `*Z*` end to start (`rev`). The next-best character `*T*` fits 1.86 times worse. Module, spot and dead zone as fitted. Position along the side sensor's path. CPU time, line steps and work used. |
| `NEW no read: best *Z* rev, margin 1.06 over *T* rev; ...` | The best guess, but not sure enough (needs 1.30). Not a read. |
| `NEW no read: the 6 widest bars fit no character (8.5 mm off); at ...` | The six largest humps aren't a Code 39 wide-bar layout (often extra dark beside the card: a line, a sheet's edge). |
| `NEW no read: dark for 520 mm, longer than a card` | Too long to be one card. |
| `NEW skipped: still fitting the last card` | Two cards were already waiting. Rare. |
| `NEW decoder: 3 read, 2 no read, 0 skipped, 0 lines lost (margin needed 1.30)` | Totals, after STOP, just before the `barcodes:` line. |

`, cuts joined` at the end of a line means two cuts were joined into one card.

## How to test

Same set-up as the normal S3-U07 runs: 1 cm sensor height on the side mount, broker on the laptop,
START on the car.

1. **Flash** `bench_motion_mqtt_TRIAL.uf2` (copy the file onto the Pico's drive; don't drag it out
   of a zip).
2. **Start the broker** on the laptop (Docker, `eclipse-mosquitto:2`, port 1883, as in TESTING.md
   §3) and **log everything** from Git Bash in the repo folder:

   ```bash
   docker exec mqtt mosquitto_sub -t inf2004/is29/telemetry -N -W 10800 >> testing/subsystem3/logs/$(date +%F)_S3-U07_trial_live.txt
   ```

   To watch the codes live in a second window:

   ```bash
   tail -n 0 -f testing/subsystem3/logs/$(date +%F)_S3-U07_trial_live.txt | grep -a --line-buffered -o "DRIVE start.*\|CODE .*\|GROUP .*\| NEW .*\|MISSION aborted.*\|barcodes: .*"
   ```

3. **Run full laps** with START, small and big cards both on the track, and STOP at the end. The
   car must not be backed up to reread a card: a miss stays a miss.
4. **Write down the cards.** Before each run, note every card's letter and size in track order
   (e.g. `small Z, big A, small Y`). Other people print A or Y cards sometimes, so check, don't
   assume. Without this list the log can't be scored.
5. If you can, try a few different letters on small cards (any A-Z from
   <https://barcode.tec-it.com/en/Code39>). The trial was built for small cards.

**Send back:** the `_trial_live.txt` log(s) and the card list for each run. A photo of the cards on
the track helps.

**What to look for:**

- **Small cards:** `NEW *X*` lines where there is no `CODE` line. That is what this trial is for.
- **Wrong reads:** any `NEW *X*` whose letter isn't the card's. These matter most. None happened
  in any test so far.
- **CPU:** the `ms` and `steps` figures, and whether the car drives as smoothly as before (no
  encoder `err`, no FAULT). The fit's CPU time on the RP2040 has never been measured, only
  estimated (below).

**Roll back:** flash `bench_motion_mqtt_CURRENT.uf2`, the current decoder built for the same Wi-Fi.

## The evidence so far, and its limits

All offline: there is no car at hand. The C code was run in the host simulator (`sim`, the
firmware's own source with a simulated car). The "current" column is the darkness-rule decoder on
the same passes. Right / wrong / no read, margin 1.3:

| Test passes | Current | NEW | Either right |
|---|---|---|---|
| Small cards, 1 cm, bars from tec-it's images, 2nd sensor model (104) | 0 / 0 / 104 | **84 / 0 / 20** | 84 |
| Small cards, sharper spot, same (104) | 10 / 1 / 93 | 36 / 0 / 68 | 44 |
| Big cards, 1 cm, same (52) | 8 / 0 / 44 | 14 / 0 / 38 | 20 |
| Small cards, spot 2-3 mm, decoder's own model, noisy (104) | 8 / 0 / 96 | **95 / 0 / 9** | 96 |
| Small cards, spot 2-3 mm, decoder's own model (104) | 64 / 0 / 40 | 95 / 0 / 9 | 104 |
| Small cards, spot 3-4 mm, decoder's own model, noisy (104) | 0 / 0 / 104 | **68 / 0 / 36** | 68 |
| Small cards, spot 3-4 mm, decoder's own model (104) | 0 / 0 / 104 | 68 / 0 / 36 | 68 |

The synthetic passes are A-Z (plus digits as rivals). They use bar widths taken from the tec-it
images and blur measured on the 30 Oct big `*A*` passes. The "2nd sensor model" is a different
formula (round spot, S-shaped response) fitted to the same passes, so the decoder isn't only tested
against its own assumptions.

**Real recordings** (30 Oct cuts and the 29-30 Oct traces):

| Pass | Current | NEW |
|---|---|---|
| Big `*Z*`, 30 Oct static | `*Z*` | `*Z*` (1.86) |
| Big `*Z*`, run 4 trace | `*Z*` | `*Z*` (2.22) |
| `*F*` (as read), 30 Oct mission | `*F*` | `*F*` (1.32) |
| Big `*Z*`, three 30 Oct mission passes | `*Z*` | no read: best `*Z*` at 1.06 and 1.10; one pass at 1.00 |
| Small card, run 6 trace | `*Y*` | no read (the sheet's edge adds a hump) |
| Two small cards, 30 Oct (letters not known) | no read | no read |

Read this honestly:

- **The case for NEW rests on simulated small cards.** On the only real small-card passes recorded
  (three), it read none, and the current decoder read one.
- **On real big cards the current decoder is surer.** NEW only abstains (no read), never misreads.
  On those 30 Oct passes the narrow bars are almost invisible, so `*Z*` and `*T*` (same wide bars)
  differ by a few percent. The Python version of the matcher agreed (1.03).
- **No wrong NEW read** in any test at margin 1.3: 676 synthetic passes plus the real ones.
- Old decoder unchanged, checked by:
  - the sim output with NEW lines removed, byte for byte, on 150 synthetic passes;
  - every recorded cut;
  - the standard regressions (traces `*Z*` + `*Y*`; cuts 6 `*Z*` + 2 `*F*`; batch 28 right,
    0 wrong).
- **CPU on the RP2040 is estimated, not measured** (see the build notes). The fit slices itself
  to 1.5 ms per line step, so even if the estimate is off, the step stays short and only the fit
  takes longer. The NEW line reports the real figure.

## Build notes

Checked on trial source `cc972ba5` (current source `f63947b6`), with the bench-only check set:

- strict compile (`-Wall -Wextra -Wshadow -Wpedantic`) for `BENCH_MOTION` 0 and 1, with no warnings;
- the offline, mqtt and mqtt-motion bench images all build;
- the sim regressions and the comparisons above.

| | Current | Trial |
|---|---|---|
| Flash (text) | 425,576 B | 442,628 B (+17 KB) |
| Static RAM (bss) | 120,444 B | 144,036 B (+24 KB, of 264 KB) |
| Deepest stack to the fit | — | about 1.7 KB of the bench task's 4 KB |

**CPU, estimated from the compiled inner loop** (about 31 cycles per bin test, at 125 MHz):

| Card | CPU | Driving time while it's fitted (1.5 ms per 6.75 ms step) |
|---|---|---|
| Big | about 1.1 s | about 6 s |
| Small | about 0.6 s | about 3 s |

At 80 mm/s that is 0.3–0.5 m of track. A second card in that time waits in the second slot; only a
third is skipped. The control loop never gets more than 1.5 ms more per step. A coarser search
halved the work but lost reads (big cards 14 → 7), so it was not used.

## Files

| File | What |
|---|---|
| `bench_main.c` | The full trial source (the current source with the decoder added) |
| `bench_main.orig.c` | The current source, `f63947b6`, for comparison |
| `trial.patch` | The difference: one changed line, the rest added |
| `fit_block.c` | The decoder alone, as added at the end of the file |
| `TRIAL_DECODER.md` | This file |

The hooks in `bench_main.c`, each marked `TRIAL`:

- copy each finished cut before it is dropped (`bench_fit_begin`);
- a slice of the fit every line step (`bench_fit_run`);
- the NEW line goes out with the CODE lines (`bench_fit_text`);
- reset at the start;
- finish and count after STOP.

To rebuild (WSL, a copy of the repo with the trial source put over `firmware/bench/bench_main.c`):

```bash
bash firmware/bench/build.sh mqtt motion <laptop IP>
```

The image holds the Wi-Fi name and password in plain text: don't pass it on.
