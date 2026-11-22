# Subsystem 3 — test log

**Subsystem:** Optical: IR line following, barcodes (Buddy 3, Tan Yi Jian Ethan)  ·  **Student ID:** 2501528  ·  **Code:** `app_program/subsystem3/` (`line_ctrl.c/.h`, `barcode.c/.h`, `course_cfg.c/.h`)

See `testing/README.md` for how to run tests and record results, and
`TESTING.md` for the set-up. P2 = after TESTING.md section 4; bench = with a
bench program (TESTING.md section 5).

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | Line controller and barcode decoder (exhaustive and lockstep) | FR1, FR2 | auto | `bash scripts/dev.sh host-tests test_line_main` | "Passed 1 executable(s) in both host configurations" |
| S3-U02 | IR sensors respond | FR1 | manual | P2: move each sensor between black tape and white paper | Grove status LEDs for GP6 (left) and GP7 (barcode) toggle; the right module's own D0 LED toggles |
| S3-U03 | IR A0 readings on black and white | FR1, A3 | manual | Bench: at least 20 readings per sensor on the track's black and white | Clear separation; `ROBOT_IR_*_WHITE/BLACK` entered |
| S3-U04 | Narrowest bar is resolvable | FR2, R3 | manual | Calculate counts per 3.0 mm bar from S2-U05 at the chosen speed | At least 5 counts per narrow bar |
| S3-U05 | Codebook | FR2, A4 | manual | Measure bar widths on the printed sample; confirm codes and actions with the lecturer; enter in `course_cfg.c` | Lecturer-confirmed codebook entered |
| S3-U06 | Junction approach distance | FR3, A4 | manual | Bench: measure travel from junction-edge detection to the axle over the crossbar centre | `ROBOT_SENSOR_AXLE_OFFSET_M` entered |
| S3-U07 | Barcode read on the track at run time, no codebook | FR2 | manual | Motion bench image, BARCODE test (gain set 4), car on the line with about 10 cm of white before a card on its left, START. Each card on the route, both directions | The card's Code 39 text decoded from the bar widths alone, read in order or end to start; the 1 mm trace shows every bar |

*Method:* **auto** = run by `scripts/dev.sh`, result in the saved log;
**manual** = a person observes the result (telemetry, console, meter, video).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_S3-U01.txt` | |
| 2026-10-29 | S3-U03 | `166e23f` + uncommitted | Tan Yi Jian Ethan | FAIL | `logs/2026-10-29_S3-U07_summary.txt` | Barcode sensor only, school track: white ~205, line ~1975. Clear separation, but the tape values in `robot_config.h` (137/4043) do not fit this track and were not changed |
| 2026-10-29 | S3-U07 | `166e23f` + uncommitted | Tan Yi Jian Ethan | FAIL | `logs/2026-10-29_S3-U07_summary.txt` | 3 runs (bench images `a066e62a`, `3e27c236`). Only the 18 mm thick bars show; the 6 mm thin bars and the spec-size card do not resolve with the sensor 2 cm up. Defined after the first run |
| 2026-10-31 | S3-U07 | `28ff7fc` + uncommitted (bench image `3414a534`) | Tan Yi Jian Ethan | FAIL | `logs/2026-10-31_S3-U07_summary.txt`, `logs/2026-10-31_S3-U07_live.txt` | MISSION at 80 mm/s, sensor 1 cm up on the side mount. 53 runs: 8 confirmed codes (`*Z*` normal x2, `*A*` normal x4, big `*Z*` and big `*A*` once each), all correct, no wrong code. Most misses lost bars (blurred narrow gaps merge bars). Width spec check refused 2 probably-right `*A*` frames and 2 probably-wrong (`*M*`, `*L*`). Sensor white rose to ~1400 for a few runs, then recovered. Only read in order (end-to-start not seen) |
