# Integration — test log

Tests that two or more subsystems work together. The owners on both sides
agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results, and
`TESTING.md` for the set-up. P4 = after TESTING.md section 6.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | Remote STOP brakes the car | FR10, FR11; S1 → Safety (S5) → Motion (S2) | manual | P4: while driving, `docker exec -i mqtt mosquitto_pub -t inf2004/is29/command -s < stop.bin` | Car stops; `state` FAULT, `stop_source` REMOTE |
| INT-02 | Remote STOP latency under load | NFR5; S1, S2 | manual | P4: 100 STOPs while driving; GPIO toggle at STOP validation and at motor brake, logic analyser (needs instrumentation) | Median, p99 and maximum recorded |
| INT-03 | Local STOP at speed | NFR5, FR11; Safety (S5), S2 | manual | P4: press GP20 at 0.08 m/s | Drive zero within 10 ms; stopping distance recorded |
| INT-04 | Encoder counts while driving | FR4; GPIO ISR (S5), S2 | manual | P4: drive both wheels; compare counts with wheel turns | No lost counts; no false `ENCODER` fault |
| INT-05 | Stall fault stops the car | FR11; S2, Safety (S5) | manual | P4: hold one wheel while driving | `LEFT_STALL` or `RIGHT_STALL`; car stops |
| INT-06 | Telemetry round trip | FR9; S1 and S3, S4, S5 | manual | P2/P4: decoded telemetry against known conditions (box distance, tilt angle, barcode passed) | Decoded values match the source |
| INT-07 | Barcode decoding while driving | FR2; S3, S2 | manual | P4: each barcode, 10 passes | Zero wrong actions; unknown codes ignored (`bu` increases) |
| INT-08 | Junction turn after a barcode | FR3; S3, S2, Vehicle (S5) | manual | P4: barcode, then junction | Axle over the crossbar at the pivot; line reacquired |
| INT-09 | Hump height | FR5; S4, S2 | manual | P4: humps of known height, e.g. 20, 40, 60 mm, 5 runs each | Reported `pk` against true height, error recorded |
| INT-10 | No false terrain faults | FR6; S4, S2 | manual | P4: braking, pivots, hump crests | **No** `TILT` or `COLLISION` fault |
| INT-11 | Obstacle detection while following | FR7; S5, S3, S2 | manual | P4: obstacle on the line, 10 runs | Detected; `sw` and `ss` correct |
| INT-12 | Bypass and line reacquisition | FR8; S5, S2, S3 | manual | P4: bypass left, right, and with heading error | No collision; no false line lock; original line reacquired; bypass time recorded |
| INT-13 | Broker outage during a run | FR9; S1, Vehicle (S5) | manual | P4: `docker stop mqtt` for 60 s mid-run, then `docker start mqtt` | Car keeps running; reconnects; `drop` recorded |
| INT-14 | Shared fault mask under stress (simulated) | FR11; all | auto | `bash scripts/dev.sh app-tests` | `fault_stress: ... failures=0` |

*Method:* **auto** = run by `scripts/dev.sh`, result in the saved log;
**manual** = a person observes the result (telemetry, console, meter, video).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_INT-01.jsonl` | |
| 2026-11-23 | INT-01 | image n `60499512` | Dian Ze | PASS | `../system/logs/2026-11-23_dashboard.txt` | 12 STOPs sent from the dashboard while driving (14:43–15:22): each ended the run ("MISSION aborted"). That line reached the laptop 0.15–3.1 s after the broker had the STOP (median 1.2 s), which includes the line's own trip back. `state` and `stop_source` were not checked |
| 2026-11-23 | INT-07 | image n `60499512` | Dian Ze | PASS | `../system/logs/2026-11-23_dashboard.txt` | 8 junction decisions from cards, 14:46–15:24, each the move the map in force gave: F → forward (2), Q → U-turn (3), Z → right (1) with map F/Q/A/Z; F → U-turn with map Q/F/A/Z (15:17); A → left with map Q/Y/A/Z (15:24). No wrong action; no run had decoders that disagreed. Not 10 passes per card |
| 2026-11-23 | INT-08 | image n `60499512` | Dian Ze | PASS | `../system/logs/2026-11-23_dashboard.txt` | Turned on the crossing and followed the line on: U-turn +179° 4 times (14:46, 14:47, 15:00, 15:17) and RIGHT −89° once (15:02). A LEFT was chosen once (15:24:44), but that run was stopped 2.6 s later, before the turn |
| 2026-11-23 | INT-09 | images k and n | Dian Ze | PASS | `../system/logs/2026-11-23_dashboard.txt` | The mission reports the hump in its SIDE lines; the status frame's `pk` is not fed and stays 0. Track hump (10 mm): 11, 10, 6, 10, 11 mm on image k (13:28–13:47) and 10 mm on image n (15:00) |
| 2026-11-23 | INT-11 | images k and n | Dian Ze | PASS | `../system/logs/2026-11-23_dashboard.txt` | Obstacle on the line: the car stopped to measure it 4 of 4 times, at 102 and 116 mm (13:45, 13:47, image k), 115 mm (15:02, image n) and 100 mm (17:57, image n, stopped by STOP before measuring). Width by the creeps 77, 112 and 119 mm (true width not recorded). The mission reports the width in its AVOID lines, not in `sw`/`ss`. Not 10 runs |
| 2026-11-23 | INT-12 | images k and n | Dian Ze | FAIL | `../system/logs/2026-11-23_dashboard.txt` | Back on the original line 2 of 3: 13:47:41 (image k) and 15:03:29 (image n, 34.0 s after the obstacle stop). 13:46 (image k): the run ended with a RIGHT_STALL fault while stepping back to the line |
