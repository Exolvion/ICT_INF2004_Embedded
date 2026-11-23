# Subsystem 5 — test log

**Subsystem:** Obstacles: sonar, servo, bypass; Safety and Vehicle (Buddy 5, Siang Hong)  ·  **Student ID:** 2503147  ·  **Code:** `app_program/subsystem5/` (`scanner.c/.h`, `vehicle.c/.h`), the shared `app_program/robot_app.c` (Safety, Vehicle and Scan tasks) and the sonar, servo and button parts of `app_program/board.c`; since 22 Nov 2026 also `mission_avoid.c/.h` (the mission's go-around), `robot_scan.c/.h` (Scan task); the mission's core `mission.c`, `mission_board.c`

See `testing/README.md` for how to run tests and record results, and
`TESTING.md` for the set-up. P2 = after TESTING.md section 4; bench = with a
bench program (TESTING.md section 5).

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | Scanner module (the same executable also tests terrain, see S4-U01) | FR7 | auto | `bash scripts/dev.sh host-tests test_sensor_main` | "Passed 1 executable(s) in both host configurations" |
| S5-U02 | Vehicle state machine (the same executable also tests Motion, see S2-U01) | FR8, FR11 | auto | `bash scripts/dev.sh host-tests test_motion_main` | "Passed 1 executable(s) in both host configurations" |
| S5-U03 | STOP button | FR11, A8 | manual | 1. Multimeter: GP20 low only while pressed. 2. P2: press STOP. | `state` FAULT, `stop` 1, `stop_source` LOCAL. FAULT at boot without a press means wrong polarity. |
| S5-U04 | Sonar range | FR7 | manual | P2: box at 200, 250, 500 and 1000 mm | `rg` matches the tape (error recorded); `rv` 1 |
| S5-U05 | No echo | FR7 | manual | P2: unplug ECHO | `NO_ECHO` in `faults`; no stop on its own |
| S5-U06 | Servo endpoints and centre | FR7 | manual | Bench: drive the servo to both ends and centre | Sweeps ±`ROBOT_SCAN_HALF_ANGLE_DEG` without touching anything or buzzing (40° on the IS29 car: the sonar's board meets the Robo Pico's connectors at about ±50°); values entered in `robot_config.h` |
| S5-U07 | Obstacle profile from a stationary scan (Week 10 metrics: detection, width error, side choice) | FR7, Briefing p8 | manual | Motion bench image, SCAN test (TESTING.md 5.2), run where the S5-U08 test stopped the car (the car finds the obstacle itself; nobody gives it the distance). Obstacle of known width (the 1.5 l bottle: measure its diameter once): on the line, then moved off-centre left and right so one side is clearly open. 5 scans per placement | Every scan: obstacle seen, nearest agrees with the S5-U08 stop range within 20 mm, width within 10% of the obstacle (band A), a side chosen when centred and the open side when offset, no refused trigger, scan time recorded. Obstacle, room and results logged |
| S5-U08 | Obstacle stop while following the line (Week 10 metric: detection rate) | FR7, FR8, Briefing p8 | manual | Motion bench image, LINE + stop within 10 cm test (TESTING.md 5.2), gain set 4, 120 mm/s on the 1 m line. Obstacle: a 1.5 l shop water bottle standing upright on the line at a random place before the T (at least 0.4 m from the start), moved each run. The car finds it with its own sonar; nobody gives it the distance. 10 runs | Every run stops for the bottle (detection 10/10 = band A) within 10 cm of it without touching it (range at detection at most 100 mm, range 1 s later above 20 mm); readings that are not VALID counted (each would halt the robot) |

*Method:* **auto** = run by `scripts/dev.sh`, result in the saved log;
**manual** = a person observes the result (telemetry, console, meter, video).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_S5-U01.txt` | |
| 2026-10-26 | S5-U05 | `2d121bc` + uncommitted | Siang Hong | PASS | `logs/2026-10-26_S5-U05_summary.txt`, `logs/2026-10-26_S5-U05.jsonl` | Car on the stand in INIT. ECHO unplugged about 12 s: `NO_ECHO` in faults 0.8 s after the range went invalid, no stop source, no fault latched; cleared with the first valid range. A moving car was not tested. Written up 2026-11-23 from the recording |
| 2026-10-26 | S5-U04 | not recorded | Siang Hong | PASS | `logs/2026-10-26_S5-U04_summary.txt` | Functional: range valid, stable and tracks the distance. Accuracy not measured that day (see the 27 Oct row) |
| 2026-10-27 | S5-U04 | not recorded | Siang Hong | PASS | `logs/2026-10-27_S5-U04_tape_summary.txt` | Against a tape at 200–1000 mm: range valid in every frame, largest error 7 mm, spread at most 9 mm |
| 2026-10-28 | S5-U08 | `2d121bc` + uncommitted | Siang Hong | PASS | `logs/2026-10-28_S5-U08_summary.txt`, `logs/2026-10-28_S5-U08_line_stop.txt` | 2 of 2 runs: bottle on the line detected, stopped within 10 cm, no contact. The test case asks for 10 runs |
