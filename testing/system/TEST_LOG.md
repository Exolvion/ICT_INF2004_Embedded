# System — test log

Tests of the whole car against its requirements. Run the full set before
every demo and before hand-in.

See `testing/README.md` for how to run tests and record results, and
`TESTING.md` for the set-up and the steps referred to below.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | All module tests | FR1–FR11 (module level) | auto | `bash scripts/dev.sh host-tests` | "Passed N executable(s) in both host configurations"; Python tests `OK` |
| SYS-02 | Coding standard and lock discipline | NFR2 | auto | `bash scripts/dev.sh style` | "mechanical findings: 0"; "Lock discipline: PASS" |
| SYS-03 | Application scenarios on simulated sensors | FR1–FR11 | auto | `bash scripts/dev.sh app-tests` | "Scenario executions: 122; failures: 0" |
| SYS-04 | Board, installer, radio and network fixtures | NFR1, FR9 | auto | `bash scripts/dev.sh integration` | "integration host checks passed" |
| SYS-05 | Target images build | NFR1, NFR3 | auto | `bash scripts/dev.sh build offline` and `bash scripts/dev.sh build mqtt --compile-only` | "build passed" for both |
| SYS-06 | First flash: start-up refusal | NFR6 | manual | TESTING.md section 2 | Console prints `ROBOT STARTUP REFUSED: ROBOT_WIRING_CONFIRMED=0` |
| SYS-07 | First power-up with WIRING set | FR9, FR11 | manual | TESTING.md section 4.3 | `state` INIT at about 5 Hz; wheels never move |
| SYS-08 | Line following | FR1 | manual | 5 laps under normal light and 5 under bright light | Line losses counted (target 0) |
| SYS-09 | STOP, RESET, START | FR11 | manual | Drive; STOP; RESET (GP21); START (GP21) | Stops; RESET only when eligible; a separate START needed |
| SYS-10 | Full course against Briefing p.8 bands A–D | FR1–FR9 | manual | Full course runs | Detection and reacquisition 100 / 95 / 90 / 80 %; width error ≤ 10 / 20 / 30 / 40 %; bypass < 3 / 5 / 7 / 10 s |
| SYS-11 | 10-minute continuous run | NFR4 | manual | Run the course for 10 minutes with telemetry saved | No resets; no unexpected faults |
| SYS-12 | Task timing, interrupt time, stack high-water | NFR3, NFR4 | manual | Task timing: every run's end report has a TASKS line (each task's longest step and skipped periods since START; ROBOT_APP_MISSION). ISR, masked time and stack high-water still need instrumentation (ChangeRequirement/README.md section 12) | No missed release; ISR and masked time ≤ 20 µs (proposed); stack margin ≥ 25 % (proposed) |
| SYS-13 | Mission in the robot app, host sim | FR1–FR8, FR11 | auto | `bash firmware/robot/sim/make.sh`, then `firmware/robot/sim/robot_sim junction`, `corner`, `long 22`, `ramp 10 100 50 4` (`SIM_FROM=0.05`), `obstacle`; `python3 tools/dashboard/bench_codec.py check` on each output | Card read by both decoders; corner a turn, LEFT at the crossing; card at 22 m read; side hump 7 mm; obstacle measured, passed and rejoined; every frame matches its line |

*Method:* **auto** = run by `scripts/dev.sh`, result in the saved log;
**manual** = a person observes the result (telemetry, console, meter, video).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | SYS-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_SYS-01.txt` | |
| 2026-10-28 | SYS-01 | not recorded | | PASS | `logs/2026-10-28_SYS-01.txt` | Passed 7 executable(s) in both host configurations |
| 2026-10-28 | SYS-02 | not recorded | | PASS | `logs/2026-10-28_SYS-02.txt` | 55 files: mechanical findings 0, manual-review findings 0; lock discipline PASS |
| 2026-10-28 | SYS-03 | not recorded | | PASS | `logs/2026-10-28_SYS-03.txt` | Scenario executions 122, failures 0 |
| 2026-10-28 | SYS-04 | not recorded | | PASS | `logs/2026-10-28_SYS-04.txt` | Integration host checks passed |
| 2026-10-28 | SYS-05 | not recorded | | PASS | `logs/2026-10-28_SYS-05_offline.txt`, `logs/2026-10-28_SYS-05_mqtt.txt` | Build passed, both profiles |
| 2026-11-23 | SYS-01 | `04b9260` + uncommitted (final tree) | Dian Ze | PASS | `logs/2026-11-23_SYS-01.txt` | Passed 8 executable(s) in both host configurations; Python tests OK (8) |
| 2026-11-23 | SYS-02 | `04b9260` + uncommitted (final tree) | Dian Ze | FAIL | `logs/2026-11-23_SYS-02.txt` | 89 files: 21 mechanical findings (functions over 100 lines, BARR-C 6.2.a) and 16 manual-review findings (`continue`), all in the mission files and kept on purpose (README "Known deviations"). Lock discipline PASS (`check_locks.py` run on its own: `dev.sh style` stops at the findings) |
| 2026-11-23 | SYS-03 | `04b9260` + uncommitted (final tree) | Dian Ze | PASS | `logs/2026-11-23_SYS-03.txt` | Scenario executions 122, failures 0 |
| 2026-11-23 | SYS-04 | `04b9260` + uncommitted (final tree) | Dian Ze | PASS | `logs/2026-11-23_SYS-04.txt` | Integration host checks passed |
| 2026-11-23 | SYS-05 | `04b9260` + uncommitted (final tree) | Dian Ze | PASS | `logs/2026-11-23_SYS-05_offline.txt`, `logs/2026-11-23_SYS-05_mqtt.txt` | Build passed: offline, and MQTT with placeholder settings (`--compile-only`). The MQTT build with the car's network settings is byte-identical to image n (`60499512`) |
| 2026-11-23 | SYS-13 | `04b9260` + uncommitted (final tree) | Dian Ze | PASS | `logs/2026-11-23_SYS-13.txt` | junction: `*A*` read by Decoder 1 and 2. corner: A → LEFT, turned +89° on the crossing. long: `*A*` read at 22 m. ramp: side hump 7 mm. obstacle: measured (170 mm for the 100 mm box: the sim's sonar beam), passed and back on course; NO_ECHO flickers without latching. Every frame matched its line in all five |
| 2026-11-23 | SYS-08 | image n `60499512` (and k, l, m before it) | Dian Ze | PASS | `logs/2026-11-23_dashboard.txt` | Course track, 13:27–18:01. One line loss: 13:32:38 on image k, 75.5 s into the run. None on image n (14:43–18:01). Not the 5 + 5 laps of the test case |
| 2026-11-23 | SYS-09 | image n `60499512` | Dian Ze | PASS | `logs/2026-11-23_dashboard.txt` | START and STOP over MQTT from the dashboard. 12 STOPs (14:43–15:22) each ended the run; its "MISSION aborted" line reached the laptop 0.15–3.1 s after the broker had the STOP (median 1.2 s), which includes the line's own trip back. Lift stop: "lifted (IMU)" at 150 mm on image m, 14:26:54. Image n ended three runs with "all three sensors dark (lifted, or off the mat)" (14:46:48, 15:23:38, 17:55:14). RESET and the GP21 button not exercised |
| 2026-11-23 | SYS-10 | image n `60499512` | Dian Ze | FAIL | `logs/2026-11-23_dashboard.txt`, 15:00:06–15:03:32 | One run over most of the course. Map: forward F, U-turn Q, left A, right Z. Cards read: `*F*`, `*Q*`, `*Z*`, `*Z*`, `*Q*` (5 codes, no reader disagreed). Junction 1 (3908 mm): Q → U-turn, turned +179° on the crossing. Junction 2 (11395 mm): Z → RIGHT, turned −89°. Side hump 10 mm at 3640 mm. Obstacle stopped at 115 mm, width by the creeps 119 mm (true width not recorded), back on course 34.0 s after the obstacle stop: outside band D (< 10 s). Ended by STOP |
| 2026-11-23 | SYS-12 | image n `60499512` | Dian Ze | FAIL | `logs/2026-11-23_dashboard.txt`, the "tasks, longest step us/skipped periods" line of the 14 runs 14:43–15:23 | Task timing only. Longest steps: Safety 91–124 µs, Motion 1.33–1.43 ms, Line 220–254 µs, Vehicle 8.5–25.3 ms (about 24–25 ms in runs with a junction wait, where Decoder 2 gets 20 ms steps; the counter's maximum, 65535 µs, in the 14:59:54 run), IMU 5.7–14.0 ms, Scan 1.5–13.0 ms, Comm 3.7–10.5 ms, Telemetry 0.4–10.0 ms. Skipped periods: none for Safety, Motion and Telemetry; Line, Vehicle, IMU, Scan and Comm skipped periods in every run (15:03:31: Line, Vehicle, Scan and Comm past the counter's 4095, IMU 185). ISR time, masked time and stack high-water not measured |
