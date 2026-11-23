# Bench image telemetry: minimum-bit frames (Change Requirement 1)

Tribunal 1's change, minimum-bit data representation, was built into the
robot app first (`ChangeRequirement/README.md`). The demo ran the bench
image (`bench_main.c`), which until 22 Nov 2026 sent its live log over MQTT
as text. Since then nothing it sends is text: every line goes out as one
binary frame whose fields use only the bits their range needs.

Later on 22 Nov the mission moved into the robot app (its Vehicle task,
`app_program/robot_app.c`). Since 23 Nov the robot's
mission is split into its subsystems (`app_program/subsystem5/mission.c`,
its core, plus `subsystemN/mission_*.c`, sharing
`app_program/mission_private.h`, which holds `bench_kind_t`), and the bench
image builds from its own frozen copy, `firmware/bench/bench_main.c`. The
frames, kinds and templates below are the same in both builds; the decoder
is `tools/dashboard/bench_codec.py`. The robot app adds two kinds
to the end report: TASKS (each task's longest step and skipped periods) and
IMU (the IMU task's failed and saturated reads, the longest gap between
fresh samples, and the samples terrain refused).

What already met the requirement and is unchanged:

- laptop to car: 1-byte commands (`mqttcmd.h` codes 0xA9-0xAF) and the
  3-byte junction map (`navmap.h`, 5 bits a letter);
- between the bench's two tasks: a 1-byte command and a 1-bit STOP flag;
- ISR-shared words: the robot app's `board.c`, already packed;
- task-private working values (the `double`s): kept at full precision, as
  agreed with the lecturer for Tribunal 1.

## The frame

```
byte 0   0xBD  telemetry.h's 2-bit version 1, frame type 3 (the type the
               robot reserves), then the pattern 1011b: never printable
               text, never a robot frame
byte 1   seq   8 bits, counted per frame sent: a gap is a lost frame
then     kind  7 bits (bench_kind_t), then the line's fields in order,
               LSB first, little-endian (bitpack.h), zero padding
```

At most 64 bytes, one MQTT message each. Unsigned fields saturate. Signed
fields are two's complement and saturate at +/-(2^(n-1)-1); the most negative
code is never made (bitpack.h's convention). Every saturation is counted
(`g_frame_sat`), and the sim check below would show it as a difference.

The constant text is not sent. `tools/dashboard/bench_codec.py` holds each
kind's template, which mirrors the firmware's calls for that line, and
rebuilds the same line. The dashboard (`tools/dashboard/server.py`) decodes
each frame into that line and handles it as before, so the panels and the
log files are unchanged.

## How the firmware builds it

Each line still builds its text in `g_text`, which is private to the control
task, as before. Nothing about the car's timing or behaviour changes. Beside
it, the same calls build the frame, and only the frame goes to the radio:

| Call | Frame | Text |
|---|---|---|
| `bench_line(BK_...)` | header and kind | none |
| `bench_u/i/s(v, bits)` | v in bits (unsigned, signed) | as `bench_dec` / `bench_signed` |
| `bench_x/ux(v, places, bits)` | v in bits | as `bench_fixed` |
| `bench_flag(b)` | 1 bit; returns b | none |
| `bench_code(v, bits)` | v in bits (a branch or a count) | none |
| `bench_name(names, i, bits)` | i in bits | `names[i]` |
| `bench_str(text, bits)` | length, then 6 bits a Code 39 character | the text |
| `bench_char` / `bench_marks` | a character / a map row from an alphabet | the text |
| `bench_text("...")` | nothing (the template has it) | the text |

A line with no kind, or one that prints a number through the old
`bench_dec` (no width), is never sent. It is counted in `g_frame_unsent`,
and the offline console shows it as `#T`. In this image START runs only
MISSION, so the bench's test modes are never reached, and their lines would
not be sent if they were.

`BENCH_FRAMES 0` builds the old text image (for debugging).

## Widths

Each width comes from the field's valid range at the resolution the line
prints. Run counts and paths are sized for a 1-hour run, now that MISSION
has no time limit. Rates are taken from the car's own 2-minute runs:

- time: 0.1 s to 6553.5 s (16 bits);
- centre path: +-1048575 mm (21 bits);
- side sensor path: 0-2097151 mm (21 bits);
- ADC: 0-4095 (12 bits);
- IMU angles: +-204.7 deg in tenths (12 bits);
- per-run counts: 0-1023 (10 bits), more where the car's rate needs it
  (gaps 16, jolts 12, readings 22).

The full table is at the end.

## Long-run faults found by the check (fixed)

Checking every field against its range, with a sim run far past what a
2-minute run covers, found these. All of them came in with MISSION's
120-second limit coming off; the old limit had hidden them:

| Fault | Effect with no time limit | Fix |
|---|---|---|
| Barcode samples stored as `uint16_t` units (0.32 mm) | Past 21 m (~3 laps) the store took nothing more ("STORE FULL"): no card read again | `uint32_t` units (bench_code_sample_t, D0 edges) |
| Decoder 1 kept at most 12 groups a run | Past the 12th group (card or junction tape) Decoder 1 never reported again | Sent groups give their room back (`bench_code_found_drop`) |
| REC path stored as `int16_t` | Wrapped past 32.7 m | `int32_t` |
| Live no-echo count, 8 bits | 2903 during a 40 s go-around | 16 bits |

The sim's `long` scenario puts a card at 22 m and at 40 m. Before the fix,
Decoder 1 did not read either card and the junction went "no read". After the
fix, both decoders read `*A*` and the junction takes code A.

## Verification

- **Sim, every frame against its line.** The offline build prints each frame
  in hex (`#F`) before the line it stands for. `bench_codec.py check`
  decodes each one and compares. The scenarios are junction, ramp (with lift),
  S-bend, obstacle go-around (a box for the sonar), the 40 m long run,
  recorded card traces (stream), a sharp turn before a real crossing
  (corner), and `lines` (rare lines called directly).
  Result: 5633 frames identical, 12 more longer than the text because the
  firmware's 160-byte text buffer cut the line short and the frame does not;
  0 differ; no line unsent.
- **Through the broker.** The same frames were published to the MQTT broker
  and read by the dashboard: its log matched the firmware's text line for
  line (4475 lines).
- **Kinds the sim never produces:** rare go-around branches (blocked ways,
  far edges out of reach), the straight-barcode image's line, BOOT, ENC_ZERO
  and MAG (no compass in the sim). Their templates follow the same calls,
  but only the car will show them.
- **On the car:** not yet. The dashboard should show the same lines as the
  text image did.

```
cd firmware/bench/offline/sim && ./make.sh && ./sim lines > out.txt
python3 ../../../../tools/dashboard/bench_codec.py check out.txt
python3 ../../../../tools/dashboard/bench_codec.py table out.txt     # the table below
```

## Sizes

Measured over the sim runs above: text 422,941 B, frames 209,074 B (49.4%).
The raw sample rows (`R`) dominate that total because they were already a
compact character code (7 bits now against 8). The lines the dashboard reads
most shrink the most:

| Line | Text | Frame |
|---|---|---|
| live status | 69.4 B | 17.2 B |
| REC row (3 samples, 15 fields each) | 123.3 B | 48.9 B |
| CODE / GROUP | 113.6 B | 29.0 B |
| pose | 16.5 B | 9.0 B |

Image (bench, MQTT, motion), before and after:

- text 493,676 -> 501,508 B;
- bss 169,676 -> 192,052 B: +15.4 KB the 32-bit barcode store (the long-run
  fix), +8 KB the 10 s recorder (22 Nov), about 2.3 KB frame buffers, less
  4.6 KB of queued text the MQTT build no longer keeps. That leaves about
  2 KB under the linker's 64 KiB heap reserve (256 KiB RAM): the next
  RAM must come from somewhere else.

Since the mission moved into the robot app (22 Nov), the robot app sends
these frames too, so its telemetry slots are 8 x 64 B with bursts of up to
16 frames per 100 ms, as the bench build's were (`net_mqtt_ipc.h`). Until
then the robot app alone had the Post-Tribunal sizes (4 x 20 B, one frame
per 100 ms). `build.sh` still defines `NET_MQTT_IPC_BENCH`; it changes
nothing now.

## Frame table

Field codes:

- `u21` unsigned 21 bits;
- `i13` signed;
- `x12.1` / `ux16.1` signed / unsigned fixed point, 1 decimal;
- `n3` a name;
- `f[a/b]` a 1-bit flag and its two branches;
- `w2 (a/b/..)` a 2-bit branch code and its branches;
- `r6*(..)` a 6-bit count and its repeats;
- `s4` text (4-bit length, 6 bits a character);
- `c5` one character;
- `41*m3` a map row.

Lines, Text B and Frame B are measured over the sim runs ("-": not produced
by the sim).

| # | Kind | Fields after the 23-bit header | Lines | Text B | Frame B |
|---|---|---|---|---|---|
| 1 | LIVE | ux16.1 i21 n3 u12 u12 u10 u10 u12 u16 f[x12.1 x12.1 i10/] | 580 | 69.4 | 17.2 |
| 2 | POSE | i17 i17 i9 f[i13 i13 i13/] | 1260 | 16.5 | 9.0 |
| 3 | REC_HEAD | u9 | 10 | 85.0 | 4.0 |
| 4 | REC_ROW | u8 r2*(i21 i9 u7 u7 u7 u7 u2 i8 i8 i2 i2 u7 u7 i12 i12) | 1669 | 123.3 | 48.9 |
| 5 | REC_END | none | 10 | 8.0 | 3.0 |
| 6 | CODE | f[s4 f f f/u8 w2 (s4 f/s4 n2/s4 s4) (s4 s4)] u21 u21 ux9.1 u8 | 44 | 113.6 | 29.0 |
| 7 | FIT | w2 (u16/ux10.1/f s1 f ux14.2 s1 f ux11.2 ux8.1 ux7.2) u21 u21 (u16 u16 u20 f) | 17 | 100.9 | 16.5 |
| 8 | D0 | f[s4 f/u10 w2 (s4 s4)] u21 u21 | 2 | 41.0 | 10.0 |
| 9 | D0_WIDTHS | r6*(u9) | 2 | 75.5 | 27.5 |
| 10 | FIT_TOTALS | u10 u10 u10 u10 ux8.2 | 11 | 76.0 | 9.0 |
| 11 | CAR | w2 (s4/s4) w2 (s4) w2 (s4) w2 (s4) u21 u21 | 28 | 72.4 | 10.5 |
| 12 | CAR_TOTALS | u11 u10 u10 u10 u10 f[u10/] | 11 | 113.9 | 10.1 |
| 13 | RAW_HEAD | u10 u23 u21 u13 u12 u12 | 37 | 143.4 | 15.0 |
| 14 | RAW_ROW | u10 r7*c7 | 1651 | 71.2 | 60.8 |
| 15 | MAP | c5 c5 c5 c5 | 1 | 46.0 | 6.0 |
| 16 | NAV_MAP | c5 c5 c5 c5 | 1 | 47.0 | 6.0 |
| 17 | NAV_RESTORED | u8 u8 u8 u8 u8 u8 u8 u8 u8 | 1 | 84.0 | 12.0 |
| 18 | NAV_CARD | u21 u21 w2 (s1) w2 (s1) w2 (s1) f[s1 f[n2/f]/] | 29 | 103.4 | 9.5 |
| 19 | NAV_JUNCTION | u10 i21 f[w2 (s1) w2 (s1) w2 (s1)/] f[s1 f f[n2 (u9)/f]/w2] | 6 | 100.8 | 10.0 |
| 20 | NAV_TURNED | i10 | 1 | 57.0 | 5.0 |
| 21 | NAV_WAIT | i21 w2 (s1) w2 (s1) w2 (s1) | 7 | 110.3 | 7.0 |
| 22 | NAV_WAITED | u16 w2 w2 (s1) w2 (s1) w2 (s1) | 7 | 67.9 | 6.6 |
| 23 | RAMP_OFF | f | 8 | 27.0 | 3.0 |
| 24 | RAMP_FLAT | f u10 x12.1 x12.1 | 2 | 81.0 | 8.0 |
| 25 | RAMP | u10 i10 i21 ux16.1 x12.1 f u8 x12.1 x12.1 i21 | 1 | 126.0 | 19.0 |
| 26 | SIDE | f[n3 f u9 x12.1 u9 x12.1 x12.1 i21 ux16.1 u16/u10 f u9 x12.1 u9 x12.1 x12.1 i21 ux16.1 u16] | 1 | 130.0 | 18.0 |
| 27 | TERRAIN | f i10 | 1 | 33.0 | 5.0 |
| 28 | CAL | u12 u12 u12 u12 u12 u12 u12 u12 u12 u12 u12 u12 i21 | 10 | 112.6 | 24.0 |
| 29 | RAMP_HIGHEST | i10 f[u10 i21 ux16.1 x12.1 f u8/] | 2 | 56.0 | 9.0 |
| 30 | RAMP_OPEN | i10 i21 ux16.1 x12.1 f u8 | - | - | - |
| 31 | RAMP_RISES | u10 u10 u9 x12.1 x12.1 | 2 | 108.0 | 10.0 |
| 32 | RAMP_LEANS | f[f u9 x12.1 u9 x12.1 x12.1 i21 ux16.1 u16/] u10 u12 | 2 | 102.5 | 13.0 |
| 33 | RAMP_HELD | u10 u16 u10 | 2 | 67.5 | 8.0 |
| 34 | RAMP_OUT | u10 u10 u10 u10 u10 | 2 | 138.0 | 10.0 |
| 35 | RAMP_WILD | u9 u10 | 2 | 56.0 | 6.0 |
| 36 | LIFT | f u12 u12 u12 u16 | 2 | 106.0 | 10.0 |
| 37 | RAMP_ROLL | x12.1 i21 x12.1 i21 u19 u19 | 2 | 85.5 | 16.0 |
| 38 | DRIVE_START | f[/f] | 10 | 72.0 | 4.0 |
| 39 | STRAIGHT | u9 | - | - | - |
| 40 | AUTO_FIRST | none | 10 | 101.0 | 3.0 |
| 41 | END | f[/n5] n5 f[f ux16.1/] ux16.1 f | 10 | 38.6 | 6.6 |
| 42 | CENTRE | i21 ux16.1 i10 u7 u7 u7 u7 u7 | 10 | 106.7 | 14.0 |
| 43 | CENTRE_GAPS | u16 u10 u12 u16 u12 u12 | 10 | 78.2 | 13.0 |
| 44 | AVOID_DROPPED | u10 | - | - | - |
| 45 | D0_TOTALS | u22 u12 u12 u4 u10 u10 u22 u22 f | 11 | 102.2 | 18.0 |
| 46 | BARCODES | u22 ux14.2 u10 u10 u10 u16 u10 f[u10/] f u12 u12 u12 u10 u10 | 11 | 159.0 | 22.0 |
| 47 | AV_OBSTACLE | u12 | 1 | 52.0 | 5.0 |
| 48 | AV_OBSTACLE_AVOID | u12 | - | - | - |
| 49 | AV_MEASURE | none | 1 | 44.0 | 3.0 |
| 50 | AV_LEFT_EDGE | i13 i9 f | 1 | 67.0 | 6.0 |
| 51 | AV_BACKED | i13 f | 1 | 70.0 | 5.0 |
| 52 | AV_CREEP | f | 3 | 73.7 | 3.0 |
| 53 | AV_FAR_BACK | u12 u7 | 1 | 105.0 | 6.0 |
| 54 | AV_STEP_OUT | f | 1 | 33.0 | 3.0 |
| 55 | AV_CLEAR | none | 1 | 60.0 | 3.0 |
| 56 | AV_ALONG | i13 | 1 | 40.0 | 5.0 |
| 57 | AV_AHEAD | u12 | - | - | - |
| 58 | AV_PAST | none | 1 | 51.0 | 3.0 |
| 59 | AV_WAY_CLEAR | i13 | - | - | - |
| 60 | AV_RETURN | none | 1 | 30.0 | 3.0 |
| 61 | AV_ON_LINE | u12 u12 f[x12.1/] | 1 | 85.0 | 8.0 |
| 62 | AV_ALONG_LINE | f f | - | - | - |
| 63 | AV_ON_COURSE | none | 1 | 41.0 | 3.0 |
| 64 | AV_STILL_WAY | u12 i13 | - | - | - |
| 65 | AV_BLOCKED | u12 | - | - | - |
| 66 | AV_WAY_IN | n1 w1 i13 i13 | - | - | - |
| 67 | AV_WAY_IN_BOTH | i13 i13 | - | - | - |
| 68 | AV_CROSSED | i13 x12.1 i13 i13 | 1 | 92.0 | 10.0 |
| 69 | AV_TURN_LINE | x12.1 x12.1 f[i13 i13 i13/] | 1 | 85.0 | 6.0 |
| 70 | AV_TURN_NONE | w1 u8 (u12 u12/u12 u12 f[i13 i13 i13/]) | 1 | 77.0 | 7.0 |
| 71 | AV_TURN_BACK | f x12.1 f[i13 i13 i13/] | - | - | - |
| 72 | AV_HOME_NONE | f[i13 i13 i13/] | - | - | - |
| 73 | AV_HOME | x12.1 u8 f[i13 i13 i13/] | 1 | 87.0 | 6.0 |
| 74 | AV_IR | u12 f u12 u12 u12 f u12 u12 | 1 | 75.0 | 13.0 |
| 75 | AV_NO_ECHO | i9 i13 i9 | - | - | - |
| 76 | AV_ON_FACE | i13 i13 u4 u4 | 1 | 85.0 | 8.0 |
| 77 | AV_PAST_REACH | i13 | - | - | - |
| 78 | AV_FAR_EDGE | f i13 f f[/i13 u12] | 1 | 97.0 | 8.0 |
| 79 | AV_EDGE | i13 f[/i13 u12] i13 i13 | 1 | 81.0 | 11.0 |
| 80 | AV_WIDTH | i13 f i13 i13 i13 i13 | 1 | 81.0 | 12.0 |
| 81 | AV_OTHER | none | - | - | - |
| 82 | AV_FACE | i13 i13 f i13 f i13 f u8 | 1 | 119.0 | 11.0 |
| 83 | AV_PAST_SWEEP | i13 u4 u4 | - | - | - |
| 84 | AV_PATH | i13 i13 w2 i13 i13 w2 | 1 | 91.0 | 10.0 |
| 85 | AV_NO_WAY | none | - | - | - |
| 86 | AV_GOING | f i13 u7 f | 1 | 101.0 | 6.0 |
| 87 | AV_OBST | i17 i17 x12.1 i13 i13 f i13 f i13 w2 i13 i13 i13 | 1 | 91.0 | 21.0 |
| 88 | AVOID_MAP_NONE | none | - | - | - |
| 89 | AVOID_MAP_HEAD | u5 | 1 | 115.0 | 4.0 |
| 90 | AVOID_MAP_ROW | i11 41*m3 | 4 | 49.0 | 20.0 |
| 91 | BEAMS | i12 r4*(i9 u12) f[u10/] | 95 | 21.5 | 8.0 |
| 92 | BOOT | f f | - | - | - |
| 93 | ENC_ZERO | none | - | - | - |
| 94 | IR | f[u12 u12 u12 u12 u12 u12 u12 u12 u12/] | 2 | 24.5 | 10.0 |
| 95 | ACC | u10 (i13 i13 i13 i13 i13 i13 i13 i13 i13) | 2 | 36.5 | 12.0 |
| 96 | MAG | i13 i13 i13 | - | - | - |
| 97 | ENC | i25 i25 i22 i22 u16 | 2 | 50.0 | 17.0 |
| 98 | LIFT_STOP | u12 | 1 | 36.0 | 5.0 |
| 99 | AUTO | w2 ux10.1 u8 u8 x12.1 | 3 | 85.7 | 8.0 |
| 100 | AUTO_WHY | n3 | 1 | 70.0 | 4.0 |
| 101 | AUTO_FOUND | f[x12.1/] f[x12.1/] | 3 | 84.0 | 5.3 |
| 102 | AUTO_END | f[x12.1 x12.1 x12.1/x12.1] u12 u12 | 3 | 66.0 | 9.0 |
| 103 | NAV_TURN | u9 i21 w2 (s1) w2 (s1) w2 (s1) | 4 | 139.0 | 8.5 |
| | all | | 5645 | 422941 total | 209074 total (49.4%) |
