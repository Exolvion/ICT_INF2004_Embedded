# INF2004 Embedded Systems Project — IS29

This repository contains the robot firmware, the bench MISSION firmware used
for the car tests, the laptop tools (MQTT broker and dashboard), test logs, the
Week 6 design report, Pre-Tribunal document, tribunal briefing materials and
assignment references for Team IS29.

The project is an autonomous robot built around the Raspberry Pi Pico W and
Robo Pico board. Its intended mission is to follow a line, read navigation
barcodes, make controlled turns, estimate hump height, detect and bypass
obstacles, recover the track and report its state over Wi-Fi/MQTT. The firmware
is written in C using the Pico SDK and the supplied micro T-Kernel RP2040 port.

## Project documents

| Material | File |
|---|---|
| Week 6 design report | [Word document](Week6_Report/INF2004_Week6_Design_Review.docx) · [PDF](Week6_Report/INF2004_Week6_Design_Review.pdf) |
| Tribunal 1 | [Pre-Tribunal](Tribunal1/IS29_Pre-Tribunal1.docx) · [Immediately after](Tribunal1/IS29_Imm-Tribunal1.docx) · [Post-Tribunal](Tribunal1/IS29_Post-Tribunal1.docx) |
| Change requirement 1 | [Minimum-bit data representation](<References/Change Requirement.docx>) · [Week 13 evidence](ChangeRequirement/README.md) |
| Week 6 report feedback | [Original lecturer feedback](<Tribunal1/Sources/IS29_feedback for our week 6 report.txt>) |
| Tribunal briefing | [Notice](Tribunal1/Sources/tribunal.txt) · [Portal screenshot](<Tribunal1/Sources/Screenshot 2026-10-15 221220.png>) |
| Official tribunal template | [Week8_Tribunal1_Template.docx](Tribunal1/Sources/Week8_Tribunal1_Template.docx) |
| Assignment requirements | [Project Write-up](<References/Project Write-up.pdf>) · [Project Briefing](<References/Project Briefing.pdf>) |
| Module and assessment overview | [W0 Module Overview](<References/W0_Module Overview.pdf>) |
| Track and barcode requirements | [Barcode & Line Specification](<References/Barcode & Line Specification.pdf>) · [Barcode Sample](<References/Barcode Sample.pdf>) |
| Reference index | [Project sources](References/README.md) · [Hardware and device guides](References/Device_Guides/README.md) |

The official tribunal template includes the assessment stages. Tribunal 1
has three submissions: the Pre-Tribunal (Version A), the document written
immediately after the live session (Version B), and the Post-Tribunal, which
repeats A and B and adds the AI-vetted gap check (Version C), the final
summary and the implementation evidence.

## Repository layout

The repository follows the course project template
([sirfonzie/INF2004_ProjectTemplate](https://github.com/sirfonzie/INF2004_ProjectTemplate)):
micro T-Kernel 3.0 for the Pico W, its drivers and build files are committed
here, and the robot application is in `app_program/`.

| Location | Contents |
|---|---|
| [app_program/](app_program/) | The robot application. Shared code at the top: `app_main.c` (tasks), `robot_app.c` (task steps), `board.c` (hardware), `robot_config.h` (pins and calibration), `app_cfg.c`, `core.c`, `bitpack.c`. |
| [app_program/subsystem1/](app_program/subsystem1/) | Buddy 1, communications: MQTT client (`net_mqtt*.c`), telemetry frames, command codec. |
| [app_program/subsystem2/](app_program/subsystem2/) | Buddy 2, motion: motion requests, PID, odometry (`motion.c`). |
| [app_program/subsystem3/](app_program/subsystem3/) | Buddy 3, optical: line controller, barcode decoder, course codebook. |
| [app_program/subsystem4/](app_program/subsystem4/) | Buddy 4, terrain: tilt, motion class, hump height (`terrain.c`). |
| [app_program/subsystem5/](app_program/subsystem5/) | Buddy 5, obstacles: scanner and Vehicle mission state machine. |
| [build_make/](build_make/), [config/](config/), [kernel/](kernel/), [lib/](lib/), [device/](device/), [include/](include/), [etc/](etc/), [tools/](tools/) | The project template: kernel, drivers, build files and kernel configuration. `build_make/robot_project.mk` adds the robot's include paths and SDK drivers. |
| [testing/](testing/) and [videos/](videos/) | Test logs per subsystem, integration and system tests, and the video index, in the template's format. |
| [TESTING.md](TESTING.md) | Steps to test the code, set up the car and run each buddy's test cases. |
| [TEAM.md](TEAM.md) | Team, owners, pins, tasks and interfaces. |
| [firmware/tests/](firmware/tests/) and [firmware/scripts/](firmware/scripts/) | Host tests, application scenarios, integration fixtures, build and measurement scripts. |
| [firmware/bench/](firmware/bench/) | The bench MISSION firmware used for the car tests, its build script, and offline tools in `offline/`: a host simulator, recorded barcode passes and replay scripts. See [The bench MISSION firmware](#the-bench-mission-firmware). |
| [testing/barcode_lab/](testing/barcode_lab/) | Offline barcode decoder experiments in Python. See [Barcode decoding](#barcode-decoding). |
| [tools/broker/](tools/broker/) and [tools/dashboard/](tools/dashboard/) | Laptop tools: an MQTT broker and the live dashboard. See [Laptop tools](#laptop-tools-broker-dashboard-and-commands). |
| [scripts/](scripts/) | Dependency setup and the development command runner. |
| [tools/](tools/) | The template's UF2 converter, plus the barcode sampling-budget tool and report-maintenance scripts. |
| [Week6_Report/](Week6_Report/) | Current report in Word and PDF. |
| [Tribunal1/](Tribunal1/) | Pre-Tribunal, original briefing materials, template and feedback. |
| [References/](References/) | Assignment specifications, lecture transcript, templates and hardware references. |
| [firmware/third_party_notices/](firmware/third_party_notices/) | Dependency licence notices. |

### Changes to the template

The template files here are the template at commit `c3b0954`, with the
changes below. Each change is marked `INF2004` in its file; changes that say
`ROBOT_MQTT=1` are built only into the MQTT profile.

| File | Change | Why |
|---|---|---|
| `kernel/sysdepend/cpu/core/armv6m/sys_timer.h` | Clears `SYST_CVR` before SysTick starts | Fixes a template bug. The register is unknown after reset, so the first tick can come up to 134 ms late and every task delay waits for it. On the car this tripped the MOTION fault at start-up. Any project on this kernel port can hit it. |
| `config/config.h` | 1 ms tick (was 10 ms); 2 KiB temporary stack (was 256 B); sample device drivers off | The Safety, Line and Scan tasks run every 2 ms; room for an interrupt on the idle path; `board.c` owns the ADC, I2C and their pins |
| `include/sys/inittask.h` | Initial task stack 4 KiB (was 1 KiB) | Robot start-up and the start-up refusal path |
| `etc/linker/pico_rp2040/tkernel_map.ld` | Link fails unless 64 KiB of heap fits below the reset stack | Memory budget check |
| `kernel/sysdepend/pico_rp2040/hw_setting.c` | Calls `board_boot_timebase()` after the clocks start | Starts the 1 µs timer before the radio or any task |
| `kernel/sysdepend/pico_rp2040/devinit.c` | `ROBOT_MQTT=1`: brakes the motors before the radio starts; refuses start-up if the radio fails | `usermain()` is skipped after that error, so the motors must already be safe |
| `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c` | `ROBOT_MQTT=1`: up to 3 radio start attempts; the robot client joins Wi-Fi itself; `net_mqtt_poll()` after `sys_check_timeouts()`; no console prints | The MQTT client runs in the Wi-Fi task ([TEAM.md](TEAM.md) section 3); GP0/GP1 belong to the sonar after start-up |
| `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_kernel.c` | `ROBOT_MQTT=1`: Wi-Fi task priority 9 (was 4); deletes the task if it fails to start | Radio work must never delay STOP or the motor loop ([TEAM.md](TEAM.md) section 4) |
| `lib/liblwip/sysdepend/pico_rp2040/lwipopts.h` | `ROBOT_MQTT=1`: MQTT buffer sizes and timeouts | Bounded packet storage for the robot client |
| `build_make/pico_rp2040.mk` | `ROBOT_MQTT` option (needs `NET=lwip`, refuses `MQTT=1`); builds lwIP's MQTT client; includes `robot_project.mk` | Builds the robot client instead of the heartbeat demo |
| `build_make/makefile` | Adds `$(LDLIBS)` to the link line | Links the maths library that `robot_project.mk` adds |

Added files: `build_make/robot_project.mk` (robot include paths and Pico SDK
drivers), `build_make/robot_sdk_headers/pico/version.h` (replaces the BSP's
out-of-date SDK version header), the two `config/*.example.h` network
settings, and the report and barcode scripts in `tools/`.

## Development setup

Use Ubuntu 24.04 or Ubuntu under WSL on Windows. Run the commands below in the
Ubuntu terminal from the repository root. The scripts do not support Git Bash.
Python 3.10 or later is required by the project tools.

Clone the repository if you do not already have a local copy:

```sh
git clone https://github.com/Exolvion/ICT_INF2004_Embedded.git
cd ICT_INF2004_Embedded
```

Install the host tools and set up the project:

```sh
sudo apt-get update
sudo apt-get install -y build-essential python3 git xz-utils curl ca-certificates
bash scripts/setup.sh
```

Setup downloads Pico SDK 2.2.0 (with its lwIP and CYW43 submodules) and Arm
GNU Toolchain 14.2.Rel1. The kernel and drivers are already in the
repository. Internet access is required. The default dependency directory is
`~/inf2004-deps`; keep it on the Linux filesystem without spaces.

For a different dependency directory, set `ROBOT_DEPS_DIR` before running both
setup and development commands. Using `setup.sh --deps` alone does not configure
later `dev.sh` commands.

Edit the robot code in `app_program/`. The firmware builds directly from this
repository with the template's makefile in `build_make/`; `scripts/dev.sh build`
runs it with the robot's profile (single core, UART console).

## Run checks and build

Run these commands after setup, from the repository root:

| Command | Purpose |
|---|---|
| `bash scripts/dev.sh host-tests` | Portable module tests in release and sanitizer configurations, plus the Python telemetry decoder's golden vectors. |
| `bash scripts/dev.sh style` | Selected BARR-C coding checks and the lock-discipline check for shared packed data. |
| `bash scripts/dev.sh app-tests` | Application scenarios using simulated inputs. |
| `bash scripts/dev.sh integration` | Board, image, build-script, radio and network integration fixtures. |
| `bash scripts/dev.sh build offline` | Build the offline firmware profile. |
| `bash scripts/dev.sh build mqtt --compile-only` | Build the MQTT profile with placeholder network settings. |

Telemetry and commands on MQTT are compact binary frames (see
[ChangeRequirement/README.md](ChangeRequirement/README.md)). Decode them on the
laptop with `mosquitto_sub -t <telemetry topic> -F %x | python3
firmware/scripts/telemetry_codec.py decode -`, and send a command byte with
`python3 firmware/scripts/telemetry_codec.py command STOP --raw |
mosquitto_pub -t <command topic> -s`.

Local check records are written under `firmware/validation/local/`; target build
outputs are under `build-target/offline/` and `build-target/mqtt/`. These generated
locations are ignored by Git. This repository currently supplies local checks;
it does not include a GitHub Actions workflow.

## Configuration and implementation status

Hardware settings are in [robot_config.h](app_program/robot_config.h), and
the course barcode configuration is in [course_cfg.c](app_program/subsystem3/course_cfg.c).
Three of the eight confirmation flags are set (`ROBOT_SERVO_CONFIRMED`,
`ROBOT_WIRING_CONFIRMED`, `ROBOT_IMU_MOUNT_CONFIRMED`); the calibration,
barcode, single-channel direction, course and corridor flags are still `0`,
and the production barcode codebook is empty. The robot firmware's start-up
and motion stay inhibited until those gates are satisfied, so the car tests
below were run with the bench MISSION firmware.

Measured values are entered in `robot_config.h` with the test that measured
them: encoder counts per wheel turn (S2-U05), wheel radius and track width
(S2-U06), IR white and black levels (S3-U03) and the sensor positions. The
logs are in `testing/`.

Host tests use simulated inputs. A successful target build establishes
compilation and linking; it does not establish physical robot operation,
calibration, live network performance or course completion.

### Wi-Fi and broker settings (fill these in)

The repository holds no network settings. Before any MQTT build, copy the two
examples and fill in the copies:

```sh
cp config/wifi_credentials.example.h config/wifi_credentials.h
cp config/net_mqtt_config.example.h config/net_mqtt_config.h
```

| File | Fill in |
|---|---|
| `config/wifi_credentials.h` | `UTK_WIFI_SSID` and `UTK_WIFI_PASSWORD` (the hotspot the car and laptop share; 2.4 GHz), then set `UTK_WIFI_CREDENTIALS_SET` to `1` |
| `config/net_mqtt_config.h` | `NET_MQTT_BROKER_IPV4`: the laptop's IP address on that hotspot, then set `NET_MQTT_CONFIGURED` to `1` |

Both copies are ignored by Git. Until they are filled in, the build stops with
an error that names the missing setting. The SSID can be 1-32 characters and
the password 8-63. An MQTT image has these settings compiled in as plain text,
so never commit or share a built `.uf2`.

## The bench MISSION firmware

[firmware/bench/bench_main.c](firmware/bench/bench_main.c) is a separate
firmware image for testing on the car. It is built from the same robot
modules as `app_program/` (motion, line controller, barcode, scanner,
terrain) but replaces the robot's task set with its own test loop. It is what
ran on the car in the system and S3-U07 tests. It is not the robot firmware:
flash the robot image back after bench testing.

### What MISSION does

START runs MISSION, which runs until STOP:

- **Line following:** the robot's line controller with a smooth steering law.
  At the start the car pivots to centre itself on the line (AUTO-CENTRE).
- **Barcodes:** the side sensor is read every 1 ms while driving. Three
  decoders read each card. Decoder 1 counts bars and spaces by darkness.
  Decoder 2 fits the blurred sensor reading to every Code 39 character (see
  [Barcode decoding](#barcode-decoding)). D0 times the sensor module's digital
  output. The car's answer is a vote: any decoder that reads gives the answer;
  if those that read disagree, the code read by more than half wins, else no
  code. A wrong code is worse than none.
- **Junction moves:** before the run, the laptop sends a map from four letters
  to the four moves (forward, U-turn, left, right). At a junction the car
  makes the move of the last card's letter: forward goes straight on, the
  others advance until the axle is over the crossing, pivot 90 or 180 degrees
  and rejoin the line. No letter is fixed in the firmware.
- **Hump height:** see [Hump height](#hump-height).
- **Obstacles:** when the front sonar finds an obstacle, the car stops, turns
  to map it with the sonar (both edges and the width), goes round the
  shorter clear side and rejoins the line on its original course.
- **Lift stop:** lifting the car ends the run and brakes the motors. The
  accelerometer's upward movement is integrated to a height; past 8 cm the car
  counts as lifted. Movement right after a motor changes direction is
  ignored, so the jolt of reversing does not stop a run.

### Kernel tasks

The image uses micro T-Kernel 3 tasks and kernel objects:

| Priority | Task | Job |
|---|---|---|
| 1 | init | Creates the kernel objects, starts the tasks, then sleeps |
| 2 | safety | Woken every 5 ms by a cyclic handler. Brakes at once on STOP (button or MQTT) and sets an event flag; passes every other command to control through a message buffer. It is the only reader of the buttons and MQTT |
| 5 | control | The test loop: line following, barcode reading, telemetry, the tests |
| 9 | Wi-Fi | The radio and MQTT task of the template |

A second cyclic handler wakes the barcode reading every 1 ms. Line following
and barcode reading stay in one task on purpose: an earlier version split them
into five tasks, and the separate sampler and telemetry tasks made barcodes
read worse and delayed the live log.

### Build, flash and commands

Fill in the network settings above first, then build in WSL from the
repository root:

```sh
bash firmware/bench/build.sh mqtt motion <laptop IP>   # MISSION image, live MQTT output; the IP overrides NET_MQTT_BROKER_IPV4
bash firmware/bench/build.sh offline motion            # no Wi-Fi; text on UART0 (GP0, 115200 8N1)
```

The image goes to `build-target/bench/bench_motion_<profile>.uf2`. Hold
BOOTSEL, plug in the Pico and copy the file onto the `RPI-RP2` drive. The
build works on a scratch copy of the project, so the working tree is never
changed.

Commands are one byte on the command topic (the car's START and STOP buttons
do the same as the first two):

| Byte | Command |
|---|---|
| `0xAA` | START: run MISSION |
| `0xA9` | STOP: end the run, brake |
| `0xAB` | Zero the wheel distance |
| 3 bytes | The letter-to-move map (`app_program/subsystem1/navmap.h`): four 5-bit letters and a 4-bit check pattern, 24 bits in all (Change Requirement 1, minimum bits) |

The car reports in plain text on the telemetry topic: a status line every
second, each barcode decoder's result, junctions, humps, obstacles, and a
report at the end of the run.

The other bench tests (duty sweep, PID steps, 1 m drive, pivots, the line
tests, the obstacle scan and the stand-alone obstacle run) are still in the
source for the unit tests in [TESTING.md](TESTING.md) section 5.

## Laptop tools: broker, dashboard and commands

All three run on the laptop on the same hotspot as the car. Topics are
`inf2004/is29/telemetry` (car to laptop), `inf2004/is29/command` (laptop to
car) and `inf2004/is29/status` (`1` when the car connects, `0` as its last
will).

### MQTT broker

Either broker works on port 1883: the Docker `eclipse-mosquitto` container
described in [TESTING.md](TESTING.md) section 3, or the self-contained broker
in [tools/broker/](tools/broker/), which needs only Node.js:

```sh
cd tools/broker
npm install        # once; installs into ./node_modules only
npm start          # listens on 0.0.0.0:1883
```

It prints the laptop's IP addresses (one of them goes into
`config/net_mqtt_config.h`) and then logs every message, so it also works as a
telemetry viewer. The car connects from another device, so Windows Firewall
must allow inbound TCP 1883 (once, as administrator):

```powershell
New-NetFirewallRule -DisplayName "mosquitto 1883" -Direction Inbound -Protocol TCP -LocalPort 1883 -Action Allow
```

### Sending commands and the junction map

```sh
cd tools/broker
npm install mqtt                          # once, for send_cmd.js
node send_cmd.js map F=A U=Z L=B R=C      # letter for forward, U-turn, left, right
node send_cmd.js start | stop | reset
```

The car answers the map with `MAP set: forward A, U-turn Z, left B, right C`.
Send the map before START (or during a run); the car keeps it until it is powered off. The
dashboard has the same map box and buttons.

### Dashboard

[tools/dashboard/](tools/dashboard/) is a web page that shows the car live and
sends it commands. It works on the hotspot without internet.

```sh
pip install paho-mqtt                     # once
```

Start the broker, then double-click `start_dashboard.bat` (Windows). It starts
the server, writes its log to `testing/system/logs/<date>_dashboard.txt` and
opens http://localhost:8080. A phone on the same hotspot can open
`http://<laptop IP>:8080`. Anyone who can open the page can press START, so
keep it to the team. To start it by hand:

```sh
cd tools/dashboard
python server.py --log ../../testing/system/logs/<date>_dashboard.txt
```

Options: `--broker HOST` (default `127.0.0.1`), `--mqtt-port 1883`,
`--topic-base inf2004/is29`, `--port 8080`, `--log FILE`.

What the page shows:

| Panel | Content |
|---|---|
| Header | Dashboard, broker and car status (online while telemetry arrived in the last 3 s); RUNNING or IDLE |
| Overview | The latest run, the card the car holds and its move, the highest hump, wheel distance |
| Mission | The ten mission requirements, each lit from what the car reported this run, and a timeline of cards, junctions, turns, humps and obstacles |
| Barcodes | Per card: the side sensor's raw profile, the bars and spaces D0 measured drawn to scale, each decoder's result and the car's vote |
| 3D map | The path the car drove, each junction, each card as the Code 39 it read, humps and obstacles with their sonar echoes. Drag to turn, scroll to zoom. Uses three.js 0.160, included under `static/vendor/` (MIT licence), so it works offline |
| Live map | About 1 m ahead of the car; at an obstacle, the sonar beams, the measured box and the planned way round |
| Controls | START, STOP (or Space), Reset distance, Next test, Next gains, and the junction map |
| Live sensors | IR left, right and barcode (raw 0-4095), wheel distance, encoder counts, accelerometer, a 60 s IR chart |
| Run results | The end-of-run report, heading and steering over the run, trace rows, CSV download; the last 12 runs |
| Event log | Every line from the car and every command seen on the broker; downloadable |

With the robot firmware instead of the bench image, the server decodes its
binary frames with `firmware/scripts/telemetry_codec.py` and shows state,
wheel speed, sonar range and faults. The robot firmware acts on STOP only.

## Barcode decoding

The barcode cards are Code 39 beside the line, read by the side IR sensor
about 1 cm above the mat. The small cards have 3 mm narrow and 9 mm wide bars;
the big cards are twice that.

### What the sensor sees

Recorded passes over the big card fit one sensor model: the sensor sees a
blurred spot (a Gaussian, σ about 3.3-3.8 mm, about 8 mm across at half
height), and its reading only starts to move once about half the spot is
dark. Scaled to the small card this predicts:

- a narrow 3 mm bar covers about a third of the spot, below that dead zone, so
  on its own it gives no signal;
- a wide 9 mm bar peaks about 1800 above white, matching the car's readings
  (1500-1900);
- a narrow bar next to a wide bar shows only as a shoulder on the wide bar.

So a decoder that counts bars cannot see the small card's narrow bars: it
finds 5-14 bars where Code 39 has 15. What survives the blur is the wide bars.
Every letter has exactly two, so a card `*X*` always has six, and their
positions alone split the 26 letters into 10 groups. Five letters (G H I J S)
are unique from the wide bars; the others come in groups (A/K/U, B/L, C/M,
D/N/V, E/O/W, F/P, Q/X, R/Y, T/Z) that differ only in the faint shoulders.

### Decoder 2: fitting the reading

Decoder 2 does not count bars. For each Code 39 character it predicts what the
sensor would read through its blur and picks the character that fits best:

1. The six largest dark humps are taken as the wide bars and fitted to each
   candidate's six wide-bar centres, both reading directions. The 10 closest
   candidates go on.
2. Each candidate's bars are blurred by the sensor model and scaled to the
   pass's own white and black. Spot size and dead zone are fitted once per
   pass; module width and start position for every candidate.
3. The best fit is a read only if the next different character fits at least
   1.3 times worse. Otherwise the decoder reports no read and names both.

Nothing about the cards or the track is fixed: the candidates are all 39
Code 39 characters with two wide bars (digits compete with letters), and module
width, start, direction, spot and dead zone are measured on every pass. The
fit runs in slices of at most 1.5 ms per line step, so the control loop is
never delayed by more than that.

### Results

Offline, on synthetic small cards (104 passes per set, every letter A-Z,
random direction, angle stretch, speed changes and missing samples), counted
as right / wrong / no read:

| Sensor | Decoder 1 | Decoder 2 |
|---|---|---|
| Sharper spot (σ 2-3 mm), noisy readings | 8 / 0 / 96 | 95 / 0 / 9 |
| The 1 cm fit (σ 3-4 mm), noisy readings | 0 / 0 / 104 | 68 / 0 / 36 |
| A second sensor model, tested against the decoder's own | 0 / 0 / 104 | 84 / 0 / 20 |

No wrong read in any test at margin 1.3 (676 synthetic passes plus the
recorded ones).

On the car, 10 counted rounds on the tape track (4 Nov, 5 small and 5 big
cards):

| Decoder | Small (5) | Big (5) | All 10 |
|---|---|---|---|
| Decoder 1 | 5 | 4 | 9 |
| Decoder 2 | 0 | 5 | 5 |
| D0 | 3 | 5 | 8 |
| Any decoder that reads, majority on disagreement (the vote above) | | | 10 |

No decoder read a card wrong. Decoder 2 held back its best guess on four small
cards, each a wrong letter (K three times, G once), by the 1.3 margin. The vote
rule was chosen from this table. Logs: `testing/subsystem3/logs/`.

The hardest pairs at 1 cm are A/K/U and T/Z, which differ only in faint
shoulders. A narrower sensor spot (a slit or tube over the sensor) would help
more than any decoder change.

### Offline tools

[firmware/bench/offline/](firmware/bench/offline/) replays recorded readings
through the bench decoder on a PC, without the car. Use it to check a decoder
change before building and flashing:

| Folder | Content |
|---|---|
| `sim/` | Host simulator: `bench_main.c` compiled with a simulated car (`sim.c`) |
| `recordings/` | Card passes cut from the 30 Oct, 31 Oct and 2 Nov runs, one file per pass |
| `traces/` | 1 mm traces of whole runs, 29-30 Oct |
| `scripts/` | `evalcuts.sh` (replay every cut), `raw2dump.py`, `splitdumps.py`, `rawplot.py`, `uf2grep.py` |

```sh
cd firmware/bench/offline/sim && bash make.sh     # builds ./sim from the current source
./sim stream ../traces/run4_trace.txt ../traces/run6_trace.txt | grep -a "CODE\|barcodes:"
bash ../scripts/evalcuts.sh ./sim ../recordings/2026-10-30_cuts ../recordings/2026-10-31_cuts
./sim batch | tail -n 1                           # 384 synthetic MISSION runs
./sim ramp H LEN TOP ROLL                         # a simulated hump
```

[testing/barcode_lab/](testing/barcode_lab/) holds the Python experiments
behind Decoder 2: the sensor model fit (`calib.py`), the template matcher
(`fast.py`, `realrun.py`), the two-stage decoder (`twostage.py`), the
synthetic card generator (`synth_bench.py`), the wide-bar groups
(`widepos.py`) and Code 39 bar widths taken from the tec-it generator
(`tecit_*.py`, `tecit_bars.json`). Results and plots are in `results/`. Run
them from that folder with Python 3 and `numpy scipy matplotlib`:

```sh
python inventory.py                        # parse every recorded pass
python calib.py "L30:13573:*A*:f:80:420"   # fit the sensor model -> calib.png
python realrun.py --gap=55 "L30:" "C30:" "T:" "T29"
python synth_bench.py gen 4 7 synB 3.0,4.0 0.42,0.58
SYN=synB python synth_bench.py match 8 && python score.py synB 1.2 1.3
python twostage.py synB
```

## Hump height

The lecturer's announcement of 27 Oct set the final ramp at about the height
of the smaller practice ramp (slightly lower or higher), so the higher ramp can
be left out. The task is to estimate the height reached using the IMU.

The track's hump is one-sided: one wheel rides over it and the car rolls
rather than pitches. The car rolls more than its 130 mm track predicts: on the
1 cm hump the single largest roll read 17-18 mm. The firmware therefore takes
the most tilted mean roll over 30 mm of path, and the height is a lever times
the sine of that roll. The lever is calibrated on the 1 cm hump: 0.102 m, set
on 19 Nov from three passes that then read 10.0, 10.7 and 9.4 mm. A rise or
lean counts only on a straight: one during which the heading changes more than
20 degrees, or a lean pitched more than 10 degrees, is reported as left out. Each
hump is reported live, and the run report gives the highest.

## Results on the car so far

| What | Result | Log |
|---|---|---|
| Barcodes, 10 counted rounds (4 Nov) | 10 of 10 with the vote, no wrong read | `testing/subsystem3/logs/2026-11-04_S3-U07_straight_live.txt` |
| Junction moves from the map (6 Nov) | 7 of 7 correct moves where the card was read and the junction seen (5 U-turns, 1 left, 1 right) | `testing/subsystem3/logs/2026-11-06_S3-U07_live.txt` |
| Hump height, 1 cm hump (9 and 19 Nov) | 9-10.7 mm | `testing/system/logs/2026-11-09_dashboard.txt`, `2026-11-19_dashboard.txt` |
| Obstacle stop while following the line (28 Oct) | 2 of 2 stopped within 10 cm, no contact | `testing/subsystem5/logs/2026-10-28_S5-U08_summary.txt` |

The robot firmware itself has been host-tested and built; its car tests are in
the `TEST_LOG.md` files under `testing/`.

## Working with the report

The current editable report is
[Week6_Report/INF2004_Week6_Design_Review.docx](Week6_Report/INF2004_Week6_Design_Review.docx).
Its PDF is included for reading.

The report-generation scripts in `tools/` retain the original
`Part1_Week6_Report/` paths and expect report data and figure assets that are not
included in this simplified repository. They are not required to build the
robot firmware and are not a working regeneration path for this copy. Maintain
the supplied Word document directly, or restore the generator inputs and update
its paths before using those scripts.
