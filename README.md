# INF2004 Embedded Systems Project — IS29

This repository contains the robot firmware, its tests and test logs, the
Week 6 design report and the Tribunal 1 documents for Team IS29.

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
| Change requirement 1 | [Minimum-bit data representation: Week 13 evidence](ChangeRequirement/README.md) |
| Week 6 report feedback | [Original lecturer feedback](<Tribunal1/Sources/IS29_feedback for our week 6 report.txt>) |
| Tribunal briefing | [Notice](Tribunal1/Sources/tribunal.txt) · [Portal screenshot](<Tribunal1/Sources/Screenshot 2026-10-15 221220.png>) |
| Official tribunal template | [Week8_Tribunal1_Template.docx](Tribunal1/Sources/Week8_Tribunal1_Template.docx) |

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
| [app_program/](app_program/) | The robot application. Shared code at the top: `app_main.c` (tasks), `robot_app.c` (task steps), `board.c` (hardware), `robot_config.h` (pins and calibration), `app_cfg.c`, `core.c`, `bitpack.c`. The mission (line, barcodes, junction moves, humps, go-around) runs in the Vehicle task: its core is `subsystem5/mission.c`, and each subsystem holds its own part as `mission_*.c`. |
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
| [firmware/bench/](firmware/bench/) and [firmware/robot/sim/](firmware/robot/sim/) | The bench program used to develop and test the mission on the car before it moved into `app_program/`, its frame format (`FRAMES.md`) and the host simulator of the robot's mission. |
| [scripts/](scripts/) | Dependency setup and the development command runner. |
| [tools/](tools/) | The template's UF2 converter, the laptop dashboard and MQTT broker, the barcode sampling-budget tool, the log splitter (`split_logs.py`) and the subsystem split script (`reorg/split_c.py`). |
| [Week6_Report/](Week6_Report/) | Current report in Word and PDF. |
| [Tribunal1/](Tribunal1/) | Pre-Tribunal, original briefing materials, template and feedback. |
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
settings, and the laptop tools in `tools/` (dashboard, broker, barcode sampling
budget, log splitter and subsystem split script).

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
Four of the eight confirmation flags are set to `1`:
`ROBOT_WIRING_CONFIRMED`, `ROBOT_CALIBRATION_CONFIRMED`,
`ROBOT_SERVO_CONFIRMED` and `ROBOT_IMU_MOUNT_CONFIRMED`, which the motors,
servo and IMU code check before they act. The other four stay `0`:
`ROBOT_SINGLE_CHANNEL_DIRECTION_CONFIRMED` (the car uses two-channel
encoders), and `ROBOT_BARCODE_CONFIRMED`, `ROBOT_COURSE_CONFIRMED` and
`ROBOT_CORRIDOR_CONFIRMED`, which only the earlier build without the mission
uses (`ROBOT_APP_MISSION=0`: the Vehicle state machine in `vehicle.c` and its
codebook decoder). That build's barcode codebook is empty; the mission instead
takes the letter-to-move map over MQTT before a run.

Host tests use simulated inputs. A successful target build establishes
compilation and linking; it does not establish physical robot operation,
calibration, live network performance or course completion.

`config/wifi_credentials.example.h` and `config/net_mqtt_config.example.h` are
configuration examples. For a real MQTT build, copy them to
`config/wifi_credentials.h` and `config/net_mqtt_config.h` and configure the
copies. These filenames are ignored by Git. The `--compile-only`
profile uses placeholders and cannot connect to a network. Keep real credentials
and firmware images built with them out of the repository.

## Known deviations

Departures from the Week 6 design and from BARR-C that the team chose to keep
for the final demo.

**Remote START and RESET.** Week 6 FR10 allowed remote STOP only. The final
firmware also accepts the START (`0xAA`) and RESET (`0xAB`) command bytes over
MQTT (`app_program/subsystem1/robot_comm.c`). The Vehicle task acts on them
only while no run is under way, and a START still waits until no stop or fault
blocks it, as with the car's button. Every decision during a run comes from the
car's own sensors. STOP (`0xA9`) works at any time, from the
button or over MQTT, where the radio passes it on through its own priority
channel.

The letter-to-move junction map (3 bytes, `navmap.h`) is also sent over MQTT,
because the brief's barcode table is only an example. It is sent before the
run; a map that arrives during a run replaces the current one.

**Function length (BARR-C 6.2.a).** `bash scripts/dev.sh style` reports 21
functions over 100 lines, all in the mission files (`mission_*.c`), the longest
being `bench_line_step` (450 lines) and `bench_avoid_drive` (427 lines). They
are the line follower and the obstacle go-around tested on the car, moved over
from the bench program. They were left whole rather than split this close to
the demo without another full run on the car.

**`continue` (BARR-C 1.7).** The same check lists 16 `continue` statements for
review, also in the mission files. Fifteen are in the barcode decoder
(`mission_barcode.c`) and skip a sample or candidate it cannot use; one skips
an empty reader slot in the junction logic (`mission_line.c`). They were left
as they are for the same reason. The checker labels them
"BARR-C 8.5".

## Working with the report

The current editable report is
[Week6_Report/INF2004_Week6_Design_Review.docx](Week6_Report/INF2004_Week6_Design_Review.docx).
Its PDF is included for reading.
