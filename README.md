# INF2004 Embedded Systems Project — IS29

This repository contains the robot firmware, Week 6 design report,
Pre-Tribunal document, tribunal briefing materials and assignment references
for Team IS29.

The project is an autonomous robot built around the Raspberry Pi Pico W and
Robo Pico board. Its intended mission is to follow a line, read navigation
barcodes, make controlled turns, estimate hump height, detect and bypass
obstacles, recover the track and report its state over Wi-Fi/MQTT. The firmware
is written in C using the Pico SDK and the supplied micro T-Kernel RP2040 port.

## Project documents

| Material | File |
|---|---|
| Week 6 design report | [Word document](Week6_Report/INF2004_Week6_Design_Review.docx) · [PDF](Week6_Report/INF2004_Week6_Design_Review.pdf) |
| Pre-Tribunal | [IS29_Pre-Tribunal1.docx](Tribunal1/IS29_Pre-Tribunal1.docx) |
| Week 6 report feedback | [Original lecturer feedback](<Tribunal1/Sources/IS29_feedback for our week 6 report.txt>) |
| Tribunal briefing | [Notice](Tribunal1/Sources/tribunal.txt) · [Portal screenshot](<Tribunal1/Sources/Screenshot 2026-10-15 221220.png>) |
| Official tribunal template | [Week8_Tribunal1_Template.docx](Tribunal1/Sources/Week8_Tribunal1_Template.docx) |
| Assignment requirements | [Project Write-up](<References/Project Write-up(2).pdf>) · [Project Briefing](<References/Project Briefing(1).pdf>) |
| Module and assessment overview | [W0 Module Overview](<References/W0_Module Overview.pdf>) |
| Track and barcode requirements | [Barcode & Line Specification](<References/Barcode & Line Specification(1).pdf>) · [Barcode Sample](<References/Barcode Sample.pdf>) |
| Reference index | [Project sources](References/README.md) · [Hardware and device guides](References/Device_Guides/README.md) |

The official tribunal template includes the assessment stages. The prepared
tribunal document supplied here is the Pre-Tribunal.

## Repository layout

| Location | Contents |
|---|---|
| [firmware/src/](firmware/src/) and [firmware/include/](firmware/include/) | Portable modules for motion, navigation, line/barcode processing, terrain, scanning, commands and telemetry. |
| [firmware/target/](firmware/target/) | Pico hardware drivers, task scheduling, application integration, network adapters and configuration. |
| [firmware/tests/](firmware/tests/) and [firmware/scripts/](firmware/scripts/) | Module tests, application scenarios, integration checks, installation and build verification. |
| [scripts/](scripts/) | Dependency setup and the development command runner. |
| [local/](local/) | Example Wi-Fi and MQTT configuration headers. |
| [Tools/](Tools/) | Barcode sampling-budget tool and existing report-maintenance scripts. |
| [Week6_Report/](Week6_Report/) | Current report in Word and PDF. |
| [Tribunal1/](Tribunal1/) | Pre-Tribunal, original briefing materials, template and feedback. |
| [References/](References/) | Assignment specifications, lecture transcript, templates and hardware references. |
| [firmware/third_party_notices/](firmware/third_party_notices/) | Dependency licence notices. |

## Development setup

Use Ubuntu 24.04 or Ubuntu under WSL on Windows. Run the commands below in the
Ubuntu terminal from the repository root. The scripts do not support Git Bash.
Python 3.10 or later is required by the project tools.

Clone the repository if you do not already have a local copy:

```sh
git clone https://github.com/teck1341/ICT_INF2004_Embedded.git
cd ICT_INF2004_Embedded
```

Install the host tools and set up the project:

```sh
sudo apt-get update
sudo apt-get install -y build-essential python3 git xz-utils curl ca-certificates
bash scripts/setup.sh
```

Setup downloads the pinned micro T-Kernel RP2040 port, Pico SDK 2.2.0 and Arm GNU
Toolchain 14.2.Rel1, then installs the application into the generated port
checkout. Internet access is required. The default dependency directory is
`~/inf2004-deps`; keep it on the Linux filesystem without spaces.

For a different dependency directory, set `ROBOT_DEPS_DIR` before running both
setup and development commands. Using `setup.sh --deps` alone does not configure
later `dev.sh` commands.

Edit source files in `firmware/`. The port checkout under the dependency
directory is generated from those files.

## Run checks and build

Run these commands after setup, from the repository root:

| Command | Purpose |
|---|---|
| `bash scripts/dev.sh host-tests` | Portable module tests in release and sanitizer configurations. |
| `bash scripts/dev.sh style` | Selected BARR-C coding checks. |
| `bash scripts/dev.sh app-tests` | Application scenarios using simulated inputs. |
| `bash scripts/dev.sh integration` | Board, installer, image, build-script, radio and network integration fixtures. |
| `bash scripts/dev.sh build offline` | Build the offline firmware profile. |
| `bash scripts/dev.sh build mqtt --compile-only` | Build the MQTT profile with placeholder network settings. |

Local check records are written under `firmware/validation/local/`; target build
outputs are under `build-target/offline/` and `build-target/mqtt/`. These generated
locations are ignored by Git. This repository currently supplies local checks;
it does not include a GitHub Actions workflow.

## Configuration and implementation status

Hardware settings are in [robot_config.h](firmware/target/robot_config.h), and
the course barcode configuration is in [course_cfg.c](firmware/target/course_cfg.c).
The supplied configuration has all eight hardware/course confirmation flags set
to `0`, and the production barcode codebook is empty. Startup and motion remain
inhibited until the required configuration and commissioning gates are satisfied.
The configured physical values are hypotheses that still require measurement.

Host tests use simulated inputs. A successful target build establishes
compilation and linking; it does not establish physical robot operation,
calibration, live network performance or course completion.

The two files in `local/` are configuration examples. For a real MQTT build,
copy them to `local/wifi_credentials.h` and `local/net_mqtt_config.h` and configure
the local copies. These filenames are ignored by Git. The `--compile-only`
profile uses placeholders and cannot connect to a network. Keep real credentials
and firmware images built with them out of the repository.

## Working with the report

The current editable report is
[Week6_Report/INF2004_Week6_Design_Review.docx](Week6_Report/INF2004_Week6_Design_Review.docx).
Its PDF is included for reading.

The report-generation scripts in `Tools/` retain the original
`Part1_Week6_Report/` paths and expect report data and figure assets that are not
included in this simplified repository. They are not required to build the
robot firmware and are not a working regeneration path for this copy. Maintain
the supplied Word document directly, or restore the generator inputs and update
its paths before using those scripts.
