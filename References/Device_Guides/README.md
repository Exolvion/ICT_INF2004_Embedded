# Course device guides (xSITe "Datasheet (for Default Project)")

Source: the module page *INF2004-Embedded Systems Programming [2026/27 T1] -
Datasheet (for Default Project)*, downloaded as
`INF2004-Embedded Systems Programming [202627 T1] - 9222026 - 404 PM.zip`
(25,847,016 bytes, SHA-256
`2851ff57fbd54028c0f88e87b0e1e798193f26e501a869b673eda412eb386580`) and
extracted on 22 September 2026. Items 1–10 and the table of contents are
byte-identical to their zip entries; item 11 came from the lecturer's
24 September post (see its row) and item 11a is derived from it. They
cover the Pico W, the Robo Pico, the drive motors with encoders, and the
IR, ultrasonic and IMU modules. The SG90 servo appears only as a stock
picture in item 11, and the slot sensor, the power bank and the Raspberry
Pi Debug Probe have no document here. The cross-check of the project
against them is `docs/Device_Guide_Crosscheck_2026-09-22.md` (section 8
covers item 11).

**Course material from xSITe: private, never published.** Neither these
files nor the zip at the repository root may enter the public team
repository, and the private repository that holds them must never be made
public or have its history pushed to a public remote (see the root
`README.md`, section 1a).

Page numbers are 1-based PDF pages. A "`PicoCar.pptx` slide N" elsewhere in
the repository means item 10; its slides 1–4 are slides 2, 5, 6 and 7 of
item 11.

| # | File | SHA-256 | Bytes | What it establishes for this project |
|---|---|---|---:|---|
| 1 | `RP-008312-DS-2-pico-w-datasheet.pdf` (Raspberry Pi, release 7, July 2026) | `ba8649ba98437301b9f1621504569a27f1d2ba6111795b4df76783adb08de7c5` | 20175431 | Pico W: GP26–GP28 are ADC0–2, GP23/24/25/29 are internal (wireless, VSYS/3), I/O fixed at 3.3 V, ADC-capable pins must stay below IOVDD + 0.3 V, VSYS 1.8–5.5 V, 3V3 output load under 300 mA (pp8–9, 14); pinout: I2C1 on GP2/GP3, UART0 on GP0/GP1 (p5 figure) |
| 2 | `Robo_Pico.pdf` (Cytron Rev 1.0, April 2023) | `f4bc6b22ea12e97fb7098ac8d57ef6273dd6eaaa4d647c3639ea9e04b49a78ff` | 1969548 | M1A GP8, M1B GP9, M2A GP10, M2B GP11 (p5); brake/forward/backward/coast truth table (p9); motor PWM at most 20 kHz, 1 A continuous, 1.5 A peak (p7); servo ports GP12–GP15, buttons GP20/GP21, NeoPixel GP18, buzzer GP22, Grove port map (p4); VBAT sense jumper on GP28 (p5); 3.6–6 V input, Grove 3V3 total 300 mA (p7) |
| 3 | `tcrt5000.pdf` (Vishay) | `fbf04bfbc82a74ab339756fd0c975522bef64c75cdbd78ea97b348166c2b61b4` | 176657 | Bare reflective sensor: 950 nm emitter, peak response at 2.5 mm, usable 0.2–15 mm (p1) |
| 4 | `HCSR04.pdf` (Elecfreaks) | `4ebdc3e1f70d84a1ca856d8fcd7f8b1f9e548a94e4012cd86d14ca0b30543b06` | 80625 | 5 V, 15 mA, 10 µs trigger, eight-cycle 40 kHz burst, µs/58 = cm, cycle over 60 ms, 2–400 cm, 15° (pp1–2). The table's "working frequency 40Hz" is the 40 kHz carrier with a unit typo |
| 5 | `Infrared-Reflective-Sensor-UserManual.pdf` (Waveshare) | `37f8190738ac2c15e8d2721cd89ce7e146da3bcd8b649b11a531d5bdcdf1885f` | 27401 | Module: ST188 sensor, LM393 comparator, 3.0–5.3 V, pins DOUT, AOUT, GND, VCC (p1); 3.3 V hookup example (p2) |
| 6 | `LM393-D.PDF` (onsemi) | `2806d40bbb23d56be226b284d0de10753c946d824a28fd5b5b25d4d431df25fd` | 448851 | Dual comparator used on the Waveshare module; the TCRT5000 line-tracker module pictured in item 11 slide 4 carries an 8-pin comparator, typically this part (inference from the photo); no wiring of the assembled module |
| 7 | `lsm303dlhc.pdf` (ST DocID018771 Rev 2) | `b9a85ab640056878fb44a2adfa1e6fa4d554e725c68ecd8fe7332515eaf45fd7` | 833044 | Accelerometer address 0x19, magnetometer 0x1E (pp21–22); CTRL_REG1_A/CTRL_REG4_A, STATUS_REG_A and OUT registers (pp23–29); 1 mg/LSB at ±2 g (p10); Vdd 2.16–3.6 V (p12); 10 kΩ pull-ups (p17); no gyroscope |
| 8 | `OKY3231-2.pdf` | `afd6696cd45e7d5e18d55bb4c5444656f0d6efdc180d75948ad90192107dcf1d` | 117148 | GY-511 module carrying the LSM303DLHC, 3–5 V input, I2C |
| 9 | `Robo Pico Board Datasheet.pdf` (Cytron Rev 1.0) | `b2e62f614824e64ae884bde0a3f26ded4dc0252877061e544561e02c62beab1b` | 2250520 | Same content as item 2 with different pagination (truth table p8) plus the power tree (p9): buck-boost converter 800 mA total for the 3V3 loads |
| 10 | `PicoCar.pptx` (four image-only slides) | `c6081d9f157e47ac37546794da05c7a85cf44714e385746bb9dead1e8e14cd01` | 945660 | Slide 1: drive motor with encoder, PH2.0 6-pin interface: 1 encoder ground, 2 encoder output A phase, 3 encoder output B phase, 4 encoder power supply, 5 motor line +, 6 motor line −. Slide 2: GY-511 header VIN, 3.3V, GND, SCL, SDA, INT2, INT1, DRDY (module silkscreen, `OKY3231-2.pdf` p1, item 8; the slide's annotation swaps INT1/INT2); only the supply, GND, SCL and SDA are used; wire by the silkscreen. Slide 3: Pico W pinout. Slide 4: Robo Pico board functions (RGB LED labelled NeoPixel WS2812) |
| 11 | `PicoCar_RoboCar_Connections_2026-09-24.pptx` (seven slides; lecturer post "Default Project - RoboCar Connections", 24 September 2026, 4:49 PM, supplied by the team coordinator on 25 September) | `cc42368b3b6d6bf83cc60fef75bb7da384317c9bade5a9d41cda1e45b570651d` | 1241001 | The lecturer's ink wiring diagrams; the GPIO choice is left to the teams. Slide by slide: see *Item 11, slide by slide* below the table |
| 11a | `PicoCar_RoboCar_Connections_2026-09-24_ink_render.pdf` (derived, not course-issued) | `6bf216d6f36b33b5abb14096bf1110cd3d60b6f0874c9241a172a5ce173b6603` | 950220 | LibreOffice render of item 11 with PowerPoint's stored ink fallback images, so the wiring is visible in any PDF viewer (LibreOffice and many previewers skip the ink) |
| – | `Table of Contents.html` | `f0037686080e55af3490d3d193c9497d58c890fd41eafdbff53238ad1b5fdf6a` | 1657 | The xSITe listing of items 1–10 (item 11 came from the lecturer's post, not from this page) |

## Item 11, slide by slide

The wiring is drawn as ink ("you will need to download it to see the
wiring").

- Slide 1: the power bank goes to the VIN terminal (red to +, black to −;
  silkscreen "3.6-6V VIN"; no voltage or model given). Each motor's PH2.0
  pin 5 (motor +) goes to the MxB screw and pin 6 (motor −) to MxA of a
  MOTOR terminal, both motors alike. Encoder pin 4 (supply) goes to 3.3 V
  and pin 1 to GND; the ink ends at "3.3v" and "GND" labels, not at a board
  contact. "PIN 2 & 3 can use any GPIO pins on the Pico".
- Slide 2: the PH2.0-6P pin list (as item 10 slide 1).
- Slide 3: a stock picture of an SG90 9 g micro servo (brown −, red +,
  orange signal) wired to the servo header. The board picture is rotated
  and mirrored, and the ink ends on the GP15 column (servo port 4). That is
  illustrative only: this project keeps the servo on servo port 1 (GP12).
- Slide 4: a 4-pin TCRT5000 IR line-tracker module (stock photo, alt text
  "TCRT 5000 IR Infrared Sensor Line tracker Module"; header VCC, GND, D0,
  A0; an 8-pin comparator on the board) with VCC to 3.3 V and GND:
  "*D0 = Digital Out (Can use any GPIO [High/Low]) / *A0 = Analog Out (need
  to use ADC) / *Pick One". Also an HC-SR04 with Vcc to 3.3 V and GND,
  "TRIG & ECHO can use any GPIO pins on the Pico", and no divider drawn.
  The photo prints "HC-SR04"; its alt text is the title of an article on
  the HC-SR04P, the 3.0–5.5 V variant.
- Slides 5–7: as item 10 slides 2–4.

## Answered by the lecturer's wiring slides

The slides (item 11, described above) cover the power source, the encoder
supply, the HC-SR04 supply, the servo model and the IR board. They are
instructions and stock pictures, not measurements, so each is confirmed
on the issued kit at stage 1. Where the project adds to or departs from them:

- Power: with only VIN connected, the motor and servo V+ equal VIN
  (item 2 p7).
- Encoder supply: the slide does not say which 3.3 V contact; the project
  uses the 3V3 contact of the encoder's own Grove port.
- HC-SR04: a 3.3 V supply suits only a 3.3 V-capable module (HC-SR04P
  type); item 4 specifies 5 V. The kit photos of 26 September 2026 show the
  classic 5 V module, so the project proposes 5 V from servo port 3 "+"
  with the 2.2 kΩ/3.3 kΩ ECHO divider to GP15. This departs from slide 4
  and waits for the lecturer's approval of the pin request
  ([record](../../Evidence/Kit_Photos_2026-09-26_Set2/RECORD.md)).
- IR board: the project uses A0 only. The Waveshare ST188 module (item 5)
  has the same functions in the order DOUT, AOUT, GND, VCC, so every module
  is wired by its printed labels.

## What no guide establishes

- Encoder output type (push-pull or open-drain, with or without on-board
  pull-ups) and level, pulses per revolution, gearbox ratio, and whether
  pulses are counted at the motor or wheel shaft.
- Motor rated voltage, stall current and left/right mounting.
- Servo safe pulse range, centre and settling time; no servo datasheet is
  in the set. The kit servo's label reads "SG90" (kit photo 7).
- Which IR board sits at each of the three positions (the expected TCRT5000
  module or a Waveshare ST188 module), and the A0 polarity over black and
  white.
- The polarity of the Robo Pico push buttons on GP20/GP21.
- Which accelerometer chip the kit's GY-511 carries. Its eighth pin reads
  SA0 where item 8 shows DRDY (kit photos 9 and 10), so it may not be the
  LSM303DLHC of item 7. The stage 1 I2C bus scan expects 0x19; a board
  that answers only at 0x1D or 0x1E needs a firmware change before
  `ROBOT_WIRING_CONFIRMED` is set at stage 2.
- The HC-SR04's real range and beam width against the course obstacles
  (item 4 gives only the nominal 2–400 cm and 15°), its no-echo behaviour,
  and its 5 V supply under load. Pico W VBUS is 0 V without USB (item 1,
  p9), so the servo header "+" pins carry VIN (5 V from the power bank)
  through the 3.6–6 V input selector (item 9, p9), shared with the motors
  and servo; measure it at the module with the motors running.
- The power bank's output under load and whether it switches off at light
  load. Its label, read on 26 September 2026, gives 5 V, 2.1 A per output,
  20000 mAh. A switch-off, or a trip on motor inrush, resets the Pico.
- Whether the Robo Pico ON/OFF switch removes USB power from the motor
  driver. The driver has no separate supply: it runs from the highest of
  VIN, USB and LiPo (item 9, p9), and from VUSB − 0.4 V on USB alone
  (item 2 p7). Unplugging the power bank therefore does not isolate the
  motors while USB is connected.
- The numbering of the Grove contacts. The board image (item 10 slide 4 and
  item 11 slide 7, the same as item 2 p3) prints each port as GND, 3V3,
  GPa, GPb; with the standard Grove order that makes the GPa contact pin 2
  and the outer GPb contact pin 1. The project names contacts by their
  silkscreen GPIO label and confirms each one with a continuity meter.

Everything in this list stays a measured commissioning input; see
`docs/Hardware_Testing_Handoff.md`.

**Re-download, 26 September 2026.** The same xSITe page was downloaded again
as `INF2004-Embedded Systems Programming [202627 T1] - 9262026 - 119 PM.zip`
(26,127,099 bytes, SHA-256
`ecd2695880bef0c687e75082b703b523409b07cb115485b04959e96453a883d0`; not
committed, because it adds no new file). It holds the same 11 files: items
1–9 and `Table of Contents.html` are byte-identical to the copies here, and
its `PicoCar.pptx` is byte-identical to item 11
(`PicoCar_RoboCar_Connections_2026-09-24.pptx`, SHA-256 `cc42368b…70651d`).
So the lecturer replaced the four-slide deck (item 10) on the portal with
the seven-slide wiring deck under the same name. Item 10 stays here as the
earlier version that older citations "`PicoCar.pptx` slide N" refer to.

Git keeps `*.html` and `*.pptx` here byte-identical (`../.gitattributes`).
