# IS29 car dashboard

A web page that shows the car live over MQTT and sends it commands.

- **Header:** dashboard, broker and car status. The car counts as online while a telemetry message
  arrived in the last 3 s. The bench image goes quiet while a test drives, so a running test shows
  "Car driving". The state badge shows RUNNING or IDLE (or the robot firmware's own state).
- **Overview:** car, the latest run, the card the car holds (letter -> move), the highest hump this
  run, wheel distance, last run distance.
- **Mission:** the 10 mission requirements, each lit from what the car reported in the current run
  (`NAV:` card / junction / turned lines, `MAP set:`, `RAMP` / `SIDE` / `TERRAIN` humps and the
  `ramp: highest` report, obstacle stops, `OBST`, the go-around and `back on course`). The four
  junction moves show the letters the car confirmed; the held move lights up. A timeline lists the
  run's cards, junctions, turns, humps and obstacles. Parsed in `server.py` (`mission_line`).
- **Barcodes:** one entry per card this run. For the picked card (the newest with a read by default):
  the side sensor's raw profile (`RAW` / `R` rows, darkness above the card's white, square-root
  scale), the bars and spaces D0 measured drawn to scale on white (`D0 widths u:`, bar first), the
  characters those widths make (the firmware's own Code 39 table; 3 widest of every 9 are wide, as
  D0 decides; wide elements marked), the ideal Code 39 bars of the answer over the same span, and
  each decoder's result: Decoder 1 (`CODE` / `GROUP`), Decoder 2 (the model fit), D0 and the car's
  vote (`CAR`, after the run). Logs from before 19 Nov call Decoder 1 and 2 `CURRENT` and `NEW`;
  both spellings are read. Hover a bar for its width; the list shows each card's bars as a small
  icon. Parsed in `server.py` (`barcode_line`, `on_rawhead`, `on_rawrow`).
- **Map marks:** cards, junctions and humps are pinned where the car met them along its path (by
  distance, `mark_at`), not where the car is when the line arrives. The map stays after a run until
  the next START.
- **3D map:** the default map view (the "3D" button; "Ahead of car" and "Whole run" are the 2D
  views, and the page remembers the last one picked). It opens centred on the car and follows it;
  "Show whole run" zooms out to the whole run, still centred on the car. A white floor with the path the
  car drove on the line as black tape (dashed where it went round an obstacle), each junction it
  detected as a crossing tape, each card printed as the Code 39 it read under the barcode sensor
  (80 mm left), humps under the right wheel, obstacles (measured face and width; depth and height
  assumed) with their sonar echoes, and the car modelled from the team's photos. Drag to turn,
  right-drag to move, scroll to zoom; "Follow car" or "Show whole run". `static/map3d.js`, with
  three.js 0.160 vendored in `static/vendor/` (MIT, `three-LICENSE.txt`), so it works offline.
- **Live map:** "Ahead of car" (default) turns with the car, heading up, and shows about 1 m in
  front in cm. When the car stops at an obstacle it switches to the obstacle: the line heading at the
  stop up, fitted to the whole obstacle. While it measures, the sonar beams (`BEAMS` lines) and the
  first estimate (`AVOID: face … WIDTH ~…`) are drawn live; once `OBST` arrives, the box with its
  width, assumed depth and face distance, the track line through it, and the planned way round.
  It goes back to following the car at `AVOID: back on course`. "Whole run" shows the full path.
  The panel beside it gives the obstacle's size, face distance, left/right edges, way round and echo
  count. Pose lines may carry the compass's ` C x y z` (6 Nov images on).
- **Theme:** follows the system light/dark setting; the half-moon button in the header switches it.
- **Control:** START (0xAA), STOP (0xA9, or press Space), RESET (0xAB), and **Send map to car**:
  the four letters for forward, U-turn, left and right, sent as the 3-byte junction map
  (`app_program/subsystem1/navmap.h`); the car answers `MAP set: ...`. Send the map before START.
- **Live sensors:** IR left, right and barcode raw (0 to 4095, with min and max of the last second),
  wheel distance and encoder counts, accelerometer, and a 60 s chart of the three IR raws.
- **Run results:** the end-of-run report (result, test, gains, distance, mean speed, time per phase,
  gaps, crossings, searches, junctions, loop dt), the report lines as printed, a chart of heading and
  steering over the trace with the GAP / SEARCH / CROSS phases shaded, the trace rows, and a CSV
  download. The last 12 runs since the server started can be picked from the list.
- **Event log:** every line from the car except the periodic report and trace rows, plus every
  command seen on the broker. Download saves it as text.

It works on the hotspot without internet. A phone on the same hotspot can open it too.

## Setup (once)

```
pip install paho-mqtt
```

The broker is the contained Node broker in `tools/broker/` (`npm start`, see its README) or the
Docker container `mqtt` (eclipse-mosquitto), on port 1883. The dashboard never starts a broker.

## Launch

1. Start the broker if it is not running: `npm start` in `tools/broker`, or `docker start mqtt`.
2. Double-click **`start_dashboard.bat`**. It checks port 1883, starts the server (log in
   `testing\system\logs\<date>_dashboard.txt`) and opens http://localhost:8080.
3. Power on the car. Within a few seconds the header shows **Car online** and the IR values move.

From a phone on the same hotspot: open `http://<laptop IP>:8080` (`ipconfig`, the hotspot adapter's
IPv4 address). If Windows Firewall asks, allow Python on private networks. Anyone on the hotspot who
opens the page can press START, so keep the page to the team.

### Manual launch

```
cd tools\dashboard
python server.py --log ..\..\testing\system\logs\2026-10-29_dashboard.txt
```

`server.py` options: `--broker HOST` (default 127.0.0.1), `--mqtt-port 1883`,
`--topic-base inf2004/is29`, `--port 8080`, `--log FILE`.

## What it reads and sends

| Topic | Direction | Content |
|---|---|---|
| `inf2004/is29/telemetry` | car to laptop | The mission's lines, one minimum-bit frame per line (first byte 0xBD, `firmware/bench/FRAMES.md`), decoded back to the line by `tools/dashboard/bench_codec.py`; and the robot's binary status, perception and diagnostic frames. |
| `inf2004/is29/status` | car to laptop | `1` when the car connects, `0` as its last will. |
| `inf2004/is29/command` | laptop to car | One byte: 0xAA START, 0xA9 STOP, 0xAB RESET; or the 3-byte junction map. |

The mission (`app_program/subsystem5/mission.c` and its `mission_*.c` parts, in the robot image;
the bench image ran the same mission until 22 Nov) prints:

| Line | Shown as |
|---|---|
| `IR L 499 476-513 R 2267 2248-2291 B 228 195-256` | IR tiles and chart (mean, min-max of the last second) |
| `ACC mg n 52 x +31 -52..+141 y ... z ...` | Accelerometer tile |
| `ENC counts L -44 R -1 mm L -4 R +0 err 0` | Wheel distance and encoder tiles |
| `MODE <test>, gains 4 P 3.00 I 15.00 D 0.00` | Selected test and gains |
| `DRIVE start: ...` (or PID, SWEEP, SCAN start) | A new run |
| `<test> stopped: line lost after 83.0 s: gains ..., dt max 8.0 ms` | Run result |
| ` centre: ...`, ` gaps ...`, ` junctions crossed straight on 6` and other indented lines | Run report |
| ` 500 d +71 L 263 R 1545 B 2070 s +0 TRACKING h 2.0 c GAP w ... u ...` and ` end ...` | Trace |

A message the car could not queue is lost, which cuts one line and glues the next onto it. The
server splits such lines at the known line starts, keeps the complete part and counts the cut part
(shown under the event log).

The robot image (`app_program`) also sends binary status, perception and diagnostic frames; the
server decodes them with `firmware/scripts/telemetry_codec.py` and shows state, wheel speed, sonar
range and faults. The robot image acts on START, STOP, RESET and the junction map (README "Known
deviations"); a START waits until no stop or fault blocks it.

## Files

| File | What it does |
|---|---|
| `server.py` | Subscribes to the three topics, joins and parses the car's lines, streams them to the page (Server-Sent Events, `/events`), serves `/api/state` and `/api/run?id=N`, and publishes command bytes (`POST /api/cmd`). |
| `static/index.html` | The page (no external libraries). |
| `start_dashboard.bat` | One-click launcher. |
