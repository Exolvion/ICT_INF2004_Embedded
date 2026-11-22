#!/usr/bin/env bash
# Host harness: the copy's bench_main.c in one translation unit with a
# simulated car (sim.c), against the copy's portable robot modules.
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
copy=${1:-$(cd -- "$here/../../../.." && pwd -P)}
work=$here/proj
rm -rf "$work"; mkdir -p "$work"
cp -r "$copy/app_program" "$work/"
cp "$copy/firmware/bench/bench.h" "$work/"
# The bench mission, frozen as one file (firmware/bench/bench_main.c).
cp "$copy/firmware/bench/bench_main.c" "$work/bench_main.c"
# The bench copy's robot_config.h edits (build.sh), motion image.
python3 - "$work/app_program/robot_config.h" <<'PY'
import re, sys
p = sys.argv[1]; t = open(p).read()
for name, value in (("ROBOT_CALIBRATION_CONFIRMED", "1"), ("ROBOT_SERVO_CONFIRMED", "1"),
                    ("ROBOT_MAX_DUTY", "0.60"), ("ROBOT_SERVO_MIN_US", "500U"),
                    ("ROBOT_SERVO_MAX_US", "2500U")):
    t, n = re.subn(rf'(?m)^#define {name} \S+$', f'#define {name} {value}', t)
    assert n == 1, name
open(p, 'w').write('#define BENCH_IMAGE 1\n#define BENCH_ROBOT_MAX_DUTY 0.35\n#define BENCH_ROBOT_SERVO_MIN_US 1390U\n'
                   '#define BENCH_ROBOT_SERVO_MAX_US 2240U\n#define BENCH_MOTION 1\n' + t)
PY
a=$work/app_program
inc=(-I"$here/inc" -I"$work" -I"$a" -I"$a/subsystem1" -I"$a/subsystem2" -I"$a/subsystem3" -I"$a/subsystem4" -I"$a/subsystem5")
src=("$a/bitpack.c" "$a/core.c" "$a/app_cfg.c" "$a/subsystem1/navmap.c" "$a/subsystem2/motion.c" "$a/subsystem3/barcode.c"
     "$a/subsystem3/line_ctrl.c" "$a/subsystem4/terrain.c" "$a/subsystem5/scanner.c" "$a/subsystem5/vehicle.c")
gcc -std=gnu11 -O1 -g -Wall -Wextra -Wshadow -DROBOT_MQTT=0 -fsanitize=address,undefined \
    -fno-sanitize-recover=all "${inc[@]}" "$here/sim.c" "${src[@]}" -lm -o "$here/sim"
echo built "$here/sim"
