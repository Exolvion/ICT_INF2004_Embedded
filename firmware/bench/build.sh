#!/usr/bin/env bash
# BENCH IMAGE, NOT THE ROBOT FIRMWARE (see app_program/subsystem5/mission.c
# for the commands).
# Builds from a scratch copy of the project, so the working tree is never
# modified. Run in WSL from the repository root:
#   bash firmware/bench/build.sh offline|mqtt [motion] [BROKER_IPV4]
# The image goes to build-target/bench/bench_<profile>.uf2, or with
# "motion" to bench_motion_<profile>.uf2 (git-ignored). An mqtt image holds
# the Wi-Fi settings in plain text: never share or commit it, share its hash
# only.
#
# The copy's robot_config.h opens the motor and servo outputs for the bench
# (ROBOT_CALIBRATION_CONFIRMED and ROBOT_SERVO_CONFIRMED 1), allows 60% duty
# for the sweep and widens the servo limits to 500-2500 us for finding the
# endpoints. It also records the robot's own ROBOT_MAX_DUTY as
# BENCH_ROBOT_MAX_DUTY for the motion tests and its servo endpoints as
# BENCH_ROBOT_SERVO_MIN_US/MAX_US for the SCAN test, and "motion" sets
# BENCH_MOTION.
# The robot firmware's own settings are unchanged.
set -euo pipefail

usage='usage: bash firmware/bench/build.sh offline|mqtt [motion] [BROKER_IPV4]'
profile=${1:-}
motion=0
broker=
for arg in "${@:2}"; do
    if [[ "$arg" == motion ]]; then
        motion=1
    elif [[ "$arg" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ && -z "$broker" ]]; then
        broker=$arg
    else
        printf '%s\n' "$usage" >&2
        exit 2
    fi
done
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)
src="$repo/firmware/bench"
# shellcheck disable=SC1091
source "${ROBOT_DEPS_DIR:-$HOME/inf2004-deps}/env.sh"

case "$profile" in
    offline) net=(WIFI=none NET=none MQTT=0 ROBOT_MQTT=0)
             image=mtk3pico_smp0_uart ;;
    mqtt) net=(WIFI=cyw43 NET=lwip MQTT=0 ROBOT_MQTT=1)
          image=mtk3pico_smp0_uart_wifi_lwip ;;
    *) printf '%s\n' "$usage" >&2
       exit 2 ;;
esac

work=$(mktemp -d "${TMPDIR:-/tmp}/inf2004_bench_XXXXXX")
trap 'rm -rf -- "$work"' EXIT
project="$work/project"
python3 - "$repo/firmware/scripts" "$project" <<'PY'
import sys
from pathlib import Path
sys.path.insert(0, sys.argv[1])
from robot_paths import copy_project
copy_project(Path(sys.argv[2]))
PY
# The bench source is firmware/bench/bench_main.c: the mission as one file,
# frozen when the robot app's mission was split into its subsystems (22 Nov
# 2026), BENCH_IMAGE making it the bench loop. It replaces the robot's
# mission parts in the copy.
rm -f -- "$project"/app_program/subsystem*/mission*.c \
         "$project/app_program/mission_private.h"
cp -- "$src/bench_main.c" "$project/app_program/subsystem5/mission.c"
cp -- "$src/bench.h" "$project/app_program/"

# Point usermain at the bench loop; the robot's start-up is kept but unused.
python3 - "$project/app_program/app_main.c" <<'PY'
import re, sys
path = sys.argv[1]
text = open(path, newline='').read()
def entry(match):
    nl = match.group(1)
    return ('INT usermain (void)' + nl + '{' + nl +
            '    /* BENCH IMAGE: bench measurements; the robot never starts. */' + nl +
            '    bench_loop();' + nl + '}' + nl + nl +
            'static __attribute__((unused)) INT robot_usermain (void)' + nl + '{' + nl)
text, count = re.subn(r'INT usermain \(void\)(\r?\n)\{\r?\n', entry, text, count=1)
assert count == 1, 'usermain definition not found'
text, count = re.subn(r'(#include "board\.h")(\r?\n)', r'\1\2#include "bench.h"\2', text, count=1)
assert count == 1, 'board.h include not found'
open(path, 'w', newline='').write(text)
PY

# Bench-only settings in the copy; each edited line must exist exactly once.
python3 - "$project/app_program/robot_config.h" "$motion" <<'PY'
import re, sys
path = sys.argv[1]
text = open(path, newline='').read()
robot_max = re.findall(r'(?m)^#define ROBOT_MAX_DUTY (\S+)$', text)
assert len(robot_max) == 1, 'expected one ROBOT_MAX_DUTY'
robot_servo = {}
for name in ("ROBOT_SERVO_MIN_US", "ROBOT_SERVO_MAX_US"):
    found = re.findall(rf'(?m)^#define {name} (\S+)$', text)
    assert len(found) == 1, f'expected one {name}'
    robot_servo[name] = found[0]
for name, value in (("ROBOT_CALIBRATION_CONFIRMED", "1"),
                    ("ROBOT_SERVO_CONFIRMED", "1"),
                    ("ROBOT_I2C_HZ", "100000U"),  # as hump4 was tested
                    ("ROBOT_MAX_DUTY", "0.60"),
                    ("ROBOT_SERVO_MIN_US", "500U"),
                    ("ROBOT_SERVO_MAX_US", "2500U")):
    text, count = re.subn(rf'(?m)^#define {name} \S+$',
                          f'#define {name} {value}', text)
    assert count == 1, f'expected one {name}'
header = ('/* BENCH IMAGE ONLY: outputs opened for bench measurements. */\n'
          '#define BENCH_IMAGE 1\n'
          f'#define BENCH_ROBOT_MAX_DUTY {robot_max[0]}\n'
          f'#define BENCH_ROBOT_SERVO_MIN_US {robot_servo["ROBOT_SERVO_MIN_US"]}\n'
          f'#define BENCH_ROBOT_SERVO_MAX_US {robot_servo["ROBOT_SERVO_MAX_US"]}\n')
if sys.argv[2] == '1':
    header += '#define BENCH_MOTION 1\n'
open(path, 'w', newline='').write(header + text)
PY

# The telemetry slots (net_mqtt_ipc.h) are the mission's 64-byte frames, 8
# deep, sent in bursts, in both builds since 22 Nov: nothing to change.

if [[ "$profile" == mqtt ]]; then
    cp -- "$repo/config/wifi_credentials.h" "$repo/config/net_mqtt_config.h" \
        "$project/config/"
    if [[ -n "$broker" ]]; then
        sed -i -E "s/(#define[[:space:]]+NET_MQTT_BROKER_IPV4[[:space:]]+)\"[0-9.]+\"/\1\"$broker\"/" \
            "$project/config/net_mqtt_config.h"
        grep -q "\"$broker\"" "$project/config/net_mqtt_config.h"
    fi
fi

# ROBOT_ROM_MATHS=0: the frozen bench keeps libgcc/libm, as hump4 was tested
# (the robot app uses the boot ROM's maths since 23 Nov, robot_project.mk).
make -C "$project/build_make" -j4 SMP=0 CONSOLE=uart \
    PICO_SDK_PATH="$ROBOT_SDK" "${net[@]}" ROBOT_ROM_MATHS=0 all

out="$repo/build-target/bench"
name=bench_$profile.uf2
if [[ "$motion" == 1 ]]; then
    name=bench_motion_$profile.uf2
fi
mkdir -p -- "$out"
cp -- "$project/build_make/$image.uf2" "$out/$name"
arm-none-eabi-size "$project/build_make/$image.elf"
sha256sum "$out/$name"
