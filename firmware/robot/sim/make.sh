#!/usr/bin/env bash
# Host sim of the robot app running the mission (robot_sim.c). robot_app.c is
# compiled with the host tests' strict flags (firmware/scripts/
# test_application.py) and -Werror; mission.c as the bench sim compiles it,
# with MISSION_SIM (its console on). Run in WSL from anywhere:
#   bash firmware/robot/sim/make.sh && firmware/robot/sim/robot_sim corner
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
repo=$(cd -- "$here/../../.." && pwd -P)
a=$repo/app_program
inc=(-I"$repo/firmware/bench/offline/sim/inc" -I"$a" -I"$a/subsystem1"
     -I"$a/subsystem2" -I"$a/subsystem3" -I"$a/subsystem4" -I"$a/subsystem5")
strict=(-std=c99 -Wall -Wextra -Wpedantic -Werror -Wconversion
        -Wsign-conversion -Wshadow -Wstrict-prototypes -Wmissing-prototypes
        -Wformat=2 -Wundef -Wcast-qual -Wwrite-strings -Wswitch-enum
        -Wnull-dereference)
san=(-O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all)
work=$here/obj
rm -rf "$work"; mkdir -p "$work"
# The robot app's parts: its core (app_program/robot_app.c) and one task
# file per subsystem (robot_*.c), sharing app_program/robot_app_private.h.
robot_objs=()
for part in "$a/robot_app.c" "$a"/subsystem*/robot_*.c; do
    obj="$work/$(basename "$part" .c).o"
    gcc "${strict[@]}" "${san[@]}" -DROBOT_MQTT=0 "${inc[@]}" \
        -c "$part" -o "$obj"
    robot_objs+=("$obj")
done
# The mission's parts: its core in subsystem5 and one file or two per
# subsystem (mission_*.c), sharing app_program/mission_private.h.
mission_objs=()
for part in "$a"/subsystem*/mission*.c; do
    obj="$work/$(basename "$part" .c).o"
    gcc -std=gnu11 -Wall -Wextra -Wshadow "${san[@]}" -DROBOT_MQTT=0 \
        -DMISSION_SIM "${inc[@]}" -c "$part" -o "$obj"
    mission_objs+=("$obj")
done
src=("$a/bitpack.c" "$a/core.c" "$a/app_cfg.c" "$a/subsystem1/navmap.c"
     "$a/subsystem1/mqttcmd.c" "$a/subsystem1/telemetry.c"
     "$a/subsystem2/motion.c" "$a/subsystem3/barcode.c"
     "$a/subsystem3/course_cfg.c" "$a/subsystem3/line_ctrl.c"
     "$a/subsystem4/terrain.c" "$a/subsystem5/scanner.c"
     "$a/subsystem5/vehicle.c")
gcc -std=gnu11 -Wall -Wextra -Wshadow "${san[@]}" -DROBOT_MQTT=0 "${inc[@]}" \
    "$here/robot_sim.c" "${src[@]}" "${robot_objs[@]}" "${mission_objs[@]}" \
    -lm -o "$here/robot_sim"
echo built "$here/robot_sim"
