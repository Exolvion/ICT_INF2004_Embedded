#!/usr/bin/env bash
# Full check set on a WSL-native copy of the working tree, two parallel waves.
set -u
src=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd -P)
tree="$HOME/inf2004_wip/check_tree"
out=/tmp/inf2004_checks
broker=${1:?usage: fullcheck.sh <broker IPv4>}
mkdir -p "$tree" "$out"
rm -f "$out"/*.log "$out"/*.rc
rsync -a --delete --exclude build-target --exclude 'testing/*/logs' \
    --exclude References "$src/" "$tree/"
cd "$tree" || exit 2
step() {
    local name=$1
    shift
    local t0=$SECONDS
    "$@" > "$out/$name.log" 2>&1
    local rc=$?
    echo "$rc" > "$out/$name.rc"
    printf '%-18s %s %4ds\n' "$name" "$([ $rc -eq 0 ] && echo PASS || echo "FAIL rc=$rc")" \
        $((SECONDS - t0))
}
# Wave 1: tests and style.
step host-tests bash scripts/dev.sh host-tests &
step style bash scripts/dev.sh style &
step app-tests bash scripts/dev.sh app-tests &
step integration bash scripts/dev.sh integration &
step strict bash firmware/bench/wip_2026-10-29/strict.sh &
wait
# Wave 2: images. The robot builds share build_make's objects: one at a time.
(step build_offline bash scripts/dev.sh build offline
 step build_mqtt bash scripts/dev.sh build mqtt) &
step bench_offline bash firmware/bench/build.sh offline &
step bench_mqtt bash firmware/bench/build.sh mqtt &
step bench_motion bash firmware/bench/build.sh mqtt motion "$broker" &
wait
(cd ~/inf2004_wip/barcode2 && bash make.sh > /tmp/inf2004_checks/sim_make.log 2>&1 \
    && echo "sim_make PASS" && ./sim stream ../traces/run4_trace.txt ../traces/run6_trace.txt \
    | grep -a 'CODE\|barcodes:')
mkdir -p "$src/build-target/bench"
cp build-target/bench/bench_motion_mqtt.uf2 "$src/build-target/bench/bench_motion_mqtt.uf2"
sha256sum build-target/bench/bench_motion_mqtt.uf2 | cut -c1-8
grep -c "\"$broker\"" build-target/bench/bench_motion_mqtt.uf2 || true
fail=0
for f in "$out"/*.rc; do [ "$(cat "$f")" = 0 ] || fail=1; done
echo "OVERALL $([ $fail -eq 0 ] && echo PASS || echo FAIL)"
