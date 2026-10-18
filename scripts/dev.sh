#!/usr/bin/env bash
# Developer task runner used by the VS Code tasks and by GitHub Actions.
#
#   bash scripts/dev.sh setup [setup options]     dependencies, toolchain, install
#                                                 (--no-install while .import-stage exists)
#   bash scripts/dev.sh host-tests [pattern]      firmware/run_tests.sh (release + ASan/UBSan)
#   bash scripts/dev.sh style                     selected BARR-C lexical checks
#   bash scripts/dev.sh app-tests [options]       application scenario tests
#   bash scripts/dev.sh integration               board IRQ, image gate, build script,
#                                                 installer memory, preflight, radio,
#                                                 network, import-stage guard
#   bash scripts/dev.sh install                   refresh the generated port checkout from firmware/
#   bash scripts/dev.sh build offline             uncommissioned offline image
#   bash scripts/dev.sh build mqtt                MQTT image with local/ credentials
#                                                 (in plain text in the ELF/UF2/map;
#                                                 share hashes only)
#   bash scripts/dev.sh build mqtt --compile-only MQTT image with labelled placeholder settings
#   bash scripts/dev.sh all                       everything CI runs; while .import-stage
#                                                 exists, app-tests and builds are skipped
#                                                 as in CI
#   bash scripts/dev.sh clean                     remove local build products
#
# Outputs go to $ROBOT_VALIDATION_DIR (default firmware/validation/local, which
# is git-ignored) and build products to build-target/<profile>/ (git-ignored).
# During the staged public import (.import-stage present) app-tests, install
# and builds need modules that are not imported yet, so they are refused.
set -euo pipefail

log() { printf '[dev] %s\n' "$*"; }
die() { printf '[dev] ERROR: %s\n' "$*" >&2; exit 1; }

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
firmware="$repo_root/firmware"
scripts="$firmware/scripts"

case "$(uname -s)" in
    MINGW*|MSYS*|CYGWIN*)
        printf 'Run this inside Ubuntu (WSL):  wsl.exe --cd "%s" -e bash scripts/dev.sh %s\n' "$repo_root" "$*"
        exit 2 ;;
esac

deps_dir="${ROBOT_DEPS_DIR:-$HOME/inf2004-deps}"
if [[ -f "$deps_dir/env.sh" ]]; then
    # shellcheck disable=SC1091
    source "$deps_dir/env.sh"
fi
port="${ROBOT_PORT:-$deps_dir/mtk3smp-rp2040}"
sdk="${ROBOT_SDK:-$deps_dir/pico-sdk}"
out="${ROBOT_VALIDATION_DIR:-$firmware/validation/local}"
export ROBOT_VALIDATION_DIR="$out"
mkdir -p "$out"

need_deps() {
    [[ -d "$port/.git" && -d "$sdk/.git" ]] \
        || die "dependencies missing in $deps_dir; run: bash scripts/setup.sh (dev.sh reads only \$ROBOT_DEPS_DIR, default ~/inf2004-deps: after setup.sh --deps <dir>, export ROBOT_DEPS_DIR=<dir>)"
}
need_install() {
    need_deps
    [[ -f "$port/robot_install_manifest.json" ]] || die "firmware not installed; run: bash scripts/dev.sh install"
}
# Commands that need every module cannot run during the staged import. A
# marker that contradicts the files present is reported as the error CI gives.
refuse_during_import() {
    if [[ -f "$repo_root/.import-stage" ]]; then
        python3 "$scripts/import_stage.py" >/dev/null \
            || die ".import-stage contradicts the files present; CI fails the same way (see errors above)"
        die "$1 needs every module, but the staged import is in progress (.import-stage = $(tr -d '[:space:]' < "$repo_root/.import-stage")). CI skips it too; it runs after the fifth import removes .import-stage."
    fi
}

cmd_host_tests() { (cd "$firmware" && bash run_tests.sh "$@"); }
cmd_style() { python3 "$scripts/check_style.py" --report-dir "$out/style" "$@"; }
cmd_app_tests() {
    refuse_during_import "app-tests"
    python3 "$scripts/test_application.py" "$@"
}

cmd_integration() {
    need_deps
    log "board ISR fixtures with pinned SDK headers"
    python3 "$scripts/test_board_irq.py" --repository "$port" --sdk "$sdk" --output "$out/board_irq"
    log "image structure gate fixtures"
    python3 "$scripts/test_inspect_target.py"
    log "build script failure-path fixtures"
    python3 "$scripts/test_build_target.py"
    log "installer memory guards"
    python3 "$scripts/test_install_memory.py" --repository "$port" --sdk "$sdk" --output "$out/install-memory.json" > "$out/install-memory.log"
    log "preflight source-integrity gate"
    python3 "$scripts/test_verify_install.py" --repository "$port" --sdk "$sdk" --output "$out/verify-install.json" > "$out/verify-install.log"
    log "radio startup with mocked driver/kernel"
    python3 "$scripts/test_radio_startup.py" --repository "$port" --sdk "$sdk" --output "$out/radio-startup"
    log "MQTT adapter with real SDK types and mocked transport"
    python3 "$scripts/test_network_adapter.py" --repository "$port" --sdk "$sdk" --output "$out/network"
    log "staged-import guard fixtures"
    (cd "$scripts" && python3 test_import_stage.py) > "$out/import-stage.log" 2>&1 \
        || { cat "$out/import-stage.log" >&2; die "import-stage guard fixtures failed"; }
    log "integration host checks passed; reports in $out"
}

cmd_install() {
    refuse_during_import "install"
    need_deps
    if ! python3 "$scripts/install_project.py" --repo "$port" --sdk "$sdk" > "$out/install.log" 2>&1; then
        cat "$out/install.log" >&2
        die "installer refused; see message above (after hand edits to the generated checkout use: bash scripts/setup.sh --reset)"
    fi
    log "generated checkout refreshed from firmware/ (manifest: $port/robot_install_manifest.json)"
}

# Write MQTT profile headers into the generated checkout. Real settings come
# from the git-ignored local/ folder; --compile-only writes labelled
# placeholders that cannot join any network.
prepare_mqtt_config() {
    local mode=$1
    if [[ "$mode" == compile-only ]]; then
        python3 - "$port" "$firmware" <<'PY'
from pathlib import Path
import re, sys
repo, firmware = Path(sys.argv[1]), Path(sys.argv[2])
wifi = (repo / 'config/wifi_credentials.example.h').read_text()
wifi, count = re.subn(r'(#define\s+UTK_WIFI_CREDENTIALS_SET\s+)0', r'\g<1>1', wifi)
assert count == 1, 'unexpected wifi_credentials.example.h shape'
assert 'replace-with-ssid' in wifi and 'replace-with-password' in wifi
wifi = wifi.replace('replace-with-ssid', 'COMPILE-ONLY-NO-ACCESS-POINT')
wifi = wifi.replace('replace-with-password', 'COMPILE-ONLY-NO-CREDENTIAL')
(repo / 'config/wifi_credentials.h').write_text(
    '/* COMPILE-ONLY PLACEHOLDER: generated by scripts/dev.sh; not a real network. */\n' + wifi)
mqtt = (firmware / 'target/net_mqtt_config.example.h').read_text()
old = '#define NET_MQTT_CONFIGURED 0'
assert mqtt.count(old) == 1
(repo / 'config/net_mqtt_config.h').write_text(
    '/* COMPILE-ONLY PLACEHOLDER: broker 192.0.2.1 is a documentation address. */\n'
    + mqtt.replace(old, '#define NET_MQTT_CONFIGURED 1'))
PY
        log "MQTT profile uses COMPILE-ONLY placeholder Wi-Fi/broker settings"
    else
        local local_dir="$repo_root/local"
        for name in wifi_credentials.h net_mqtt_config.h; do
            [[ -f "$local_dir/$name" ]] || die "missing $local_dir/$name; copy the matching .example.h, fill in real values (never commit them), or use --compile-only"
        done
        cp -- "$local_dir/wifi_credentials.h" "$port/config/wifi_credentials.h"
        cp -- "$local_dir/net_mqtt_config.h" "$port/config/net_mqtt_config.h"
        log "MQTT profile uses local/ settings (not shown, not committed)"
        log "WARNING: the resulting ELF/UF2/map contain the Wi-Fi SSID/password and broker settings in plain text; never share or commit them, share hashes only"
    fi
}

cmd_build() {
    local profile=${1:-} mode=real
    shift || true
    case "$profile" in offline|mqtt) ;; *) die "usage: build offline|mqtt [--compile-only]" ;; esac
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --compile-only) mode=compile-only; shift ;;
            *) die "unknown build option: $1" ;;
        esac
    done
    refuse_during_import "build $profile"
    need_deps
    # Always refresh the generated checkout so the preflight gate sees
    # the current firmware/ sources (the installer is idempotent).
    cmd_install
    command -v arm-none-eabi-gcc >/dev/null 2>&1 || die "arm-none-eabi-gcc not found; run: bash scripts/setup.sh"
    if [[ "$profile" == mqtt ]]; then
        prepare_mqtt_config "$mode"
    fi
    local evidence="$port/robot_build_evidence/$profile"
    local dest="$repo_root/build-target/$profile"
    local status=0
    bash "$scripts/build_target.sh" "$port" "$sdk" "$profile" || status=$?
    rm -rf -- "$dest"
    mkdir -p "$dest"
    cp -r -- "$evidence/." "$dest/"
    {
        printf 'Profile: %s\n' "$profile"
        printf 'Source revision: %s\n' "$(git -C "$repo_root" rev-parse HEAD 2>/dev/null || echo unknown)"
        if [[ -z "$(git -C "$repo_root" status --porcelain 2>/dev/null)" ]]; then
            printf 'Working tree: clean\n'
        else
            printf 'Working tree: has uncommitted changes\n'
        fi
        python3 - "$dest/toolchain.json" <<'PY'
import json, sys
try:
    data = json.load(open(sys.argv[1], encoding='utf-8'))
except (OSError, ValueError):
    print('Toolchain: not recorded (the build stopped before the toolchain check)')
    raise SystemExit(0)
version = data.get('arm-none-eabi-gcc', {}).get('version', 'unknown')
if data.get('toolchain_matches_pin'):
    print('Toolchain: %s (the pinned toolchain CI uses)' % version)
else:
    print('Toolchain: WARNING %s is not the pinned %s; image hashes are not '
          'comparable with CI' % (version, data.get('pinned_arm_toolchain', 'toolchain')))
PY
        printf 'Commissioning: UNCOMMISSIONED. All ROBOT_*_CONFIRMED flags are 0; the image refuses to start motors.\n'
        if [[ "$profile" == mqtt && "$mode" == compile-only ]]; then
            printf 'MQTT settings: COMPILE-ONLY placeholders (no real SSID, credential or broker). Cannot connect.\n'
        elif [[ "$profile" == mqtt ]]; then
            printf 'MQTT settings: REAL settings from local/. The ELF, UF2 and map in this folder contain the Wi-Fi SSID/password and broker settings in plain text: never share, upload or commit them; share their SHA-256 hashes only.\n'
        fi
    } > "$dest/BUILD_LABEL.txt"
    if [[ $status -ne 0 ]]; then
        die "build failed (exit $status); see $dest/build.log and $dest/build-status.json"
    fi
    log "build passed; ELF/UF2/map and evidence in $dest"
    if [[ "$profile" == mqtt && "$mode" == real ]]; then
        log "WARNING: these images hold the local/ network settings in plain text; share hashes only (see BUILD_LABEL.txt)"
    fi
}

cmd_all() {
    local stage
    # Same guard as the CI prepare job: the marker must match the files.
    stage=$(python3 "$scripts/import_stage.py") \
        || die ".import-stage does not match the files present (CI fails the same way); see the errors above"
    cmd_host_tests
    cmd_style
    if [[ "$stage" != complete ]]; then
        log "import stage $stage (.import-stage present): app-tests and target builds are skipped, as in CI"
        cmd_integration
        log "checks for import stage $stage completed; app-tests and target builds were NOT run"
        return 0
    fi
    cmd_app_tests
    cmd_integration
    cmd_build offline
    cmd_build mqtt --compile-only
    log "all checks and builds completed"
}

cmd_clean() {
    rm -rf -- "$firmware/build-host" "$firmware/build-apphost" "$repo_root/build-target" "$firmware/validation/local"
    log "local build products removed"
}

cmd=${1:-help}
shift || true
case "$cmd" in
    setup) exec bash "$repo_root/scripts/setup.sh" "$@" ;;
    host-tests) cmd_host_tests "$@" ;;
    style) cmd_style "$@" ;;
    app-tests) cmd_app_tests "$@" ;;
    integration) cmd_integration ;;
    install) cmd_install ;;
    build) cmd_build "$@" ;;
    all) cmd_all ;;
    clean) cmd_clean ;;
    help|-h|--help) sed -n '2,/^set -euo pipefail/p' "${BASH_SOURCE[0]}" | sed -n 's/^#//p' ;;
    *) die "unknown command: $cmd (try: bash scripts/dev.sh help)" ;;
esac
