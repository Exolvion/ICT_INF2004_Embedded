#!/usr/bin/env python3
"""Run actual board ISR/acquisition with SDK types and host MMIO/I2C doubles.

This regression exercises encoder IRQ and accelerometer sample status. It is
not an ARM build, hardware test, IRQ latency test or GPIO electrical model.

Logs go to --output, else $ROBOT_VALIDATION_DIR/board_irq, else the
git-ignored firmware/validation/local/board_irq; never into dated evidence.
"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import re
import subprocess
import tempfile


SDK_INCLUDES = (
    "common/pico_base_headers", "common/pico_binary_info",
    "common/pico_time", "common/hardware_claim", "rp2040/pico_platform",
    "rp2040/hardware_regs", "rp2040/hardware_structs",
    "rp2_common/pico_platform_common", "rp2_common/pico_platform_compiler",
    "rp2_common/pico_platform_sections", "rp2_common/pico_platform_panic",
    "rp2_common/hardware_base", "rp2_common/hardware_gpio",
    "rp2_common/hardware_adc", "rp2_common/hardware_i2c",
    "rp2_common/hardware_pwm", "rp2_common/hardware_irq",
    "rp2_common/hardware_resets", "rp2_common/hardware_clocks",
    "rp2_common/hardware_timer", "rp2_common/hardware_sync",
    "rp2_common/hardware_sync_spin_lock",
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository", type=Path, required=True)
    parser.add_argument("--sdk", type=Path, required=True)
    parser.add_argument("--output", type=Path,
                        help="log folder (default: $ROBOT_VALIDATION_DIR/"
                             "board_irq, else firmware/validation/local/"
                             "board_irq)")
    parser.add_argument("--cc", default="gcc")
    args = parser.parse_args()
    firmware = Path(__file__).resolve().parents[1]
    sdk = args.sdk.resolve()
    repository = args.repository.resolve()
    if args.output is None:
        args.output = Path(os.environ.get(
            "ROBOT_VALIDATION_DIR", firmware / "validation" / "local")) / "board_irq"
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    version = (sdk / "pico_sdk_version.cmake").read_text()
    values = {}
    for key in ("MAJOR", "MINOR", "REVISION"):
        match = re.search(r"set\(PICO_SDK_VERSION_" + key + r" (\d+)\)",
                          version)
        if match is None:
            raise ValueError("Missing SDK version component " + key)
        values["PICO_SDK_VERSION_" + key] = match[1]
    values["PICO_SDK_VERSION_STRING"] = ".".join(values.values())
    if values["PICO_SDK_VERSION_STRING"] != "2.2.0":
        raise ValueError("This regression requires Pico SDK 2.2.0")
    # LeakSanitizer is off by project choice until it has been tried on the
    # CI runner; ROBOT_LSAN=1 enables it for a local run.
    leak_checks = int(os.environ.get("ROBOT_LSAN") == "1")
    report = {"scope": "host MMIO/I2C, actual board.c ISR/acquisition, SDK headers",
              "hardware_test": False, "arm_build": False, "checks": [],
              "leak_sanitizer": ("enabled (ROBOT_LSAN=1)" if leak_checks else
                                 "disabled by project choice pending a CI "
                                 "trial; ROBOT_LSAN=1 enables it"),
              "source_sha256": {}}
    for name in ("target/board.c", "target/board.h", "target/robot_config.h",
                 "tests/boardirq_test_main.c", "tests/boardirq_test_main.h",
                 "scripts/test_board_irq.py"):
        report["source_sha256"][name] = hashlib.sha256(
            (firmware / name).read_bytes()).hexdigest()
    failed = False
    with tempfile.TemporaryDirectory(prefix="inf2004_board_irq_") as folder:
        temporary = Path(folder)
        generated = temporary / "pico"
        generated.mkdir()
        template = (sdk / "src/common/pico_base_headers/include/pico/"
                    "version.h.in").read_text()
        for key, value in values.items():
            template = template.replace("${" + key + "}", value)
        (generated / "version.h").write_text(template)
        command = [args.cc, "-std=gnu11", "-Wall", "-Wextra", "-Werror",
                   "-D__unused=__attribute__((__unused__))", "-DNDEBUG",
                   "-DPICO_RP2040=1", "-DPICO_32BIT=1", "-DPICO_ON_DEVICE=1",
                   "-DPICO_NO_HARDWARE=0", "-DTM_WIFI_CYW43=0",
                   "-ffunction-sections", "-fdata-sections",
                   "-I" + str(firmware / "include"),
                   "-I" + str(firmware / "target"), "-I" + str(temporary),
                   "-I" + str(repository /
                               "lib/libtm/sysdepend/pico_rp2040/usb")]
        for relative in SDK_INCLUDES:
            command += ["-isystem", str(sdk / "src" / relative / "include")]
        command += [str(firmware / "tests/boardirq_test_main.c"),
                    "-Wl,--gc-sections", "-lm"]
        for variant, channels, confirmed in (("quadrature", 2, 0),
                                             ("single_unconfirmed", 1, 0),
                                             ("single_confirmed", 1, 1)):
            # Copy only into a temporary fixture; shipped hardware gates stay 0.
            fixture = temporary / variant
            fixture.mkdir()
            (fixture / "board.c").write_bytes(
                (firmware / "target/board.c").read_bytes())
            config = (firmware / "target/robot_config.h").read_text()
            config = config.replace("#define ROBOT_ENCODER_CHANNELS 2",
                                    "#define ROBOT_ENCODER_CHANNELS " +
                                    str(channels))
            config = config.replace(
                "#define ROBOT_SINGLE_CHANNEL_DIRECTION_CONFIRMED 0",
                "#define ROBOT_SINGLE_CHANNEL_DIRECTION_CONFIRMED " +
                str(confirmed))
            (fixture / "robot_config.h").write_text(config)
            for mode, flags in (("release", ["-O2"]),
                                ("sanitized", ["-O1", "-g",
                                 "-fsanitize=address,undefined",
                                 "-fno-sanitize-recover=all",
                                 "-fno-omit-frame-pointer"])):
                label = variant + "_" + mode
                binary = temporary / label
                build = command[:1] + ["-I" + str(fixture)] + command[1:] + flags + ["-o", str(binary)]
                result = subprocess.run(build, text=True, capture_output=True)
                (output / (label + "_build.log")).write_text(
                    result.stdout + result.stderr)
                record = {"variant": variant, "mode": mode, "command": build,
                          "build_exit": result.returncode}
                if result.returncode == 0:
                    environment = os.environ.copy()
                    environment["ASAN_OPTIONS"] = (f"detect_leaks={leak_checks}"
                                                   ":halt_on_error=1")
                    environment["UBSAN_OPTIONS"] = "halt_on_error=1:print_stacktrace=1"
                    run = subprocess.run([str(binary)], text=True,
                                         capture_output=True, env=environment)
                    (output / (label + "_test.log")).write_text(
                        run.stdout + run.stderr)
                    record.update({"test_exit": run.returncode,
                                   "stdout": run.stdout, "stderr": run.stderr})
                    failed = failed or run.returncode != 0
                    print(label + ": " + run.stdout.strip())
                else:
                    failed = True
                    print(label + ": compile failed; see " + str(output))
                report["checks"].append(record)
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
