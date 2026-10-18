#!/usr/bin/env python3
"""Install this application into a clean, pinned professor checkout.

No fetching, flashing, credential creation or compiler installation occurs.
The --dry-run option checks inputs and reports the planned installation.
An installation manifest records every generated/patched file's hash.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import re
import shutil
import subprocess
from pathlib import Path

PORT_COMMIT = "2e8e6ad480c2d84a369b12995694b9bf5e9ddfb3"
SDK_COMMIT = "a1438dff1d38bd9c65dbd693f0e5db4b9ae91779"
MANIFEST = "robot_install_manifest.json"
INCLUDE_MARKER = "include robot_project.mk"
OLD_PIN_NOTE = "/* INF2004: GP16 belongs to ultrasonic trigger. */"
PIN_NOTE = "/* INF2004: GP16 is the right wheel encoder A input; no LED. */"
INTEGRATION_REVISION = "2026-09-11"
MINIMUM_HEAP_BYTES = 65536
TEMPORARY_STACK_BYTES = 2048
LINKER_PATH = "etc/linker/pico_rp2040/tkernel_map.ld"
MEMORY_GUARD = '''
/* INF2004 memory guard v1: budget only; measure actual stack high water. */
__robot_heap_start = ALIGN(__bss_end, 8);
__robot_heap_limit = ORIGIN(RAM) + LENGTH(RAM) - 2048;
ASSERT(__robot_heap_start + 65536 <= __robot_heap_limit,
       "INF2004: reserve 64 KiB heap below the 2048-byte reset stack")
'''

SDK_INCLUDES = [
    "common/pico_base_headers", "common/pico_binary_info",
    "common/pico_time", "common/hardware_claim",
    "rp2040/pico_platform", "rp2040/hardware_regs",
    "rp2040/hardware_structs", "rp2_common/pico_platform_common",
    "rp2_common/pico_platform_compiler",
    "rp2_common/pico_platform_sections",
    "rp2_common/pico_platform_panic", "rp2_common/hardware_base",
    "rp2_common/hardware_gpio", "rp2_common/hardware_adc",
    "rp2_common/hardware_i2c", "rp2_common/hardware_pwm",
    "rp2_common/hardware_irq", "rp2_common/hardware_resets",
    "rp2_common/hardware_clocks", "rp2_common/hardware_timer",
    "rp2_common/hardware_sync", "rp2_common/hardware_sync_spin_lock",
]


def git(repo: Path, *args: str) -> str:
    """Run read-only git commands with argument boundaries preserved."""
    return subprocess.check_output(
        ["git", "-C", str(repo), *args], text=True
    ).strip()


def digest(path: Path) -> str:
    """Return the SHA-256 of a regular file."""
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"Expected a regular, non-symlink file: {path}")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def replace_once(text: str, pattern: str, replacement: str) -> str:
    """Fail on unexpected source shapes instead of guessing an edit."""
    updated, count = re.subn(pattern, replacement, text, count=0)
    if count != 1:
        raise ValueError(f"Expected exactly one source anchor: {pattern}")
    return updated


def memory_config(config: str) -> str:
    """Preserve the pinned SMP branch; enlarge only single-core stacks."""
    for name, expected in (("CNF_EXC_STACK_SIZE", "2048"),
                           ("CNF_SYSTEMAREA_TOP", "0"),
                           ("CNF_SYSTEMAREA_END", "0")):
        values = re.findall(rf"^#define[ \t]+{name}[ \t]+"
                            r"\(?([0-9]+)\)?[ \t]*(?:/\*|$)",
                            config, flags=re.MULTILINE)
        if values != [expected]:
            raise ValueError(f"Memory guard requires {name}={expected}")
    branch = (r"(#if CNF_SMP\n"
              r"#define[ \t]+CNF_TMP_STACK_SIZE[ \t]+\(1024\)"
              r"[ \t]+/\* Temporary stack size \*/\n#else\n"
              r"#define[ \t]+CNF_TMP_STACK_SIZE[ \t]+)"
              r"\((?:256|2048)\)"
              r"([ \t]+/\* Temporary stack size \*/\n#endif)")
    if len(re.findall(r"^#define[ \t]+CNF_TMP_STACK_SIZE\b", config,
                      flags=re.MULTILINE)) != 2:
        raise ValueError("Expected exactly two temporary-stack branches")
    return replace_once(config, branch, r"\g<1>(2048)\g<2>")


def memory_linker(current: str, baseline: str) -> str:
    """Accept only the pinned linker, or our exact idempotent extension."""
    baseline = baseline.rstrip() + "\n"
    guarded = baseline + MEMORY_GUARD
    if current.strip() not in (baseline.strip(), guarded.strip()):
        raise ValueError("Unexpected linker layout; memory guard needs review")
    if not re.search(r"RAM \(xrw\)\s*: ORIGIN = 0x20000000, LENGTH = 256K",
                     baseline):
        raise ValueError("Expected pinned 256 KiB RAM layout")
    if baseline.count("__bss_end = .;") != 1:
        raise ValueError("Expected one end-of-BSS symbol for the heap guard")
    return guarded


def project_make() -> str:
    """Generate a peripheral-only SDK import; the BSP retains startup."""
    includes = " \\\n    ".join(f'-I"$(PICO_SDK_PATH)/src/{part}/include"'
                  for part in SDK_INCLUDES)
    return f'''# Generated INF2004 application integration. SDK 2.2.0 only.
ifneq ($(SMP),0)
$(error This application uses core-0 IRQ masking and requires SMP=0)
endif
ifneq ($(CONSOLE),uart)
$(error This reviewed profile reserves UART0 GP0/GP1; use CONSOLE=uart)
endif
ifeq ($(wildcard $(PICO_SDK_PATH)/src/rp2_common/hardware_adc/adc.c),)
$(error Set PICO_SDK_PATH to the pinned Pico SDK 2.2.0 checkout)
endif

# Override the BSP's stale 2.1.0 version header with the actual SDK version.
CFLAGS := -I"./robot_sdk_headers" $(CFLAGS)
CFLAGS += -I"../app_program" \\
    -I"../lib/libtm/sysdepend/pico_rp2040/usb" \\
    -DPICO_RP2040=1 -DPICO_32BIT=1 -DPICO_ON_DEVICE=1 \\
    -DPICO_NO_HARDWARE=0 -DNDEBUG -ffunction-sections -fdata-sections \\
    -fstack-usage \\
    {includes}
LFLAGS += -Wl,--gc-sections
LDLIBS += -lm

ROBOT_SDK_OBJS := ./mtkernel_3/robot_sdk/adc.o \\
    ./mtkernel_3/robot_sdk/i2c.o \\
    ./mtkernel_3/robot_sdk/timeout_helper.o
ifneq ($(WIFI),cyw43)
ROBOT_SDK_OBJS += ./mtkernel_3/robot_sdk/gpio.o
endif
OBJS += $(ROBOT_SDK_OBJS)
C_DEPS += $(ROBOT_SDK_OBJS:.o=.d)

mtkernel_3/robot_sdk/adc.o: $(PICO_SDK_PATH)/src/rp2_common/hardware_adc/adc.c
\t@mkdir -p "$(@D)"
\t$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -c "$<" -o "$@"

mtkernel_3/robot_sdk/i2c.o: $(PICO_SDK_PATH)/src/rp2_common/hardware_i2c/i2c.c
\t@mkdir -p "$(@D)"
\t$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -c "$<" -o "$@"

mtkernel_3/robot_sdk/gpio.o: $(PICO_SDK_PATH)/src/rp2_common/hardware_gpio/gpio.c
\t@mkdir -p "$(@D)"
\t$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -c "$<" -o "$@"

mtkernel_3/robot_sdk/timeout_helper.o: $(PICO_SDK_PATH)/src/common/pico_time/timeout_helper.c
\t@mkdir -p "$(@D)"
\t$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -c "$<" -o "$@"
'''


def validate_clean(repo: Path) -> None:
    """Require a clean tracked baseline or an intact earlier installation."""
    path = repo / MANIFEST
    if path.exists():
        previous = json.loads(path.read_text())
        for name, expected in previous["installed_sha256"].items():
            if digest(repo / name) != expected:
                raise ValueError(
                    f"Local edits in {name}; preserve them before reinstalling"
                )
        changed = set(git(repo, "diff", "--name-only", "HEAD").splitlines())
        unrelated = changed - set(previous["installed_sha256"])
        if unrelated:
            raise ValueError(f"Unrelated checkout edits exist: {unrelated}")
        return
    if git(repo, "diff", "--name-only", "HEAD"):
        raise ValueError("Tracked checkout changes exist; use a clean clone")


def install(repo: Path, sdk: Path, dry_run: bool) -> dict:
    """Validate pins, copy project sources, apply reviewed integration."""
    firmware = Path(__file__).resolve().parents[1]
    repo = repo.resolve()
    sdk = sdk.resolve()
    if any(character.isspace() for character in str(repo) + str(sdk)):
        raise ValueError("Use checkout paths without whitespace for GNU Make")
    if git(repo, "rev-parse", "HEAD") != PORT_COMMIT:
        raise ValueError(f"Professor checkout must be at {PORT_COMMIT}")
    if git(sdk, "rev-parse", "HEAD") != SDK_COMMIT:
        raise ValueError(f"Pico SDK must be 2.2.0 at {SDK_COMMIT}")
    if git(sdk, "diff", "--name-only", "HEAD"):
        raise ValueError("Pico SDK has local tracked changes; use pinned sources")
    validate_clean(repo)
    sources = sorted((firmware / "src").glob("*.c"))
    sources += sorted((firmware / "include").glob("*.h"))
    sources += sorted((firmware / "target").glob("*.c"))
    sources += sorted((firmware / "target").glob("*.h"))
    names = [path.name for path in sources]
    if len(names) != len(set(names)):
        raise ValueError("Flattening would overwrite duplicate source names")
    previous_path = repo / MANIFEST
    previous = json.loads(previous_path.read_text()) if previous_path.exists() else {}
    removed = set(previous.get("app_files", [])) - set(names)
    if removed:
        raise ValueError(f"Source set changed: use a fresh clone to remove {removed}")
    required = {"board.c", "board.h", "app_main.c", "app_main.h",
                "robot_app.c", "robot_config.h", "net_mqtt.c"}
    if not required.issubset(set(names)):
        raise ValueError(f"Incomplete application: missing {required - set(names)}")
    for path in sources:
        digest(path)
        relative = "app_program/" + path.name
        destination = repo / relative
        if (destination.exists() and
            relative not in previous.get("installed_sha256", {}) and
            not git(repo, "ls-files", "--", relative)):
            raise ValueError(f"Untracked destination already exists: {relative}")
    allowed_c = set(names) | {"demo_tasks.c"}
    unknown_c = {path.name for path in (repo / "app_program").glob("*.c")}
    unknown_c -= allowed_c
    if unknown_c:
        raise ValueError(f"Unexpected application sources: {unknown_c}")
    for part in SDK_INCLUDES:
        if not (sdk / "src" / part / "include").is_dir():
            raise ValueError(f"SDK include directory missing: {part}")
    config_path = repo / "config/config.h"
    config = config_path.read_text(encoding="utf-8-sig")
    if not re.search(r"#define\s+USE_PTMR\s+\(0\)", config):
        raise ValueError("USE_PTMR must be 0; application owns PWM slices")
    config = replace_once(config,
                          r"(#define\s+CNF_TIMER_PERIOD\s+)"
                          r"(?:\([0-9]+\)|[0-9]+)",
                          r"\g<1>(1)")
    # The sample device layer otherwise also initialises ADC and I2C.
    config = replace_once(config,
                          r"(#define\s+USE_SDEV_DRV\s+)\([01]\)",
                          r"\g<1>(0)")
    config = memory_config(config)
    linker = memory_linker(
        (repo / LINKER_PATH).read_text(),
        git(repo, "show", f"{PORT_COMMIT}:{LINKER_PATH}")
    )
    init_path = repo / "include/sys/inittask.h"
    init_header = replace_once(
        init_path.read_text(),
        r"(#define\s+INITTASK_STKSZ\s+)\([14]\*1024\)",
        r"\g<1>(4*1024)"
    )
    radio_path = repo / "lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_kernel.c"
    radio_kernel = replace_once(
        radio_path.read_text(), r"(#define\s+CYW43_TASK_PRIORITY\s+)[49]",
        r"\g<1>9"
    )
    if "INF2004 radio follows" not in radio_kernel:
        radio_kernel = replace_once(
            radio_kernel, r"(#define\s+CYW43_TASK_PRIORITY\s+9)",
            r"/* INF2004 radio follows app priorities 1..8; measure WCET. */\n"
            r"\g<1>"
        )
    clock_header = repo / "include/sys/sysdepend/pico_rp2040/sysdef.h"
    if not re.search(r"#define\s+TMCLK_MHz\s+\(125\)",
                     clock_header.read_text()):
        raise ValueError("BSP clock changed; PWM/I2C timing requires review")
    make_path = repo / "build_make/makefile"
    make = make_path.read_text()
    if "$(OBJS) $(LDLIBS)" not in make:
        make = replace_once(make,
                            r'( -o "\$\(EXE_FILE\)\.elf" \$\(OBJS\))',
                            r"\g<1> $(LDLIBS)")
    pico_path = repo / "build_make/pico_rp2040.mk"
    pico_make = pico_path.read_text()
    if INCLUDE_MARKER not in pico_make:
        pico_make += f"\n# INF2004 peripheral-only SDK adapter\n{INCLUDE_MARKER}\n"
    app_make_path = repo / "build_make/mtkernel_3/app_program/subdir.mk"
    app_make = app_make_path.read_text()
    if "filter-out ../app_program/demo_tasks.c" not in app_make:
        app_make = replace_once(
            app_make,
            r"TEMP_SRCS = \$\(wildcard ../app_program/\*\.c\)",
            "TEMP_SRCS = $(filter-out ../app_program/demo_tasks.c,"
            "$(wildcard ../app_program/*.c))"
        )
    startup_path = repo / "kernel/sysdepend/pico_rp2040/hw_setting.c"
    startup = startup_path.read_text()
    if OLD_PIN_NOTE in startup:
        # A checkout patched by the earlier pin map keeps the LED removed.
        startup = startup.replace(OLD_PIN_NOTE, PIN_NOTE)
    elif PIN_NOTE not in startup:
        startup = replace_once(
            startup,
            r"\s*/\* External LED \*/\n"
            r"\s*\{GPIO_CTRL\(BOARD_LED_PIN\),[^\n]+\n"
            r"\s*\{GPIO_OE,[^\n]+\n",
            "\n\t" + PIN_NOTE + "\n"
        )
    if "board_boot_timebase();" not in startup:
        startup = replace_once(
            startup, r'(#include "sysdepend.h"\n)',
            r"\g<1>\n/* Fixed-width peripheral adapter; no SDK headers here. */\n"
            r"IMPORT void board_boot_timebase(void);\n"
        )
        startup = replace_once(
            startup,
            r"(startup_clock\(CLKATR_USB \| CLKATR_ADC \| CLKATR_RTC \| "
            r"CLKATR_PREI\);)",
            r"\g<1>\n\t/* Explicit 1 us timer before radio or tasks start. */\n"
            r"\tboard_boot_timebase();"
        )
    hook_path = firmware / "scripts/install_network.py"
    if not hook_path.is_file():
        raise ValueError("Required network integration script is missing")
    report = {"professor_commit": PORT_COMMIT, "pico_sdk_commit": SDK_COMMIT,
              "sdk_version": "2.2.0", "profile": "SMP=0 CONSOLE=uart",
              "integration_revision": INTEGRATION_REVISION,
              "minimum_heap_bytes": MINIMUM_HEAP_BYTES,
              "temporary_stack_bytes": TEMPORARY_STACK_BYTES,
              "kernel_tick_ms": 1, "app_files": names, "dry_run": dry_run}
    if dry_run:
        return report
    for folder in (repo / "app_program", repo / "build_make"):
        if folder.is_symlink():
            raise ValueError(f"Refusing symlink output directory: {folder}")
    written = set()
    for path in sources:
        target = repo / "app_program" / path.name
        if target.is_symlink():
            raise ValueError(f"Refusing symlink output file: {target}")
        shutil.copy2(path, target)
        written.add(str(target.relative_to(repo)))
    content_map = {
        "config/config.h": config,
        LINKER_PATH: linker,
        "include/sys/inittask.h": init_header,
        "lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_kernel.c": radio_kernel,
        "build_make/makefile": make,
        "build_make/pico_rp2040.mk": pico_make,
        "build_make/mtkernel_3/app_program/subdir.mk": app_make,
        "kernel/sysdepend/pico_rp2040/hw_setting.c": startup,
        "build_make/robot_project.mk": project_make(),
        "build_make/robot_sdk_headers/pico/version.h":
        '#ifndef PICO_VERSION_H\n#define PICO_VERSION_H\n'
        '#define PICO_SDK_VERSION_MAJOR 2\n#define PICO_SDK_VERSION_MINOR 2\n'
        '#define PICO_SDK_VERSION_REVISION 0\n'
        '#define PICO_SDK_VERSION_STRING "2.2.0"\n#endif\n',
    }
    for name, content in content_map.items():
        target = repo / name
        if target.is_symlink() or any(
            parent.is_symlink() for parent in target.parents if parent != repo
        ):
            raise ValueError(f"Refusing symlink output path: {target}")
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(content)
        written.add(name)
    specification = importlib.util.spec_from_file_location(
        "install_network", hook_path
    )
    if specification is None or specification.loader is None:
        raise ValueError("Cannot load the network installer")
    network = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(network)
    written.update(network.install_network(repo))
    written.update({"build_make/pico_rp2040.mk",
                    "kernel/sysdepend/pico_rp2040/devinit.c",
                    "lib/libnet/lwip/include/lwipopts.h",
                    "lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c",
                    "lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_kernel.c"})
    report["installed_sha256"] = {
        name: digest(repo / name) for name in sorted(written)
    }
    if (repo / MANIFEST).is_symlink():
        raise ValueError("Refusing symlink installation manifest")
    (repo / MANIFEST).write_text(json.dumps(report, indent=2) + "\n")
    return report


def main() -> int:
    """Run the CLI and distinguish install planning from actual building."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--sdk", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    try:
        report = install(args.repo, args.sdk, args.dry_run)
    except (ValueError, OSError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"Installation stopped: {error}\n")
    print(json.dumps(report, indent=2))
    print("No target compilation or hardware validation is claimed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
