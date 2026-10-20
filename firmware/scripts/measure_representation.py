#!/usr/bin/env python3
"""Measure application RAM, ARM type sizes and stack frames from a build.

Reads one build folder written by scripts/dev.sh build <profile>
(build-target/<profile>/): the ELF for section sizes and DWARF type sizes,
the linker map for each application variable's .bss/.data bytes, and the
compiler's per-function stack reports. --compare BEFORE AFTER prints the
Week 13 before/after tables from two saved reports.

This is static build evidence. It does not measure stack high-water marks,
execution time, interrupt latency or behaviour on the robot.
"""
from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

APP_MARKER = "app_program/"
MAP_START = "Linker script and memory map"
ONE_LINE = re.compile(r"^ (\.(?:bss|data|text)\.(\S+))\s+0x([0-9a-f]+)\s+"
                      r"0x([0-9a-f]+)\s+(\S+)$")
NAME_ONLY = re.compile(r"^ (\.(?:bss|data|text)\.(\S+))\s*$")
CONTINUED = re.compile(r"^\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)\s+(\S+)$")
DIE = re.compile(r"^\s*<(\d+)><([0-9a-f]+)>: Abbrev Number: \d+"
                 r"(?: \((DW_TAG_\w+)\))?")
ATTRIBUTE = re.compile(r"^\s*<[0-9a-f]+>\s+(DW_AT_\w+)\s*:\s*(.*)$")
STACK_LINE = re.compile(r"^(?P<file>[^:]+):\d+:\d+:(?P<function>\w+)\t"
                        r"(?P<bytes>\d+)\t(?P<kind>[\w,]+)$")
TAGS = ("DW_TAG_typedef", "DW_TAG_structure_type", "DW_TAG_union_type",
        "DW_TAG_enumeration_type", "DW_TAG_base_type", "DW_TAG_array_type")


def tool(name: str) -> str:
    """Locate a toolchain program on PATH (source inf2004-deps/env.sh)."""
    path = shutil.which(name)
    if path is None:
        raise SystemExit(f"{name} not found; source ~/inf2004-deps/env.sh")
    return path


def section_sizes(elf: Path) -> dict:
    """Allocated section sizes from arm-none-eabi-size -A."""
    output = subprocess.check_output([tool("arm-none-eabi-size"), "-A",
                                      str(elf)], text=True)
    sizes = {}
    for line in output.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0] in (".text", ".data", ".bss"):
            sizes[parts[0]] = int(parts[1])
    return sizes


def map_symbols(map_path: Path) -> list[dict]:
    """Per-symbol .bss/.data/.text input sections of application objects
    (-fdata-sections and -ffunction-sections give one section each)."""
    symbols = []
    pending = None
    started = False
    for line in map_path.read_text(errors="replace").splitlines():
        if not started:
            started = line.startswith(MAP_START)
            continue
        match = ONE_LINE.match(line)
        if match:
            section, name, _address, size, obj = match.groups()
        elif pending is not None and CONTINUED.match(line):
            section, name = pending
            _address, size, obj = CONTINUED.match(line).groups()
        else:
            named = NAME_ONLY.match(line)
            pending = named.groups() if named else None
            continue
        pending = None
        if APP_MARKER in obj and int(size, 16) > 0:
            symbols.append({"symbol": name, "section": section.split(".")[1],
                            "bytes": int(size, 16),
                            "object": obj.rsplit("/", 1)[-1]})
    return sorted(symbols, key=lambda item: (-item["bytes"], item["symbol"]))


def dwarf_types(elf: Path) -> dict:
    """byte_size of every named type declared in an application unit."""
    output = subprocess.run([tool("arm-none-eabi-readelf"),
                             "--debug-dump=info", str(elf)],
                            capture_output=True, text=True, check=True).stdout
    dies: dict[int, dict] = {}
    current = None
    in_app = False
    for line in output.splitlines():
        die = DIE.match(line)
        if die:
            depth, offset, tag = die.groups()
            current = {"depth": int(depth), "tag": tag, "app": in_app}
            dies[int(offset, 16)] = current
            continue
        attribute = ATTRIBUTE.match(line)
        if current is None or attribute is None:
            continue
        key, value = attribute.groups()
        if key == "DW_AT_name":
            value = value.rsplit(": ", 1)[-1].strip()
            if current["tag"] == "DW_TAG_compile_unit":
                in_app = APP_MARKER in value
                current["app"] = in_app
        elif key == "DW_AT_type":
            value = int(value.strip("<> "), 16)
        elif key == "DW_AT_byte_size":
            value = int(value.split()[0], 0)
        current[key] = value
    sizes: dict[str, set] = {}
    for die in dies.values():
        if (not die["app"]) or die["tag"] not in TAGS or die["depth"] != 1:
            continue
        name = die.get("DW_AT_name")
        target = die
        hops = 0
        while "DW_AT_byte_size" not in target and "DW_AT_type" in target:
            target = dies.get(target["DW_AT_type"], {})
            hops += 1
            if hops > 8:
                break
        if name and "DW_AT_byte_size" in target:
            sizes.setdefault(name, set()).add(target["DW_AT_byte_size"])
    return {name: sorted(values)[0] if len(values) == 1 else sorted(values)
            for name, values in sorted(sizes.items())}


def stack_frames(report: Path) -> dict:
    """Static frame bytes of application functions (-fstack-usage)."""
    if not report.is_file():
        return {}
    frames = {}
    data = json.loads(report.read_text())
    for name, text in data.get("reports", {}).items():
        if APP_MARKER not in name:
            continue
        for line in text.splitlines():
            match = STACK_LINE.match(line)
            if match:
                frames[match["function"]] = {"bytes": int(match["bytes"]),
                                             "kind": match["kind"]}
    return dict(sorted(frames.items()))


def measure(folder: Path, label: str) -> dict:
    """Collect every static measurement from one build-target folder."""
    elves = sorted(folder.glob("*.elf"))
    maps = sorted(folder.glob("*.map"))
    if len(elves) != 1 or len(maps) != 1:
        raise SystemExit(f"{folder}: expected one ELF and one map file")
    entries = map_symbols(maps[0])
    symbols = [item for item in entries if item["section"] != "text"]
    functions = {item["symbol"]: item["bytes"] for item in entries
                 if item["section"] == "text"}
    return {
        "label": label, "folder": str(folder), "elf": elves[0].name,
        "sections": section_sizes(elves[0]),
        "application_ram_bytes": sum(item["bytes"] for item in symbols),
        "application_code_bytes": sum(functions.values()),
        "symbols": symbols, "functions": dict(sorted(functions.items())),
        "types": dwarf_types(elves[0]),
        "stack_frames": stack_frames(folder / "stack-usage.json"),
        "scope": "Static ELF/map/DWARF/-fstack-usage evidence; no runtime "
                 "high-water mark, timing or hardware measurement.",
    }


def delta(before, after) -> str:
    """Signed difference, or a dash when either side is absent."""
    if not isinstance(before, int) or not isinstance(after, int):
        return "-"
    return f"{after - before:+d}"


def compare(before: dict, after: dict, types: list[str],
            functions: list[str]) -> str:
    """Markdown before/after tables for sections, variables and types."""
    lines = [f"## {before['label']} vs {after['label']}", "",
             "| Section | Before (B) | After (B) | Change |", "|---|---:|---:|---:|"]
    for name in (".data", ".bss"):
        b_size = before["sections"].get(name)
        a_size = after["sections"].get(name)
        lines.append(f"| {name} | {b_size} | {a_size} | {delta(b_size, a_size)} |")
    b_app = before["application_ram_bytes"]
    a_app = after["application_ram_bytes"]
    lines.append(f"| application variables | {b_app} | {a_app} | "
                 f"{delta(b_app, a_app)} |")
    lines += ["", "| Variable | Object | Before (B) | After (B) | Change |",
              "|---|---|---:|---:|---:|"]
    b_vars = {item["symbol"]: item for item in before["symbols"]}
    a_vars = {item["symbol"]: item for item in after["symbols"]}
    for name in sorted(set(b_vars) | set(a_vars),
                       key=lambda key: -max(b_vars.get(key, {}).get("bytes", 0),
                                            a_vars.get(key, {}).get("bytes", 0))):
        b_size = b_vars.get(name, {}).get("bytes")
        a_size = a_vars.get(name, {}).get("bytes")
        obj = (a_vars.get(name) or b_vars.get(name))["object"]
        if b_size != a_size:
            lines.append(f"| {name} | {obj} | {b_size if b_size is not None else '-'}"
                         f" | {a_size if a_size is not None else '-'} | "
                         f"{delta(b_size, a_size)} |")
    if functions:
        lines += ["", "| Function (ARM code) | Before (B) | After (B) | "
                  "Change | Stack before (B) | Stack after (B) |",
                  "|---|---:|---:|---:|---:|---:|"]
        for name in functions:
            b_size = before.get("functions", {}).get(name)
            a_size = after.get("functions", {}).get(name)
            b_stack = before["stack_frames"].get(name, {}).get("bytes")
            a_stack = after["stack_frames"].get(name, {}).get("bytes")
            lines.append(f"| {name} | {b_size if b_size is not None else '-'}"
                         f" | {a_size if a_size is not None else '-'} | "
                         f"{delta(b_size, a_size)} | "
                         f"{b_stack if b_stack is not None else '-'} | "
                         f"{a_stack if a_stack is not None else '-'} |")
    if types:
        lines += ["", "| Type (ARM sizeof) | Before (B) | After (B) | Change |",
                  "|---|---:|---:|---:|"]
        for name in types:
            b_size = before["types"].get(name)
            a_size = after["types"].get(name)
            lines.append(f"| {name} | {b_size if b_size is not None else '-'} | "
                         f"{a_size if a_size is not None else '-'} | "
                         f"{delta(b_size, a_size)} |")
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, help="build-target/<profile>")
    parser.add_argument("--label", default="build")
    parser.add_argument("--output", type=Path, help="write report JSON")
    parser.add_argument("--compare", nargs=2, type=Path,
                        metavar=("BEFORE", "AFTER"))
    parser.add_argument("--types", nargs="*", default=[],
                        help="type names for the comparison table")
    parser.add_argument("--functions", nargs="*", default=[],
                        help="function names for the code/stack table")
    args = parser.parse_args()
    if args.compare:
        before, after = (json.loads(path.read_text()) for path in args.compare)
        sys.stdout.write(compare(before, after, args.types, args.functions))
        return 0
    if args.build is None:
        parser.error("--build or --compare is required")
    report = measure(args.build.resolve(), args.label)
    text = json.dumps(report, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text)
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
