#!/usr/bin/env python3
"""Check installer memory guards without compiling or executing ARM code.

Uses new temporary local clones only. GNU ld receives host assembly objects
to test the real generated linker script at its memory boundary. An optional
intact legacy installation is copied into a temporary clone before upgrade.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import re
import subprocess
import tempfile
from pathlib import Path


def run(arguments: list[str], cwd: Path | None = None) -> subprocess.CompletedProcess:
    """Capture each command; failures are inspected by the caller."""
    return subprocess.run(arguments, cwd=cwd, text=True, capture_output=True,
                          check=False)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository", type=Path, required=True)
    parser.add_argument("--sdk", type=Path, required=True)
    parser.add_argument("--legacy-repository", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    module_path = Path(__file__).with_name("install_project.py")
    spec = importlib.util.spec_from_file_location("install_project", module_path)
    assert spec is not None and spec.loader is not None
    installer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(installer)
    checks: list[str] = []
    link_results: list[dict] = []

    def check(condition: bool, description: str) -> None:
        if not condition:
            raise AssertionError(description)
        checks.append(description)

    def rejects(function, description: str) -> None:
        try:
            function()
        except ValueError:
            checks.append(description)
        else:
            raise AssertionError(description)

    def clone(destination: Path) -> None:
        result = run(["git", "clone", "--quiet", "--no-hardlinks",
                      str(args.repository.resolve()), str(destination)])
        if result.returncode != 0:
            raise RuntimeError(result.stderr)

    def verify_manifest(repo: Path, report: dict) -> None:
        check(report["integration_revision"] == "2026-09-11",
              "Manifest records integration revision")
        check(report["minimum_heap_bytes"] == 65536 and
              report["temporary_stack_bytes"] == 2048,
              "Manifest records heap and stack budgets")
        expected = {installer.LINKER_PATH,
                    "kernel/sysdepend/pico_rp2040/devinit.c",
                    "lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_kernel.c"}
        check(expected <= set(report["installed_sha256"]),
              "Manifest includes linker and radio startup patches")
        check(all(installer.digest(repo / name) == digest
                  for name, digest in report["installed_sha256"].items()),
              "All installed manifest hashes match")

    with tempfile.TemporaryDirectory(prefix="inf2004-memory-") as folder:
        workspace = Path(folder)
        repo = workspace / "fresh"
        clone(repo)
        baseline_config = (repo / "config/config.h").read_text()
        baseline_linker = (repo / installer.LINKER_PATH).read_text()
        patched_config = installer.memory_config(baseline_config)
        check(installer.memory_config(patched_config) == patched_config,
              "Stack configuration patch is idempotent")
        for name, old, new in (("CNF_EXC_STACK_SIZE", "2048", "1024"),
                               ("CNF_SYSTEMAREA_TOP", "0", "1"),
                               ("CNF_SYSTEMAREA_END", "0", "1")):
            invalid = re.sub(rf"(#define[ \t]+{name}[ \t]+\(?){old}",
                             rf"\g<1>{new}", baseline_config)
            rejects(lambda value=invalid: installer.memory_config(value),
                    f"Changed {name} is rejected")
        invalid = baseline_config.replace("(256)", "(512)")
        rejects(lambda: installer.memory_config(invalid),
                "Unexpected single-core stack size is rejected")
        invalid = baseline_config.replace("#if CNF_SMP", "#if OTHER_PROFILE")
        rejects(lambda: installer.memory_config(invalid),
                "Unexpected stack branch is rejected")
        guarded = installer.memory_linker(baseline_linker, baseline_linker)
        check(installer.memory_linker(guarded, baseline_linker) == guarded,
              "Linker guard is idempotent")
        rejects(lambda: installer.memory_linker(
            baseline_linker.replace("256K", "128K"), baseline_linker),
            "Unexpected linker layout is rejected")
        rejects(lambda: installer.memory_linker(
            guarded.replace("+ 65536", "+ 32768"), baseline_linker),
            "Altered existing heap guard is rejected")

        first = installer.install(repo, args.sdk, False)
        verify_manifest(repo, first)
        second = installer.install(repo, args.sdk, False)
        check(first == second, "Fresh install then reinstall is byte-stable")
        check("-fstack-usage" in (repo / "build_make/robot_project.mk").read_text(),
              "Generated compiler flags request stack-usage evidence")
        for profile, expected in ((0, "2048"), (1, "1024")):
            result = run(["cc", "-E", "-dM", "-x", "c",
                          f"-DCNF_SMP={profile}", str(repo / "config/config.h")])
            check(result.returncode == 0 and re.search(
                rf"^#define CNF_TMP_STACK_SIZE \({expected}\)$",
                result.stdout, re.MULTILINE) is not None,
                f"C preprocessor selects {expected} bytes for SMP={profile}")

        # Check the actual generated script, including its boot2 assertion.
        # Objects contain host instructions and are never executed.
        boundary = 256 * 1024 - 2048 - 65536
        linker = repo / installer.LINKER_PATH
        for adjustment, should_pass in ((-8, True), (0, True), (4, False)):
            name = f"boundary_{adjustment:+d}".replace("+", "plus")
            assembly = workspace / f"{name}.S"
            object_path = workspace / f"{name}.o"
            elf = workspace / f"{name}.elf"
            assembly.write_text(
                '.section .boot2,"a"\n.zero 256\n'
                '.section .data_vector,"aw",@nobits\n.zero 192\n'
                '.section .text\n.globl Reset_Handler\nReset_Handler:\n'
                '.byte 0xc3\n.section .bss\n'
                f'.zero {boundary + adjustment - 192}\n'
                '.section .note.GNU-stack,"",@progbits\n')
            built = run(["cc", "-c", str(assembly), "-o", str(object_path)])
            if built.returncode != 0:
                raise RuntimeError(built.stderr)
            linked = run(["ld", "-T", str(linker), "-o", str(elf),
                          str(object_path)])
            check((linked.returncode == 0) == should_pass,
                  f"GNU ld boundary {adjustment:+d} bytes has expected result")
            if not should_pass:
                check("INF2004: reserve 64 KiB heap" in linked.stderr,
                      "Excess BSS is rejected by the intended memory assertion")
            symbols = run(["nm", "-n", str(elf)]).stdout if should_pass else ""
            if should_pass:
                check(f"{0x20000000 + boundary + adjustment:016x} B __bss_end"
                      in symbols, "Synthetic BSS ends at intended boundary")
            link_results.append({"adjustment_bytes": adjustment,
                                 "expected_success": should_pass,
                                 "return_code": linked.returncode,
                                 "stderr": linked.stderr.strip(),
                                 "symbols": [line for line in symbols.splitlines()
                                             if "__bss_end" in line or
                                             "__robot_heap_" in line]})

        if args.legacy_repository is not None:
            legacy_source = args.legacy_repository.resolve()
            legacy_report = json.loads(
                (legacy_source / installer.MANIFEST).read_text())
            check("integration_revision" not in legacy_report,
                  "Migration source is the earlier manifest format")
            upgrade = workspace / "legacy-upgrade"
            clone(upgrade)
            for name, expected in legacy_report["installed_sha256"].items():
                payload = (legacy_source / name).read_bytes()
                if hashlib.sha256(payload).hexdigest() != expected:
                    raise ValueError(f"Legacy installation was modified: {name}")
                destination = upgrade / name
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_bytes(payload)
            (upgrade / installer.MANIFEST).write_text(
                json.dumps(legacy_report, indent=2) + "\n")
            upgraded = installer.install(upgrade, args.sdk, False)
            verify_manifest(upgrade, upgraded)
            check(installer.install(upgrade, args.sdk, False) == upgraded,
                  "Intact old installation upgrades and reinstalls unchanged")

        modified = repo / "config/config.h"
        modified.write_text(modified.read_text() + "\n/* local edit */\n")
        before = modified.read_bytes()
        rejects(lambda: installer.install(repo, args.sdk, False),
                "Installer refuses post-install local edits")
        check(modified.read_bytes() == before,
              "Rejected install preserves the local edit")

    report = {"status": "passed", "checks": len(checks),
              "check_names": checks, "linker_boundary_results": link_results,
              "legacy_upgrade_executed": args.legacy_repository is not None,
              "installer_sha256": hashlib.sha256(module_path.read_bytes()).hexdigest(),
              "test_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "host_ld": run(["ld", "--version"]).stdout.splitlines()[0],
              "scope": "Host installer, preprocessing and linker-expression tests; "
                       "no ARM build, target execution or stack high-water proof."}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
