#!/usr/bin/env python3
"""Exercise the preflight gate using disposable local source checkouts."""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import subprocess
import sys
import tempfile
from pathlib import Path


def run(arguments: list[str]) -> subprocess.CompletedProcess:
    return subprocess.run(arguments, text=True, capture_output=True, check=False)


def load(name: str):
    specification = importlib.util.spec_from_file_location(
        name, Path(__file__).with_name(f"{name}.py"))
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository", type=Path, required=True)
    parser.add_argument("--sdk", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    installer, gate = load("install_project"), load("verify_install")
    checks: list[str] = []
    refusals: list[dict] = []

    def check(condition: bool, name: str) -> None:
        if not condition:
            raise AssertionError(name)
        checks.append(name)

    def clone(source: Path, target: Path) -> None:
        result = run(["git", "clone", "--quiet", "--shared", str(source), str(target)])
        if result.returncode:
            raise RuntimeError(result.stderr)

    def different_commit(checkout: Path) -> subprocess.CompletedProcess:
        # A local empty commit also works when the provided history is shallow.
        return run(["git", "-C", str(checkout), "-c", "user.name=Preflight Fixture",
                    "-c", "user.email=fixture@example.invalid", "commit", "--quiet",
                    "--allow-empty", "-m", "Temporary pin-refusal fixture"])

    with tempfile.TemporaryDirectory(prefix="inf2004-preflight-") as temporary:
        folder = Path(temporary)
        repo, sdk = folder / "port", folder / "sdk"
        clone(args.repository.resolve(), repo)
        clone(args.sdk.resolve(), sdk)
        installation = installer.install(repo, sdk, False)
        baseline = (repo / gate.MANIFEST).read_bytes()
        first = gate.verify(repo, sdk, "offline")
        check(first["status"] == "passed", "Fresh installation passes offline preflight")
        check(first == gate.verify(repo, sdk, "offline"),
              "Repeated preflight is deterministic")
        check((repo / gate.MANIFEST).read_bytes() == baseline,
              "Preflight leaves installation manifest unchanged")

        def rejects(name: str, phrase: str, profile: str = "offline") -> None:
            try:
                gate.verify(repo, sdk, profile)
            except ValueError as error:
                check(phrase in str(error), name)
                refusals.append({"case": name, "reason": str(error)})
            else:
                raise AssertionError(f"Unexpected acceptance: {name}")

        def manifest_case(name: str, phrase: str, change) -> None:
            manifest = json.loads(baseline)
            change(manifest)
            (repo / gate.MANIFEST).write_text(json.dumps(manifest))
            try:
                rejects(name, phrase)
            finally:
                (repo / gate.MANIFEST).write_bytes(baseline)

        rejects("MQTT refuses uninitialized libraries", "not initialized", "mqtt")
        for name in gate.SUBMODULE_COMMITS:
            clone(args.sdk.resolve() / name, sdk / name)
        rejects("MQTT requires local credentials", "local configuration header", "mqtt")
        credentials = repo / "config/wifi_credentials.h"
        settings = repo / "config/net_mqtt_config.h"
        credentials.write_text("/* private test fixture, never printed */\n")
        settings.write_text("/* private test fixture, never printed */\n")
        mqtt = gate.verify(repo, sdk, "mqtt")
        check(mqtt["status"] == "passed" and
              mqtt["initialized_sdk_submodules"] == gate.SUBMODULE_COMMITS,
              "Initialized pinned radio libraries and local headers pass MQTT preflight")
        check("private test fixture" not in json.dumps(mqtt),
              "Preflight report does not expose local configuration contents")
        manifest_case("Old integration revision is refused", "integration_revision",
                      lambda value: value.update(integration_revision="2026-09-09"))
        manifest_case("Reduced heap budget is refused", "minimum_heap_bytes",
                      lambda value: value.update(minimum_heap_bytes=32768))
        manifest_case("Reduced temporary stack is refused", "temporary_stack_bytes",
                      lambda value: value.update(temporary_stack_bytes=256))
        manifest_case("Boolean tick disguised as integer is refused", "kernel_tick_ms",
                      lambda value: value.update(kernel_tick_ms=True))
        manifest_case("Malformed hash is refused", "Invalid installed SHA-256",
                      lambda value: value["installed_sha256"].update({"app_program/core.c": "bad"}))
        manifest_case("Missing integration path is refused", "manifest path set",
                      lambda value: value["installed_sha256"].pop("build_make/robot_project.mk"))
        manifest_case("Escaping manifest path is refused", "escaping path",
                      lambda value: value["installed_sha256"].update({"../outside.c": "0" * 64}))
        manifest_case("Absolute manifest path is refused", "escaping path",
                      lambda value: value["installed_sha256"].update({"/tmp/outside.c": "0" * 64}))
        manifest_case("Noncanonical path is refused", "escaping path",
                      lambda value: value["installed_sha256"].update({"app_program/./core.c": "0" * 64}))
        manifest_case("Duplicate application names are refused", "source set is stale",
                      lambda value: value["app_files"].append(value["app_files"][0]))

        source = repo / "app_program/core.c"
        source_before = source.read_bytes()
        source.write_bytes(source_before + b"\n/* local source edit */\n")
        rejects("Installed source drift is refused", "Installed file drift")
        check(source.read_bytes() != source_before, "Refusal preserves local source edits")
        manifest_case("Rehashed stale application is refused", "stale or changed",
                      lambda value: value["installed_sha256"].update(
                          {"app_program/core.c": hashlib.sha256(source.read_bytes()).hexdigest()}))
        source.write_bytes(source_before)
        source.unlink()
        source.symlink_to(repo / "app_program/vehicle.c")
        rejects("Installed source symlink is refused", "Symlink input")
        source.unlink()
        source.write_bytes(source_before)
        directory = repo / "build_make/robot_sdk_headers"
        moved = repo / "saved_sdk_headers"
        directory.rename(moved)
        directory.symlink_to(moved, target_is_directory=True)
        rejects("Intermediate directory symlink is refused", "Symlink input")
        directory.unlink()
        moved.rename(directory)

        unknown = repo / "app_program/unreviewed.c"
        unknown.write_text("int unreviewed(void) { return 1; }\n")
        rejects("Additional application C source is refused", "Unexpected application C source")
        unknown.unlink()
        unrelated = repo / "include/tk/typedef.h"
        unrelated_before = unrelated.read_bytes()
        unrelated.write_bytes(unrelated_before + b"\n/* unrelated tracked edit */\n")
        rejects("Unrelated tracked checkout edit is refused", "Unreviewed tracked checkout edits")
        manifest_case("Manifest cannot authorize extra tracked source", "manifest path set",
                      lambda value: value["installed_sha256"].update(
                          {"include/tk/typedef.h": hashlib.sha256(unrelated.read_bytes()).hexdigest()}))
        unrelated.write_bytes(unrelated_before)

        sdk_file = sdk / "pico_sdk_init.cmake"
        sdk_before = sdk_file.read_bytes()
        sdk_file.write_bytes(sdk_before + b"\n# tracked SDK edit\n")
        rejects("Tracked SDK edit is refused", "Pico SDK has tracked edits")
        sdk_file.write_bytes(sdk_before)
        lwip_file = sdk / "lib/lwip/src/core/init.c"
        lwip_before = lwip_file.read_bytes()
        lwip_file.write_bytes(lwip_before + b"\n/* tracked library edit */\n")
        rejects("Tracked radio library edit is refused", "tracked edits", "mqtt")
        lwip_file.write_bytes(lwip_before)
        result = different_commit(sdk / "lib/lwip")
        check(result.returncode == 0, "Local fixture can select another library revision")
        rejects("Changed radio library revision is refused", "changed submodules", "mqtt")
        result = run(["git", "-C", str(sdk / "lib/lwip"), "checkout", "--quiet",
                      gate.SUBMODULE_COMMITS["lib/lwip"]])
        check(result.returncode == 0, "Library revision restored in disposable fixture")

        manifest_file = repo / gate.MANIFEST
        manifest_file.write_text('{"dry_run":false,"dry_run":false}')
        rejects("Duplicate manifest JSON keys are refused", "Duplicate installation-manifest key")
        manifest_file.write_bytes(baseline)
        for checkout, expected, label in ((repo, gate.PORT_COMMIT, "professor"),
                                          (sdk, gate.SDK_COMMIT, "SDK")):
            result = different_commit(checkout)
            check(result.returncode == 0, f"Disposable {label} pin fixture changed")
            rejects(f"Wrong {label} pin is refused", "required commit")
            result = run(["git", "-C", str(checkout), "update-ref", "HEAD", expected])
            check(result.returncode == 0, f"Disposable {label} pin fixture restored")
        check(gate.verify(repo, sdk, "mqtt")["status"] == "passed",
              "Restored inputs pass after all negative cases")
        credentials.unlink()
        credentials.symlink_to(settings)
        rejects("Local credentials symlink is refused", "Symlink input", "mqtt")
        credentials.unlink()
        credentials.write_text("/* private test fixture */\n")

        manifest_file.write_text("{invalid json")
        output = folder / "cli-failure.json"
        result = run([sys.executable, str(Path(__file__).with_name("verify_install.py")),
                      "--repo", str(repo), "--sdk", str(sdk), "--profile", "offline",
                      "--output", str(output)])
        check(result.returncode == 1 and json.loads(output.read_text())["status"] == "failed",
              "CLI writes machine-readable failure evidence and exits nonzero")
        check(json.loads(result.stdout) == json.loads(output.read_text()),
              "CLI stdout and saved failure evidence agree")
        manifest_file.write_bytes(baseline)

    report = {
        "status": "passed", "checks": len(checks), "check_names": checks,
        "refusal_cases": refusals, "installed_file_count": len(installation["installed_sha256"]),
        "script_sha256": {name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                          for name in ("verify_install.py", "test_verify_install.py", "install_project.py")},
        "scope": "Disposable local Git/source-integrity tests; no network, ARM build or hardware execution.",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
