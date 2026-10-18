#!/usr/bin/env python3
"""Refuse stale or modified target inputs before invoking GNU Make.

This is a read-only source-integrity gate, not an ARM build or hardware test.
Local Wi-Fi and MQTT configuration headers may contain private settings; this
tool records neither their contents nor their hashes.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
from pathlib import Path, PurePosixPath

PORT_COMMIT = "2e8e6ad480c2d84a369b12995694b9bf5e9ddfb3"
SDK_COMMIT = "a1438dff1d38bd9c65dbd693f0e5db4b9ae91779"
SUBMODULE_COMMITS = {
    "lib/lwip": "77dcd25a72509eb83f72b033d219b1d40cd8eb95",
    "lib/cyw43-driver": "dd7568229f3bf7a37737b9e1ef250c26efe75b23",
}
MANIFEST = "robot_install_manifest.json"
INTEGRATION_FILES = {
    "build_make/makefile",
    "build_make/mtkernel_3/app_program/subdir.mk",
    "build_make/pico_rp2040.mk",
    "build_make/robot_project.mk",
    "build_make/robot_sdk_headers/pico/version.h",
    "config/config.h",
    "etc/linker/pico_rp2040/tkernel_map.ld",
    "include/sys/inittask.h",
    "kernel/sysdepend/pico_rp2040/devinit.c",
    "kernel/sysdepend/pico_rp2040/hw_setting.c",
    "lib/libnet/lwip/include/lwipopts.h",
    "lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c",
    "lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_kernel.c",
}


def git(repo: Path, *arguments: str) -> str:
    """Capture read-only Git output without executing a shell."""
    result = subprocess.run(["git", "-C", str(repo), *arguments],
                            text=True, capture_output=True, check=False)
    if result.returncode:
        raise ValueError(f"Git input check failed in {repo}: "
                         f"{result.stderr.strip()}")
    return result.stdout


def regular_file(root: Path, name: str) -> Path:
    """Validate a canonical relative path and every directory component."""
    if not isinstance(name, str) or not name or "\\" in name:
        raise ValueError("Invalid manifest/source path")
    relative = PurePosixPath(name)
    if (relative.is_absolute() or ".." in relative.parts or
            relative.as_posix() != name or name == "."):
        raise ValueError(f"Non-canonical or escaping path: {name}")
    candidate = root
    for component in relative.parts:
        candidate = candidate / component
        if candidate.is_symlink():
            raise ValueError(f"Symlink input is not allowed: {name}")
    if not candidate.is_file():
        raise ValueError(f"Required regular file is missing: {name}")
    if not candidate.resolve().is_relative_to(root.resolve()):
        raise ValueError(f"Path escapes its input root: {name}")
    return candidate


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def unique_object(pairs: list[tuple[str, object]]) -> dict:
    """Reject duplicate JSON keys instead of accepting the last value."""
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate installation-manifest key: {key}")
        result[key] = value
    return result


def tracked_changes(repo: Path) -> set[str]:
    return set(filter(None, git(repo, "diff", "--name-only", "-z", "HEAD",
                                "--ignore-submodules=untracked").split("\0")))


def check_submodules(sdk: Path, profile: str) -> dict[str, str]:
    """Check every initialized gitlink; require radio libraries for MQTT."""
    observed: dict[str, str] = {}

    def visit(repository: Path, prefix: str, depth: int) -> None:
        if depth > 16:
            raise ValueError("Unexpectedly deep SDK submodule tree")
        for entry in git(repository, "ls-files", "--stage", "-z").split("\0"):
            if not entry:
                continue
            metadata, name = entry.split("\t", 1)
            mode, expected, stage = metadata.split()
            if mode != "160000":
                continue
            if stage != "0":
                raise ValueError("SDK contains an unresolved submodule merge")
            relative = f"{prefix}/{name}" if prefix else name
            child = repository / name
            if child.is_symlink() or any(parent.is_symlink()
                    for parent in child.parents if parent != sdk):
                raise ValueError(f"Symlink SDK submodule: {relative}")
            if not (child / ".git").exists():
                if profile == "mqtt" and relative in SUBMODULE_COMMITS:
                    raise ValueError(f"MQTT SDK submodule is not initialized: {relative}")
                continue
            top = Path(git(child, "rev-parse", "--show-toplevel").strip()).resolve()
            if top != child.resolve():
                raise ValueError(f"SDK submodule resolves to another repository: {relative}")
            actual = git(child, "rev-parse", "HEAD").strip()
            if actual != expected or (relative in SUBMODULE_COMMITS and
                                      actual != SUBMODULE_COMMITS[relative]):
                raise ValueError(f"SDK submodule pin mismatch: {relative}")
            if tracked_changes(child) or git(child, "ls-files", "--unmerged", "-z"):
                raise ValueError(f"SDK submodule has tracked edits: {relative}")
            observed[relative] = actual
            visit(child, relative, depth + 1)

    visit(sdk, "", 0)
    if profile == "mqtt" and not SUBMODULE_COMMITS.keys() <= observed.keys():
        raise ValueError("Required MQTT SDK submodules are absent")
    return observed


def verify(repo: Path, sdk: Path, profile: str) -> dict:
    """Return evidence only after all current-source checks succeed."""
    if profile not in ("offline", "mqtt"):
        raise ValueError("Profile must be offline or mqtt")
    repo, sdk = repo.resolve(), sdk.resolve()
    if any(character.isspace() for character in str(repo) + str(sdk)):
        raise ValueError("Use checkout paths without whitespace for GNU Make")
    for path, expected, label in ((repo, PORT_COMMIT, "professor checkout"),
                                  (sdk, SDK_COMMIT, "Pico SDK")):
        if Path(git(path, "rev-parse", "--show-toplevel").strip()).resolve() != path:
            raise ValueError(f"Use the root directory of the {label}")
        if git(path, "rev-parse", "HEAD").strip() != expected:
            raise ValueError(f"The {label} is not at its required commit {expected}")
        if git(path, "ls-files", "--unmerged", "-z"):
            raise ValueError(f"The {label} contains an unresolved merge")
    if tracked_changes(sdk):
        raise ValueError("Pico SDK has tracked edits or changed submodules")
    submodules = check_submodules(sdk, profile)

    manifest_path = regular_file(repo, MANIFEST)
    manifest = json.loads(manifest_path.read_text(), object_pairs_hook=unique_object)
    if not isinstance(manifest, dict):
        raise ValueError("Installation manifest must be a JSON object")
    for name, expected in {
        "professor_commit": PORT_COMMIT, "pico_sdk_commit": SDK_COMMIT,
        "sdk_version": "2.2.0", "profile": "SMP=0 CONSOLE=uart",
        "integration_revision": "2026-09-11", "minimum_heap_bytes": 65536,
        "temporary_stack_bytes": 2048, "kernel_tick_ms": 1, "dry_run": False,
    }.items():
        if type(manifest.get(name)) is not type(expected) or manifest[name] != expected:
            raise ValueError(f"Missing or outdated installation field: {name}")
    installed = manifest.get("installed_sha256")
    if not isinstance(installed, dict) or not installed:
        raise ValueError("Installation manifest has no installed file hashes")
    for name, expected in installed.items():
        path = regular_file(repo, name)
        if not isinstance(expected, str) or not re.fullmatch("[0-9a-f]{64}", expected):
            raise ValueError(f"Invalid installed SHA-256 for {name}")
        if sha256(path) != expected:
            raise ValueError(f"Installed file drift: {name}; preserve edits before reinstalling")

    firmware = Path(__file__).resolve().parents[1]
    sources = []
    for folder, extension in (("src", "c"), ("include", "h"),
                               ("target", "c"), ("target", "h")):
        sources.extend(sorted((firmware / folder).glob(f"*.{extension}")))
    names = [source.name for source in sources]
    if not sources or len(set(names)) != len(names):
        raise ValueError("The current source set is empty or has duplicate flattened names")
    app_files = manifest.get("app_files")
    if (not isinstance(app_files, list) or
            not all(isinstance(name, str) for name in app_files) or
            len(app_files) != len(names) or set(app_files) != set(names)):
        raise ValueError("Installed application source set is stale; reinstall current sources")
    expected_paths = INTEGRATION_FILES | {f"app_program/{name}" for name in names}
    if set(installed) != expected_paths:
        raise ValueError("Installation manifest path set differs from the reviewed integration: "
                         f"missing={sorted(expected_paths - set(installed))}, "
                         f"extra={sorted(set(installed) - expected_paths)}")
    source_hashes = {}
    for source in sources:
        relative = source.relative_to(firmware).as_posix()
        digest = sha256(regular_file(firmware, relative))
        if digest != installed[f"app_program/{source.name}"]:
            raise ValueError(f"Installed application is stale or changed: {source.name}; reinstall")
        source_hashes[relative] = digest
    unrelated = tracked_changes(repo) - expected_paths
    if unrelated:
        raise ValueError(f"Unreviewed tracked checkout edits: {sorted(unrelated)}")
    # Make uses a top-level *.c wildcard. Inspect the filesystem, including
    # ignored files, rather than relying on Git's untracked-file reporting.
    allowed_c = set(names) | {"demo_tasks.c"}
    unknown_c = {path.name for path in (repo / "app_program").glob("*.c")} - allowed_c
    if unknown_c:
        raise ValueError(f"Unexpected application C source: {sorted(unknown_c)}")
    for name in ("config/wifi_credentials.h", "config/net_mqtt_config.h"):
        path = repo / name
        if path.exists() or path.is_symlink():
            regular_file(repo, name)
        elif profile == "mqtt":
            raise ValueError(f"MQTT profile requires a local configuration header: {name}")
    return {
        "status": "passed", "profile": profile,
        "integration_revision": "2026-09-11",
        "professor_commit": PORT_COMMIT, "pico_sdk_commit": SDK_COMMIT,
        "initialized_sdk_submodules": submodules,
        "installed_file_count": len(installed), "application_file_count": len(names),
        "manifest_sha256": sha256(manifest_path), "source_sha256": source_hashes,
        "minimum_heap_bytes": 65536, "temporary_stack_bytes": 2048,
        "scope": "Read-only source-integrity gate; no compilation or hardware validation.",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--sdk", type=Path, required=True)
    parser.add_argument("--profile", choices=("offline", "mqtt"), required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        report = verify(args.repo, args.sdk, args.profile)
    except (ValueError, OSError, KeyError, TypeError) as error:
        report = {"status": "failed", "profile": args.profile, "reason": str(error),
                  "scope": "Preflight stopped before compilation."}
    payload = json.dumps(report, indent=2) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload)
    print(payload, end="")
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
