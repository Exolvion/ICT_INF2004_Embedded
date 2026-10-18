#!/usr/bin/env python3
"""Check the staged-import marker against the files that are present.

During the sequential public import the repository root holds `.import-stage`
with the number (1-4) of the last imported stage. CI and scripts/dev.sh skip
the application tests and ARM builds while it exists. This check stops a
marker from skipping them in a tree that has moved on: every file of the
stages up to the marker must exist and no file of a later stage may exist.

    python3 firmware/scripts/import_stage.py           # prints "complete" or N
    python3 firmware/scripts/import_stage.py --github  # GitHub Actions step

Exit status 0: no marker, or a marker that matches the files present.
Exit status 1: an invalid marker, or one that contradicts the files present.
"""
from __future__ import annotations

from pathlib import Path
import argparse
import os
import sys

MARKER = ".import-stage"

# Files that each later import stage adds; stage 1 is everything else. The
# private package builder splits the stage 2-5 handover packages by these
# lists and its verifier checks every package against them, so change a
# stage's files here and nowhere else.
STAGE_FILES = {
    2: ("Diagrams/motion.dot", "Diagrams/motion.png", "Diagrams/motion.svg",
        "firmware/include/motion.h", "firmware/include/vehicle.h",
        "firmware/src/motion.c", "firmware/src/vehicle.c",
        "firmware/tests/test_motion_main.c",
        "firmware/tests/test_motion_main.h"),
    3: ("Diagrams/optical.dot", "Diagrams/optical.png", "Diagrams/optical.svg",
        "Tools/barcode_sampling_budget.py",
        "firmware/include/barcode.h", "firmware/include/line_ctrl.h",
        "firmware/src/barcode.c", "firmware/src/line_ctrl.c",
        "firmware/target/course_cfg.c", "firmware/target/course_cfg.h",
        "firmware/tests/test_line_main.c", "firmware/tests/test_line_main.h"),
    4: ("Diagrams/terrain.dot", "Diagrams/terrain.png", "Diagrams/terrain.svg",
        "firmware/include/terrain.h", "firmware/src/terrain.c"),
    5: ("Diagrams/scan.dot", "Diagrams/scan.png", "Diagrams/scan.svg",
        "firmware/include/scanner.h", "firmware/src/scanner.c",
        "firmware/tests/apphost_test_main.c",
        "firmware/tests/apphost_test_main.h",
        "firmware/tests/test_sensor_main.c",
        "firmware/tests/test_sensor_main.h"),
}


def inspect(root: Path) -> tuple[str | None, list[str]]:
    """Return (marker value, or None without a marker; list of errors)."""
    marker = root / MARKER
    if not marker.exists():
        return None, []
    stage = "".join(marker.read_text(encoding="utf-8",
                                     errors="replace").split())
    if stage not in {"1", "2", "3", "4"}:
        return stage, [f"Invalid {MARKER} value {stage!r}; expected 1-4. "
                       "The fifth import removes the marker."]
    errors = []
    for first, paths in sorted(STAGE_FILES.items()):
        for path in paths:
            present = (root / path).exists()
            if int(stage) >= first and not present:
                errors.append(f"{MARKER}={stage} but stage-{first} file "
                              f"{path} is missing")
            elif int(stage) < first and present:
                errors.append(f"{MARKER}={stage} but stage-{first} file "
                              f"{path} is already present")
    if errors:
        if any((root / path).exists() for path in STAGE_FILES[5]):
            errors.append("The stage-5 files are present: the fifth import "
                          f"must delete the marker (git rm {MARKER}).")
        else:
            errors.append(f"Copy {MARKER} from the package being imported "
                          "and check that all of its files were copied.")
    return stage, errors


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--root", type=Path,
                        default=Path(__file__).resolve().parents[2],
                        help="repository root (default: this checkout)")
    parser.add_argument("--github", action="store_true",
                        help="write GitHub Actions outputs, annotations and "
                             "a step summary")
    args = parser.parse_args()
    stage, errors = inspect(args.root.resolve())
    if errors:
        for error in errors:
            if args.github:
                print("::error::" + error)
            else:
                print("ERROR: " + error, file=sys.stderr)
        return 1
    if not args.github:
        print(stage or "complete")
        return 0
    with open(os.environ["GITHUB_OUTPUT"], "a", encoding="utf-8") as output:
        output.write(f"import_complete={'false' if stage else 'true'}\n")
        output.write(f"import_stage={stage or 'complete'}\n")
    if stage:
        print(f"::warning title=Import in progress::{MARKER}={stage}; "
              "application tests and ARM builds are skipped until the "
              "final import removes the marker.")
        summary = os.environ.get("GITHUB_STEP_SUMMARY")
        if summary:
            with open(summary, "a", encoding="utf-8") as handle:
                handle.write(f"Import in progress (stage {stage}); the marker "
                             "matches the files present. Full checks resume "
                             f"when {MARKER} is removed.\n")
    else:
        print("No import marker: every check runs.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
