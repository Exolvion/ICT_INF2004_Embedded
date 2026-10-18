#!/usr/bin/env python3
"""Exercise the staged-import guard with disposable file trees.

No repository file is written. Only import_stage.py is used in place;
test_application.py and scripts/dev.sh are copied into the disposable trees
to check their import-stage refusals (before anything is built).
"""
from __future__ import annotations

from pathlib import Path
import importlib.util
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

import import_stage

SCRIPT = Path(__file__).with_name("import_stage.py")
APPLICATION = Path(__file__).with_name("test_application.py")
DEV_SH = Path(__file__).resolve().parents[2] / "scripts/dev.sh"


def make_tree(root: Path, last_stage: int, marker: str | None) -> None:
    """Create the files of stages 2..last_stage and an optional marker."""
    for stage, paths in import_stage.STAGE_FILES.items():
        if stage <= last_stage:
            for path in paths:
                (root / path).parent.mkdir(parents=True, exist_ok=True)
                (root / path).write_text("fixture\n")
    if marker is not None:
        (root / import_stage.MARKER).write_text(marker)


class ImportStageTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="import-stage-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def test_consistent_markers_are_accepted(self):
        for stage in (1, 2, 3, 4):
            with self.subTest(stage=stage):
                with tempfile.TemporaryDirectory() as folder:
                    root = Path(folder)
                    make_tree(root, stage, f"{stage}\n")
                    self.assertEqual(import_stage.inspect(root),
                                     (str(stage), []))

    def test_complete_tree_without_marker_is_complete(self):
        make_tree(self.root, 5, None)
        self.assertEqual(import_stage.inspect(self.root), (None, []))

    def test_marker_left_in_complete_tree_is_rejected(self):
        for marker in ("1\n", "4\n", "4\r\n", " 2 "):
            with self.subTest(marker=marker):
                with tempfile.TemporaryDirectory() as folder:
                    root = Path(folder)
                    make_tree(root, 5, marker)
                    stage, errors = import_stage.inspect(root)
                    self.assertTrue(errors)
                    self.assertIn("git rm .import-stage", errors[-1])

    def test_marker_behind_or_ahead_of_files_is_rejected(self):
        for files, marker in ((3, "2"), (2, "3"), (1, "4"), (4, "1")):
            with self.subTest(files=files, marker=marker):
                with tempfile.TemporaryDirectory() as folder:
                    root = Path(folder)
                    make_tree(root, files, marker)
                    self.assertTrue(import_stage.inspect(root)[1])

    def test_invalid_marker_values_are_rejected(self):
        for marker in ("0", "5", "", "abc", "12", "1\n2\n"):
            with self.subTest(marker=marker):
                (self.root / import_stage.MARKER).write_text(marker)
                self.assertIn("Invalid", import_stage.inspect(self.root)[1][0])

    def test_github_mode_outputs_and_exit_status(self):
        make_tree(self.root, 2, "2\n")
        output = self.root / "github-output.txt"
        summary = self.root / "summary.md"
        environment = dict(os.environ, GITHUB_OUTPUT=str(output),
                           GITHUB_STEP_SUMMARY=str(summary))
        good = subprocess.run([sys.executable, str(SCRIPT), "--root",
                               str(self.root), "--github"], env=environment,
                              text=True, capture_output=True, check=False)
        self.assertEqual(good.returncode, 0, good.stdout + good.stderr)
        self.assertEqual(output.read_text(),
                         "import_complete=false\nimport_stage=2\n")
        self.assertIn("::warning", good.stdout)
        make_tree(self.root, 5, "2\n")
        bad = subprocess.run([sys.executable, str(SCRIPT), "--root",
                              str(self.root), "--github"], env=environment,
                             text=True, capture_output=True, check=False)
        self.assertEqual(bad.returncode, 1)
        self.assertIn("::error::", bad.stdout)
        (self.root / import_stage.MARKER).unlink()
        output.unlink()
        done = subprocess.run([sys.executable, str(SCRIPT), "--root",
                               str(self.root), "--github"], env=environment,
                              text=True, capture_output=True, check=False)
        self.assertEqual(done.returncode, 0)
        self.assertEqual(output.read_text(),
                         "import_complete=true\nimport_stage=complete\n")


class ApplicationRefusalTests(unittest.TestCase):
    """test_application.py stops with exit status 2 during a staged import."""

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="app-refusal-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def run_application(self) -> subprocess.CompletedProcess:
        script = self.root / "firmware/scripts/test_application.py"
        script.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(APPLICATION, script)
        environment = dict(os.environ,
                           ROBOT_VALIDATION_DIR=str(self.root / "reports"))
        return subprocess.run([sys.executable, "-B", str(script)],
                              env=environment, text=True,
                              capture_output=True, check=False)

    def assert_nothing_built(self):
        self.assertFalse((self.root / "reports").exists())
        self.assertFalse((self.root / "firmware/build-apphost").exists())

    def test_marker_present_is_refused_with_exit_2(self):
        make_tree(self.root, 2, "2\n")
        result = self.run_application()
        self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
        self.assertIn("ERROR: import in progress (.import-stage=2): "
                      "app-tests need all five stages", result.stderr)
        self.assertIn("tests/apphost_test_main.c", result.stderr)
        self.assert_nothing_built()

    def test_missing_stage_5_sources_are_refused_without_marker(self):
        make_tree(self.root, 4, None)
        result = self.run_application()
        self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
        self.assertIn("ERROR: required app-test sources missing (",
                      result.stderr)
        self.assertIn("src/scanner.c", result.stderr)
        self.assert_nothing_built()

    def test_required_sources_arrive_with_the_final_stage(self):
        spec = importlib.util.spec_from_file_location("test_application",
                                                      APPLICATION)
        application = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(application)
        stage_of = {path: stage
                    for stage, paths in import_stage.STAGE_FILES.items()
                    for path in paths}
        stages = {stage_of.get("firmware/" + name, 1)
                  for name in application.REQUIRED_SOURCES}
        # Some required source is a stage-5 file, so app-tests cannot run
        # before the fifth import, which also removes the marker.
        self.assertEqual(max(stages), 5)


@unittest.skipIf(os.name == "nt" or shutil.which("bash") is None,
                 "scripts/dev.sh runs in a Linux shell")
class DevScriptRefusalTests(unittest.TestCase):
    """scripts/dev.sh refuses whole-tree commands during a staged import and
    reports a marker that contradicts the files as the CI failure it is."""

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="dev-refusal-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        (self.root / "scripts").mkdir()
        shutil.copy2(DEV_SH, self.root / "scripts/dev.sh")
        guard = self.root / "firmware/scripts/import_stage.py"
        guard.parent.mkdir(parents=True)
        shutil.copy2(SCRIPT, guard)

    def run_dev(self, *arguments: str) -> subprocess.CompletedProcess:
        environment = dict(os.environ,
                           ROBOT_DEPS_DIR=str(self.root / "no-deps"),
                           ROBOT_VALIDATION_DIR=str(self.root / "reports"))
        return subprocess.run(["bash", str(self.root / "scripts/dev.sh"),
                               *arguments], env=environment, text=True,
                              capture_output=True, check=False)

    def test_consistent_marker_is_skipped_as_in_ci(self):
        make_tree(self.root, 2, "2\n")
        result = self.run_dev("app-tests")
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn("staged import is in progress (.import-stage = 2)",
                      result.stderr)
        self.assertIn("CI skips it too", result.stderr)

    def test_contradicting_marker_reports_the_ci_failure(self):
        make_tree(self.root, 5, "4\n")
        for command in (("app-tests",), ("install",), ("build", "offline")):
            with self.subTest(command=command):
                result = self.run_dev(*command)
                self.assertEqual(result.returncode, 1,
                                 result.stdout + result.stderr)
                self.assertIn(".import-stage contradicts the files present; "
                              "CI fails the same way", result.stderr)
                self.assertIn("git rm .import-stage", result.stderr)
                self.assertNotIn("CI skips it too", result.stderr)


if __name__ == "__main__":
    unittest.main()
