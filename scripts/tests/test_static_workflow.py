"""Isolated NAV fixtures; fake Git/tools, no compilation, ROS, or Git writes."""

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from check_boundaries import collect, compare_baseline, local_file


class StaticWorkflowTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / ".git").mkdir()
        for name in ("src/ros2", "config", "launch", "scripts/tests", "bin"):
            (self.root / name).mkdir(parents=True)
        for name in ("verify_static.py", "check_boundaries.py", "verify.sh", "format.sh"):
            shutil.copyfile(Path(__file__).resolve().parents[1] / name, self.root / "scripts" / name)
        (self.root / "src/input.hpp").write_text("int value;\n")
        (self.root / "config/input.yml").write_text("value: 1\n")
        (self.root / "launch/run.launch.py").write_text("raise RuntimeError('must not execute launch')\n")
        (self.root / "package.xml").write_text("<package/>\n")
        (self.root / ".clang-format").write_text("BasedOnStyle: LLVM\n")
        (self.root / "CMakeLists.txt").write_text("project(nav)\n")
        (self.root / "scripts/static_baseline.json").write_text(
            '{"schema":1,"entries":{},"revision":"fixture","reason":"isolated test"}')
        (self.root / "scripts/tests/test_fixture.py").write_text(
            "import unittest\nclass FixtureTest(unittest.TestCase):\n    def test_ok(self): pass\n")
        self.log = self.root / "tool_calls.jsonl"
        stub = f'''#!{sys.executable}
import json, os, pathlib, sys
name=pathlib.Path(sys.argv[0]).name
with open(os.environ["TOOL_LOG"], "a") as f:
    f.write(json.dumps([name, *sys.argv[1:]])+"\\n")
if name == "clang-format":
    if "--version" in sys.argv:
        print("clang-format version " + os.environ.get("FORMAT_VERSION", "18.1.8"))
    elif os.environ.get("FAIL_FORMAT"):
        sys.exit(7)
    elif "-i" not in sys.argv:
        path=pathlib.Path(sys.argv[-1])
        offset=os.environ.get("FORMAT_OFFSET")
        print("<replacements>" + (f'<replacement offset="{{offset}}" length="0"> </replacement>' if offset else "") + "</replacements>")
elif name == "git":
    if sys.argv[1] == "show":
        old=os.environ.get("HEAD_TEXT")
        if old is None: sys.exit(128)
        print(old, end="")
    elif sys.argv[1] == "log": print("a"*40)
    elif sys.argv[1] not in {{"status", "ls-tree"}}: sys.exit(99)
else:
    sys.exit(98)
'''
        for name in ("git", "clang-format", "cmake", "ninja", "colcon", "make", "clang-tidy"):
            path = self.root / "bin" / name
            path.write_text(stub)
            path.chmod(0o755)
        self.env = dict(os.environ, PATH=f"{self.root / 'bin'}:{os.environ['PATH']}",
                        TOOL_LOG=str(self.log), PYTHONDONTWRITEBYTECODE="1")
        for key in ("HEAD_TEXT", "FORMAT_OFFSET", "FAIL_FORMAT", "FORMAT_VERSION"):
            self.env.pop(key, None)

    def invoke(self, *args, **env):
        protected = {str(p.relative_to(self.root)): p.read_bytes() for name in ("src", "config", "launch")
                     for p in (self.root / name).rglob("*") if p.is_file() and not p.is_symlink()}
        result = subprocess.run(["bash", str(self.root / "scripts/verify.sh"), *args],
                                cwd=self.root.parent, env=dict(self.env, **env), capture_output=True, text=True)
        for name, content in protected.items():
            self.assertEqual((self.root / name).read_bytes(), content)
        calls = [json.loads(line) for line in self.log.read_text().splitlines()] if self.log.exists() else []
        self.assertFalse(any(call[0] in {"cmake", "ninja", "colcon", "make", "clang-tidy"} for call in calls))
        return result

    def test_default_static_is_read_only_and_does_not_execute_launch(self):
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("build/runtime/sanitizers NOT_VERIFIED", result.stdout)

    def test_empty_cpp_is_valid_and_parent_git_is_not_discovered(self):
        (self.root / "src/empty.cpp").touch()
        self.assertEqual(self.invoke("--full-scan").returncode, 0)
        (self.root / ".git").rmdir()
        self.assertEqual(self.invoke().returncode, 1)

    def test_rejects_build_options_before_invoking_tools(self):
        for option in ("--with-ros2", "--tsan", "--benchmark", "--build"):
            result = self.invoke(option)
            self.assertEqual(result.returncode, 2)
        self.assertFalse(self.log.exists())

    def test_duplicate_yaml_and_invalid_launch_or_xml_fail(self):
        cases = {"config/input.yml": "key: 1\nkey: 2\n", "launch/run.launch.py": "def (\n",
                 "package.xml": "<package>"}
        for name, broken in cases.items():
            with self.subTest(name=name):
                path = self.root / name
                original = path.read_text()
                path.write_text(broken)
                self.assertEqual(self.invoke().returncode, 1)
                path.write_text(original)

    def test_empty_or_missing_scope_fails(self):
        (self.root / "src/input.hpp").unlink()
        self.assertEqual(self.invoke().returncode, 1)
        shutil.rmtree(self.root / "src")
        self.assertEqual(self.invoke().returncode, 1)

    def test_core_ros_is_rejected_but_ros_entry_is_allowed(self):
        (self.root / "src/ma_nav.cpp").write_text('#include <rclcpp/rclcpp.hpp>\n')
        (self.root / "src/ros2/node.cpp").write_text('#include <nav_msgs/msg/path.hpp>\n')
        self.assertEqual(self.invoke().returncode, 0)
        (self.root / "src/input.hpp").write_text('#include <sensor_msgs/msg/point_cloud2.hpp>\n')
        self.assertEqual(self.invoke().returncode, 1)

    def test_parent_and_external_symlink_are_rejected(self):
        with self.assertRaises(ValueError):
            local_file(self.root, "src/../../external.hpp")
        (self.root / "src/external.hpp").symlink_to(Path(__file__))
        self.assertEqual(self.invoke().returncode, 1)

    def test_excluded_build_tree_is_not_scanned(self):
        path = self.root / "src/build"
        path.mkdir()
        (path / "invalid.yaml").write_text("x: [")
        self.assertEqual(self.invoke().returncode, 0)

    def test_baseline_records_debt_and_full_scan_still_fails(self):
        (self.root / "src/input.hpp").write_text('#include <rclcpp/rclcpp.hpp>\n')
        self.assertEqual(self.invoke("--write-baseline", "test fixture only").returncode, 0)
        self.assertEqual(self.invoke().returncode, 0)
        self.assertEqual(self.invoke("--full-scan").returncode, 1)
        with (self.root / "src/input.hpp").open("a") as stream:
            stream.write('#include <rclcpp/rclcpp.hpp>\n')
        self.assertEqual(self.invoke().returncode, 1)

    def test_baseline_handles_moved_lines_without_allowing_new_duplicates(self):
        finding = {"key": "rule:path:hash", "line": 3}
        self.assertEqual(compare_baseline([finding], {finding["key"]: 1}), [])
        self.assertEqual(len(compare_baseline([finding, finding], {finding["key"]: 1})), 1)

    def test_new_and_changed_line_formatting_fail_but_untouched_lines_do_not(self):
        self.assertEqual(self.invoke(FORMAT_OFFSET="1").returncode, 1)
        path = self.root / "src/input.hpp"
        path.write_text("int value;\nint next;\n")
        self.assertEqual(self.invoke(HEAD_TEXT="int value;\n", FORMAT_OFFSET="1").returncode, 0)
        self.assertEqual(self.invoke(HEAD_TEXT="int value;\n", FORMAT_OFFSET="12").returncode, 1)
        self.assertEqual(self.invoke("--full-scan", HEAD_TEXT="int value;\nint next;\n", FORMAT_OFFSET="1").returncode, 1)

    def test_tool_failure_and_wrong_version_propagate(self):
        self.assertEqual(self.invoke(FAIL_FORMAT="1").returncode, 1)
        self.assertEqual(self.invoke(FORMAT_VERSION="14.0.0").returncode, 1)

    def test_missing_local_style_and_test_suite_fail(self):
        path = self.root / ".clang-format"
        path.unlink()
        self.assertEqual(self.invoke().returncode, 1)
        path.write_text("BasedOnStyle: LLVM\n")
        (self.root / "scripts/tests/test_fixture.py").unlink()
        self.assertEqual(self.invoke().returncode, 1)

    def test_report_must_stay_in_local_artifact_directory(self):
        self.assertEqual(self.invoke("--report", ".cache/verify/report.json").returncode, 0)
        self.assertTrue((self.root / ".cache/verify/report.json").is_file())
        self.assertEqual(self.invoke("--report", "report.json").returncode, 1)

    def test_format_requires_explicit_local_source_files(self):
        for args in ([], ["../external.hpp"], ["config/input.yml"]):
            result = subprocess.run(["bash", str(self.root / "scripts/format.sh"), *args],
                                    env=self.env, capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)


if __name__ == "__main__":
    unittest.main()
