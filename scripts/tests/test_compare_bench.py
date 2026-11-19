"""Isolated unit tests for scripts/compare_bench.py."""

from __future__ import annotations

import contextlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from compare_bench import CompareError, compare_reports, main, render_markdown


class CompareBenchTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def write_json(self, name: str, payload: dict) -> Path:
        path = self.root / name
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    def test_compare_aggregates_and_unit_normalization(self) -> None:
        base = self.write_json(
            "before.json",
            {
                "context": {"library_build_type": "release", "host_name": "box", "num_cpus": 8},
                "benchmarks": [
                    {
                        "name": "GridMap/UpdateESDF_median",
                        "run_name": "GridMap/UpdateESDF",
                        "run_type": "aggregate",
                        "aggregate_name": "median",
                        "real_time": 10.0,
                        "cpu_time": 10.0,
                        "time_unit": "ms",
                    },
                    {
                        "name": "MaMap/GetSnapshot",
                        "run_name": "MaMap/GetSnapshot",
                        "run_type": "iteration",
                        "real_time": 100.0,
                        "cpu_time": 95.0,
                        "time_unit": "ns",
                    },
                    {
                        "name": "Removed/Case",
                        "run_name": "Removed/Case",
                        "run_type": "iteration",
                        "real_time": 5.0,
                        "cpu_time": 5.0,
                        "time_unit": "us",
                    },
                ],
            },
        )
        cand = self.write_json(
            "after.json",
            {
                "context": {"library_build_type": "release", "host_name": "box", "num_cpus": 8},
                "benchmarks": [
                    {
                        "name": "GridMap/UpdateESDF_median",
                        "run_name": "GridMap/UpdateESDF",
                        "run_type": "aggregate",
                        "aggregate_name": "median",
                        "real_time": 5000.0,  # 5.0 ms in us
                        "cpu_time": 5000.0,
                        "time_unit": "us",
                    },
                    {
                        "name": "MaMap/GetSnapshot",
                        "run_name": "MaMap/GetSnapshot",
                        "run_type": "iteration",
                        "real_time": 102.0,
                        "cpu_time": 96.0,
                        "time_unit": "ns",
                    },
                    {
                        "name": "Added/Case",
                        "run_name": "Added/Case",
                        "run_type": "iteration",
                        "real_time": 2.0,
                        "cpu_time": 2.0,
                        "time_unit": "ms",
                    },
                ],
            },
        )

        report = compare_reports(base, cand, threshold=5.0)
        self.assertEqual(report["verdict"], "PASS")
        self.assertEqual(report["counts"]["matched"], 2)
        self.assertEqual(report["counts"]["improved"], 1)
        self.assertEqual(report["counts"]["neutral"], 1)
        self.assertEqual(report["counts"]["regressed"], 0)
        self.assertEqual(report["counts"]["added"], 1)
        self.assertEqual(report["counts"]["removed"], 1)

        by_name = {row["name"]: row for row in report["rows"]}
        self.assertAlmostEqual(by_name["GridMap/UpdateESDF"]["speedup"], 2.0, places=5)
        self.assertEqual(by_name["GridMap/UpdateESDF"]["status"], "IMPROVED")
        self.assertEqual(by_name["MaMap/GetSnapshot"]["status"], "NEUTRAL")
        self.assertEqual(by_name["Added/Case"]["status"], "ADDED")
        self.assertEqual(by_name["Removed/Case"]["status"], "REMOVED")

        md = render_markdown(report)
        self.assertIn("| `GridMap/UpdateESDF` |", md)
        self.assertIn("verdict=PASS", md)

    def test_regression_detection_and_exit_code(self) -> None:
        base = self.write_json(
            "before.json",
            {
                "context": {"library_build_type": "release"},
                "benchmarks": [
                    {
                        "name": "FsmReplan/FullReplan",
                        "run_name": "FsmReplan/FullReplan",
                        "run_type": "iteration",
                        "real_time": 10.0,
                        "cpu_time": 10.0,
                        "time_unit": "ms",
                    }
                ],
            },
        )
        cand = self.write_json(
            "after.json",
            {
                "context": {"library_build_type": "debug"},
                "benchmarks": [
                    {
                        "name": "FsmReplan/FullReplan",
                        "run_name": "FsmReplan/FullReplan",
                        "run_type": "iteration",
                        "real_time": 12.0,
                        "cpu_time": 12.0,
                        "time_unit": "ms",
                    }
                ],
            },
        )

        report = compare_reports(base, cand, threshold=5.0)
        self.assertEqual(report["verdict"], "REGRESSION_DETECTED")
        self.assertEqual(report["counts"]["regressed"], 1)
        self.assertTrue(any("library_build_type" in w for w in report["warnings"]))

        # Default mode prints report and exits 0; --fail-on-regression exits 1
        with contextlib.redirect_stdout(io.StringIO()) as buf:
            self.assertEqual(main([str(base), str(cand)]), 0)
        self.assertIn("REGRESSION_DETECTED", buf.getvalue())
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(main([str(base), str(cand), "--fail-on-regression"]), 1)

    def test_iteration_fallback_mean_and_filter(self) -> None:
        base = self.write_json(
            "iter_base.json",
            {
                "benchmarks": [
                    {
                        "name": "GridMap/A",
                        "run_name": "GridMap/A",
                        "run_type": "iteration",
                        "real_time": 10.0,
                        "cpu_time": 10.0,
                        "time_unit": "ms",
                    },
                    {
                        "name": "GridMap/A",
                        "run_name": "GridMap/A",
                        "run_type": "iteration",
                        "real_time": 20.0,
                        "cpu_time": 14.0,
                        "time_unit": "ms",
                    },
                    {
                        "name": "MaMap/B",
                        "run_name": "MaMap/B",
                        "run_type": "iteration",
                        "real_time": 5.0,
                        "cpu_time": 5.0,
                        "time_unit": "ms",
                    },
                ]
            },
        )
        cand = self.write_json(
            "iter_cand.json",
            {
                "benchmarks": [
                    {
                        "name": "GridMap/A",
                        "run_name": "GridMap/A",
                        "run_type": "iteration",
                        "real_time": 10.0,
                        "cpu_time": 6.0,
                        "time_unit": "ms",
                    },
                    {
                        "name": "GridMap/A",
                        "run_name": "GridMap/A",
                        "run_type": "iteration",
                        "real_time": 10.0,
                        "cpu_time": 6.0,
                        "time_unit": "ms",
                    },
                ]
            },
        )
        report = compare_reports(
            base,
            cand,
            metric="cpu_time",
            aggregate="mean",
            threshold=5.0,
            filter_pattern="^GridMap/",
        )
        self.assertEqual(report["verdict"], "PASS")
        self.assertEqual(len(report["rows"]), 1)
        self.assertAlmostEqual(report["rows"][0]["speedup"], 2.0, places=5)
        self.assertEqual(report["rows"][0]["status"], "IMPROVED")

    def test_rejects_error_occurred_and_invalid_inputs(self) -> None:
        good = self.write_json(
            "good.json",
            {
                "benchmarks": [
                    {
                        "name": "Controller/Mpc_Solve",
                        "run_name": "Controller/Mpc_Solve",
                        "run_type": "iteration",
                        "real_time": 50.0,
                        "cpu_time": 50.0,
                        "time_unit": "us",
                    }
                ]
            },
        )
        errored = self.write_json(
            "errored.json",
            {
                "benchmarks": [
                    {
                        "name": "Controller/Mpc_Solve",
                        "run_name": "Controller/Mpc_Solve",
                        "run_type": "iteration",
                        "error_occurred": True,
                        "error_message": "MPC solve failed",
                    }
                ]
            },
        )
        with self.assertRaises(CompareError):
            compare_reports(good, errored)
        with self.assertRaises(CompareError):
            compare_reports(self.root / "missing.json", good)
        with self.assertRaises(CompareError):
            compare_reports(good, good, threshold=-1.0)
        with self.assertRaises(CompareError):
            compare_reports(good, good, filter_pattern="[unclosed")

    def test_cli_readonly_and_json_out(self) -> None:
        base = self.write_json(
            "b.json",
            {
                "benchmarks": [
                    {
                        "name": "PathPlanning/JPS",
                        "run_name": "PathPlanning/JPS",
                        "run_type": "iteration",
                        "real_time": 1.0,
                        "cpu_time": 1.0,
                        "time_unit": "ms",
                    }
                ]
            },
        )
        before_files = set(self.root.iterdir())
        script = Path(__file__).resolve().parents[1] / "compare_bench.py"
        proc = subprocess.run(
            [sys.executable, str(script), str(base), str(base)],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertIn("PathPlanning/JPS", proc.stdout)
        self.assertEqual(before_files, set(self.root.iterdir()))

        out_json = self.root / "out" / "diff.json"
        proc_out = subprocess.run(
            [sys.executable, str(script), str(base), str(base), "--json-out", str(out_json)],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(proc_out.returncode, 0, proc_out.stderr)
        parsed = json.loads(out_json.read_text(encoding="utf-8"))
        self.assertEqual(parsed["verdict"], "PASS")


if __name__ == "__main__":
    unittest.main()
