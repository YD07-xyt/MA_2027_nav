#!/usr/bin/env python3
"""Compare two Google Benchmark JSON reports (baseline vs candidate).

Read-only by default; prints a GitHub-flavored Markdown comparison table to stdout.
Uses Python 3.10+ standard library only.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import statistics
import sys
from typing import Any

UNIT_TO_NS: dict[str, float] = {
    "ns": 1.0,
    "us": 1e3,
    "ms": 1e6,
    "s": 1e9,
}

AGGREGATE_SUFFIXES = ("_mean", "_median", "_stddev", "_cv")


class CompareError(ValueError):
    """Raised when input files or arguments are invalid."""


def to_ns(value: Any, unit: Any, label: str) -> float:
    if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise CompareError(f"invalid numeric metric for {label}: {value!r}")
    if unit not in UNIT_TO_NS:
        raise CompareError(f"unsupported time_unit for {label}: {unit!r}")
    return float(value) * UNIT_TO_NS[unit]


def format_ns_pair(base_ns: float | None, cand_ns: float | None) -> tuple[str, str]:
    ref = base_ns if (base_ns is not None and base_ns > 0.0) else (cand_ns or 0.0)
    if ref >= 1e9:
        scale, suffix = 1e9, "s"
    elif ref >= 1e6:
        scale, suffix = 1e6, "ms"
    elif ref >= 1e3:
        scale, suffix = 1e3, "us"
    else:
        scale, suffix = 1.0, "ns"

    def fmt(val: float | None) -> str:
        if val is None:
            return "-"
        return f"{val / scale:.3f} {suffix}"

    return fmt(base_ns), fmt(cand_ns)


def normalize_run_name(entry: dict[str, Any]) -> str:
    run_name = entry.get("run_name")
    if isinstance(run_name, str) and run_name:
        return run_name
    name = entry.get("name")
    if not isinstance(name, str) or not name:
        raise CompareError(f"benchmark entry missing name/run_name: {entry!r}")
    for suffix in AGGREGATE_SUFFIXES:
        if name.endswith(suffix):
            return name[: -len(suffix)]
    return name


def load_benchmark_json(path: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    if not path.is_file():
        raise CompareError(f"benchmark JSON not found: {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CompareError(f"failed to parse JSON {path}: {exc}") from exc
    if not isinstance(payload, dict) or not isinstance(payload.get("benchmarks"), list):
        raise CompareError(f"invalid Google Benchmark JSON schema in {path}: missing 'benchmarks' list")
    context = payload.get("context", {})
    if not isinstance(context, dict):
        context = {}
    return context, payload["benchmarks"]


def extract_series(
    entries: list[dict[str, Any]],
    metric: str,
    aggregate: str,
    name_filter: re.Pattern[str] | None,
    source_label: str,
) -> dict[str, dict[str, Any]]:
    grouped: dict[str, dict[str, Any]] = {}
    for entry in entries:
        if not isinstance(entry, dict):
            raise CompareError(f"invalid benchmark entry in {source_label}: {entry!r}")
        run_name = normalize_run_name(entry)
        if name_filter and not name_filter.search(run_name):
            continue
        if entry.get("error_occurred"):
            msg = entry.get("error_message", "unknown error")
            raise CompareError(f"benchmark error in {source_label} [{run_name}]: {msg}")

        bucket = grouped.setdefault(run_name, {"iterations": [], "aggregates": {}})
        run_type = entry.get("run_type", "iteration")
        if run_type == "aggregate":
            agg_name = entry.get("aggregate_name")
            if agg_name in ("median", "mean") and metric in entry:
                bucket["aggregates"][agg_name] = to_ns(entry[metric], entry.get("time_unit"), run_name)
            elif agg_name == "cv" and metric in entry and isinstance(entry[metric], (int, float)):
                bucket["aggregates"]["cv"] = float(entry[metric])
        elif run_type == "iteration":
            if metric not in entry:
                raise CompareError(f"metric '{metric}' missing in {source_label} [{run_name}]")
            bucket["iterations"].append(to_ns(entry[metric], entry.get("time_unit"), run_name))

    result: dict[str, dict[str, Any]] = {}
    for run_name, bucket in grouped.items():
        if aggregate in bucket["aggregates"]:
            value_ns = bucket["aggregates"][aggregate]
            source = f"aggregate:{aggregate}"
        elif bucket["iterations"]:
            if aggregate == "mean":
                value_ns = statistics.fmean(bucket["iterations"])
            else:
                value_ns = statistics.median(bucket["iterations"])
            source = f"iterations:{aggregate}"
        else:
            raise CompareError(f"no usable samples for {source_label} [{run_name}]")
        result[run_name] = {
            "value_ns": value_ns,
            "samples": len(bucket["iterations"]),
            "source": source,
            "cv": bucket["aggregates"].get("cv"),
        }
    return result


def check_context_warnings(base_ctx: dict[str, Any], cand_ctx: dict[str, Any]) -> list[str]:
    warnings: list[str] = []
    for key in ("library_build_type", "host_name", "num_cpus"):
        if key in base_ctx and key in cand_ctx and base_ctx[key] != cand_ctx[key]:
            warnings.append(f"context mismatch for '{key}': baseline={base_ctx[key]!r}, candidate={cand_ctx[key]!r}")
    if base_ctx.get("cpu_scaling_enabled") or cand_ctx.get("cpu_scaling_enabled"):
        warnings.append("cpu_scaling_enabled=true detected; small deltas (<5%) may reflect CPU frequency jitter")
    return warnings


def compare_reports(
    baseline_path: Path,
    candidate_path: Path,
    metric: str = "real_time",
    aggregate: str = "median",
    threshold: float = 5.0,
    filter_pattern: str | None = None,
) -> dict[str, Any]:
    if threshold < 0.0 or not math.isfinite(threshold):
        raise CompareError(f"threshold must be finite and >= 0, got {threshold}")
    compiled_filter = None
    if filter_pattern:
        try:
            compiled_filter = re.compile(filter_pattern)
        except re.error as exc:
            raise CompareError(f"invalid regex --filter {filter_pattern!r}: {exc}") from exc

    base_ctx, base_entries = load_benchmark_json(baseline_path)
    cand_ctx, cand_entries = load_benchmark_json(candidate_path)
    base_series = extract_series(base_entries, metric, aggregate, compiled_filter, str(baseline_path))
    cand_series = extract_series(cand_entries, metric, aggregate, compiled_filter, str(candidate_path))

    all_names = list(dict.fromkeys([*base_series.keys(), *cand_series.keys()]))
    rows: list[dict[str, Any]] = []
    speedup_logs: list[float] = []
    counts = {"matched": 0, "improved": 0, "regressed": 0, "neutral": 0, "added": 0, "removed": 0}

    for name in all_names:
        in_base = name in base_series
        in_cand = name in cand_series
        if in_base and in_cand:
            counts["matched"] += 1
            base_ns = base_series[name]["value_ns"]
            cand_ns = cand_series[name]["value_ns"]
            if base_ns > 0.0 and cand_ns > 0.0:
                delta_pct = (cand_ns - base_ns) / base_ns * 100.0
                speedup = base_ns / cand_ns
                speedup_logs.append(math.log(speedup))
            elif base_ns == cand_ns:
                delta_pct = 0.0
                speedup = 1.0
                speedup_logs.append(0.0)
            else:
                delta_pct = math.inf if cand_ns > base_ns else -100.0
                speedup = 0.0 if cand_ns > base_ns else math.inf

            if delta_pct <= -threshold:
                status = "IMPROVED"
                counts["improved"] += 1
            elif delta_pct >= threshold:
                status = "REGRESSED"
                counts["regressed"] += 1
            else:
                status = "NEUTRAL"
                counts["neutral"] += 1

            rows.append(
                {
                    "name": name,
                    "baseline_ns": base_ns,
                    "candidate_ns": cand_ns,
                    "delta_pct": delta_pct,
                    "speedup": speedup,
                    "status": status,
                }
            )
        elif in_cand:
            counts["added"] += 1
            rows.append(
                {
                    "name": name,
                    "baseline_ns": None,
                    "candidate_ns": cand_series[name]["value_ns"],
                    "delta_pct": None,
                    "speedup": None,
                    "status": "ADDED",
                }
            )
        else:
            counts["removed"] += 1
            rows.append(
                {
                    "name": name,
                    "baseline_ns": base_series[name]["value_ns"],
                    "candidate_ns": None,
                    "delta_pct": None,
                    "speedup": None,
                    "status": "REMOVED",
                }
            )

    geomean_speedup = math.exp(statistics.fmean(speedup_logs)) if speedup_logs else None
    if counts["matched"] == 0:
        verdict = "NO_MATCHED_BENCHMARKS"
    elif counts["regressed"] > 0:
        verdict = "REGRESSION_DETECTED"
    else:
        verdict = "PASS"

    return {
        "baseline": str(baseline_path),
        "candidate": str(candidate_path),
        "metric": metric,
        "aggregate": aggregate,
        "threshold_pct": threshold,
        "filter": filter_pattern,
        "warnings": check_context_warnings(base_ctx, cand_ctx),
        "counts": counts,
        "geomean_speedup": geomean_speedup,
        "verdict": verdict,
        "rows": rows,
    }


def render_markdown(report: dict[str, Any]) -> str:
    lines: list[str] = []
    lines.append(
        f"Benchmark Comparison ({report['metric']}, {report['aggregate']}, threshold=+/-{report['threshold_pct']:.1f}%)"
    )
    lines.append(f"- Baseline : `{report['baseline']}`")
    lines.append(f"- Candidate: `{report['candidate']}`")
    if report["filter"]:
        lines.append(f"- Filter   : `{report['filter']}`")
    for warn in report["warnings"]:
        lines.append(f"- [WARN] {warn}")
    lines.append("")
    lines.append("| Benchmark | Baseline | Candidate | Delta (%) | Speedup | Status |")
    lines.append("| :--- | ---: | ---: | ---: | ---: | :--- |")

    for row in report["rows"]:
        base_str, cand_str = format_ns_pair(row["baseline_ns"], row["candidate_ns"])
        if row["delta_pct"] is None:
            delta_str = "-"
            speedup_str = "-"
        else:
            delta_str = f"{row['delta_pct']:+.2f}%"
            speedup_str = f"{row['speedup']:.2f}x"
        lines.append(
            f"| `{row['name']}` | {base_str} | {cand_str} | {delta_str} | {speedup_str} | {row['status']} |"
        )

    counts = report["counts"]
    gm = f"{report['geomean_speedup']:.2f}x" if report["geomean_speedup"] is not None else "N/A"
    lines.append("")
    lines.append(
        f"Summary: verdict={report['verdict']}, matched={counts['matched']}, "
        f"improved={counts['improved']}, regressed={counts['regressed']}, "
        f"neutral={counts['neutral']}, added={counts['added']}, removed={counts['removed']}, "
        f"geomean_speedup={gm}"
    )
    return "\n".join(lines)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Compare two Google Benchmark JSON files (before vs after code changes)."
    )
    parser.add_argument("baseline", type=Path, help="Path to baseline (before) Google Benchmark JSON.")
    parser.add_argument("candidate", type=Path, help="Path to candidate (after) Google Benchmark JSON.")
    parser.add_argument(
        "--metric",
        choices=("real_time", "cpu_time"),
        default="real_time",
        help="Timing field to compare (default: real_time).",
    )
    parser.add_argument(
        "--aggregate",
        choices=("median", "mean"),
        default="median",
        help="Statistic to use when multiple repetitions exist (default: median).",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=5.0,
        help="Percentage threshold for IMPROVED/REGRESSED status (default: 5.0).",
    )
    parser.add_argument(
        "--filter",
        default=None,
        help="Optional regex to filter benchmark names.",
    )
    parser.add_argument(
        "--fail-on-regression",
        action="store_true",
        help="Return exit code 1 if any benchmark regresses beyond --threshold or no benchmarks match.",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=None,
        help="Optional path to write machine-readable comparison JSON.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        report = compare_reports(
            baseline_path=args.baseline,
            candidate_path=args.candidate,
            metric=args.metric,
            aggregate=args.aggregate,
            threshold=args.threshold,
            filter_pattern=args.filter,
        )
    except CompareError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    print(render_markdown(report))
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    if args.fail_on_regression and report["verdict"] != "PASS":
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
