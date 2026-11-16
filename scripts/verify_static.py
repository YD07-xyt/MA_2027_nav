#!/usr/bin/env python3
"""Self-contained static entry point. Every subprocess is a read-only tool or Python test."""

import argparse
from collections import Counter
import difflib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

from check_boundaries import (CPP_SUFFIXES, ROOT, collect, compare_baseline, local_file)

BASELINE = ROOT / "scripts/static_baseline.json"


def run(args, **kwargs):
    return subprocess.run(args, cwd=ROOT, check=True, **kwargs)


def require_format():
    if not local_file(ROOT, ".clang-format").is_file():
        raise ValueError("NAV .clang-format missing; parent configuration discovery is disabled")
    executable = shutil.which("clang-format")
    if not executable:
        raise ValueError("required tool missing: clang-format 18")
    version = run([executable, "--version"], capture_output=True, text=True).stdout
    if not re.search(r"version 18\.", version):
        raise ValueError(f"expected clang-format 18, got: {version.strip()}")
    return executable


def git_bytes(*args):
    return run(["git", *args], capture_output=True).stdout


def changed_lines(path, text):
    relative = path.relative_to(ROOT).as_posix()
    # Compare the final working tree with HEAD, covering staged and unstaged edits.
    proc = subprocess.run(["git", "show", f"HEAD:{relative}"], cwd=ROOT, capture_output=True)
    if proc.returncode:
        # Only a genuinely new file may use whole-file checking; Git failure is fatal.
        git_bytes("log", "-1", "--format=%H")
        tracked = git_bytes("ls-tree", "-z", "HEAD", "--", relative)
        if tracked:
            raise ValueError(f"cannot read HEAD version: {relative}")
        return None
    old = proc.stdout.decode("utf-8").splitlines(keepends=True)
    new = text.splitlines(keepends=True)
    changed = set()
    for kind, _, _, start, end in difflib.SequenceMatcher(None, old, new, autojunk=False).get_opcodes():
        if kind != "equal":
            changed.update(range(max(1, start + 1), max(start + 1, end) + 1))
    return changed


def format_findings(executable, path, full):
    data = path.read_bytes()
    if not data:
        return [], 0
    lines = None if full else changed_lines(path, data.decode("utf-8"))
    if lines == set():
        return [], 0
    xml = run([executable, "--output-replacements-xml", str(path)], capture_output=True).stdout
    document = ET.fromstring(xml)
    if document.get("incomplete_format") == "true":
        raise ValueError(f"incomplete clang-format result: {path.relative_to(ROOT)}")
    problems, historical = [], 0
    for replacement in document.findall("replacement"):
        offset, length = int(replacement.attrib["offset"]), int(replacement.attrib["length"])
        start = data[:offset].count(b"\n") + 1
        end = start + data[offset:offset + length].count(b"\n")
        if lines is None or any(line in lines for line in range(start, end + 1)):
            problems.append(f"{path.relative_to(ROOT)}:{start}: formatting")
        else:
            historical += 1
    return problems, historical


def main():
    parser = argparse.ArgumentParser(description="NAV static checks; builds are not implemented or authorized here")
    parser.add_argument("--static-only", action="store_true")
    parser.add_argument("--full-scan", action="store_true")
    parser.add_argument("--report", help="JSON output under .cache/verify/")
    parser.add_argument("--write-baseline", metavar="REASON", help="explicitly record current heuristic debt")
    parser.add_argument("--format", nargs="+", metavar="FILE", help="explicitly format named NAV C++ files")
    args = parser.parse_args()
    if not (ROOT / ".git").exists():
        raise ValueError("NAV root must contain its own .git; parent repository discovery is disabled")
    if args.format and (args.static_only or args.full_scan or args.report or args.write_baseline):
        parser.error("--format cannot be combined with verification options")
    executable = require_format()
    if args.format:
        paths = [local_file(ROOT, path) for path in args.format]
        for path in paths:
            if not path.is_file() or path.suffix not in CPP_SUFFIXES or path.relative_to(ROOT).parts[0] != "src":
                raise ValueError(f"format expects explicit src/ C++ files: {path}")
            if any(part in {"3rd", "build", "install", "log"} for part in path.relative_to(ROOT).parts):
                raise ValueError(f"excluded format path: {path}")
        run([executable, "-i", *map(str, paths)])
        return 0
    files, errors, findings = collect(ROOT)
    git_bytes("status", "--porcelain")  # Root must be a usable Git repository.
    revision = git_bytes("log", "-1", "--format=%H").decode().strip()
    if args.write_baseline:
        local_file(ROOT, BASELINE)
        if errors:
            raise ValueError("syntax errors cannot be baselined: " + "\n".join(errors))
        payload = {"schema": 1, "revision": revision, "reason": args.write_baseline,
                   "entries": dict(Counter(item["key"] for item in findings)), "findings": findings}
        BASELINE.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n")
        print(f"Recorded {len(findings)} heuristic findings in scripts/static_baseline.json; no format/syntax exemptions.")
        return 0
    local_file(ROOT, BASELINE)
    baseline = json.loads(BASELINE.read_text())
    if (baseline.get("schema") != 1 or not isinstance(baseline.get("entries"), dict)
            or any(not isinstance(value, int) or value < 1 for value in baseline["entries"].values())
            or not baseline.get("reason") or not baseline.get("revision")):
        raise ValueError("invalid baseline schema")
    new = compare_baseline(findings, baseline["entries"])
    format_errors, historical = [], 0
    for path in files:
        if path.suffix in CPP_SUFFIXES:
            problems, old = format_findings(executable, path, args.full_scan)
            format_errors.extend(problems)
            historical += old
    # Parse Python without executing launch/doc extraction or generating bytecode.
    import ast
    for path in sorted((ROOT / "scripts").rglob("*.py")):
        local_file(ROOT, path)
        ast.parse(path.read_text(), filename=str(path))
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    if not list((ROOT / "scripts/tests").glob("test_*.py")):
        raise ValueError("no verification tool tests found")
    run([sys.executable, "-m", "unittest", "discover", "-s", "scripts/tests", "-p", "test_*.py"], env=env)
    blocked_findings = findings if args.full_scan else new
    passed = not errors and not format_errors and not blocked_findings
    result = {"status": "PASS" if passed else "NEEDS_FIX", "scope": "full" if args.full_scan else "incremental",
              "revision": revision, "scanned_files": len(files), "syntax_errors": errors,
              "heuristic_findings": findings, "new_findings": new, "format_errors": format_errors,
              "historical_format_replacements_in_changed_files": historical,
              "runtime_tests": "NOT_VERIFIED", "build": "NOT_VERIFIED"}
    print(f"Scanned {len(files)} NAV files; heuristic debt={len(findings)}, new={len(new)}, format failures={len(format_errors)}")
    print(f"Heuristic baseline: {baseline['revision']}; {baseline['reason']}")
    for error in errors + format_errors[:30]:
        print(error)
    for finding in findings[:30]:
        state = "NEW" if finding in new else "BASELINE"
        print(f"[{state}] {finding['path']}:{finding['line']}: {finding['rule']}: {finding['code']}")
    if args.report:
        path = local_file(ROOT, args.report)
        path.relative_to(ROOT / ".cache/verify")
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(f"{result['status']}: {result['scope']} static gate only; build/runtime/sanitizers NOT_VERIFIED.")
    return 0 if passed else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (ValueError, OSError, subprocess.CalledProcessError, ET.ParseError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
