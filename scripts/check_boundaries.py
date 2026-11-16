#!/usr/bin/env python3
"""NAV-local syntax checks and heuristic boundary findings. No build/import of launch."""

import ast
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parent.parent
CPP_SUFFIXES = {".h", ".hpp", ".hh", ".hxx", ".cpp", ".cc", ".cxx"}
EXCLUDED = {".git", "3rd", "build", "install", "log", "__pycache__", ".cache"}
ROS = re.compile(r"\b(?:rclcpp|rcl|ament_\w*|[a-zA-Z0-9_]+_msgs)\b")
HEAVY = re.compile(r'#\s*include\s*[<"](?:pcl/|opencv2/|boost/asio|OsqpEigen/)')
EIGEN = re.compile(r"\bauto\b[^;=]*=\s*[^;]*\.(?:transpose|inverse|block|head|tail|segment|row|col)(?:<[^;]*?>)?\s*\(")


class DuplicateKeyLoader(yaml.SafeLoader):
    pass


def mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise yaml.MarkedYAMLError("mapping", node.start_mark,
                                      f"duplicate key: {key}", key_node.start_mark)
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


DuplicateKeyLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, mapping)


def local_file(root, path):
    path = Path(path)
    if not path.is_absolute():
        path = root / path
    relative = path.relative_to(root)
    if ".." in relative.parts:
        raise ValueError(f"parent traversal in scan path: {relative}")
    current = root
    for part in relative.parts:
        current /= part
        if current.is_symlink():
            raise ValueError(f"symlink in scan path: {relative}")
    path.resolve().relative_to(root.resolve())
    return path


def inventory(root):
    files = []
    for name in ("src", "config", "launch"):
        directory = local_file(root, name)
        if not directory.is_dir():
            raise ValueError(f"required directory missing: {name}")
        selected = []
        for path in directory.rglob("*"):
            relative = path.relative_to(root)
            if any(part in EXCLUDED for part in relative.parts):
                continue
            local_file(root, path)
            if path.is_file() and (path.suffix in CPP_SUFFIXES | {".yaml", ".yml", ".rviz", ".py", ".xml"}
                                   or path.name == "CMakeLists.txt"):
                selected.append(path)
        if not selected:
            raise ValueError(f"empty scan scope: {name}")
        files.extend(selected)
    for name in ("package.xml", "CMakeLists.txt"):
        path = local_file(root, name)
        if not path.is_file():
            raise ValueError(f"required file missing: {name}")
        files.append(path)
    return sorted(files)


def collect(root):
    files = inventory(root)
    errors, findings = [], []
    for path in files:
        relative = path.relative_to(root).as_posix()
        text = path.read_text(encoding="utf-8")
        try:
            if path.suffix in {".yaml", ".yml", ".rviz"}:
                yaml.load(text, Loader=DuplicateKeyLoader)
            elif path.suffix == ".xml":
                ET.fromstring(text)
            elif path.suffix == ".py":
                ast.parse(text, filename=relative)
        except (yaml.YAMLError, SyntaxError, ET.ParseError, ValueError, TypeError) as exc:
            errors.append(f"{relative}: {exc}")
        if path.suffix not in CPP_SUFFIXES and path.name != "CMakeLists.txt":
            continue
        ros_allowed = relative.startswith("src/ros2/") or relative in {"src/ma_nav.cpp", "CMakeLists.txt"}
        for number, line in enumerate(text.splitlines(), 1):
            code = line.split("//", 1)[0]
            if path.name == "CMakeLists.txt":
                code = line.split("#", 1)[0]
            rules = []
            if not ros_allowed and ROS.search(code):
                rules.append("core-ros-reference")
            if path.suffix in CPP_SUFFIXES:
                if "/include/" in relative and not ros_allowed and HEAVY.search(code):
                    rules.append("heavy-public-header")
                if EIGEN.search(code):
                    rules.append("eigen-auto-expression")
            for rule in rules:
                normalized = " ".join(code.split())
                digest = hashlib.sha256(normalized.encode()).hexdigest()[:16]
                findings.append({"key": f"{rule}:{relative}:{digest}", "rule": rule,
                                 "path": relative, "line": number, "code": code.strip()})
    return files, errors, findings


def compare_baseline(findings, baseline):
    remaining = Counter(baseline)
    new = []
    for finding in findings:
        key = finding["key"]
        if remaining[key] > 0:
            remaining[key] -= 1
        else:
            new.append(finding)
    return new


if __name__ == "__main__":
    try:
        files, errors, findings = collect(ROOT)
        print(json.dumps({"files": len(files), "errors": errors, "findings": findings},
                         ensure_ascii=False, indent=2))
        sys.exit(bool(errors or findings))
    except (ValueError, OSError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
