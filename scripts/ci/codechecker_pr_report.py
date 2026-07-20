#!/usr/bin/env python3
"""Create advisory GitHub output from a CodeChecker JSON report."""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
from pathlib import Path
from typing import Any


MARKER = "<!-- codechecker-advisory-report -->"
SOURCE_SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx"}
DIFF_HEADER = re.compile(r"^\+\+\+ b/(.+)$")
DIFF_RANGE = re.compile(r"^@@ -\d+(?:,\d+)? \+(\d+)(?:,(\d+))? @@")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--analysis-dir", type=Path)
    parser.add_argument("--comment-output", type=Path, required=True)
    parser.add_argument("--base", default="")
    parser.add_argument("--head", default="HEAD")
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    return parser.parse_args()


def changed_lines(repo_root: Path, base: str, head: str) -> dict[str, list[range]]:
    if not base:
        return {}

    command = [
        "git",
        "diff",
        "--unified=0",
        "--no-color",
        f"{base}...{head}",
        "--",
        "*.c",
        "*.cc",
        "*.cpp",
        "*.cxx",
        "*.h",
        "*.hh",
        "*.hpp",
        "*.hxx",
    ]
    result = subprocess.run(
        command,
        cwd=repo_root,
        check=True,
        capture_output=True,
        text=True,
    )

    current_file: str | None = None
    changed: dict[str, list[range]] = {}
    for raw_line in result.stdout.splitlines():
        header = DIFF_HEADER.match(raw_line)
        if header:
            current_file = header.group(1)
            if Path(current_file).suffix.lower() not in SOURCE_SUFFIXES:
                current_file = None
            continue

        hunk = DIFF_RANGE.match(raw_line)
        if current_file and hunk:
            start = int(hunk.group(1))
            count = int(hunk.group(2) or "1")
            if count:
                changed.setdefault(current_file, []).append(range(start, start + count))

    return changed


def report_path(report: dict[str, Any], repo_root: Path) -> str | None:
    file_value = report.get("file", {})
    candidates: list[str] = []
    if isinstance(file_value, dict):
        candidates.extend(
            str(file_value.get(key, "")) for key in ("original_path", "path", "id")
        )
    elif file_value:
        candidates.append(str(file_value))

    repo_root = repo_root.resolve()
    repo_name = repo_root.name
    for candidate in candidates:
        if not candidate:
            continue
        normalized = candidate.replace("\\", "/")
        candidate_path = Path(candidate)
        if candidate_path.is_absolute():
            try:
                return candidate_path.resolve().relative_to(repo_root).as_posix()
            except ValueError:
                pass

        marker = f"/{repo_name}/"
        if marker in normalized:
            return normalized.split(marker, 1)[1]
        if normalized.startswith(f"{repo_name}/"):
            return normalized[len(repo_name) + 1 :]
        if (repo_root / normalized).exists():
            return normalized

    return None


def load_reports(report_file: Path) -> list[dict[str, Any]]:
    if not report_file.is_file():
        return []
    data = json.loads(report_file.read_text(encoding="utf-8"))
    if isinstance(data, dict):
        reports = data.get("reports", [])
    else:
        reports = data
    return [report for report in reports if isinstance(report, dict)]


def escape_annotation(value: str) -> str:
    return (
        value.replace("%", "%25")
        .replace("\r", "%0D")
        .replace("\n", "%0A")
        .replace(":", "%3A")
        .replace(",", "%2C")
    )


def markdown_escape(value: str) -> str:
    return value.replace("|", "\\|").replace("\n", " ").replace("\r", " ")


def main() -> int:
    args = parse_args()
    repo_root = args.repo_root.resolve()
    try:
        changed = changed_lines(repo_root, args.base, args.head)
    except subprocess.CalledProcessError as error:
        print(f"Could not calculate changed lines: {error}")
        changed = {}

    all_reports = load_reports(args.report)
    analyzer_errors = (
        len(list(args.analysis_dir.rglob("*.err")))
        if args.analysis_dir and args.analysis_dir.is_dir()
        else 0
    )
    findings: list[dict[str, Any]] = []
    for report in all_reports:
        path = report_path(report, repo_root)
        line = int(report.get("line") or 1)
        if path not in changed or not any(line in lines for lines in changed[path]):
            continue
        if str(report.get("severity", "")).upper() == "STYLE":
            continue
        finding = dict(report)
        finding["repo_path"] = path
        finding["line"] = line
        findings.append(finding)

    severity_order = {
        "CRITICAL": 0,
        "HIGH": 1,
        "MEDIUM": 2,
        "LOW": 3,
        "UNSPECIFIED": 4,
    }
    findings.sort(
        key=lambda item: (
            severity_order.get(str(item.get("severity", "UNSPECIFIED")).upper(), 5),
            item["repo_path"],
            item["line"],
        )
    )

    for finding in findings[:50]:
        severity = str(finding.get("severity") or "UNSPECIFIED").upper()
        level = "warning" if severity in {"CRITICAL", "HIGH", "MEDIUM"} else "notice"
        checker = str(finding.get("checker_name") or "CodeChecker")
        message = str(finding.get("message") or "Static analysis finding")
        print(
            f"::{level} file={escape_annotation(finding['repo_path'])},"
            f"line={finding['line']},title={escape_annotation(checker)}::"
            f"{escape_annotation(f'[{severity}] {message}')}"
        )

    run_url = os.environ.get("GITHUB_RUN_URL", "")
    report_available = args.report.is_file()
    lines = [
        MARKER,
        "## CodeChecker static analysis (advisory)",
        "",
        "This report is informational and does not block merging. "
        "The complete codebase report is available as the `codechecker-report` workflow artifact.",
        "",
    ]
    if analyzer_errors:
        lines.extend(
            [
                f"**Coverage note:** {analyzer_errors} analyzer invocation(s) could not be "
                "completed. Their diagnostic logs are included in the downloadable artifact.",
                "",
            ]
        )
    if not report_available:
        lines.extend(
            [
                "CodeChecker did not produce a JSON report. Check the downloadable analysis log "
                "for build or analyzer errors.",
                "",
            ]
        )
    elif findings:
        lines.extend(
            [
                f"Found **{len(findings)}** non-style issue(s) on lines changed by this PR.",
                "",
                "| Severity | Location | Analyzer/checker | Message |",
                "| --- | --- | --- | --- |",
            ]
        )
        for finding in findings[:50]:
            severity = markdown_escape(str(finding.get("severity") or "UNSPECIFIED"))
            location = f"`{finding['repo_path']}:{finding['line']}`"
            checker = markdown_escape(str(finding.get("checker_name") or "CodeChecker"))
            message = markdown_escape(str(finding.get("message") or "Static analysis finding"))
            lines.append(f"| {severity} | {location} | `{checker}` | {message} |")
        if len(findings) > 50:
            lines.extend(["", f"Only the first 50 of {len(findings)} findings are shown here."])
        lines.append("")
    else:
        lines.extend(
            [
                "No non-style findings were reported on lines changed by this PR.",
                "",
            ]
        )

    if run_url:
        lines.append(f"[Open this workflow run to download the complete report]({run_url})")
        lines.append("")

    args.comment_output.parent.mkdir(parents=True, exist_ok=True)
    args.comment_output.write_text("\n".join(lines), encoding="utf-8")

    summary_file = os.environ.get("GITHUB_STEP_SUMMARY")
    if summary_file:
        with Path(summary_file).open("a", encoding="utf-8") as summary:
            summary.write("\n".join(lines[1:]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
