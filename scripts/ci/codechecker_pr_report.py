#!/usr/bin/env python3
"""Create advisory GitHub output from a CodeChecker JSON report."""

from __future__ import annotations

import argparse
from collections import Counter
from html import escape as html_escape
import json
import os
import plistlib
import re
import subprocess
from pathlib import Path
from typing import Any
from urllib.parse import quote


MARKER = "<!-- codechecker-advisory-report -->"
SOURCE_SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx"}
DIFF_HEADER = re.compile(r"^\+\+\+ b/(.+)$")
DIFF_RANGE = re.compile(r"^@@ -\d+(?:,\d+)? \+(\d+)(?:,(\d+))? @@")
ANALYZER_RESULT = re.compile(
    r"_(?P<analyzer>[A-Za-z0-9_-]+)_[0-9a-f]+\.plist(?P<failed>\.err)?$"
)
ANSI_ESCAPE = re.compile(r"(?:\\x1b|\x1b)\[[0-9;]*m")
DIAGNOSTIC = re.compile(
    r"(?P<path>[^\r\n]+?):(?P<line>\d+):(?P<column>\d+):\s*"
    r"(?:fatal\s+)?error:\s*(?P<message>[^\r\n]+)"
)
MAX_DETAILS = 100
CONTEXT_LINES = 1
MAX_SNIPPET_LINE_LENGTH = 180


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
            str(file_value.get(key, "")) for key in ("path", "original_path", "id")
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

        if (repo_root / normalized).exists():
            return normalized
        parts = Path(normalized).parts
        for index in range(1, len(parts)):
            suffix = Path(*parts[index:])
            if (repo_root / suffix).exists():
                return suffix.as_posix()
        marker = f"/{repo_name}/"
        if marker in normalized:
            return normalized.rsplit(marker, 1)[1]
        if normalized.startswith(f"{repo_name}/"):
            return normalized[len(repo_name) + 1 :]

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


def analyzer_results(analysis_dir: Path | None) -> tuple[Counter[str], Counter[str]]:
    completed: Counter[str] = Counter()
    failed: Counter[str] = Counter()
    if not analysis_dir or not analysis_dir.is_dir():
        return completed, failed

    for path in analysis_dir.iterdir():
        match = ANALYZER_RESULT.search(path.name)
        if not match:
            continue
        destination = failed if match.group("failed") else completed
        destination[match.group("analyzer")] += 1
    return completed, failed


def analyzer_failures(
    analysis_dir: Path | None, repo_root: Path
) -> list[dict[str, str | int]]:
    failures: list[dict[str, str | int]] = []
    if not analysis_dir or not analysis_dir.is_dir():
        return failures

    for path in sorted(analysis_dir.glob("*.plist.err")):
        match = ANALYZER_RESULT.search(path.name)
        analyzer = match.group("analyzer") if match else "unknown"
        source = path.name.split(f"_{analyzer}_", 1)[0]
        line = 1
        message = "Analyzer invocation failed; see the artifact for diagnostics."
        try:
            data = plistlib.loads(path.read_bytes())
            stderr = ANSI_ESCAPE.sub("", str(data.get("stderr", "")))
            diagnostic = DIAGNOSTIC.search(stderr)
            if diagnostic:
                candidate = report_path(
                    {"file": {"original_path": diagnostic.group("path")}}, repo_root
                )
                source = candidate or Path(diagnostic.group("path")).name
                line = int(diagnostic.group("line"))
                message = diagnostic.group("message").strip()
        except (OSError, plistlib.InvalidFileException, ValueError):
            pass
        failures.append(
            {"analyzer": analyzer, "path": source, "line": line, "message": message}
        )
    return failures


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


def source_url(path: str, line: int, repository: str, revision: str) -> str | None:
    if not repository or not revision:
        return None
    return f"https://github.com/{repository}/blob/{revision}/{quote(path, safe='/')}#L{line}"


def source_location(path: str, line: int, repository: str, revision: str) -> str:
    label = f"{path}:{line}"
    url = source_url(path, line, repository, revision)
    if not url:
        return f"`{label}`"
    return f"[`{label}`]({url})"


def source_location_html(path: str, line: int, repository: str, revision: str) -> str:
    label = html_escape(f"{path}:{line}")
    url = source_url(path, line, repository, revision)
    if not url:
        return f"<code>{label}</code>"
    return f'<a href="{html_escape(url, quote=True)}"><code>{label}</code></a>'


def finding_table(
    reports: list[dict[str, Any]], repository: str, revision: str
) -> list[str]:
    lines = [
        "| Severity | Location | Analyzer/checker | Message |",
        "| --- | --- | --- | --- |",
    ]
    for finding in reports:
        severity = markdown_escape(str(finding.get("severity") or "UNSPECIFIED"))
        location = source_location(
            str(finding["repo_path"]), int(finding["line"]), repository, revision
        )
        analyzer = markdown_escape(str(finding.get("analyzer_name") or "unknown"))
        checker = markdown_escape(str(finding.get("checker_name") or "CodeChecker"))
        message = markdown_escape(str(finding.get("message") or "Static analysis finding"))
        lines.append(f"| {severity} | {location} | `{analyzer}/{checker}` | {message} |")
    return lines


def finding_range(
    finding: dict[str, Any], repo_root: Path
) -> tuple[int, int, int, int]:
    line = int(finding["line"])
    start_col = int(finding.get("column") or 1)
    end_line = line
    end_col = start_col
    for event in reversed(finding.get("bug_path_events") or []):
        if not isinstance(event, dict):
            continue
        event_path = report_path(event, repo_root)
        event_range = event.get("range") or {}
        if event_path != finding["repo_path"] or int(event.get("line") or 0) != line:
            continue
        start_col = int(event_range.get("start_col") or event.get("column") or start_col)
        end_line = int(event_range.get("end_line") or line)
        end_col = int(event_range.get("end_col") or start_col)
        break
    return line, start_col, end_line, end_col


def source_excerpt(finding: dict[str, Any], repo_root: Path) -> str | None:
    source = repo_root / str(finding["repo_path"])
    if not source.is_file():
        return None
    try:
        source_lines = source.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return None

    line, start_col, end_line, end_col = finding_range(finding, repo_root)
    if line < 1 or line > len(source_lines):
        return None
    first = max(1, line - CONTEXT_LINES)
    last = min(len(source_lines), max(line, end_line) + CONTEXT_LINES)
    width = len(str(last))
    excerpt: list[str] = []
    for current in range(first, last + 1):
        prefix = ">" if line <= current <= end_line else " "
        rendered = source_lines[current - 1].expandtabs(4)
        if len(rendered) > MAX_SNIPPET_LINE_LENGTH:
            rendered = f"{rendered[: MAX_SNIPPET_LINE_LENGTH - 3]}..."
        excerpt.append(f"{prefix} {current:>{width}} | {rendered}")
        if current == line:
            original = source_lines[current - 1]
            expanded_prefix = original[: max(0, start_col - 1)].expandtabs(4)
            if end_line == line:
                selected = original[
                    max(0, start_col - 1) : max(start_col, end_col)
                ].expandtabs(4)
                marker_width = max(1, len(selected))
            else:
                marker_width = max(1, len(rendered) - len(expanded_prefix))
            excerpt.append(
                f"  {'':>{width}} | {' ' * len(expanded_prefix)}{'^' * marker_width}"
            )
    return "\n".join(excerpt)


def context_section(
    reports: list[dict[str, Any]],
    repo_root: Path,
    repository: str,
    revision: str,
) -> list[str]:
    details = reports[:MAX_DETAILS]
    if not details:
        return []

    lines = [
        "### Complete-codebase findings",
        "",
        "Each finding includes source context. The `>` line and carets identify the "
        "expression reported by the analyzer.",
        "",
        "<details>",
        f"<summary><strong>Show detailed findings with source context ({len(details)}"
        f" of {len(reports)})</strong></summary>",
        "",
    ]
    for finding in details:
        severity = html_escape(str(finding.get("severity") or "UNSPECIFIED"))
        message = html_escape(str(finding.get("message") or "Static analysis finding"))
        location = source_location_html(
            str(finding["repo_path"]), int(finding["line"]), repository, revision
        )
        excerpt = source_excerpt(finding, repo_root)
        lines.extend(
            [
                "<details>",
                f"<summary><strong>{severity}</strong> — {location} — {message}</summary>",
                "",
            ]
        )
        if excerpt:
            lines.extend(["```text", excerpt, "```", ""])
        else:
            lines.extend(["Source context is unavailable in the workflow checkout.", ""])
        lines.extend(["</details>", ""])
    lines.extend(["</details>", ""])
    return lines


def main() -> int:
    args = parse_args()
    repo_root = args.repo_root.resolve()
    try:
        changed = changed_lines(repo_root, args.base, args.head)
    except subprocess.CalledProcessError as error:
        print(f"Could not calculate changed lines: {error}")
        changed = {}

    all_reports = [
        report
        for report in load_reports(args.report)
        if str(report.get("severity", "")).upper() != "STYLE"
    ]
    completed_runs, failed_runs = analyzer_results(args.analysis_dir)
    failures = analyzer_failures(args.analysis_dir, repo_root)
    analyzer_errors = sum(failed_runs.values())
    findings: list[dict[str, Any]] = []
    for report in all_reports:
        path = report_path(report, repo_root)
        line = int(report.get("line") or 1)
        if path not in changed or not any(line in lines for lines in changed[path]):
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

    normalized_reports: list[dict[str, Any]] = []
    for report in all_reports:
        path = report_path(report, repo_root)
        if not path:
            continue
        normalized = dict(report)
        normalized["repo_path"] = path
        normalized["line"] = int(report.get("line") or 1)
        normalized_reports.append(normalized)
    normalized_reports.sort(
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
    repository = os.environ.get("GITHUB_REPOSITORY", "")
    revision = args.head
    report_available = args.report.is_file()
    severity_counts = Counter(
        str(report.get("severity") or "UNSPECIFIED").upper()
        for report in normalized_reports
    )
    analyzer_counts = Counter(
        str(report.get("analyzer_name") or "unknown") for report in normalized_reports
    )
    lines = [
        MARKER,
        "## CodeChecker static analysis (advisory)",
        "",
        "> [!NOTE]",
        "> This report is informational. It does not block merging and does not change code.",
        "",
        "### At a glance",
        "",
        "| Scope | Result |",
        "| --- | ---: |",
        f"| Findings on lines changed by this PR | **{len(findings)}** |",
        f"| Findings in the complete codebase | **{len(normalized_reports)}** |",
        f"| Analyzer invocations completed | **{sum(completed_runs.values())}** |",
        f"| Analyzer invocations failed | **{analyzer_errors}** |",
        "",
    ]
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
                "### Findings on changed lines",
                "",
                f"Found **{len(findings)}** non-style issue(s) on lines changed by this PR.",
                "",
            ]
        )
        lines.extend(finding_table(findings[:50], repository, revision))
        if len(findings) > 50:
            lines.extend(["", f"Only the first 50 of {len(findings)} findings are shown here."])
        lines.append("")
    else:
        lines.extend(
            [
                "### Findings on changed lines",
                "",
                "No non-style findings were reported on lines changed by this PR.",
                "",
            ]
        )

    if completed_runs or failed_runs:
        lines.extend(
            [
                "### Analyzer coverage",
                "",
                "| Analyzer | Completed | Failed |",
                "| --- | ---: | ---: |",
            ]
        )
        for analyzer in sorted(completed_runs.keys() | failed_runs.keys()):
            lines.append(
                f"| `{analyzer}` | {completed_runs[analyzer]} | {failed_runs[analyzer]} |"
            )
        lines.append("")

    if failures:
        lines.extend(
            [
                "<details>",
                f"<summary><strong>{len(failures)} failed analyzer invocation(s)</strong></summary>",
                "",
                "These files received reduced analysis coverage. The first compiler diagnostic "
                "from each invocation is shown below.",
                "The reason is that the clang frontend was not able to link them because of issues like implicit declaration. Target gcc build may still succeed.",
                "",
                "| Analyzer | Source | First diagnostic |",
                "| --- | --- | --- |",
            ]
        )
        for failure in failures:
            location = source_location(
                str(failure["path"]), int(failure["line"]), repository, revision
            )
            lines.append(
                f"| `{markdown_escape(str(failure['analyzer']))}` | {location} | "
                f"{markdown_escape(str(failure['message']))} |"
        )
        lines.extend(["", "</details>", ""])

    if report_available:
        severity_summary = ", ".join(
            f"**{severity_counts.get(severity, 0)} {severity.lower()}**"
            for severity in ("CRITICAL", "HIGH", "MEDIUM", "LOW", "UNSPECIFIED")
            if severity_counts.get(severity, 0)
        ) or "no findings"
        analyzer_summary = ", ".join(
            f"`{name}`: {count}" for name, count in sorted(analyzer_counts.items())
        ) or "none"
        lines.extend(
            [
                "### Complete-codebase summary",
                "",
                f"Severity: {severity_summary}  ",
                f"Reported by analyzer: {analyzer_summary}",
                "",
            ]
        )
        lines.extend(
            context_section(normalized_reports, repo_root, repository, revision)
        )

    if run_url:
        lines.append(
            "Open the workflow run and download the complete CodeChecker "
            f"[report]({run_url}) for a detailed view. Check out "
            "`codechecker.html/index.html`."
        )
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
