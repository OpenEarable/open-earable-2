#!/usr/bin/env python3
"""Render Twister results as a concise GitHub pull-request comment."""

from __future__ import annotations

import argparse
import html
import json
import re
from pathlib import Path

MARKER = "<!-- openearable-unit-test-results -->"
UNITY_RESULT = re.compile(r"^.*:\d+:test_.+:(?:PASS|FAIL)(?::.*)?$")
IMPORTANT_OUTPUT = re.compile(
    r"(?:\b(?:ERROR|FAIL|FAILED)\b|PROJECT EXECUTION|Tests? \d+ Failures)",
    re.IGNORECASE,
)


def load_suites(report_path: Path) -> list[dict]:
    if not report_path.is_file():
        return []

    try:
        report = json.loads(report_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return []
    return report.get("testsuites", [])


def status_icon(status: str) -> str:
    return {
        "passed": "✅",
        "failed": "❌",
        "error": "🚨",
        "skipped": "⏭️",
        "filtered": "⏭️",
        "blocked": "🚫",
    }.get(status.lower(), "❔")


def failure_details(console_path: Path, suites: list[dict]) -> str:
    console = ""
    if console_path.is_file():
        console = console_path.read_text(encoding="utf-8", errors="replace")

    selected = []
    for line in console.splitlines():
        stripped = line.strip()
        if UNITY_RESULT.match(stripped) or IMPORTANT_OUTPUT.search(stripped):
            selected.append(stripped)

    if not selected:
        for suite in suites:
            if suite.get("status", "").lower() not in {"passed", "skipped", "filtered"}:
                reason = suite.get("reason") or "No detailed failure message was reported."
                selected.append(f"{suite.get('name', 'unknown suite')}: {reason}")

    if not selected and console:
        selected = [line for line in console.splitlines() if line.strip()][-40:]

    # Keep comments readable and below GitHub's comment-size limit.
    return "\n".join(selected[-100:])[-12000:]


def render(args: argparse.Namespace) -> str:
    suites = load_suites(args.report)
    statuses = [str(suite.get("status", "unknown")).lower() for suite in suites]
    failed = sum(status not in {"passed", "skipped", "filtered"} for status in statuses)
    passed = statuses.count("passed")
    skipped = sum(status in {"skipped", "filtered"} for status in statuses)
    succeeded = args.outcome == "success" and failed == 0

    heading = "✅ Unit tests passed" if succeeded else "❌ Unit tests failed"
    lines = [MARKER, f"## {heading}", ""]
    lines.append(
        f"**{passed} passed**, **{failed} failed/error**, **{skipped} skipped** "
        f"— [view workflow run]({args.run_url})"
    )

    if suites:
        lines.extend(["", "| Test scenario | Platform | Result |", "|---|---|---|"])
        for suite in suites:
            status = str(suite.get("status", "unknown"))
            lines.append(
                f"| `{suite.get('name', 'unknown')}` "
                f"| `{suite.get('platform', 'unknown')}` "
                f"| {status_icon(status)} {status} |"
            )
    elif not succeeded:
        lines.extend(["", "Twister did not produce a JSON report; this may be an infrastructure failure."])

    if not succeeded:
        details = failure_details(args.console, suites)
        if details:
            lines.extend(
                [
                    "",
                    "<details open>",
                    "<summary>Failure details</summary>",
                    "",
                    f"<pre>{html.escape(details)}</pre>",
                    "</details>",
                ]
            )

    lines.extend(
        [
            "",
            f"[Download the `unit-test-results` artifact]({args.artifact_url}) "
            "for full Twister reports and logs.",
        ]
    )
    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--console", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--outcome", choices=("success", "failure", "cancelled", "skipped"), required=True)
    parser.add_argument("--run-url", required=True)
    parser.add_argument("--artifact-url", required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output.write_text(render(args), encoding="utf-8")


if __name__ == "__main__":
    main()
