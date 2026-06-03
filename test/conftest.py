"""Top-level pytest hooks for the RTM harness.

Writes every session's output to a timestamped log file under
``test/log/<YYYY>/<MM>/<DD>/<HHMMSS>_pytest.log`` so a non-interactive
runner (or post-mortem inspection) can read what happened without
re-running the rig.

The file path is computed in ``pytest_configure`` and stuffed into
``--log-file`` if the user hasn't already supplied one.
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import sys

import pytest


_LOG_ROOT = Path(__file__).resolve().parent / "log"


def pytest_addoption(parser: pytest.Parser) -> None:
    parser.addoption(
        "--no-smoke-gate",
        action="store_true",
        default=False,
        help=(
            "Continue the selected suite even if a smoke-marked scenario "
            "fails or skips. By default, smoke failures stop the run."
        ),
    )


def pytest_configure(config: pytest.Config) -> None:
    if config.getoption("--log-file", default=None):
        config._rtm_log_path = config.getoption("--log-file")  # type: ignore[attr-defined]
        return  # user supplied an explicit log path; respect it
    now = datetime.now()
    day_dir = _LOG_ROOT / f"{now.year:04d}" / f"{now.month:02d}" / f"{now.day:02d}"
    day_dir.mkdir(parents=True, exist_ok=True)
    log_path = day_dir / f"{now.strftime('%H%M%S')}_pytest.log"
    config.option.log_file = str(log_path)
    config._rtm_log_path = str(log_path)  # type: ignore[attr-defined]
    # Print the path early so the user (or a wrapping script) can find
    # the log without scraping pytest output.
    print(f"\n[harness] Logging session to {log_path}\n")


def pytest_terminal_summary(
    terminalreporter: pytest.TerminalReporter,
    exitstatus: int,
    config: pytest.Config,
) -> None:
    log_path = getattr(config, "_rtm_log_path", None)
    if not log_path:
        return
    stats = terminalreporter.stats
    summary_keys = (
        "passed", "failed", "error", "skipped", "xfailed", "xpassed",
        "deselected",
    )
    lines = _session_lines(config) + [
        "",
        "===== RTM pytest summary =====",
        f"exitstatus: {exitstatus}",
    ]
    for key in summary_keys:
        reports = stats.get(key, [])
        if reports:
            lines.append(f"{key}: {len(reports)}")
    failed_reports = stats.get("failed", []) + stats.get("error", [])
    if failed_reports:
        lines.append("failures:")
        for report in failed_reports:
            nodeid = getattr(report, "nodeid", "(unknown)")
            lines.append(f"- {nodeid}")
            crash = getattr(getattr(report, "longrepr", None), "reprcrash", None)
            if crash is not None:
                lines.append(f"  {crash.message}")
    skipped_reports = stats.get("skipped", [])
    if skipped_reports:
        lines.append("skips:")
        for report in skipped_reports:
            lines.append(f"- {getattr(report, 'nodeid', '(unknown)')}")
    lines.extend(["==============================", ""])
    _append_lines(log_path, lines)


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_makereport(
    item: pytest.Item,
    call: pytest.CallInfo[object],
) -> None:
    outcome = yield
    report = outcome.get_result()
    if _smoke_gate_tripped(item, report):
        item.session.shouldstop = (
            "smoke gate stopped the suite after "
            f"{report.outcome}: {item.nodeid}"
        )


def _smoke_gate_tripped(item: pytest.Item, report: pytest.TestReport) -> bool:
    if item.config.getoption("--no-smoke-gate", default=False):
        return False
    if item.get_closest_marker("smoke") is None:
        return False
    return bool(report.failed or report.skipped)


def _append_lines(log_path: str, lines: list[str]) -> None:
    with Path(log_path).open("a", encoding="utf-8") as handle:
        for line in lines:
            handle.write(line + "\n")


def _session_lines(config: pytest.Config) -> list[str]:
    args = " ".join(sys.argv)
    markexpr = config.getoption("markexpr", default="")
    keyword = config.getoption("keyword", default="")
    return [
        "",
        "===== RTM pytest session =====",
        f"command: {args}",
        f"markexpr: {markexpr or '(none)'}",
        f"keyword: {keyword or '(none)'}",
        "==============================",
        "",
    ]
