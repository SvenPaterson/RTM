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

import pytest


_LOG_ROOT = Path(__file__).resolve().parent / "log"


def pytest_configure(config: pytest.Config) -> None:
    if config.getoption("--log-file", default=None):
        return  # user supplied an explicit log path; respect it
    now = datetime.now()
    day_dir = _LOG_ROOT / f"{now.year:04d}" / f"{now.month:02d}" / f"{now.day:02d}"
    day_dir.mkdir(parents=True, exist_ok=True)
    log_path = day_dir / f"{now.strftime('%H%M%S')}_pytest.log"
    config.option.log_file = str(log_path)
    # Print the path early so the user (or a wrapping script) can find
    # the log without scraping pytest output.
    print(f"\n[harness] Logging session to {log_path}\n")
