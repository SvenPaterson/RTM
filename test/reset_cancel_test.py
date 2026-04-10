#!/usr/bin/env python3
"""Compatibility wrapper for rig_control reset-cancel mode."""

from __future__ import annotations

import sys
from typing import Sequence

from rig_control import main as rig_main


def main(argv: Sequence[str] | None = None) -> int:
    if argv is None:
        argv = sys.argv[1:]
    return rig_main(["reset-cancel", *argv])


if __name__ == "__main__":
    raise SystemExit(main())
