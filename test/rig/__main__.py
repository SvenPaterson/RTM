"""Tiny CLI for ad-hoc rig actuation via the Teensy.

Usage from repo root (with venv active):

    python -m test.rig run        # latch RUN low (start / resume)
    python -m test.rig stop       # release RUN (pause)
    python -m test.rig reset      # 6 s RST pulse (logical reset on CC)
    python -m test.rig power on   # PSU relay on
    python -m test.rig power off  # PSU relay off
    python -m test.rig pulse-run 750
    python -m test.rig pulse-rst 6000

All commands exit non-zero on serial errors.
"""

from __future__ import annotations

import argparse
import logging
import sys

from test.rig.teensy import Teensy


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="python -m test.rig")
    p.add_argument("--port", help="Override Teensy COM port (else auto-detect).")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("run",   help="Latch RUN low (start / resume).")
    sub.add_parser("stop",  help="Release RUN (pause).")

    rst = sub.add_parser("reset", help="Drive RST high for ms (default 6000 = logical reset).")
    rst.add_argument("ms", nargs="?", type=int, default=6000)

    pr = sub.add_parser("pulse-run", help="Pulse RUN high for ms.")
    pr.add_argument("ms", type=int)

    prs = sub.add_parser("pulse-rst", help="Pulse RST high for ms.")
    prs.add_argument("ms", type=int)

    pwr = sub.add_parser("power", help="Switch the rig PSU relay.")
    pwr.add_argument("state", choices=("on", "off"))

    return p


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    args = _build_parser().parse_args(argv)

    # safe_park_on_close=False: ad-hoc CLI users want `run` to actually
    # latch RUN=1 after the process exits. (Pytest fixtures keep the
    # default True so the rig parks safely between tests.)
    with Teensy(port=args.port, safe_park_on_close=False) as t:
        if args.cmd == "run":
            t.set_run(1)
        elif args.cmd == "stop":
            t.set_run(0)
        elif args.cmd == "reset":
            t.pulse_reset(args.ms)
        elif args.cmd == "pulse-run":
            t.pulse_run(args.ms)
        elif args.cmd == "pulse-rst":
            t.pulse_reset(args.ms)
        elif args.cmd == "power":
            t.set_power(args.state == "on")
    return 0


if __name__ == "__main__":
    sys.exit(main())
