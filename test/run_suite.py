#!/usr/bin/env python3
"""
Automated firmware test-suite runner.

Drives the individual rig_control.py verbs in sequence, collects verdicts,
and prints a summary table.  Supports --quick (fast subset) and --full
(all automated tests including cold-boot).

Requires --port to specify the sniffer COM port.

Usage:
    python test/run_suite.py --port COM7
    python test/run_suite.py --port COM7 --quick
    python test/run_suite.py --port COM7 --full
"""

import argparse
import datetime
import subprocess
import sys
import time
from pathlib import Path
from typing import IO

# ---------------------------------------------------------------------------
# Test definitions — (id, name, verb + extra args)
# ---------------------------------------------------------------------------

# Quick tests — no power-cycle, safe to run while hardware is up.
QUICK_TESTS = [
    ("T8", "comms-health",   ["comms-health", "--duration-s", "15"]),
    ("T2", "run-cycle",      ["run-cycle"]),
    ("T5", "reset-cancel",   ["reset-cancel", "--count", "5"]),
]

# Extended tests — includes power-cycle and cold-boot.
EXTENDED_TESTS = [
    ("T4", "reset-pulse",    ["reset-pulse"]),
    ("T6", "reset-multi",    ["reset-pulse-multi", "--count", "3"]),
    ("T7", "run-gate",       ["run-gate"]),
    ("T1", "cold-boot",      ["cold-boot"]),
]

# Manual-interactive tests — require operator action (SD swap, visual check).
# Run these individually: python test/rig_control.py protocol-upload --port COM7
INTERACTIVE_TESTS = [
    ("T3",  "protocol-upload", ["protocol-upload"]),
    ("T10", "heat-flags",      ["heat-flags"]),
    ("T11", "heat-lifecycle",  ["heat-lifecycle"]),
]

RIG_CONTROL = str(Path(__file__).resolve().parent / "rig_control.py")
LOG_DIR     = Path(__file__).resolve().parent / "log"

POWER_OFF_DWELL_S = 3        # seconds to hold power off
BOOT_SETTLE_S     = 25       # seconds for QUIESCE + protocol load


def _timestamp() -> str:
    return datetime.datetime.now().strftime("%Y%m%d-%H%M%S")


def _emit(msg: str, log: IO[str] | None = None) -> None:
    """Print to stdout and optionally mirror to the master log file."""
    print(msg)
    if log is not None:
        log.write(msg + "\n")
        log.flush()

def establish_known_state(common_args: list[str]) -> bool:
    """Power-cycle the rig to start from a clean, known state.

    Returns True if the preamble succeeded (both commands exit 0).
    """
    print("=" * 60)
    print("  PREAMBLE — Establishing known state (power cycle)")
    print("=" * 60)

    # Power OFF
    cmd_off = [sys.executable, RIG_CONTROL] + common_args + [
        "power", "--state", "off", "--capture-s", str(POWER_OFF_DWELL_S),
    ]
    print(f"  CMD: {' '.join(cmd_off)}")
    rc = subprocess.run(cmd_off, timeout=30).returncode
    if rc != 0:
        print(f"  ✗ Power-off failed (exit {rc})")
        return False

    # Power ON and let the system boot fully
    cmd_on = [sys.executable, RIG_CONTROL] + common_args + [
        "power", "--state", "on", "--capture-s", str(BOOT_SETTLE_S),
    ]
    print(f"\n  CMD: {' '.join(cmd_on)}")
    result = subprocess.run(cmd_on, timeout=BOOT_SETTLE_S + 30,
                            capture_output=True, text=True)
    # Echo captured output so operator can see it
    if result.stdout:
        print(result.stdout, end="")
    if result.returncode != 0:
        print(f"  ✗ Power-on failed (exit {result.returncode})")
        return False

    # Verify protocol loaded — abort suite immediately if SD/protocol missing
    boot_output = result.stdout or ""
    if "Protocol Missing" in boot_output or "PROTO_RX=OK" not in boot_output:
        print("\n  ✗ PROTOCOL NOT LOADED — SD card missing or protocol file not found.")
        print("    Insert SD card with a valid protocol CSV and retry.")
        print("    Aborting test suite.")
        return False

    print("\n  ✓ Rig is in known state (powered, booted, protocol loaded)")
    print("=" * 60)
    return True


def run_test(test_id: str, label: str, verb_args: list[str],
             common_args: list[str],
             log: IO[str] | None = None) -> tuple[str, str, float]:
    """Run a single test verb and return (test_id, result, elapsed_s)."""
    cmd = [sys.executable, RIG_CONTROL] + common_args + verb_args
    _emit(f"\n{'=' * 60}", log)
    _emit(f"  [{test_id}] {label}", log)
    _emit(f"  CMD: {' '.join(cmd)}", log)
    _emit(f"{'=' * 60}\n", log)

    t0 = time.monotonic()
    try:
        result = subprocess.run(cmd, timeout=600)
        elapsed = time.monotonic() - t0
        status = "PASS" if result.returncode == 0 else "FAIL"
    except subprocess.TimeoutExpired:
        elapsed = time.monotonic() - t0
        status = "TIMEOUT"
    except Exception as exc:  # noqa: BLE001
        elapsed = time.monotonic() - t0
        _emit(f"  ERROR: {exc}", log)
        status = "ERROR"

    tag = status
    _emit(f"\n  >>> [{test_id}] {label}: {tag}  ({elapsed:.1f}s)", log)
    return test_id, tag, elapsed


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the automated firmware test suite.",
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--quick", action="store_true",
        help="Run only the quick subset (no power-cycle tests).",
    )
    group.add_argument(
        "--full", action="store_true",
        help="Run the full suite including cold-boot and multi-reset.",
    )
    parser.add_argument(
        "--port", type=str, required=True,
        help="Sniffer serial port (e.g. COM7).",
    )
    parser.add_argument(
        "--baud", type=int, default=None,
        help="USB CDC baud rate (passed through to rig_control).",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    # Default to --quick when neither flag is specified.
    if not args.quick and not args.full:
        args.quick = True

    # ---- Master log setup ----
    now = datetime.datetime.now()
    day_dir = LOG_DIR / f"{now.year}" / f"{now.month:02d}" / f"{now.day:02d}"
    day_dir.mkdir(parents=True, exist_ok=True)
    mode_tag = "full" if args.full else "quick"
    log_path = day_dir / f"{_timestamp()}_suite_{mode_tag}.log"
    log_file = open(log_path, "w", encoding="utf-8")

    _emit(f"[SUITE] Logging to {log_path}", log_file)
    _emit(f"[SUITE] Mode: {mode_tag}", log_file)

    cmd_port = args.port
    _emit(f"[SUITE] Sniffer port: {cmd_port}", log_file)

    # Build common args forwarded to every rig_control invocation.
    common: list[str] = ["--port", cmd_port]
    if args.baud:
        common += ["--baud", str(args.baud)]

    tests = list(QUICK_TESTS)
    if args.full:
        tests += EXTENDED_TESTS

    # ---- Preamble: power-cycle to known state ----
    if not establish_known_state(common):
        _emit("\n  ✗ PREAMBLE FAILED — aborting suite", log_file)
        log_file.close()
        return 1

    results: list[tuple[str, str, str, float]] = []

    for test_id, label, verb_args in tests:
        tid, status, elapsed = run_test(test_id, label, verb_args, common,
                                        log=log_file)
        results.append((tid, label, status, elapsed))

    # ---- Summary ----
    _emit("\n", log_file)
    _emit("=" * 60, log_file)
    _emit("  TEST SUITE SUMMARY", log_file)
    _emit("=" * 60, log_file)
    total_time = sum(r[3] for r in results)
    passes = sum(1 for r in results if r[2] == "PASS")
    fails = len(results) - passes

    for tid, label, status, elapsed in results:
        marker = "✓" if status == "PASS" else "✗"
        _emit(f"  {marker}  [{tid}] {label:<20s}  {status:<8s}  {elapsed:6.1f}s",
              log_file)

    _emit(f"\n  {passes} passed, {fails} failed  —  {total_time:.1f}s total",
          log_file)
    _emit("=" * 60, log_file)

    _emit(f"\n[SUITE] Master log saved to {log_path}", log_file)
    log_file.close()

    return 0 if fails == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
