#!/usr/bin/env python3
"""
Automated firmware test-suite runner.

Drives the individual rig_control.py verbs in sequence, collects verdicts,
and prints a summary table.  Supports --quick (fast subset) and --full
(all automated tests including cold-boot).

Auto-detects ClearCore, Expansion Board, and Sniffer by USB VID:PID.
Use --port to override the ClearCore port if needed.

Usage:
    python test/run_suite.py
    python test/run_suite.py --quick
    python test/run_suite.py --full
    python test/run_suite.py --port COM8 --full
"""

import argparse
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

import serial.tools.list_ports

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
    ("T3", "protocol-upload", ["protocol-upload"]),
]

RIG_CONTROL = str(Path(__file__).resolve().parent / "rig_control.py")

POWER_OFF_DWELL_S = 3        # seconds to hold power off
BOOT_SETTLE_S     = 25       # seconds for QUIESCE + protocol load

# ---------------------------------------------------------------------------
# USB VID:PID fingerprints for auto-detection
# ---------------------------------------------------------------------------
BOARD_FINGERPRINTS = {
    "clearcore": (0x2890, 0x8022),   # Teknic ClearCore
    "xpb":       (0x2341, 0x0058),   # Arduino Nano Every (Expansion Board)
    "sniffer":   (0x16C0, 0x0483),   # Teensy 4.0 (TTL Sniffer)
}


def detect_ports() -> Optional[dict[str, str]]:
    """Auto-detect COM ports for all three boards by USB VID:PID.

    Returns a dict {"clearcore": "COMx", "xpb": "COMy", "sniffer": "COMz"}
    or None if any board is missing.
    """
    ports = serial.tools.list_ports.comports()
    vid_pid_map: dict[tuple[int, int], str] = {}
    for p in ports:
        if p.vid is not None and p.pid is not None:
            vid_pid_map[(p.vid, p.pid)] = p.device

    found: dict[str, str] = {}
    missing: list[str] = []
    for name, (vid, pid) in BOARD_FINGERPRINTS.items():
        port = vid_pid_map.get((vid, pid))
        if port:
            found[name] = port
        else:
            missing.append(name)

    if missing:
        print("  ✗ Board detection FAILED — missing boards:")
        for name in missing:
            vid, pid = BOARD_FINGERPRINTS[name]
            print(f"      {name}: VID:PID={vid:#06x}:{pid:#06x} not found")
        print(f"  Detected ports: {found}" if found else "  No boards detected.")
        return None

    return found


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
             common_args: list[str]) -> tuple[str, str, float]:
    """Run a single test verb and return (test_id, result, elapsed_s)."""
    cmd = [sys.executable, RIG_CONTROL] + common_args + verb_args
    print(f"\n{'=' * 60}")
    print(f"  [{test_id}] {label}")
    print(f"  CMD: {' '.join(cmd)}")
    print(f"{'=' * 60}\n")

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
        print(f"  ERROR: {exc}")
        status = "ERROR"

    tag = status
    print(f"\n  >>> [{test_id}] {label}: {tag}  ({elapsed:.1f}s)")
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
        "--port", type=str, default=None,
        help="Sniffer serial port override (auto-detected by default).",
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

    # ---- Board detection ----
    print("=" * 60)
    print("  BOARD DETECTION")
    print("=" * 60)
    boards = detect_ports()
    if boards is None:
        print("\n  ✗ Cannot proceed — plug in all three boards and retry.")
        return 1

    for name, port in boards.items():
        print(f"  ✓  {name:<12s} → {port}")

    # Use explicit --port if given, otherwise auto-detected sniffer (Teensy)
    # rig_control.py sends commands via the sniffer, which relays to ClearCore
    cmd_port = args.port if args.port else boards["sniffer"]
    print(f"\n  Using sniffer on {cmd_port} for test commands")
    print("=" * 60)

    # Build common args forwarded to every rig_control invocation.
    common: list[str] = ["--port", cmd_port, "--drop-first-line"]
    if args.baud:
        common += ["--baud", str(args.baud)]

    tests = list(QUICK_TESTS)
    if args.full:
        tests += EXTENDED_TESTS

    # ---- Preamble: power-cycle to known state ----
    if not establish_known_state(common):
        print("\n  ✗ PREAMBLE FAILED — aborting suite")
        return 1

    results: list[tuple[str, str, str, float]] = []

    for test_id, label, verb_args in tests:
        tid, status, elapsed = run_test(test_id, label, verb_args, common)
        results.append((tid, label, status, elapsed))

    # ---- Summary ----
    print("\n")
    print("=" * 60)
    print("  TEST SUITE SUMMARY")
    print("=" * 60)
    total_time = sum(r[3] for r in results)
    passes = sum(1 for r in results if r[2] == "PASS")
    fails = len(results) - passes

    for tid, label, status, elapsed in results:
        marker = "✓" if status == "PASS" else "✗"
        print(f"  {marker}  [{tid}] {label:<20s}  {status:<8s}  {elapsed:6.1f}s")

    print(f"\n  {passes} passed, {fails} failed  —  {total_time:.1f}s total")
    print("=" * 60)

    return 0 if fails == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
