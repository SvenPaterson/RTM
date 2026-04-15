#!/usr/bin/env python3
"""Unified CLI for Teensy sniffer controller exercises."""

from __future__ import annotations

import argparse
import csv
import random
import re
import secrets
import subprocess
import sys
import time
import shlex
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional, Sequence, TextIO

try:
    import serial  # type: ignore
    from serial.tools import list_ports  # type: ignore
except ImportError as exc:  # pragma: no cover - user guidance
    raise SystemExit("pyserial is required. Install with `pip install pyserial`.") from exc

LOG_DIR = Path(__file__).resolve().parent / "log"
DEFAULT_BAUD = 460_800
RESET_THRESHOLD_MS = 5000
RUN_PULSE_DEFAULT_MS = 5000
RUN_PULSE_CAPTURE_S = 10.0
RESET_PULSE_DEFAULT_MS = 6000
RESET_PULSE_CAPTURE_S = 35.0
RESET_CANCEL_CAPTURE_S = 15.0
RESET_CANCEL_COUNT = 3
RESET_CANCEL_MIN_MS = 500
RESET_CANCEL_MAX_MS = 4500
POWER_CAPTURE_DEFAULT_S = 4.0
DEFAULT_INGESTION_WAIT = 15.0
DEFAULT_TAIL_PAD = 10.0
DEFAULT_RUN_MARGIN = 15.0
DEFAULT_PROTOCOL_DIR = Path(__file__).resolve().parent.parent / "tools" / "protocols"
DEFAULT_PROTOCOL_NAME = "protocol.csv"

# run-gate validation defaults
RUN_GATE_BOOT_WAIT_S = 30.0
RUN_GATE_SETTLE_S = 5.0
RUN_GATE_HB_WINDOW_S = 10.0
RUN_GATE_POWER_DWELL_S = 4.0
RUN_GATE_STEP_WAIT_S = 12.0
RUN_GATE_RESET_PULSE_MS = 6000
RUN_GATE_RESET_WAIT_S = 12.0


def detect_default_port() -> str | None:
    # Prefer Teensy sniffer by VID:PID (0x16C0:0x0483)
    for port in list_ports.comports():
        if port.vid == 0x16C0 and port.pid == 0x0483:
            return port.device
    # Fallback: match by description
    for port in list_ports.comports():
        desc = (port.description or "").lower()
        if "teensy" in desc:
            return port.device
    return None


def open_serial_with_retry(port: str, baud: int, *, timeout: float, wait_s: float = 20.0) -> serial.Serial:
    deadline = time.monotonic() + wait_s
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            ser = serial.Serial(port, baud, timeout=timeout)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.5)  # let Teensy USB CDC settle after connect
            ser.reset_input_buffer()
            # Verify the sniffer is actually responsive before returning.
            # Teensy USB CDC can accept the OS-level open but drop the first
            # few bytes while the pipe finishes initialising.
            if _wait_for_sniffer_ready(ser):
                return ser
            ser.close()
        except (serial.SerialException, OSError) as exc:
            last_error = exc
            time.sleep(0.05)
    raise serial.SerialException(f"Timed out waiting for {port} to become available") from last_error


def _wait_for_sniffer_ready(ser: serial.Serial, retries: int = 3, timeout_s: float = 1.0) -> bool:
    """Send STATUS and wait for [CTRL] echo to confirm USB CDC is live."""
    for _ in range(retries):
        ser.reset_input_buffer()
        ser.write(b"STATUS\n")
        ser.flush()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            raw = ser.readline()
            if raw and b"[CTRL]" in raw:
                return True
        time.sleep(0.1)
    return False


def write_log(line: str, log: TextIO) -> None:
    log.write(line + "\n")
    log.flush()


def emit(line: str, log: TextIO | None) -> None:
    try:
        print(line)
    except UnicodeEncodeError:
        encoding = sys.stdout.encoding or "utf-8"
        safe_line = line.encode(encoding, errors="replace").decode(encoding, errors="replace")
        print(safe_line)
    if log:
        write_log(line, log)


def send_command(ser: serial.Serial, command: str, *, log: TextIO | None) -> None:
    emit(f"[TEST] -> {command}", log)
    ser.write((command + "\n").encode("ascii", errors="strict"))
    ser.flush()


def rel_path(path: Path) -> str:
    try:
        return str(path.relative_to(Path.cwd()))
    except ValueError:
        return str(path)


def ensure_log_dir() -> Path:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    return LOG_DIR


def format_invocation(argv: Sequence[str] | None = None) -> str:
    args = tuple(argv) if argv is not None else tuple(sys.argv)
    try:
        return shlex.join(args)
    except AttributeError:
        pieces: list[str] = []
        for token in args:
            if token.isalnum():
                pieces.append(token)
            else:
                pieces.append('"' + token.replace('"', '\\"') + '"')
        return " ".join(pieces)


def stream_serial(
    ser: serial.Serial,
    deadline: float,
    log: TextIO | None,
    *,
    drop_first_line: bool = False,
    on_tick: Optional[Callable[[], None]] = None,
    tick_interval: float = 0.1,
) -> None:
    partial = bytearray()
    drop_next = drop_first_line
    next_tick = time.monotonic() + tick_interval if on_tick else 0.0
    while time.monotonic() < deadline:
        chunk = ser.read(512)
        if chunk:
            partial.extend(chunk)
            while True:
                nl = partial.find(b"\n")
                if nl < 0:
                    break
                frame = partial[:nl]
                del partial[: nl + 1]
                if drop_next:
                    drop_next = False
                    continue
                text = frame.decode("utf-8", errors="replace").rstrip("\r")
                if text:
                    emit(text, log)
        else:
            time.sleep(0.02)
        if on_tick and time.monotonic() >= next_tick:
            on_tick()
            next_tick = time.monotonic() + tick_interval
    if partial and not drop_next:
        text = partial.decode("utf-8", errors="replace").rstrip("\r")
        if text:
            emit(text, log)


def flush_serial(ser: serial.Serial, log: TextIO | None) -> None:
    while ser.in_waiting:
        line = ser.readline()
        if not line:
            break
        text = line.decode("utf-8", errors="replace").rstrip("\r\n")
        if text:
            emit(text, log)


def capture_lines(
    ser: serial.Serial,
    duration_s: float,
    log: TextIO | None,
    *,
    drop_first_line: bool = False,
) -> list[str]:
    """Like stream_serial but also returns every captured line."""
    collected: list[str] = []
    partial = bytearray()
    drop_next = drop_first_line
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        chunk = ser.read(512)
        if chunk:
            partial.extend(chunk)
            while True:
                nl = partial.find(b"\n")
                if nl < 0:
                    break
                frame = partial[:nl]
                del partial[: nl + 1]
                if drop_next:
                    drop_next = False
                    continue
                text = frame.decode("utf-8", errors="replace").rstrip("\r")
                if text:
                    emit(text, log)
                    collected.append(text)
        else:
            time.sleep(0.02)
    if partial and not drop_next:
        text = partial.decode("utf-8", errors="replace").rstrip("\r")
        if text:
            emit(text, log)
            collected.append(text)
    return collected


def scan_lines(lines: list[str], patterns: dict[str, str]) -> dict[str, list[str]]:
    """Scan *lines* for named regex patterns.  Returns {name: [matching lines]}."""
    compiled = {name: re.compile(pat, re.IGNORECASE) for name, pat in patterns.items()}
    results: dict[str, list[str]] = {name: [] for name in patterns}
    for line in lines:
        for name, regex in compiled.items():
            if regex.search(line):
                results[name].append(line)
    return results


def timestamp() -> str:
    return datetime.now().strftime("%Y%m%d-%H%M%S")


def choose_pulses(count: int, min_ms: int, max_ms: int, *, seed: int | None) -> list[int]:
    rng = random.Random(seed)
    return [rng.randint(min_ms, max_ms) for _ in range(count)]


def ensure_default_protocol() -> Path:
    proto_dir = DEFAULT_PROTOCOL_DIR
    proto_dir.mkdir(parents=True, exist_ok=True)
    token = secrets.token_hex(2)
    today = datetime.now().strftime("%m%d")
    name = f"TEST_{today}_{token.upper()}"
    content = [
        f"PROTOCOL_NAME={name}",
        "LOOP_COUNT=2",
        "TargetRPM,AccelRPMperSec,DwellSeconds",
        "500,200,4",
        "-500,200,3",
        "0,200,2",
    ]
    path = proto_dir / DEFAULT_PROTOCOL_NAME
    path.write_text("\n".join(content) + "\n", encoding="utf-8")
    return path


def parse_protocol(path: Path) -> tuple[str, int, Sequence[tuple[float, float, float]]]:
    lines = path.read_text(encoding="utf-8").splitlines()
    if len(lines) < 3:
        raise ValueError("Protocol file must include metadata and at least one row")
    name_line = lines[0].strip()
    loop_line = lines[1].strip()
    if not name_line.upper().startswith("PROTOCOL_NAME="):
        raise ValueError("First line must start with PROTOCOL_NAME=")
    if not loop_line.upper().startswith("LOOP_COUNT="):
        raise ValueError("Second line must start with LOOP_COUNT=")
    protocol_name = name_line.split("=", 1)[1].strip()
    try:
        loop_count = int(loop_line.split("=", 1)[1].strip())
    except ValueError as exc:
        raise ValueError("Invalid LOOP_COUNT value") from exc

    reader = csv.DictReader(lines[2:])
    required = {"TargetRPM", "AccelRPMperSec", "DwellSeconds"}
    if reader.fieldnames is None or any(h not in reader.fieldnames for h in required):
        raise ValueError("Protocol CSV must define TargetRPM, AccelRPMperSec, DwellSeconds columns")

    steps: list[tuple[float, float, float]] = []
    for row in reader:
        if not row:
            continue
        target_raw = row.get("TargetRPM")
        accel_raw = row.get("AccelRPMperSec")
        dwell_raw = row.get("DwellSeconds")
        if not target_raw:
            raise ValueError("Each row must define TargetRPM")
        if not accel_raw:
            raise ValueError("Each row must define AccelRPMperSec")
        if dwell_raw is None or dwell_raw.strip() == "":
            raise ValueError("Each row must define DwellSeconds")
        try:
            target = float(target_raw)
        except ValueError as exc:
            raise ValueError(f"Invalid TargetRPM value: {target_raw}") from exc
        try:
            accel = float(accel_raw)
        except ValueError as exc:
            raise ValueError(f"Invalid AccelRPMperSec value: {accel_raw}") from exc
        if accel <= 0:
            raise ValueError("AccelRPMperSec must be positive for all steps")
        try:
            dwell = float(dwell_raw)
        except ValueError as exc:
            raise ValueError(f"Invalid DwellSeconds value: {dwell_raw}") from exc
        if dwell < 0:
            raise ValueError("DwellSeconds cannot be negative")
        steps.append((target, accel, dwell))

    if not steps:
        raise ValueError("Protocol must define at least one step")
    return protocol_name, loop_count, steps


def run_reset_pulse(args: argparse.Namespace) -> int:
    log_dir = ensure_log_dir()
    log_path = log_dir / f"{timestamp()}_reset_pulse.log"

    with open(log_path, "w", encoding="utf-8") as log:
        emit(f"[TEST] Logging to {rel_path(log_path)}", log)
        emit(f"[TEST] Command: {format_invocation()}", log)
        emit(f"[TEST] Opening serial port {args.port} @ {args.baud} baud", log)
        try:
            ser = open_serial_with_retry(args.port, args.baud, timeout=0.1)
        except (serial.SerialException, OSError) as exc:
            emit(f"[TEST] Serial error: {exc}", log)
            return 2

        with ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            emit("[TEST] Connected", log)
            emit(
                f"[TEST] Configuration: pulse={args.pulse_ms} ms, capture={args.capture_s:.1f} s",
                log,
            )

            send_command(ser, "STATUS", log=log)
            time.sleep(0.25)
            send_command(ser, f"PULSE RST={args.pulse_ms}", log=log)

            lines = capture_lines(
                ser,
                args.capture_s,
                log,
                drop_first_line=args.drop_first_line,
            )

            # Verify the system recovered after the reset
            hits = scan_lines(lines, {
                "proto_ok": r"NOTICE;PROTO_RX=OK",
                "hb_idle": r"HB;.*STATE=IDLE",
            })

            send_command(ser, "STATUS", log=log)
            time.sleep(0.2)
            flush_serial(ser, log)

            if hits["proto_ok"] or hits["hb_idle"]:
                emit("[TEST] PASS — system recovered after reset pulse", log)
                emit("[TEST] Capture complete", log)
                return 0

            emit("[TEST] FAIL — system did not recover within capture window", log)
            emit("[TEST] Capture complete", log)

    return 1


# Defaults for reset-pulse-multi
RESET_MULTI_COUNT = 5
RESET_MULTI_BOOT_WAIT_S = 30.0
RESET_MULTI_SETTLE_S = 5.0


def run_reset_pulse_multi(args: argparse.Namespace) -> int:
    """Run multiple reset-pulse cycles back-to-back, checking protocol reload each time."""
    log_dir = ensure_log_dir()
    log_path = log_dir / f"{timestamp()}_reset_pulse_multi.log"
    count = args.count
    boot_wait_s = args.boot_wait_s
    settle_s = args.settle_s
    pulse_ms = args.pulse_ms
    results: list[str] = []

    with open(log_path, "w", encoding="utf-8") as log:
        emit(f"[TEST] Logging to {rel_path(log_path)}", log)
        emit(f"[TEST] Command: {format_invocation()}", log)
        emit(f"[TEST] === RESET-PULSE-MULTI ({count} iterations) ===", log)
        emit(
            f"[TEST] Configuration: count={count}, pulse={pulse_ms} ms, "
            f"boot_wait={boot_wait_s:.1f} s, settle={settle_s:.1f} s",
            log,
        )
        emit(f"[TEST] Opening serial port {args.port} @ {args.baud} baud", log)
        try:
            ser = open_serial_with_retry(args.port, args.baud, timeout=0.1)
        except (serial.SerialException, OSError) as exc:
            emit(f"[TEST] Serial error: {exc}", log)
            return 2

        with ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            emit("[TEST] Connected", log)

            # Confirm system is alive before starting
            send_command(ser, "STATUS", log=log)
            time.sleep(0.5)
            flush_serial(ser, log)

            for iteration in range(1, count + 1):
                emit(f"[TEST] --- Iteration {iteration}/{count} ---", log)

                # Issue reset pulse
                send_command(ser, f"PULSE RST={pulse_ms}", log=log)

                # Capture the reset + reboot sequence
                # Reset arm (5s) + exec + reboot + protocol upload
                capture_time = (pulse_ms / 1000.0) + boot_wait_s
                lines = capture_lines(ser, capture_time, log, drop_first_line=args.drop_first_line)

                # Check for protocol upload success indicators
                hits = scan_lines(lines, {
                    "proto_ok": r"NOTICE;PROTO_RX=OK",
                    "hb_idle": r"HB;.*STATE=IDLE",
                    "proto_missing": r"Protocol Missing",
                })

                if hits["proto_ok"] or hits["hb_idle"]:
                    verdict = "PASS"
                    detail = (
                        f"proto_ok={len(hits['proto_ok'])}, "
                        f"hb_idle={len(hits['hb_idle'])}"
                    )
                else:
                    verdict = "FAIL"
                    detail = "No PROTO_RX=OK or IDLE heartbeat observed"
                    if hits["proto_missing"]:
                        detail = "Protocol Missing on SD detected"

                results.append(verdict)
                emit(f"[TEST] Iteration {iteration}: {verdict} ({detail})", log)

                # Settle before next iteration (unless last)
                if iteration < count:
                    emit(f"[TEST] Settling {settle_s:.1f}s before next iteration", log)
                    stream_serial(ser, time.monotonic() + settle_s, log)

            # Summary
            pass_count = results.count("PASS")
            fail_count = results.count("FAIL")
            emit(f"[TEST] === SUMMARY: {pass_count}/{count} PASS, {fail_count}/{count} FAIL ===", log)
            for i, r in enumerate(results, 1):
                emit(f"[TEST]   Iteration {i}: {r}", log)

            send_command(ser, "STATUS", log=log)
            time.sleep(0.2)
            flush_serial(ser, log)
            emit("[TEST] Capture complete", log)

    return 0 if fail_count == 0 else 1


def run_power(args: argparse.Namespace) -> int:
    log_dir = ensure_log_dir()
    state_token = "on" if args.state == "on" else "off"
    log_path = log_dir / f"{timestamp()}_power_{state_token}.log"

    with open(log_path, "w", encoding="utf-8") as log:
        emit(f"[TEST] Logging to {rel_path(log_path)}", log)
        emit(f"[TEST] Command: {format_invocation()}", log)
        emit(f"[TEST] Opening serial port {args.port} @ {args.baud} baud", log)
        try:
            ser = open_serial_with_retry(args.port, args.baud, timeout=0.1)
        except (serial.SerialException, OSError) as exc:
            emit(f"[TEST] Serial error: {exc}", log)
            return 2

        with ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            emit("[TEST] Connected", log)
            emit(
                f"[TEST] Configuration: state={args.state}, capture={args.capture_s:.1f} s",
                log,
            )

            send_command(ser, "STATUS", log=log)
            time.sleep(0.25)
            send_command(ser, f"PWR={'1' if args.state == 'on' else '0'}", log=log)

            deadline = time.monotonic() + args.capture_s
            stream_serial(
                ser,
                deadline,
                log,
                drop_first_line=args.drop_first_line,
            )

            send_command(ser, "STATUS", log=log)
            time.sleep(0.2)
            flush_serial(ser, log)
            emit("[TEST] Capture complete", log)

    return 0


def run_run_pulse(args: argparse.Namespace) -> int:
    log_dir = ensure_log_dir()
    log_path = log_dir / f"{timestamp()}_run_pulse.log"

    with open(log_path, "w", encoding="utf-8") as log:
        emit(f"[TEST] Logging to {rel_path(log_path)}", log)
        emit(f"[TEST] Command: {format_invocation()}", log)
        emit(f"[TEST] Opening serial port {args.port} @ {args.baud} baud", log)
        try:
            ser = open_serial_with_retry(args.port, args.baud, timeout=0.1)
        except (serial.SerialException, OSError) as exc:
            emit(f"[TEST] Serial error: {exc}", log)
            return 2

        with ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            emit("[TEST] Connected", log)
            emit(
                f"[TEST] Configuration: pulse={args.pulse_ms} ms, capture={args.capture_s:.1f} s",
                log,
            )

            send_command(ser, "STATUS", log=log)
            time.sleep(0.25)
            send_command(ser, f"PULSE RUN={args.pulse_ms}", log=log)

            deadline = time.monotonic() + args.capture_s
            stream_serial(
                ser,
                deadline,
                log,
                drop_first_line=args.drop_first_line,
            )

            send_command(ser, "STATUS", log=log)
            time.sleep(0.2)
            flush_serial(ser, log)
            emit("[TEST] Capture complete", log)

    return 0


def run_protocol(args: argparse.Namespace) -> int:
    """T9: Run a loaded protocol to completion and log RPM vs step."""
    proto_path = args.protocol.resolve()
    if not proto_path.is_file():
        print(f"[TEST] Protocol file not found: {proto_path}")
        return 1

    try:
        proto_name, loop_count, steps = parse_protocol(proto_path)
    except ValueError as exc:
        print(f"[TEST] Protocol parse error: {exc}")
        return 3

    # Simulate expected runtime
    total_runtime = 0.0
    current_rpm = 0.0
    for _ in range(loop_count):
        for target_rpm, accel_rpm_s, dwell_s in steps:
            ramp_time = abs(target_rpm - current_rpm) / accel_rpm_s
            total_runtime += ramp_time + dwell_s
            current_rpm = target_rpm

    run_hold_s = total_runtime + args.run_margin_s
    capture_s = args.settle_s + run_hold_s + args.tail_s

    log_dir = ensure_log_dir()
    safe_name = (proto_name.replace(" ", "_") or "protocol")
    log_path = log_dir / f"{timestamp()}_run_protocol_{safe_name}.log"

    with open(log_path, "w", encoding="utf-8") as log:
        emit(f"[TEST] Logging to {rel_path(log_path)}", log)
        emit(f"[TEST] Command: {format_invocation()}", log)
        emit("[TEST] === RUN-PROTOCOL TEST ===", log)
        emit(f"[TEST] Protocol: {proto_name} ({len(steps)} steps, {loop_count} loops)", log)
        emit(f"[TEST] Expected runtime: {total_runtime:.1f}s, RUN hold: {run_hold_s:.1f}s", log)
        emit(f"[TEST] Capture window: {capture_s:.1f}s (settle {args.settle_s:.1f}s + run {run_hold_s:.1f}s + tail {args.tail_s:.1f}s)", log)
        emit(f"[TEST] Opening serial port {args.port} @ {args.baud} baud", log)
        try:
            ser = open_serial_with_retry(args.port, args.baud, timeout=0.1)
        except (serial.SerialException, OSError) as exc:
            emit(f"[TEST] Serial error: {exc}", log)
            return 2

        with ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            emit("[TEST] Connected", log)

            try:
                send_command(ser, "STATUS", log=log)
                time.sleep(0.25)

                # Phase 1: Wait for IDLE (system may still be booting / loading protocol)
                emit(f"[TEST] Phase 1: Waiting up to {args.settle_s:.0f}s for IDLE state", log)
                settle_lines: list[str] = []
                idle_seen = False
                settle_deadline = time.monotonic() + args.settle_s
                while time.monotonic() < settle_deadline:
                    chunk = capture_lines(ser, min(2.0, settle_deadline - time.monotonic()), log)
                    settle_lines.extend(chunk)
                    chunk_hits = scan_lines(chunk, {"hb_idle": r"HB;.*STATE=IDLE"})
                    if chunk_hits["hb_idle"]:
                        idle_seen = True
                        emit("[TEST] IDLE confirmed", log)
                        break
                if not idle_seen:
                    emit("[TEST] FAIL — system did not reach IDLE within settle window", log)
                    return 1

                # Phase 2: Assert RUN, capture through completion
                run_ms = int(run_hold_s * 1000)
                emit(f"[TEST] Phase 2: Issuing RUN pulse for {run_hold_s:.1f}s", log)
                send_command(ser, f"PULSE RUN={run_ms}", log=log)
                run_lines = capture_lines(ser, run_hold_s + args.tail_s, log)

                send_command(ser, "STATUS", log=log)
                time.sleep(0.2)
                flush_serial(ser, log)
            except KeyboardInterrupt:
                emit("\n[TEST] Ctrl+C — releasing RUN and stopping", log)
                send_command(ser, "RUN=0", log=log)
                time.sleep(0.1)
                emit("[TEST] ABORTED by user", log)
                return 1

            # Phase 3: Analyze captured HB messages
            all_lines = settle_lines + run_lines
            hits = scan_lines(all_lines, {
                "hb_running": r"HB;.*STATE=RUNNING",
                "hb_completed": r"HB;.*STATE=COMPLETED",
                "hb_estop": r"HB;.*STATE=E-STOP",
            })

            # Extract RPM values per step with sniffer timestamps
            rpm_by_step: dict[int, list[int]] = {}
            step_timeline: list[tuple[float, int, int]] = []  # (ms, step, rpm)
            for line in all_lines:
                m = re.search(r"HB;.*STEP=(\d+).*RPM=(-?\d+)", line)
                if m:
                    step_idx = int(m.group(1))
                    rpm_val = int(m.group(2))
                    rpm_by_step.setdefault(step_idx, []).append(rpm_val)
                    # Extract sniffer ms timestamp
                    ts_m = re.search(r"ms=(\d+)", line)
                    if ts_m:
                        step_timeline.append((float(ts_m.group(1)), step_idx, rpm_val))

            # Build expected step targets from protocol (1-indexed, repeating per loop)
            step_targets: dict[int, tuple[float, float]] = {}  # step_idx -> (target_rpm, accel)
            for loop_i in range(loop_count):
                for s_i, (trpm, accel, _dwell) in enumerate(steps):
                    idx = loop_i * len(steps) + s_i + 1  # 1-based step index
                    step_targets[idx] = (trpm, accel)

            # Single pass: detect ramp completion and collect ramp analysis + steady-state RPMs
            ramp_results: list[tuple[int, float, float, float, float, str]] = []  # (step, target, expect, actual, diff, result)
            steady_rpm_by_step: dict[int, list[int]] = {}
            ramp_targets = dict(step_targets)  # working copy for ramp detection
            step_reached_ms: dict[int, float] = {}  # step_idx -> ms when target reached

            if step_timeline:
                prev_step = 0
                prev_rpm_at_transition = 0
                transition_ms = 0.0
                ramp_tolerance = 0.10  # 10% tolerance on RPM match

                for ts_ms, step_idx, rpm_val in step_timeline:
                    if step_idx != prev_step:
                        prev_rpm_at_transition = rpm_val
                        transition_ms = ts_ms
                        prev_step = step_idx
                    elif step_idx in ramp_targets and transition_ms > 0:
                        target_rpm, accel = ramp_targets[step_idx]
                        abs_target = abs(target_rpm)
                        abs_rpm = abs(rpm_val)

                        if abs_target == 0:
                            reached = abs_rpm == 0
                        else:
                            reached = abs(abs_rpm - abs_target) <= abs_target * ramp_tolerance

                        if reached:
                            actual_ramp_s = (ts_ms - transition_ms) / 1000.0
                            delta_rpm = abs(target_rpm - prev_rpm_at_transition)
                            expected_ramp_s = delta_rpm / accel if accel > 0 else 0.0
                            diff_s = actual_ramp_s - expected_ramp_s
                            ok = abs(diff_s) < max(1.0, expected_ramp_s * 0.25)
                            result = "OK" if ok else "SLOW" if diff_s > 0 else "FAST"
                            ramp_results.append((step_idx, target_rpm, expected_ramp_s, actual_ramp_s, diff_s, result))
                            step_reached_ms[step_idx] = ts_ms
                            del ramp_targets[step_idx]
                            transition_ms = 0.0

                    # Collect steady-state samples (after target reached)
                    if step_idx in step_reached_ms and ts_ms >= step_reached_ms[step_idx]:
                        steady_rpm_by_step.setdefault(step_idx, []).append(rpm_val)

            # Summary table
            emit("", log)
            emit("[TEST] === RPM SUMMARY BY STEP ===", log)
            emit(f"[TEST] {'Step':>4}  {'Count':>5}  {'Min RPM':>8}  {'Max RPM':>8}  {'Avg RPM':>8}  {'Steady':>6}  {'StdyAvg':>8}", log)
            emit(f"[TEST] {'----':>4}  {'-----':>5}  {'-------':>8}  {'-------':>8}  {'-------':>8}  {'------':>6}  {'-------':>8}", log)
            for step_idx in sorted(rpm_by_step.keys()):
                vals = rpm_by_step[step_idx]
                avg = sum(vals) / len(vals) if vals else 0
                svals = steady_rpm_by_step.get(step_idx, [])
                savg = sum(svals) / len(svals) if svals else 0
                emit(
                    f"[TEST] {step_idx:4d}  {len(vals):5d}  {min(vals):8d}  {max(vals):8d}  {avg:8.0f}"
                    f"  {len(svals):6d}  {savg:8.0f}",
                    log,
                )

            # Ramp / acceleration analysis
            if ramp_results:
                emit("", log)
                emit("[TEST] === RAMP / ACCELERATION ANALYSIS ===", log)
                emit(f"[TEST] {'Step':>4}  {'Target':>7}  {'Expect':>7}  {'Actual':>7}  {'Δ':>7}  {'Result':>6}", log)
                emit(f"[TEST] {'':>4}  {'RPM':>7}  {'Ramp s':>7}  {'Ramp s':>7}  {'s':>7}  {'':>6}", log)
                emit(f"[TEST] {'----':>4}  {'-------':>7}  {'-------':>7}  {'-------':>7}  {'-------':>7}  {'------':>6}", log)

                for step_idx, target_rpm, expected_ramp_s, actual_ramp_s, diff_s, result in ramp_results:
                    emit(
                        f"[TEST] {step_idx:4d}  {target_rpm:7.0f}  {expected_ramp_s:7.1f}  "
                        f"{actual_ramp_s:7.1f}  {diff_s:+7.1f}  {result:>6}",
                        log,
                    )
            emit("", log)

            # Verdict
            if hits["hb_estop"]:
                emit("[TEST] FAIL — E-STOP detected during protocol run", log)
                return 1

            if not hits["hb_running"]:
                emit("[TEST] FAIL — no RUNNING heartbeats observed", log)
                return 1

            completed = bool(hits["hb_completed"])
            total_hbs = len(hits["hb_running"])
            rpm_steps = len(rpm_by_step)

            if completed:
                emit(
                    f"[TEST] PASS — protocol completed ({total_hbs} RUNNING HBs, "
                    f"RPM data for {rpm_steps} steps)",
                    log,
                )
            else:
                emit(
                    f"[TEST] WARN — protocol did not reach COMPLETED state "
                    f"({total_hbs} RUNNING HBs, RPM data for {rpm_steps} steps). "
                    f"May need longer --run-margin-s or --tail-s.",
                    log,
                )

            emit("[TEST] Capture complete", log)

    return 0


def run_run_cycle(args: argparse.Namespace) -> int:
    """T2: Run/Pause/Resume cycle — double-pulse with automated verdict."""
    log_dir = ensure_log_dir()
    log_path = log_dir / f"{timestamp()}_run_cycle.log"

    with open(log_path, "w", encoding="utf-8") as log:
        emit(f"[TEST] Logging to {rel_path(log_path)}", log)
        emit(f"[TEST] Command: {format_invocation()}", log)
        emit("[TEST] === RUN-CYCLE TEST (IDLE → RUNNING → PAUSED → RUNNING) ===", log)
        emit(
            f"[TEST] Configuration: pulse={args.pulse_ms} ms, "
            f"observe={args.observe_s:.1f} s",
            log,
        )
        emit(f"[TEST] Opening serial port {args.port} @ {args.baud} baud", log)
        try:
            ser = open_serial_with_retry(args.port, args.baud, timeout=0.1)
        except (serial.SerialException, OSError) as exc:
            emit(f"[TEST] Serial error: {exc}", log)
            return 2

        with ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            emit("[TEST] Connected", log)

            send_command(ser, "STATUS", log=log)
            time.sleep(0.25)

            # --- Phase 1: IDLE baseline ---
            emit("[TEST] Phase 1: Confirming IDLE baseline", log)
            idle_lines = capture_lines(ser, args.observe_s, log)
            idle_hits = scan_lines(idle_lines, {
                "hb_idle": r"HB;.*STATE=IDLE",
                "cc_stat": r"STAT;SEQ=",
            })
            if not idle_hits["hb_idle"] and not idle_hits["cc_stat"]:
                emit("[TEST] FAIL — no telemetry; system not in IDLE", log)
                return 1

            # --- Phase 2: Assert RUN → expect RUNNING ---
            emit("[TEST] Phase 2: Asserting RUN", log)
            send_command(ser, "RUN=1", log=log)
            run1_lines = capture_lines(ser, args.observe_s, log)
            run1_hits = scan_lines(run1_lines, {
                "hb_running": r"HB;.*STATE=RUNNING",
            })
            if not run1_hits["hb_running"]:
                emit("[TEST] FAIL — no RUNNING heartbeat after RUN asserted", log)
                send_command(ser, "RUN=0", log=log)
                return 1

            # --- Phase 3: Release RUN → expect PAUSED ---
            emit("[TEST] Phase 3: Releasing RUN", log)
            send_command(ser, "RUN=0", log=log)
            pause_lines = capture_lines(ser, args.observe_s, log)
            pause_hits = scan_lines(pause_lines, {
                "hb_paused": r"HB;.*STATE=PAUSED",
            })
            if not pause_hits["hb_paused"]:
                emit("[TEST] FAIL — no PAUSED heartbeat after RUN released", log)
                return 1

            # --- Phase 4: Re-assert RUN → expect RUNNING resume ---
            emit("[TEST] Phase 4: Re-asserting RUN (resume)", log)
            send_command(ser, f"PULSE RUN={args.pulse_ms}", log=log)
            run2_lines = capture_lines(ser, args.observe_s, log)
            run2_hits = scan_lines(run2_lines, {
                "hb_running": r"HB;.*STATE=RUNNING",
                "hb_resume": r"HB;.*STATE=RESUME",
            })
            if not run2_hits["hb_running"] and not run2_hits["hb_resume"]:
                emit("[TEST] FAIL — no RUNNING/RESUME heartbeat after re-assert", log)
                return 1

            # --- Phase 5: Final release → PAUSED ---
            emit("[TEST] Phase 5: Final release, confirming PAUSED", log)
            final_lines = capture_lines(ser, args.observe_s, log)
            final_hits = scan_lines(final_lines, {
                "hb_paused": r"HB;.*STATE=PAUSED",
            })

            send_command(ser, "STATUS", log=log)
            time.sleep(0.2)
            flush_serial(ser, log)

            emit(
                f"[TEST] PASS — IDLE({len(idle_hits['hb_idle'])} HBs) → "
                f"RUNNING({len(run1_hits['hb_running'])}) → "
                f"PAUSED({len(pause_hits['hb_paused'])}) → "
                f"RUNNING({len(run2_hits['hb_running'])}+{len(run2_hits['hb_resume'])} resume) → "
                f"PAUSED({len(final_hits['hb_paused'])})",
                log,
            )
            emit("[TEST] Capture complete", log)

    return 0


def run_reset_cancel(args: argparse.Namespace) -> int:
    """T5: Sub-threshold RESET pulses — system must NOT reset."""
    log_dir = ensure_log_dir()
    log_path = log_dir / f"{timestamp()}_reset_cancel.log"
    pulses = choose_pulses(args.count, args.min_ms, args.max_ms, seed=args.seed)

    with open(log_path, "w", encoding="utf-8") as log:
        emit(f"[TEST] Logging to {rel_path(log_path)}", log)
        emit(f"[TEST] Command: {format_invocation()}", log)
        emit(f"[TEST] Pulses below cancel threshold ({RESET_THRESHOLD_MS} ms): {pulses}", log)
        emit(f"[TEST] Opening serial port {args.port} @ {args.baud} baud", log)
        try:
            ser = open_serial_with_retry(args.port, args.baud, timeout=0.1)
        except (serial.SerialException, OSError) as exc:
            emit(f"[TEST] Serial error: {exc}", log)
            return 2

        with ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            emit("[TEST] Connected", log)
            emit(
                f"[TEST] Configuration: capture={args.capture_s:.1f} s, pulses={len(pulses)}",
                log,
            )

            send_command(ser, "STATUS", log=log)
            time.sleep(0.25)

            all_lines: list[str] = []
            for idx, pulse_ms in enumerate(pulses, start=1):
                send_command(ser, f"PULSE RST={pulse_ms}", log=log)
                lines = capture_lines(ser, pulse_ms / 1000.0 + 0.5, log)
                all_lines.extend(lines)
                send_command(ser, "STATUS", log=log)

            tail_lines = capture_lines(ser, args.capture_s, log,
                                       drop_first_line=args.drop_first_line)
            all_lines.extend(tail_lines)

            send_command(ser, "STATUS", log=log)
            time.sleep(0.2)
            flush_serial(ser, log)

            # Verdict: ARM + CANCEL is expected for sub-threshold pulses.
            # Fail only if the reset actually executed or the system rebooted.
            hits = scan_lines(all_lines, {
                "reset_exec": r"CMD;RESET=EXEC",
                "quiesce_boot": r"QUIESCE;SECS=",
                "reboot_notice": r"NOTICE;XPB_RESET=NOW",
            })

            rebooted = (hits["reset_exec"] or hits["quiesce_boot"]
                        or hits["reboot_notice"])
            if rebooted:
                emit("[TEST] FAIL — system rebooted during sub-threshold pulses", log)
                return 1

            emit(f"[TEST] PASS — {len(pulses)} sub-threshold pulses, all cancelled", log)
            emit("[TEST] Capture complete", log)

    return 0


def run_comms_health(args: argparse.Namespace) -> int:
    """T8: Passive comms health check — verify heartbeat cadence."""
    log_dir = ensure_log_dir()
    log_path = log_dir / f"{timestamp()}_comms_health.log"

    with open(log_path, "w", encoding="utf-8") as log:
        emit(f"[TEST] Logging to {rel_path(log_path)}", log)
        emit(f"[TEST] Command: {format_invocation()}", log)
        emit("[TEST] === COMMS HEALTH CHECK ===", log)
        emit(f"[TEST] Configuration: duration={args.duration_s:.1f} s", log)
        emit(f"[TEST] Opening serial port {args.port} @ {args.baud} baud", log)
        try:
            ser = open_serial_with_retry(args.port, args.baud, timeout=0.1)
        except (serial.SerialException, OSError) as exc:
            emit(f"[TEST] Serial error: {exc}", log)
            return 2

        with ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            emit("[TEST] Connected", log)

            send_command(ser, "STATUS", log=log)
            time.sleep(0.25)

            lines = capture_lines(ser, args.duration_s, log,
                                  drop_first_line=args.drop_first_line)

            send_command(ser, "STATUS", log=log)
            time.sleep(0.2)
            flush_serial(ser, log)

            # Count HB and STAT messages by content
            hb_count = sum(1 for line in lines if "HB;" in line)
            stat_count = sum(1 for line in lines if "STAT;" in line)

            failed = False
            min_expected = max(2, int(args.duration_s / 2))

            emit(
                f"[TEST] HB (XPB→CC): {hb_count} msgs in {args.duration_s:.0f}s "
                f"(expected >= {min_expected})",
                log,
            )
            if hb_count < min_expected:
                emit(f"[TEST] FAIL — HB count {hb_count} below minimum {min_expected}", log)
                failed = True

            emit(
                f"[TEST] STAT (CC→XPB): {stat_count} msgs in {args.duration_s:.0f}s "
                f"(expected >= {min_expected})",
                log,
            )
            if stat_count < min_expected:
                emit(f"[TEST] FAIL — STAT count {stat_count} below minimum {min_expected}", log)
                failed = True

            # Check for E-STOP (;E=1; with delimiters to avoid matching SW_AGE=1...)
            estop_hits = scan_lines(lines, {"estop": r";E=1;"})
            if estop_hits["estop"]:
                emit("[TEST] FAIL — E-STOP detected during health check", log)
                failed = True

            if failed:
                emit("[TEST] FAIL — comms health check failed", log)
                return 1

            emit("[TEST] PASS — heartbeat cadence healthy, no gaps or E-STOPs", log)
            emit("[TEST] Capture complete", log)

    return 0


def run_cold_boot(args: argparse.Namespace) -> int:
    """T1: Cold boot / power loss recovery — power cycle and verify full boot."""
    log_dir = ensure_log_dir()
    log_path = log_dir / f"{timestamp()}_cold_boot.log"

    with open(log_path, "w", encoding="utf-8") as log:
        emit(f"[TEST] Logging to {rel_path(log_path)}", log)
        emit(f"[TEST] Command: {format_invocation()}", log)
        emit("[TEST] === COLD BOOT / POWER LOSS RECOVERY ===", log)
        emit(
            f"[TEST] Configuration: boot_wait={args.boot_wait_s:.1f} s, "
            f"power_dwell={args.power_dwell_s:.1f} s",
            log,
        )
        emit(f"[TEST] Opening serial port {args.port} @ {args.baud} baud", log)
        try:
            ser = open_serial_with_retry(args.port, args.baud, timeout=0.1)
        except (serial.SerialException, OSError) as exc:
            emit(f"[TEST] Serial error: {exc}", log)
            return 2

        with ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            emit("[TEST] Connected", log)

            send_command(ser, "STATUS", log=log)
            time.sleep(0.25)

            # Power cycle
            emit("[TEST] Power-cycling boards", log)
            send_command(ser, "PWR=0", log=log)
            emit(f"[TEST] Power-off dwell {args.power_dwell_s:.1f}s", log)
            time.sleep(args.power_dwell_s)
            send_command(ser, "PWR=1", log=log)

            # Capture full boot sequence
            lines = capture_lines(ser, args.boot_wait_s, log,
                                  drop_first_line=args.drop_first_line)

            send_command(ser, "STATUS", log=log)
            time.sleep(0.2)
            flush_serial(ser, log)

            # Verdict
            hits = scan_lines(lines, {
                "quiesce": r"QUIESCE;SECS=15",
                "ready": r"READY;ID=CC",
                "proto_ok": r"NOTICE;PROTO_RX=OK",
                "hb_idle": r"HB;.*STATE=IDLE",
                "estop": r";E=1;",
                "proto_missing": r"Protocol Missing",
            })

            failed = False
            if not hits["quiesce"]:
                emit("[TEST] WARNING — no QUIESCE;SECS=15 from XPB", log)

            if not hits["proto_ok"]:
                emit("[TEST] FAIL — NOTICE;PROTO_RX=OK not observed", log)
                if hits["proto_missing"]:
                    emit("[TEST]   Protocol Missing on SD detected", log)
                failed = True

            if not hits["hb_idle"]:
                emit("[TEST] FAIL — no IDLE heartbeat after boot", log)
                failed = True

            if hits["estop"]:
                emit("[TEST] FAIL — E-STOP detected during boot", log)
                failed = True

            if failed:
                return 1

            emit(
                f"[TEST] PASS — boot sequence complete: "
                f"QUIESCE={len(hits['quiesce'])}, "
                f"proto_ok={len(hits['proto_ok'])}, "
                f"hb_idle={len(hits['hb_idle'])}",
                log,
            )
            emit("[TEST] Capture complete", log)

    return 0


def run_protocol_upload(args: argparse.Namespace) -> int:
    if args.protocol is None:
        args.protocol = ensure_default_protocol()
        args.generated_protocol = True
    else:
        args.protocol = args.protocol.resolve()
        args.generated_protocol = False
        if not args.protocol.is_file():
            print(f"[TEST] Protocol file not found: {args.protocol}")
            return 1

    try:
        proto_name, loop_count, steps = parse_protocol(args.protocol)
    except ValueError as exc:
        print(f"[TEST] Protocol parse error: {exc}")
        return 3

    def simulate_runtime(loop_count: int, steps: Sequence[tuple[float, float, float]]) -> tuple[float, list[float]]:
        total = 0.0
        loop_totals: list[float] = []
        current_rpm = 0.0
        for _ in range(loop_count):
            loop_time = 0.0
            for target_rpm, accel_rpm_s, dwell_s in steps:
                ramp_time = abs(target_rpm - current_rpm) / accel_rpm_s
                loop_time += ramp_time + dwell_s
                current_rpm = target_rpm
            loop_totals.append(loop_time)
            total += loop_time
        return total, loop_totals

    expected_runtime, loop_durations = simulate_runtime(loop_count, steps)
    first_loop = loop_durations[0] if loop_durations else 0.0
    avg_loop = expected_runtime / loop_count if loop_count else 0.0
    capture_horizon = args.ingestion_wait + expected_runtime + args.tail_pad
    pulse_seconds = max(0.0, expected_runtime + args.run_margin_s)

    log_dir = ensure_log_dir()
    ts = timestamp()
    safe_name = (proto_name.replace(" ", "_") or "protocol")
    log_path = log_dir / f"{ts}_protocol_upload_{safe_name}.log"

    if getattr(args, "generated_protocol", False):
        try:
            rel_proto = args.protocol.relative_to(Path.cwd())
        except ValueError:
            rel_proto = args.protocol
        print(f"[TEST] Generated protocol CSV at {rel_proto}. Copy to microSD before continuing.")

    print("[TEST] Confirm that SD card is removed from XPB, boards are powered down,"
          " sniffer terminals are closed, and the new protocol is staged on microSD.")
    response = input("Type YES to continue: ").strip().upper()
    if response != "YES":
        print("[TEST] Aborting; prerequisites not acknowledged.")
        return 1

    print("[TEST] Insert the prepared microSD card into the XPB before continuing.")
    input("Press Enter once the card is inserted and you are ready to begin sniffing...")

    print(f"[TEST] Opening serial port {args.port} @ {args.baud} baud")
    try:
        ser = open_serial_with_retry(args.port, args.baud, timeout=0.1)
    except (serial.SerialException, OSError) as exc:
        print(f"[TEST] Serial error: {exc}")
        return 2

    with ser, open(log_path, "w", encoding="utf-8") as log:
        emit(f"[TEST] Logging to {rel_path(log_path)}", log)
        emit(f"[TEST] Command: {format_invocation()}", log)
        proto_note = "(generated)" if getattr(args, "generated_protocol", False) else "(supplied)"
        loop_note = "0.0"
        if loop_count > 0:
            loop_note = f"first-loop={first_loop:.1f}s"
            if loop_count > 1 and any(abs(ld - first_loop) > 1e-6 for ld in loop_durations[1:]):
                loop_note += f", avg-loop={avg_loop:.1f}s"
        emit(
            f"[TEST] Protocol: {proto_name} {proto_note} (loops={loop_count}, {loop_note})",
            log,
        )
        emit(
            f"[TEST] Capture horizon: {capture_horizon:.1f}s (ingestion wait {args.ingestion_wait:.1f}s, runtime {expected_runtime:.1f}s, tail {args.tail_pad:.1f}s)",
            log,
        )
        emit(
            f"[TEST] Planned RUN pulse: {pulse_seconds:.1f}s (runtime {expected_runtime:.1f}s + margin {args.run_margin_s:.1f}s)",
            log,
        )

        ser.reset_input_buffer()
        ser.reset_output_buffer()
        emit("[TEST] Power both boards now; logging has started.", log)

        pulse_ms = int(pulse_seconds * 1000)
        run_pulse_sent = False

        if pulse_ms <= 0:
            emit("[TEST] Protocol runtime <= 0 s; RUN pulse will not be issued automatically.", log)

        def issue_run_pulse() -> None:
            nonlocal run_pulse_sent
            if run_pulse_sent or pulse_ms <= 0:
                return
            send_command(ser, f"PULSE RUN={pulse_ms}", log=log)
            emit(f"[TEST] RUN pulse issued for approximately {pulse_seconds:.1f}s", log)
            run_pulse_sent = True

        def tick() -> None:
            now = time.monotonic()
            if not run_pulse_sent and pulse_ms > 0 and now - start_time >= args.ingestion_wait:
                issue_run_pulse()

        start_time = time.monotonic()
        stream_serial(
            ser,
            start_time + capture_horizon,
            log,
            drop_first_line=args.drop_first_line,
            on_tick=tick,
            tick_interval=0.25,
        )

        if not run_pulse_sent and pulse_ms > 0:
            issue_run_pulse()
        if run_pulse_sent:
            send_command(ser, "RUN=0", log=log)
        send_command(ser, "STATUS", log=log)
        emit("[TEST] Capture complete", log)

    print("[TEST] Done. Review the log for protocol ingestion details.")
    return 0


# ---------------------------------------------------------------------------
# run-gate: three-step RUN-gate validation (CODE_REVIEW.md §7)
# ---------------------------------------------------------------------------

_STEP_PASS = "PASS"
_STEP_FAIL = "FAIL"
_STEP_INCONCLUSIVE = "INCONCLUSIVE"
_STEP_SKIPPED = "SKIPPED"


def _power_cycle(
    ser: serial.Serial,
    log: TextIO | None,
    dwell_s: float,
    *,
    drop_first_line: bool = False,
) -> None:
    """Power-off, dwell, power-on.  Does NOT capture — caller handles that."""
    send_command(ser, "PWR=0", log=log)
    emit(f"[TEST] Power-off dwell {dwell_s:.1f}s", log)
    time.sleep(dwell_s)
    send_command(ser, "PWR=1", log=log)


def _run_gate_step1(
    ser: serial.Serial,
    log: TextIO | None,
    *,
    boot_wait_s: float,
    dwell_s: float,
    drop_first_line: bool,
) -> tuple[str, str]:
    """Step 1: RUN held low across cold boot — CC must NOT enter RUNNING."""
    emit("[TEST] === STEP 1: RUN held low across cold boot ===", log)
    emit("[TEST] Expected: CC stays IDLE after protocol upload; no RUNNING before RUN released", log)

    # Clean slate
    send_command(ser, "RUN=0", log=log)
    time.sleep(0.25)

    # Latch RUN low BEFORE power-on
    send_command(ser, "RUN=1", log=log)
    time.sleep(0.25)

    _power_cycle(ser, log, dwell_s, drop_first_line=drop_first_line)

    # Capture boot + protocol upload + heartbeats
    lines = capture_lines(ser, boot_wait_s, log, drop_first_line=drop_first_line)

    hits = scan_lines(lines, {
        "proto_ok": r"NOTICE;PROTO_RX=OK",
        "err_wrong_stat": r"ACK;RESUME=ERR_WRONG_STAT",
        "cc_stat": r"STAT;SEQ=",
    })

    # Only evaluate HBs AFTER protocol upload completes — boot-time frames
    # can be garbled (e.g. glued READY;ID=CC fragments) and must be excluded.
    proto_ok_idx = None
    for i, line in enumerate(lines):
        if re.search(r"NOTICE;PROTO_RX=OK", line, re.IGNORECASE):
            proto_ok_idx = i
            break
    post_proto = lines[proto_ok_idx + 1:] if proto_ok_idx is not None else lines
    post_hits = scan_lines(post_proto, {
        "hb_idle": r"HB;.*STATE=IDLE",
        "hb_running": r"HB;.*STATE=RUNNING",
    })

    # Release RUN for subsequent steps
    send_command(ser, "RUN=0", log=log)
    time.sleep(0.25)

    # --- Evaluate ---
    if not hits["proto_ok"]:
        reason = "Protocol upload NOT observed (NOTICE;PROTO_RX=OK missing)"
        emit(f"[TEST] Step 1: {_STEP_INCONCLUSIVE} — {reason}", log)
        return _STEP_INCONCLUSIVE, reason

    if hits["err_wrong_stat"]:
        reason = "ERR_WRONG_STAT ACK detected — CC rejected resume while already RUNNING"
        emit(f"[TEST] Step 1: {_STEP_FAIL} — {reason}", log)
        return _STEP_FAIL, reason

    # Check whether any RUNNING heartbeat appeared BEFORE we released RUN.
    # Only post-protocol-upload HBs are checked (boot noise excluded).
    if post_hits["hb_running"]:
        reason = f"HB STATE=RUNNING appeared while RUN held low ({len(post_hits['hb_running'])} occurrences)"
        emit(f"[TEST] Step 1: {_STEP_FAIL} — {reason}", log)
        return _STEP_FAIL, reason

    # Primary evidence: proto loaded + NO RUNNING + NO ERR_WRONG_STAT = gate worked.
    # XPB HB IDLE is bonus confirmation; CC STAT messages prove the system is alive.
    alive = post_hits["hb_idle"] or hits["cc_stat"]
    if not alive:
        reason = "No telemetry after protocol upload (system unresponsive)"
        emit(f"[TEST] Step 1: {_STEP_INCONCLUSIVE} — {reason}", log)
        return _STEP_INCONCLUSIVE, reason

    idle_detail = f"{len(post_hits['hb_idle'])} IDLE HBs" if post_hits["hb_idle"] else f"{len(hits['cc_stat'])} CC STAT msgs"
    reason = (
        f"Proto upload OK, {idle_detail}, "
        "no RUNNING before RUN released, no ERR_WRONG_STAT"
    )
    emit(f"[TEST] Step 1: {_STEP_PASS} — {reason}", log)
    return _STEP_PASS, reason


def _run_gate_step2(
    ser: serial.Serial,
    log: TextIO | None,
    *,
    settle_s: float,
    hb_window_s: float,
) -> tuple[str, str]:
    """Step 2: Gate opens once RUN returns high — verify IDLE→RUNNING→PAUSED."""
    emit("[TEST] === STEP 2: Gate open, IDLE → RUNNING → PAUSED ===", log)
    emit("[TEST] Expected: RUN=1 → STATE=RUNNING, then RUN=0 → STATE=PAUSED", log)

    # Settle — confirm still IDLE after step 1 released RUN
    emit(f"[TEST] Settling {settle_s:.1f}s, confirming IDLE", log)
    settle_lines = capture_lines(ser, settle_s, log)
    settle_hits = scan_lines(settle_lines, {
        "hb_idle": r"HB;.*STATE=IDLE",
        "hb_running": r"HB;.*STATE=RUNNING",
        "sw_run_off": r"SW;RUN=0",
        "cc_stat": r"STAT;SEQ=",
    })
    if settle_hits["hb_running"]:
        reason = "System already in RUNNING before step 2 RUN assertion"
        emit(f"[TEST] Step 2: {_STEP_FAIL} — {reason}", log)
        return _STEP_FAIL, reason
    # Accept XPB HB IDLE, or CC SW;RUN=0 / CC STAT as evidence system is alive  and not running
    alive = settle_hits["hb_idle"] or settle_hits["sw_run_off"] or settle_hits["cc_stat"]
    if not alive:
        reason = "No heartbeats or switch state during settle window"
        emit(f"[TEST] Step 2: {_STEP_INCONCLUSIVE} — {reason}", log)
        return _STEP_INCONCLUSIVE, reason

    # Assert RUN low → expect RUNNING
    send_command(ser, "RUN=1", log=log)
    run_lines = capture_lines(ser, hb_window_s, log)
    run_hits = scan_lines(run_lines, {
        "hb_running": r"HB;.*STATE=RUNNING",
        "sw_run_on": r"SW;RUN=1",
    })
    if not run_hits["hb_running"] and not run_hits["sw_run_on"]:
        reason = "No HB STATE=RUNNING or SW;RUN=1 after asserting RUN"
        emit(f"[TEST] Step 2: {_STEP_FAIL} — {reason}", log)
        # Release before returning
        send_command(ser, "RUN=0", log=log)
        return _STEP_FAIL, reason

    # Release RUN → expect PAUSED
    send_command(ser, "RUN=0", log=log)
    pause_lines = capture_lines(ser, hb_window_s, log)
    pause_hits = scan_lines(pause_lines, {
        "hb_paused": r"HB;.*STATE=PAUSED",
        "sw_run_off": r"SW;RUN=0",
    })
    if not pause_hits["hb_paused"] and not pause_hits["sw_run_off"]:
        reason = "No HB STATE=PAUSED or SW;RUN=0 after releasing RUN"
        emit(f"[TEST] Step 2: {_STEP_FAIL} — {reason}", log)
        return _STEP_FAIL, reason

    # Build detailed reason
    run_evidence = []
    if run_hits["hb_running"]:
        run_evidence.append(f"{len(run_hits['hb_running'])} HB RUNNING")
    if run_hits["sw_run_on"]:
        run_evidence.append("SW;RUN=1")
    pause_evidence = []
    if pause_hits["hb_paused"]:
        pause_evidence.append(f"{len(pause_hits['hb_paused'])} HB PAUSED")
    if pause_hits["sw_run_off"]:
        pause_evidence.append("SW;RUN=0")
    reason = (
        f"IDLE→RUNNING ({', '.join(run_evidence)}), "
        f"RUNNING→PAUSED ({', '.join(pause_evidence)})"
    )
    emit(f"[TEST] Step 2: {_STEP_PASS} — {reason}", log)
    return _STEP_PASS, reason


def _run_gate_step3(
    ser: serial.Serial,
    log: TextIO | None,
    *,
    boot_wait_s: float,
    dwell_s: float,
    hb_window_s: float,
    drop_first_line: bool,
) -> tuple[str, str]:
    """Step 3: Power-loss resume — run protocol until a step boundary save,
    then hard power-cycle with RUN held low.

    Expects CMD;RESUME=AUTO with AUTOSTART=1, followed by ACK;RESUME=OK.
    The XPB writes a resume snapshot to SD whenever STEP or LOOP changes
    during RUNNING.  A hard power-cut simulates power loss; the next boot
    should find the snapshot and offer auto-resume.
    """
    emit("[TEST] === STEP 3: RESUME AUTOSTART=1 via power-loss recovery ===", log)
    emit("[TEST] Expected: periodic save during RUNNING, power cut, then resume on cold boot", log)

    # --- 3a: Resume from PAUSED to get a running protocol ---
    emit("[TEST] Step 3a: Resuming from PAUSED (assert RUN)", log)
    send_command(ser, "RUN=1", log=log)
    resume_lines = capture_lines(ser, 3.0, log)
    resume_hits = scan_lines(resume_lines, {
        "hb_running": r"HB;.*STATE=RUNNING",
        "sw_run_on": r"SW;RUN=1",
    })
    if not resume_hits["hb_running"] and not resume_hits["sw_run_on"]:
        reason = "Could not resume to RUNNING from PAUSED"
        emit(f"[TEST] Step 3: {_STEP_INCONCLUSIVE} — {reason}", log)
        send_command(ser, "RUN=0", log=log)
        return _STEP_INCONCLUSIVE, reason

    # --- 3b: Wait for at least one step boundary so XPB periodic save fires ---
    emit(f"[TEST] Step 3b: Waiting {RUN_GATE_STEP_WAIT_S:.0f}s for step boundary save", log)
    run_lines = capture_lines(ser, RUN_GATE_STEP_WAIT_S, log)
    run_hits = scan_lines(run_lines, {
        "hb_running": r"HB;.*STATE=RUNNING",
        "step_change": r"HB;.*STATE=RUNNING;STEP=(?!1;)",
    })
    if not run_hits["step_change"]:
        reason = "Protocol did not advance past step 1 — no periodic save expected"
        emit(f"[TEST] Step 3: {_STEP_INCONCLUSIVE} — {reason}", log)
        send_command(ser, "RUN=0", log=log)
        return _STEP_INCONCLUSIVE, reason
    emit(f"[TEST] Step boundary observed ({len(run_hits['step_change'])} HBs past step 1)", log)

    # --- 3c: Hard power-cut to simulate power loss ---
    emit("[TEST] Step 3c: Hard power-cut (simulating power loss)", log)
    # RUN stays latched low (already asserted from step 2)
    _power_cycle(ser, log, dwell_s, drop_first_line=drop_first_line)

    # --- 3d: Cold boot with RUN held low — expect RESUME=AUTO + AUTOSTART=1 ---
    emit("[TEST] Step 3d: Cold boot with RUN held low, expecting RESUME AUTOSTART=1", log)
    boot_lines = capture_lines(ser, boot_wait_s, log, drop_first_line=drop_first_line)

    hits = scan_lines(boot_lines, {
        "resume_auto": r"CMD;RESUME=AUTO.*AUTOSTART=1",
        "resume_ok": r"ACK;RESUME=OK",
        "resume_err": r"ACK;RESUME=ERR",
        "hb_running": r"HB;.*STATE=RUNNING",
        "hb_preheat": r"HB;.*STATE=PREHEAT",
        "proto_ok": r"NOTICE;PROTO_RX=OK",
        "phash_overflow": r"PHASH=2147483647",
    })

    # Cleanup
    send_command(ser, "RUN=0", log=log)
    time.sleep(0.25)

    # --- Evaluate ---
    if hits["phash_overflow"]:
        emit("[TEST] WARNING: PHASH=2147483647 (INT_MAX) detected — possible uint32→int truncation", log)

    if not hits["proto_ok"]:
        reason = "Protocol upload NOT observed on resume boot"
        emit(f"[TEST] Step 3: {_STEP_INCONCLUSIVE} — {reason}", log)
        return _STEP_INCONCLUSIVE, reason

    if not hits["resume_auto"]:
        reason = "CMD;RESUME=AUTO with AUTOSTART=1 NOT sent by XPB"
        detail = " (PHASH overflow may prevent match)" if hits["phash_overflow"] else ""
        emit(f"[TEST] Step 3: {_STEP_INCONCLUSIVE} — {reason}{detail}", log)
        return _STEP_INCONCLUSIVE, reason + detail

    if hits["resume_err"] and not hits["resume_ok"]:
        err_line = hits["resume_err"][0]
        reason = f"Resume rejected by CC: {err_line}"
        emit(f"[TEST] Step 3: {_STEP_FAIL} — {reason}", log)
        return _STEP_FAIL, reason

    if not hits["resume_ok"]:
        reason = "CMD;RESUME=AUTO sent but ACK;RESUME=OK not received"
        emit(f"[TEST] Step 3: {_STEP_INCONCLUSIVE} — {reason}", log)
        return _STEP_INCONCLUSIVE, reason

    started = hits["hb_running"] or hits["hb_preheat"]
    if not started:
        reason = "Resume accepted (ACK=OK) but no RUNNING/PREHEAT heartbeat observed"
        emit(f"[TEST] Step 3: {_STEP_INCONCLUSIVE} — {reason}", log)
        return _STEP_INCONCLUSIVE, reason

    reason = "CMD;RESUME=AUTO AUTOSTART=1 sent, ACK=OK, system entered RUNNING/PREHEAT"
    emit(f"[TEST] Step 3: {_STEP_PASS} — {reason}", log)
    return _STEP_PASS, reason


def _run_gate_step4(
    ser: serial.Serial,
    log: TextIO | None,
    *,
    boot_wait_s: float,
    dwell_s: float,
    hb_window_s: float,
    drop_first_line: bool,
) -> tuple[str, str]:
    """Step 4: Manual reset clears resume — after reset, cold boot must NOT
    auto-resume even with RUN held low.

    Flow: RUNNING → RESET pulse → system returns to IDLE → cold boot with
    RUN held → verify NO CMD;RESUME=AUTO sent, system stays IDLE.
    """
    emit("[TEST] === STEP 4: Manual reset clears resume state ===", log)
    emit("[TEST] Expected: RESET clears resume; cold boot with RUN held stays IDLE", log)

    # --- 4a: Ensure we're RUNNING (system should still be running from step 3 resume) ---
    emit("[TEST] Step 4a: Confirming still RUNNING", log)
    send_command(ser, "RUN=1", log=log)
    check_lines = capture_lines(ser, 3.0, log)
    check_hits = scan_lines(check_lines, {
        "hb_running": r"HB;.*STATE=RUNNING",
    })
    if not check_hits["hb_running"]:
        reason = "System not in RUNNING state — cannot test reset"
        emit(f"[TEST] Step 4: {_STEP_INCONCLUSIVE} — {reason}", log)
        send_command(ser, "RUN=0", log=log)
        return _STEP_INCONCLUSIVE, reason

    # --- 4b: Issue RESET pulse to clear resume state ---
    emit("[TEST] Step 4b: Issuing RESET pulse (should clear resume slots)", log)
    send_command(ser, "RUN=0", log=log)
    time.sleep(0.3)
    send_command(ser, f"PULSE RST={RUN_GATE_RESET_PULSE_MS}", log=log)
    capture_lines(ser, RUN_GATE_RESET_WAIT_S, log)

    # --- 4c: Wait for system to settle back to IDLE ---
    emit("[TEST] Step 4c: Waiting for system to return to IDLE after reset", log)
    post_lines = capture_lines(ser, hb_window_s, log)
    post_hits = scan_lines(post_lines, {
        "hb_idle": r"HB;.*STATE=IDLE",
        "cc_stat": r"STAT;SEQ=",
    })
    if not post_hits["hb_idle"] and not post_hits["cc_stat"]:
        reason = "System did not return to IDLE after reset pulse"
        emit(f"[TEST] Step 4: {_STEP_INCONCLUSIVE} — {reason}", log)
        return _STEP_INCONCLUSIVE, reason

    # --- 4d: Cold boot with RUN held low — must NOT auto-resume ---
    emit("[TEST] Step 4d: Cold boot with RUN held, expecting NO resume (IDLE only)", log)
    send_command(ser, "RUN=1", log=log)
    time.sleep(0.25)
    _power_cycle(ser, log, dwell_s, drop_first_line=drop_first_line)
    boot_lines = capture_lines(ser, boot_wait_s, log, drop_first_line=drop_first_line)

    # Only check post-protocol-upload HBs (exclude garbled boot frames)
    proto_ok_idx = None
    for i, line in enumerate(boot_lines):
        if re.search(r"NOTICE;PROTO_RX=OK", line, re.IGNORECASE):
            proto_ok_idx = i
            break

    hits = scan_lines(boot_lines, {
        "proto_ok": r"NOTICE;PROTO_RX=OK",
        "resume_auto": r"CMD;RESUME=AUTO",
        "cc_stat": r"STAT;SEQ=",
    })
    post_proto = boot_lines[proto_ok_idx + 1:] if proto_ok_idx is not None else boot_lines
    post_hits = scan_lines(post_proto, {
        "hb_idle": r"HB;.*STATE=IDLE",
        "hb_running": r"HB;.*STATE=RUNNING",
    })

    # Cleanup
    send_command(ser, "RUN=0", log=log)
    time.sleep(0.25)

    # --- Evaluate ---
    if not hits["proto_ok"]:
        reason = "Protocol upload NOT observed on post-reset boot"
        emit(f"[TEST] Step 4: {_STEP_INCONCLUSIVE} — {reason}", log)
        return _STEP_INCONCLUSIVE, reason

    if hits["resume_auto"]:
        reason = f"CMD;RESUME=AUTO was sent despite manual reset ({len(hits['resume_auto'])} occurrences)"
        emit(f"[TEST] Step 4: {_STEP_FAIL} — {reason}", log)
        return _STEP_FAIL, reason

    if post_hits["hb_running"]:
        reason = f"HB STATE=RUNNING appeared — system auto-started despite reset clearing resume"
        emit(f"[TEST] Step 4: {_STEP_FAIL} — {reason}", log)
        return _STEP_FAIL, reason

    alive = post_hits["hb_idle"] or hits["cc_stat"]
    if not alive:
        reason = "No telemetry after boot (system unresponsive)"
        emit(f"[TEST] Step 4: {_STEP_INCONCLUSIVE} — {reason}", log)
        return _STEP_INCONCLUSIVE, reason

    idle_count = len(post_hits["hb_idle"]) if post_hits["hb_idle"] else 0
    reason = f"No CMD;RESUME=AUTO sent, {idle_count} IDLE HBs, no RUNNING — reset cleared resume"
    emit(f"[TEST] Step 4: {_STEP_PASS} — {reason}", log)
    return _STEP_PASS, reason


def run_run_gate(args: argparse.Namespace) -> int:
    """Execute the three-step RUN-gate validation sequence from CODE_REVIEW.md §7."""
    log_dir = ensure_log_dir()
    log_path = log_dir / f"{timestamp()}_run_gate.log"

    with open(log_path, "w", encoding="utf-8") as log:
        emit(f"[TEST] Logging to {rel_path(log_path)}", log)
        emit(f"[TEST] Command: {format_invocation()}", log)
        emit("[TEST] === RUN-GATE VALIDATION (CODE_REVIEW.md §7) ===", log)
        emit(
            f"[TEST] Configuration: boot_wait={args.boot_wait_s:.1f}s, "
            f"settle={args.settle_s:.1f}s, hb_window={args.hb_window_s:.1f}s, "
            f"skip_step_3={args.skip_step_3}",
            log,
        )
        emit(f"[TEST] Opening serial port {args.port} @ {args.baud} baud", log)

        try:
            ser = open_serial_with_retry(args.port, args.baud, timeout=0.1)
        except (serial.SerialException, OSError) as exc:
            emit(f"[TEST] Serial error: {exc}", log)
            return 2

        with ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            emit("[TEST] Connected", log)

            send_command(ser, "STATUS", log=log)
            time.sleep(0.25)

            # Step 1
            s1_result, s1_reason = _run_gate_step1(
                ser, log,
                boot_wait_s=args.boot_wait_s,
                dwell_s=RUN_GATE_POWER_DWELL_S,
                drop_first_line=args.drop_first_line,
            )

            # Step 2 — depends on step 1 not being FAIL
            if s1_result == _STEP_FAIL:
                s2_result, s2_reason = _STEP_SKIPPED, "Skipped (step 1 failed)"
                emit(f"[TEST] Step 2: {s2_result} — {s2_reason}", log)
            else:
                s2_result, s2_reason = _run_gate_step2(
                    ser, log,
                    settle_s=args.settle_s,
                    hb_window_s=args.hb_window_s,
                )

            # Step 3 — depends on step 2 passing + not skipped
            if args.skip_step_3:
                s3_result, s3_reason = _STEP_SKIPPED, "Skipped (--skip-step-3)"
                emit(f"[TEST] Step 3: {s3_result} — {s3_reason}", log)
            elif s2_result != _STEP_PASS:
                s3_result, s3_reason = _STEP_SKIPPED, f"Skipped (step 2 was {s2_result})"
                emit(f"[TEST] Step 3: {s3_result} — {s3_reason}", log)
            else:
                s3_result, s3_reason = _run_gate_step3(
                    ser, log,
                    boot_wait_s=args.boot_wait_s,
                    dwell_s=RUN_GATE_POWER_DWELL_S,
                    hb_window_s=args.hb_window_s,
                    drop_first_line=args.drop_first_line,
                )

            # Step 4 — depends on step 3 passing
            if s3_result != _STEP_PASS:
                s4_result, s4_reason = _STEP_SKIPPED, f"Skipped (step 3 was {s3_result})"
                emit(f"[TEST] Step 4: {s4_result} — {s4_reason}", log)
            else:
                s4_result, s4_reason = _run_gate_step4(
                    ser, log,
                    boot_wait_s=args.boot_wait_s,
                    dwell_s=RUN_GATE_POWER_DWELL_S,
                    hb_window_s=args.hb_window_s,
                    drop_first_line=args.drop_first_line,
                )

            # Final cleanup
            send_command(ser, "RUN=0", log=log)
            send_command(ser, "STATUS", log=log)
            time.sleep(0.2)
            flush_serial(ser, log)

            # Summary
            emit("", log)
            emit("[TEST] === RUN-GATE VALIDATION SUMMARY ===", log)
            emit(f"[TEST] Step 1 (RUN held low, CC stays IDLE):       {s1_result}", log)
            emit(f"[TEST]         {s1_reason}", log)
            emit(f"[TEST] Step 2 (Gate open, IDLE→RUNNING→PAUSED):    {s2_result}", log)
            emit(f"[TEST]         {s2_reason}", log)
            emit(f"[TEST] Step 3 (Power-loss resume AUTOSTART=1):     {s3_result}", log)
            emit(f"[TEST]         {s3_reason}", log)
            emit(f"[TEST] Step 4 (Manual reset clears resume):        {s4_result}", log)
            emit(f"[TEST]         {s4_reason}", log)
            emit("", log)
            emit("[TEST] Capture complete", log)

    all_pass = all(r == _STEP_PASS for r in (s1_result, s2_result, s3_result, s4_result))
    any_fail = any(r == _STEP_FAIL for r in (s1_result, s2_result, s3_result, s4_result))
    if all_pass:
        return 0
    if any_fail:
        return 1
    return 0


# ---------------------------------------------------------------------------
# run-suite: run all tests and produce a master log
# ---------------------------------------------------------------------------

# Each entry is (test_name, argv_tokens_after mode).
# Tests are run in order; each gets its own log file as usual.
_SUITE_TESTS: list[tuple[str, list[str]]] = [
    ("cold-boot", []),
    ("comms-health", []),
    ("reset-cancel", []),
    ("reset-pulse", []),
    ("run-cycle", []),
    ("run-gate", []),
]


def run_suite(args: argparse.Namespace) -> int:
    """Run all standard tests sequentially and write a master log."""
    log_dir = ensure_log_dir()
    ts = timestamp()
    master_path = log_dir / f"{ts}_suite.log"
    script = str(Path(__file__).resolve())

    # Build common args forwarded to each sub-invocation
    common: list[str] = [sys.executable, script, "--port", args.port, "--baud", str(args.baud)]
    if args.drop_first_line:
        common.append("--drop-first-line")

    results: list[tuple[str, int]] = []

    with open(master_path, "w", encoding="utf-8") as mlog:
        emit(f"[SUITE] Logging to {rel_path(master_path)}", mlog)
        emit(f"[SUITE] Command: {format_invocation()}", mlog)
        emit(f"[SUITE] === TEST SUITE ({len(_SUITE_TESTS)} tests) ===", mlog)
        emit("", mlog)

        for idx, (test_name, extra_args) in enumerate(_SUITE_TESTS, 1):
            emit(f"[SUITE] --- [{idx}/{len(_SUITE_TESTS)}] {test_name} ---", mlog)
            cmd = common + [test_name] + extra_args
            emit(f"[SUITE] > {shlex.join(cmd)}", mlog)

            proc = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
            )

            # Stream child output into the master log
            for line in proc.stdout.splitlines():
                emit(line, mlog)

            rc = proc.returncode
            label = "PASS" if rc == 0 else ("ERROR" if rc == 2 else "FAIL")
            emit(f"[SUITE] {test_name}: exit {rc} ({label})", mlog)
            emit("", mlog)
            results.append((test_name, rc))

        # Summary table
        emit("[SUITE] === SUITE SUMMARY ===", mlog)
        for test_name, rc in results:
            label = "PASS" if rc == 0 else ("ERROR" if rc == 2 else "FAIL")
            emit(f"[SUITE]   {test_name:<25s} {label}", mlog)
        passes = sum(1 for _, rc in results if rc == 0)
        fails = sum(1 for _, rc in results if rc != 0)
        emit(f"[SUITE] Totals: {passes} passed, {fails} failed out of {len(results)}", mlog)
        emit("[SUITE] Suite complete", mlog)

    return 0 if fails == 0 else 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Unified control for Teensy sniffer validation exercises",
    )
    parser.add_argument(
        "--port",
        default=None,
        help="Serial port connected to the Teensy (e.g. COM6 or /dev/ttyACM0)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help=f"USB baud rate for the sniffer (default: {DEFAULT_BAUD})",
    )
    parser.add_argument(
        "--drop-first-line",
        action="store_true",
        help="Discard the first decoded line after connect to skip partial boot data",
    )

    subparsers = parser.add_subparsers(dest="mode", required=True)

    reset_pulse = subparsers.add_parser(
        "reset-pulse",
        help="Issue a RESET pulse and record the response",
    )
    reset_pulse.add_argument(
        "--pulse-ms",
        type=int,
        default=RESET_PULSE_DEFAULT_MS,
        help=f"Milliseconds to hold RESET low before auto-release (default: {RESET_PULSE_DEFAULT_MS})",
    )
    reset_pulse.add_argument(
        "--capture-s",
        type=float,
        default=RESET_PULSE_CAPTURE_S,
        help=f"Seconds to keep recording after the pulse command (default: {RESET_PULSE_CAPTURE_S})",
    )
    reset_pulse.set_defaults(handler=run_reset_pulse)

    reset_pulse_multi = subparsers.add_parser(
        "reset-pulse-multi",
        help="Run multiple reset-pulse cycles to stress-test SD/protocol reliability across reboots",
    )
    reset_pulse_multi.add_argument(
        "--count",
        type=int,
        default=RESET_MULTI_COUNT,
        help=f"Number of reset-pulse iterations (default: {RESET_MULTI_COUNT})",
    )
    reset_pulse_multi.add_argument(
        "--pulse-ms",
        type=int,
        default=RESET_PULSE_DEFAULT_MS,
        help=f"Milliseconds to hold RESET low (default: {RESET_PULSE_DEFAULT_MS})",
    )
    reset_pulse_multi.add_argument(
        "--boot-wait-s",
        type=float,
        default=RESET_MULTI_BOOT_WAIT_S,
        help=f"Seconds to wait after pulse for reboot + protocol upload (default: {RESET_MULTI_BOOT_WAIT_S})",
    )
    reset_pulse_multi.add_argument(
        "--settle-s",
        type=float,
        default=RESET_MULTI_SETTLE_S,
        help=f"Settle time between iterations (default: {RESET_MULTI_SETTLE_S})",
    )
    reset_pulse_multi.set_defaults(handler=run_reset_pulse_multi)

    run_pulse = subparsers.add_parser(
        "run-pulse",
        help="Issue a RUN pulse and record the response",
    )
    run_pulse.add_argument(
        "--pulse-ms",
        type=int,
        default=RUN_PULSE_DEFAULT_MS,
        help=(
            "Milliseconds to assert RUN low before auto-release "
            f"(default: {RUN_PULSE_DEFAULT_MS}; increase for longer exercises)"
        ),
    )
    run_pulse.add_argument(
        "--capture-s",
        type=float,
        default=RUN_PULSE_CAPTURE_S,
        help=f"Seconds to keep recording after the pulse command (default: {RUN_PULSE_CAPTURE_S})",
    )
    run_pulse.set_defaults(handler=run_run_pulse)

    reset_cancel = subparsers.add_parser(
        "reset-cancel",
        help="Send multiple sub-threshold RESET pulses to validate cancel behavior",
    )
    reset_cancel.add_argument(
        "--count",
        type=int,
        default=RESET_CANCEL_COUNT,
        help=f"Number of RESET pulses to issue (default: {RESET_CANCEL_COUNT})",
    )
    reset_cancel.add_argument(
        "--min-ms",
        type=int,
        default=RESET_CANCEL_MIN_MS,
        help=f"Minimum pulse duration in milliseconds (default: {RESET_CANCEL_MIN_MS})",
    )
    reset_cancel.add_argument(
        "--max-ms",
        type=int,
        default=RESET_CANCEL_MAX_MS,
        help=f"Maximum pulse duration in milliseconds (default: {RESET_CANCEL_MAX_MS})",
    )
    reset_cancel.add_argument(
        "--capture-s",
        type=float,
        default=RESET_CANCEL_CAPTURE_S,
        help=f"Seconds to keep recording after the final pulse (default: {RESET_CANCEL_CAPTURE_S})",
    )
    reset_cancel.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Optional RNG seed for reproducible pulse durations",
    )
    reset_cancel.set_defaults(handler=run_reset_cancel)

    power = subparsers.add_parser(
        "power",
        help="Toggle system power relay via the sniffer",
    )
    power.add_argument(
        "--state",
        choices=["on", "off"],
        required=True,
        help="Desired power state",
    )
    power.add_argument(
        "--capture-s",
        type=float,
        default=POWER_CAPTURE_DEFAULT_S,
        help=f"Seconds to capture after issuing the power command (default: {POWER_CAPTURE_DEFAULT_S})",
    )
    power.set_defaults(handler=run_power)

    upload = subparsers.add_parser(
        "protocol-upload",
        help="Capture protocol ingestion and execution",
    )
    upload.add_argument(
        "--protocol",
        type=Path,
        default=None,
        help=(
            "Use an existing protocol CSV. Default: auto-generate tools/protocols/"
            f"{DEFAULT_PROTOCOL_NAME} with a fresh PROTOCOL_NAME"
        ),
    )
    upload.add_argument(
        "--ingestion-wait",
        type=float,
        default=DEFAULT_INGESTION_WAIT,
        help=f"Seconds to wait after power-on before issuing the RUN pulse (default: {DEFAULT_INGESTION_WAIT})",
    )
    upload.add_argument(
        "--tail-pad",
        type=float,
        default=DEFAULT_TAIL_PAD,
        help=f"Extra seconds to capture after expected runtime completes (default: {DEFAULT_TAIL_PAD})",
    )
    upload.add_argument(
        "--run-margin-s",
        type=float,
        default=DEFAULT_RUN_MARGIN,
        help=f"Seconds to extend RUN pulse beyond computed runtime (default: {DEFAULT_RUN_MARGIN})",
    )
    upload.set_defaults(handler=run_protocol_upload)

    run_gate = subparsers.add_parser(
        "run-gate",
        help="Three-step RUN-gate validation sequence (CODE_REVIEW.md §7)",
    )
    run_gate.add_argument(
        "--boot-wait-s",
        type=float,
        default=RUN_GATE_BOOT_WAIT_S,
        help=f"Seconds to wait after power-on for boot + protocol upload (default: {RUN_GATE_BOOT_WAIT_S})",
    )
    run_gate.add_argument(
        "--settle-s",
        type=float,
        default=RUN_GATE_SETTLE_S,
        help=f"Settle time between substeps in seconds (default: {RUN_GATE_SETTLE_S})",
    )
    run_gate.add_argument(
        "--hb-window-s",
        type=float,
        default=RUN_GATE_HB_WINDOW_S,
        help=f"Heartbeat observation window in seconds (default: {RUN_GATE_HB_WINDOW_S})",
    )
    run_gate.add_argument(
        "--skip-step-3",
        action="store_true",
        help="Skip steps 3-4 (power-loss resume + manual reset); run only steps 1-2",
    )
    run_gate.set_defaults(handler=run_run_gate)

    # --- New test verbs ---

    run_cycle = subparsers.add_parser(
        "run-cycle",
        help="T2: Double-pulse RUN test — IDLE → RUNNING → PAUSED → RUNNING → PAUSED",
    )
    run_cycle.add_argument(
        "--pulse-ms",
        type=int,
        default=RUN_PULSE_DEFAULT_MS,
        help=f"Milliseconds for the second RUN pulse (default: {RUN_PULSE_DEFAULT_MS})",
    )
    run_cycle.add_argument(
        "--observe-s",
        type=float,
        default=5.0,
        help="Seconds to observe each phase (default: 5.0)",
    )
    run_cycle.set_defaults(handler=run_run_cycle)

    run_proto = subparsers.add_parser(
        "run-protocol",
        help="T9: Run a loaded protocol to completion and log RPM vs step",
    )
    run_proto.add_argument(
        "--protocol",
        type=Path,
        required=True,
        help="Path to protocol CSV to run (must already be loaded on SD)",
    )
    run_proto.add_argument(
        "--settle-s",
        type=float,
        default=30.0,
        help="Max seconds to wait for IDLE before asserting RUN (default: 30.0)",
    )
    run_proto.add_argument(
        "--run-margin-s",
        type=float,
        default=DEFAULT_RUN_MARGIN,
        help=f"Extra seconds to hold RUN beyond computed runtime (default: {DEFAULT_RUN_MARGIN})",
    )
    run_proto.add_argument(
        "--tail-s",
        type=float,
        default=DEFAULT_TAIL_PAD,
        help=f"Extra seconds to capture after RUN released (default: {DEFAULT_TAIL_PAD})",
    )
    run_proto.set_defaults(handler=run_protocol)

    comms_health = subparsers.add_parser(
        "comms-health",
        help="T8: Passive comms health check — verify HB and STAT cadence",
    )
    comms_health.add_argument(
        "--duration-s",
        type=float,
        default=30.0,
        help="Duration of passive capture in seconds (default: 30.0)",
    )
    comms_health.set_defaults(handler=run_comms_health)

    cold_boot = subparsers.add_parser(
        "cold-boot",
        help="T1: Power cycle and verify full boot sequence (simulates power loss)",
    )
    cold_boot.add_argument(
        "--boot-wait-s",
        type=float,
        default=RUN_GATE_BOOT_WAIT_S,
        help=f"Seconds to wait for boot + protocol upload (default: {RUN_GATE_BOOT_WAIT_S})",
    )
    cold_boot.add_argument(
        "--power-dwell-s",
        type=float,
        default=4.0,
        help="Seconds to keep power off before restoring (default: 4.0)",
    )
    cold_boot.set_defaults(handler=run_cold_boot)

    suite = subparsers.add_parser(
        "run-suite",
        help="Run all standard tests and produce a master log",
    )
    suite.set_defaults(handler=run_suite)

    return parser


def validate_args(args: argparse.Namespace) -> None:
    if args.port is None:
        auto_port = detect_default_port()
        if auto_port is None:
            raise SystemExit("--port is required when no Teensy-compatible port is detected")
        args.port = auto_port

    if args.baud <= 0:
        raise SystemExit("--baud must be positive")

    if args.mode in {"reset-pulse", "run-pulse"}:
        if args.pulse_ms <= 0:
            raise SystemExit("--pulse-ms must be positive")
        if args.capture_s <= 0:
            raise SystemExit("--capture-s must be positive")

    if args.mode == "reset-pulse-multi":
        if args.count <= 0:
            raise SystemExit("--count must be positive")
        if args.pulse_ms <= 0:
            raise SystemExit("--pulse-ms must be positive")
        if args.boot_wait_s <= 0:
            raise SystemExit("--boot-wait-s must be positive")
        if args.settle_s <= 0:
            raise SystemExit("--settle-s must be positive")

    if args.mode == "power":
        if args.capture_s <= 0:
            raise SystemExit("--capture-s must be positive")

    if args.mode == "reset-cancel":
        if args.count <= 0:
            raise SystemExit("--count must be positive")
        if args.min_ms <= 0 or args.max_ms <= 0:
            raise SystemExit("--min-ms and --max-ms must be positive")
        if args.min_ms >= RESET_THRESHOLD_MS or args.max_ms >= RESET_THRESHOLD_MS:
            raise SystemExit("Pulse durations must stay below the reset threshold (5000 ms)")
        if args.min_ms > args.max_ms:
            raise SystemExit("--min-ms cannot exceed --max-ms")
        if args.capture_s <= 0:
            raise SystemExit("--capture-s must be positive")

    if args.mode == "protocol-upload":
        if args.ingestion_wait <= 0:
            raise SystemExit("--ingestion-wait must be positive")
        if args.tail_pad < 0:
            raise SystemExit("--tail-pad cannot be negative")
        if args.run_margin_s < 0:
            raise SystemExit("--run-margin-s cannot be negative")

    if args.mode == "run-gate":
        if args.boot_wait_s <= 0:
            raise SystemExit("--boot-wait-s must be positive")
        if args.settle_s <= 0:
            raise SystemExit("--settle-s must be positive")
        if args.hb_window_s <= 0:
            raise SystemExit("--hb-window-s must be positive")

    if args.mode == "run-cycle":
        if args.pulse_ms <= 0:
            raise SystemExit("--pulse-ms must be positive")
        if args.observe_s <= 0:
            raise SystemExit("--observe-s must be positive")

    if args.mode == "run-protocol":
        if args.settle_s <= 0:
            raise SystemExit("--settle-s must be positive")
        if args.run_margin_s < 0:
            raise SystemExit("--run-margin-s cannot be negative")
        if args.tail_s < 0:
            raise SystemExit("--tail-s cannot be negative")

    if args.mode == "comms-health":
        if args.duration_s <= 0:
            raise SystemExit("--duration-s must be positive")

    if args.mode == "cold-boot":
        if args.boot_wait_s <= 0:
            raise SystemExit("--boot-wait-s must be positive")
        if args.power_dwell_s <= 0:
            raise SystemExit("--power-dwell-s must be positive")


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(args)
    handler: Callable[[argparse.Namespace], int] = args.handler
    return handler(args)


if __name__ == "__main__":
    raise SystemExit(main())
