#!/usr/bin/env python3
"""USB sniffer stream logger for Teensy-based TTL captures."""
# typical use: python tools/ttl_stream_capture.py --log tools/ttl_capture.log --timestamp

import argparse
import sys
import time
from datetime import datetime

try:
    import serial  # type: ignore
    from serial.tools import list_ports  # type: ignore
except ImportError as exc:  # pragma: no cover - guidance for users
    raise SystemExit(
        "pyserial is required. Install with `pip install pyserial`."
    ) from exc


def detect_default_port() -> str | None:
    """Return the first port that looks like a Teensy/Arduino sniffer, if any."""
    for port in list_ports.comports():
        desc = (port.description or "").lower()
        if "teensy" in desc or "usb serial" in desc or "nano every" in desc or "arduino" in desc:
            return port.device
    return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stream and optionally log tagged TTL traffic from the Teensy sniffer"
    )
    parser.add_argument(
        "--port",
        default='COM6' if sys.platform.startswith('win') else '/dev/ttyUSB0',
        help="Serial port connected to the sniffer (e.g. COM7 or /dev/ttyUSB0)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=460800,
        help="USB baud rate for the Nano sniffer (default: 460800)",
    )
    parser.add_argument(
        "--log",
        default= "tools/ttl_capture.log",
        type=argparse.FileType("a", encoding="utf-8"),
        help="Optional path to append a timestamped capture log",
    )
    parser.add_argument(
        "--timestamp",
        default=False,
        action="store_true",
        help="Prepend ISO timestamps to each emitted line",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Suppress console echo when only writing to --log",
    )
    parser.add_argument(
        "--drop-first-line",
        action="store_true",
        help="Discard the first decoded line after connect to skip partial boot data",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="Emit each decoded line as hex byte pairs instead of UTF-8 text",
    )
    args = parser.parse_args()
    if not args.port:
        auto_port = detect_default_port()
        if auto_port is None:
            parser.error("--port is required when no Teensy/Arduino-compatible port is detected")
        args.port = auto_port
    return args


def emit(line: str, args: argparse.Namespace) -> None:
    """Write line to console/log based on options."""
    stamped = line
    if args.timestamp:
        stamped = f"{datetime.now().isoformat(timespec='milliseconds')} {line}"
    if not args.quiet:
        print(stamped)
    if args.log:
        args.log.write(stamped + "\n")
        args.log.flush()


def open_serial_with_retry(port: str, baud: int, *, timeout: float, wait_s: float = 15.0) -> serial.Serial:
    """Try to open the serial port, waiting for the board to enumerate if needed."""
    deadline = time.monotonic() + wait_s
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            return serial.Serial(port, baud, timeout=timeout)
        except (serial.SerialException, OSError) as exc:
            last_error = exc
            time.sleep(0.05)
    raise serial.SerialException(f"Timed out waiting for {port} to become available") from last_error


def main() -> int:
    args = parse_args()
    try:
        try:
            ser = open_serial_with_retry(args.port, args.baud, timeout=0.2)
        except (serial.SerialException, OSError) as exc:
            print(f"Serial error: {exc}", file=sys.stderr)
            return 2
        with ser:
            emit(f"Connected to {ser.port} @ {args.baud} baud", args)
            partial = bytearray()
            drop_next_line = bool(args.drop_first_line)
            while True:
                chunk = ser.read(256)
                if not chunk:
                    # keep the loop responsive to Ctrl+C even when idle
                    time.sleep(0.02)
                    continue
                partial.extend(chunk)
                while b"\n" in partial:
                    line, _, remainder = partial.partition(b"\n")
                    partial = bytearray(remainder)
                    if drop_next_line:
                        drop_next_line = False
                        continue
                    if args.raw:
                        raw_bytes = line + b"\n"
                        hex_line = " ".join(f"{byte:02X}" for byte in raw_bytes)
                        if hex_line:
                            emit(hex_line, args)
                        continue
                    try:
                        decoded = line.decode("utf-8", errors="replace")
                    except UnicodeDecodeError:
                        decoded = line.decode("latin-1", errors="replace")
                    decoded = decoded.rstrip("\r")
                    if decoded:
                        emit(decoded, args)
    except KeyboardInterrupt:
        emit("Interrupted by user", args)
        return 0
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        return 2
    finally:
        if args.log:
            args.log.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
