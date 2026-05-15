#!/usr/bin/env python3
"""rig_trace.py — Unified live tracer for the RTM rig.

Merges three event sources into a single timestamped stream:

    [CC ]  ClearCore USB serial (default COM8)
    [XPB]  Expansion-board USB serial (default COM9, optional)
    [NET]  UDP observer frames on port 8889 (CC<->XPB heartbeat/protocol tees)

All lines share one monotonically-increasing relative timestamp so you
can see exactly when CC's dbgln() output happened relative to the HBs
and PR_* frames going out on the wire.

Usage:
    # Live to stdout, no log file. CC serial + UDP only.
    python tools/rig_trace.py

    # Tee everything to a log file, also include XPB serial.
    python tools/rig_trace.py --xpb-port COM9 --log trace.log

    # Capture for a fixed duration (seconds).
    python tools/rig_trace.py --duration-s 30 --log trace.log

Notes:
- XPB serial is shared with the upload pipeline. Close this script
  before re-flashing the XPB env or the upload will fail with
  "port busy".
- CC serial defaults to 9600 baud, XPB to 115200.
- Source tags are fixed-width so columns line up nicely:
      [+12345 ms] [CC ] STATE -> RUNNING
      [+12347 ms] [NET] CC->XPB  10.0.0.10  HB;SEQ=42;...
- This tool sends only the lightweight PC observer beacon needed to enable
    firmware debug teeing; it does not send control commands.
"""

from __future__ import annotations

import argparse
import queue
import socket
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional, TextIO

try:
    import serial  # pyserial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial", file=sys.stderr)
    sys.exit(2)


HOSTS = {
    "10.0.0.10": "CC",
    "10.0.0.11": "XPB",
    "10.0.0.100": "PC",
}
OBSERVER_TARGETS = ("10.0.0.10", "10.0.0.11")
OBSERVER_BEACON = b"OBS;PC=1\n"

DEFAULT_CC_PORT = "COM8"
DEFAULT_CC_BAUD = 9600
DEFAULT_XPB_PORT = None  # opt-in, since it conflicts with the uploader
DEFAULT_XPB_BAUD = 115200
DEFAULT_FIRMWARE_PORT = 8888
DEFAULT_UDP_PORT = 8889
DEFAULT_OBSERVER_INTERVAL_S = 1.0


# ----- helpers --------------------------------------------------------------

def classify_direction(src_ip: str) -> str:
    src = HOSTS.get(src_ip, src_ip)
    if src == "CC":
        return "CC->XPB"
    if src == "XPB":
        return "XPB->CC"
    return f"{src}->?"


def escape_payload(data: bytes) -> str:
    out = []
    for b in data:
        if b == 0x0A:
            out.append("\\n")
        elif b == 0x0D:
            out.append("\\r")
        elif b == 0x09:
            out.append("\\t")
        elif b == 0x5C:
            out.append("\\\\")
        elif 0x20 <= b < 0x7F:
            out.append(chr(b))
        else:
            out.append(f"\\x{b:02x}")
    return "".join(out)


# ----- reader threads -------------------------------------------------------

def serial_reader(
    tag: str,
    port: str,
    baud: int,
    out_q: "queue.Queue[tuple[float, str, str]]",
    start_mono: float,
    stop_evt: threading.Event,
) -> None:
    try:
        ser = serial.Serial(port, baud, timeout=0.2)
    except serial.SerialException as exc:
        out_q.put((time.monotonic() - start_mono, tag, f"<open failed: {exc}>"))
        return
    out_q.put((time.monotonic() - start_mono, tag, f"<opened {port} @ {baud}>"))
    buf = bytearray()
    try:
        while not stop_evt.is_set():
            try:
                chunk = ser.read(256)
            except serial.SerialException as exc:
                out_q.put((time.monotonic() - start_mono, tag, f"<read error: {exc}>"))
                return
            if not chunk:
                continue
            buf.extend(chunk)
            while True:
                idx = buf.find(b"\n")
                if idx < 0:
                    break
                line = bytes(buf[:idx]).rstrip(b"\r")
                del buf[: idx + 1]
                ts = time.monotonic() - start_mono
                # Decode tolerantly; some MCUs emit stray bytes during reset.
                try:
                    text = line.decode("utf-8", errors="replace")
                except Exception:
                    text = repr(line)
                out_q.put((ts, tag, text))
    finally:
        try:
            ser.close()
        except Exception:
            pass


def udp_reader(
    port: int,
    firmware_port: int,
    out_q: "queue.Queue[tuple[float, str, str]]",
    start_mono: float,
    stop_evt: threading.Event,
    observer_interval_s: float | None,
) -> None:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", port))
        sock.settimeout(0.25)
    except OSError as exc:
        out_q.put((time.monotonic() - start_mono, "NET", f"<bind failed: {exc}>"))
        return
    out_q.put((time.monotonic() - start_mono, "NET", f"<listening on :{port}>"))
    next_observer = time.monotonic()
    observer_warned = False
    try:
        while not stop_evt.is_set():
            now = time.monotonic()
            if observer_interval_s is not None and now >= next_observer:
                try:
                    for host in OBSERVER_TARGETS:
                        sock.sendto(OBSERVER_BEACON, (host, firmware_port))
                except OSError as exc:
                    if not observer_warned:
                        out_q.put((now - start_mono, "NET", f"<observer beacon failed: {exc}>"))
                        observer_warned = True
                next_observer = now + observer_interval_s
            try:
                data, addr = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError as exc:
                out_q.put((time.monotonic() - start_mono, "NET", f"<recv err: {exc}>"))
                return
            ts = time.monotonic() - start_mono
            src_ip, _src_port = addr
            direction = classify_direction(src_ip)
            payload = escape_payload(data)
            if payload.endswith("\\n"):
                payload = payload[:-2]
            out_q.put((ts, "NET", f"{direction:<8s} {src_ip:<11s} {payload}"))
    finally:
        try:
            sock.close()
        except Exception:
            pass


# ----- main loop ------------------------------------------------------------

def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--cc-port", default=DEFAULT_CC_PORT,
                   help=f"ClearCore COM port (default {DEFAULT_CC_PORT}; pass empty to disable)")
    p.add_argument("--cc-baud", type=int, default=DEFAULT_CC_BAUD)
    p.add_argument("--xpb-port", default=DEFAULT_XPB_PORT,
                   help="Expansion-board COM port (default disabled; e.g. COM9)")
    p.add_argument("--xpb-baud", type=int, default=DEFAULT_XPB_BAUD)
    p.add_argument("--udp-port", type=int, default=DEFAULT_UDP_PORT)
    p.add_argument("--firmware-port", type=int, default=DEFAULT_FIRMWARE_PORT,
                   help=f"Firmware control port to beacon (default {DEFAULT_FIRMWARE_PORT})")
    p.add_argument("--log", type=Path, default=None,
                   help="Optional path to also write timestamped lines to.")
    p.add_argument("--duration-s", type=float, default=None,
                   help="Stop after N seconds. Default: run until Ctrl-C.")
    p.add_argument("--no-udp", action="store_true",
                   help="Disable UDP capture (useful if port is already bound).")
    p.add_argument("--observer-interval-s", type=float, default=DEFAULT_OBSERVER_INTERVAL_S,
                   help="Seconds between observer beacons to CC/XPB (default 1.0).")
    p.add_argument("--no-observer-beacon", action="store_true",
                   help="Do not announce this PC as a debug observer.")
    return p.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    start_mono = time.monotonic()
    start_iso = datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]
    out_q: "queue.Queue[tuple[float, str, str]]" = queue.Queue()
    stop_evt = threading.Event()
    threads: list[threading.Thread] = []

    if args.cc_port:
        t = threading.Thread(
            target=serial_reader,
            args=("CC ", args.cc_port, args.cc_baud, out_q, start_mono, stop_evt),
            daemon=True,
        )
        t.start()
        threads.append(t)
    if args.xpb_port:
        t = threading.Thread(
            target=serial_reader,
            args=("XPB", args.xpb_port, args.xpb_baud, out_q, start_mono, stop_evt),
            daemon=True,
        )
        t.start()
        threads.append(t)
    if not args.no_udp:
        t = threading.Thread(
            target=udp_reader,
            args=(
                args.udp_port,
                args.firmware_port,
                out_q,
                start_mono,
                stop_evt,
                None if args.no_observer_beacon else args.observer_interval_s,
            ),
            daemon=True,
        )
        t.start()
        threads.append(t)

    if not threads:
        print("ERROR: no sources enabled", file=sys.stderr)
        return 2

    log_fp: Optional[TextIO] = None
    if args.log:
        log_fp = args.log.open("w", encoding="utf-8", newline="")
        log_fp.write(f"# rig_trace started {start_iso}\n")
        log_fp.flush()

    header = (
        f"[rig_trace] start {start_iso}  "
        f"cc={args.cc_port or '-'} xpb={args.xpb_port or '-'} "
        f"udp={'-' if args.no_udp else args.udp_port}"
    )
    print(header)
    if log_fp is not None:
        log_fp.write(header + "\n")
        log_fp.flush()

    deadline = (start_mono + args.duration_s) if args.duration_s else None
    try:
        while True:
            if deadline is not None and time.monotonic() >= deadline:
                break
            try:
                ts, tag, text = out_q.get(timeout=0.2)
            except queue.Empty:
                continue
            line = f"[+{ts*1000:9.1f} ms] [{tag}] {text}"
            print(line)
            if log_fp is not None:
                log_fp.write(line + "\n")
                log_fp.flush()
    except KeyboardInterrupt:
        print("\n[rig_trace] Ctrl-C, stopping...")
    finally:
        stop_evt.set()
        for t in threads:
            t.join(timeout=1.0)
        # Drain any final queued lines so nothing is lost.
        while True:
            try:
                ts, tag, text = out_q.get_nowait()
            except queue.Empty:
                break
            line = f"[+{ts*1000:9.1f} ms] [{tag}] {text}"
            print(line)
            if log_fp is not None:
                log_fp.write(line + "\n")
        if log_fp is not None:
            log_fp.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
