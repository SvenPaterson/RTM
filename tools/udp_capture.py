#!/usr/bin/env python3
"""udp_capture.py — Passive UDP listener for the CC↔XPB Ethernet link.

Bind to the rig observer UDP port (8889 by default), beacon the boards on
the production port (8888 by default), and log every received datagram with
a millisecond timestamp, the source IP/port, and the payload (stripped of
trailing newline, ASCII-escaped).

Output is always streamed to stdout. With --log <path> a CSV is also
appended:

    timestamp_iso,t_ms,src_ip,src_port,direction,payload

* `t_ms` is milliseconds since this capture process started (handy for
  diffing cadence without parsing wall-clock).
* `direction` is one of `CC->XPB`, `XPB->CC`, `PC->?`, `?->?`. It is
  inferred from `src_ip` against the known rig IP map; unknown hosts are
  tagged `?->?`.
* `payload` is the datagram body with `\\` `\\n` `\\r` `\\t` and any
  non-printable byte escaped, so the CSV stays one-row-per-datagram and
  trivially grep-able.

Usage:
    python tools/udp_capture.py                       # stdout only, default port
    python tools/udp_capture.py --log capture.csv     # tee to CSV
    python tools/udp_capture.py --duration-s 30

Requires Python 3.8+, no third-party deps. The bind uses SO_REUSEADDR so
parallel receive-only tools can share the observer port when the OS allows it.
"""

from __future__ import annotations

import argparse
import csv
import socket
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import TextIO

# Rig static IP map. Keep in sync with include/RtmNet.h (when added) and
# the nettest sketches under src/clearcore-nettest and src/exp-board-nettest.
HOSTS = {
    "10.0.0.10": "CC",
    "10.0.0.11": "XPB",
    "10.0.0.100": "PC",
}
OBSERVER_TARGETS = ("10.0.0.10", "10.0.0.11")
OBSERVER_BEACON = b"OBS;PC=1\n"

DEFAULT_FIRMWARE_PORT = 8888
DEFAULT_PORT = 8889
DEFAULT_BIND = "0.0.0.0"
DEFAULT_OBSERVER_INTERVAL_S = 1.0
RECV_BUFSIZE = 2048
CSV_HEADER = (
    "timestamp_iso",
    "t_ms",
    "src_ip",
    "src_port",
    "direction",
    "payload",
)


def classify_direction(src_ip: str) -> str:
    """Infer a CC↔XPB direction tag from the source IP.

    Returns `CC->XPB`, `XPB->CC`, or `<host>->?` for known PC/unknown
    sources. We don't know the destination from a single received
    datagram (the kernel routes by bind), so the right side of the
    arrow is always inferred.
    """
    src = HOSTS.get(src_ip, src_ip)
    if src == "CC":
        return "CC->XPB"
    if src == "XPB":
        return "XPB->CC"
    return f"{src}->?"


def escape_payload(payload: bytes) -> str:
    """Convert a datagram body into a single-line CSV-safe string."""
    out = []
    for b in payload:
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


def open_socket(bind_addr: str, port: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # Windows: SO_REUSEPORT is not available; SO_REUSEADDR alone allows
    # parallel binds for receive-only use.
    sock.bind((bind_addr, port))
    sock.settimeout(0.25)
    return sock


def send_observer_beacon(sock: socket.socket, firmware_port: int) -> None:
    for host in OBSERVER_TARGETS:
        sock.sendto(OBSERVER_BEACON, (host, firmware_port))


def capture_loop(
    sock: socket.socket,
    *,
    firmware_port: int,
    duration_s: float | None,
    csv_writer: csv.writer | None,
    csv_file: TextIO | None,
    observer_interval_s: float | None,
) -> int:
    """Receive datagrams until duration elapses or Ctrl-C. Returns count."""
    start = time.monotonic()
    received = 0
    deadline = start + duration_s if duration_s is not None else None
    next_observer = start
    observer_warned = False

    while True:
        now = time.monotonic()
        if deadline is not None and now >= deadline:
            break
        if observer_interval_s is not None and now >= next_observer:
            try:
                send_observer_beacon(sock, firmware_port)
            except OSError as exc:
                if not observer_warned:
                    print(f"[udp_capture] observer beacon failed: {exc}", file=sys.stderr)
                    observer_warned = True
            next_observer = now + observer_interval_s
        try:
            data, addr = sock.recvfrom(RECV_BUFSIZE)
        except socket.timeout:
            continue
        except OSError as exc:
            print(f"[udp_capture] recv error: {exc}", file=sys.stderr)
            return received

        now = time.monotonic()
        t_ms = (now - start) * 1000.0
        ts_iso = datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]
        src_ip, src_port = addr
        direction = classify_direction(src_ip)
        payload_str = escape_payload(data)

        # Strip a trailing escaped newline for stdout readability — keep
        # the raw form in the CSV so downstream parsers see exactly what
        # the wire carried.
        display_payload = payload_str
        if display_payload.endswith("\\n"):
            display_payload = display_payload[:-2]

        print(
            f"[{t_ms:9.1f} ms] {direction:>9s}  "
            f"{src_ip}:{src_port}  {display_payload}"
        )

        if csv_writer is not None:
            csv_writer.writerow(
                (ts_iso, f"{t_ms:.3f}", src_ip, src_port, direction, payload_str)
            )
            if csv_file is not None:
                csv_file.flush()

        received += 1

    return received


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument(
        "--bind",
        default=DEFAULT_BIND,
        help=f"Local interface address to bind (default: {DEFAULT_BIND})",
    )
    p.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
        help=f"UDP observer port to listen on (default: {DEFAULT_PORT})",
    )
    p.add_argument(
        "--firmware-port",
        type=int,
        default=DEFAULT_FIRMWARE_PORT,
        help=f"Firmware control port to beacon (default: {DEFAULT_FIRMWARE_PORT})",
    )
    p.add_argument(
        "--log",
        type=Path,
        default=None,
        help="Optional CSV path to append captured datagrams.",
    )
    p.add_argument(
        "--duration-s",
        type=float,
        default=None,
        help="Stop after N seconds. Default: run until Ctrl-C.",
    )
    p.add_argument(
        "--observer-interval-s",
        type=float,
        default=DEFAULT_OBSERVER_INTERVAL_S,
        help="Seconds between observer beacons to CC/XPB (default: 1.0).",
    )
    p.add_argument(
        "--no-observer-beacon",
        action="store_true",
        help="Do not announce this PC as a debug observer.",
    )
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    sock = open_socket(args.bind, args.port)
    print(
        f"[udp_capture] listening on {args.bind}:{args.port}"
        + (f" for {args.duration_s:.1f}s" if args.duration_s else " (Ctrl-C to stop)")
    )
    if not args.no_observer_beacon:
        print(
            "[udp_capture] observer beacon enabled "
            f"every {args.observer_interval_s:.1f}s to {', '.join(OBSERVER_TARGETS)}:"
            f"{args.firmware_port}"
        )
    if args.log:
        print(f"[udp_capture] writing CSV to {args.log}")

    csv_file: TextIO | None = None
    csv_writer: csv.writer | None = None
    if args.log is not None:
        new_file = not args.log.exists() or args.log.stat().st_size == 0
        csv_file = args.log.open("a", encoding="utf-8", newline="")
        csv_writer = csv.writer(csv_file)
        if new_file:
            csv_writer.writerow(CSV_HEADER)
            csv_file.flush()

    try:
        count = capture_loop(
            sock,
            firmware_port=args.firmware_port,
            duration_s=args.duration_s,
            csv_writer=csv_writer,
            csv_file=csv_file,
            observer_interval_s=None if args.no_observer_beacon else args.observer_interval_s,
        )
    except KeyboardInterrupt:
        print("\n[udp_capture] stopped by user")
        count = -1
    finally:
        sock.close()
        if csv_file is not None:
            csv_file.close()

    if count >= 0:
        print(f"[udp_capture] captured {count} datagram(s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
