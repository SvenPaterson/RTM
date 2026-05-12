#!/usr/bin/env python3
"""udp_probe.py — Phase 2 / 3 Ethernet bring-up tester.

Sends a UDP datagram to a target peer and waits for an echo. Reports
round-trip latency, packet loss, and decoded payload. Designed to be
extended in later phases into a full passive listener / harness peer
(see plan: tools/udp_capture.py is the production passive listener).

Usage examples:
    # Single ping to XPB (default)
    python tools/udp_probe.py

    # Ping ClearCore
    python tools/udp_probe.py --host 10.0.0.10

    # Burst of 50 packets, 20 ms apart, custom payload
    python tools/udp_probe.py -n 50 -i 0.02 --payload "HELLO"

    # Listen-only mode (passive; firmware must broadcast)
    python tools/udp_probe.py --listen

Notes:
    * Binds the local socket to the same port (8888) so any inbound reply
      from the peer reaches us regardless of NAT / firewall ephemeral-port
      assumptions. This matches the eventual production transport, where
      both peers use port 8888.
    * Requires Python 3.8+. No third-party deps.
"""

from __future__ import annotations

import argparse
import socket
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

try:
    import serial  # pyserial
except ImportError:
    serial = None  # type: ignore


DEFAULT_HOST = "10.0.0.11"   # XPB
DEFAULT_PORT = 8888
DEFAULT_BIND = "0.0.0.0"
DEFAULT_PAYLOAD = "PING"


@dataclass
class ProbeResult:
    sent: int
    recv: int
    rtts_ms: list[float]

    @property
    def loss_pct(self) -> float:
        return 0.0 if self.sent == 0 else 100.0 * (self.sent - self.recv) / self.sent

    def summary(self) -> str:
        if not self.rtts_ms:
            return f"sent={self.sent} recv={self.recv} loss={self.loss_pct:.1f}%"
        rtts = self.rtts_ms
        return (
            f"sent={self.sent} recv={self.recv} loss={self.loss_pct:.1f}% "
            f"rtt min/avg/max = {min(rtts):.2f}/{sum(rtts)/len(rtts):.2f}/{max(rtts):.2f} ms"
        )


# ---------------------------------------------------------------------------
# Serial monitor (optional, runs in a background thread).
# ---------------------------------------------------------------------------

class SerialMonitor:
    """Background reader that prints firmware serial output, timestamped
    and prefixed with [SER], so it interleaves with probe output in one
    terminal. Use as a context manager.

    The port must be free (close any other serial monitor first; PlatformIO
    upload also needs it free).
    """

    def __init__(self, port: str, baud: int = 115200) -> None:
        if serial is None:
            raise RuntimeError("pyserial not installed (pip install pyserial)")
        self._ser = serial.Serial(port, baud, timeout=0.1)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._t0 = time.perf_counter()
        self._lock = threading.Lock()

    def _stamp(self) -> str:
        return f"{(time.perf_counter() - self._t0)*1000:8.1f}ms"

    def _run(self) -> None:
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(256)
            except Exception as e:
                with self._lock:
                    print(f"[SER {self._stamp()}] <read error: {e}>")
                return
            if not chunk:
                continue
            buf.extend(chunk)
            while b"\n" in buf:
                line, _, rest = buf.partition(b"\n")
                buf = bytearray(rest)
                text = line.rstrip(b"\r").decode("utf-8", errors="replace")
                with self._lock:
                    print(f"[SER {self._stamp()}] {text}", flush=True)

    def __enter__(self) -> "SerialMonitor":
        self._thread.start()
        # Give the firmware a moment to flush any boot output that arrives
        # right after our open() (DTR/reset on some boards).
        time.sleep(0.2)
        return self

    def __exit__(self, *exc) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        try:
            self._ser.close()
        except Exception:
            pass


def make_socket(bind_addr: str, bind_port: int, timeout_s: float) -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.settimeout(timeout_s)
    s.bind((bind_addr, bind_port))
    return s


def echo_probe(host: str, port: int, payload: bytes,
               count: int, interval_s: float, timeout_s: float,
               bind_addr: str, bind_port: int, verbose: bool) -> ProbeResult:
    sock = make_socket(bind_addr, bind_port, timeout_s)
    result = ProbeResult(sent=0, recv=0, rtts_ms=[])

    try:
        for i in range(count):
            # Tag each packet with seq so we can detect reordering.
            pkt = f"{payload.decode()};seq={i}\n".encode()
            t0 = time.perf_counter()
            sock.sendto(pkt, (host, port))
            result.sent += 1

            try:
                data, src = sock.recvfrom(1024)
                rtt_ms = (time.perf_counter() - t0) * 1000.0
                result.recv += 1
                result.rtts_ms.append(rtt_ms)
                if verbose:
                    print(f"[{i:3d}] {len(data)}B from {src[0]}:{src[1]} "
                          f"rtt={rtt_ms:.2f}ms  payload={data!r}")
            except socket.timeout:
                if verbose:
                    print(f"[{i:3d}] TIMEOUT (>{timeout_s*1000:.0f}ms)")

            if i + 1 < count:
                time.sleep(interval_s)
    finally:
        sock.close()

    return result


def listen_only(bind_addr: str, bind_port: int, duration_s: float) -> None:
    """Passive listener: print everything seen on the bound port."""
    sock = make_socket(bind_addr, bind_port, timeout_s=0.5)
    print(f"Listening on {bind_addr}:{bind_port} for {duration_s}s ...")
    deadline = time.perf_counter() + duration_s
    count = 0
    try:
        while time.perf_counter() < deadline:
            try:
                data, src = sock.recvfrom(1024)
                ts = time.strftime("%H:%M:%S")
                print(f"{ts}  {src[0]}:{src[1]:>5}  {len(data):3d}B  {data!r}")
                count += 1
            except socket.timeout:
                pass
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
    print(f"Total packets received: {count}")


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description="RTM Ethernet bring-up UDP probe.")
    p.add_argument("--host", default=DEFAULT_HOST,
                   help=f"Target IP (default {DEFAULT_HOST} = XPB)")
    p.add_argument("--port", type=int, default=DEFAULT_PORT,
                   help=f"Target UDP port (default {DEFAULT_PORT})")
    p.add_argument("--payload", default=DEFAULT_PAYLOAD,
                   help=f'Payload string (default "{DEFAULT_PAYLOAD}")')
    p.add_argument("-n", "--count", type=int, default=4,
                   help="Number of packets to send (default 4)")
    p.add_argument("-i", "--interval", type=float, default=0.25,
                   help="Seconds between sends (default 0.25)")
    p.add_argument("-t", "--timeout", type=float, default=1.0,
                   help="Per-packet receive timeout in seconds (default 1.0)")
    p.add_argument("--bind", default=DEFAULT_BIND,
                   help=f"Local bind address (default {DEFAULT_BIND})")
    p.add_argument("--bind-port", type=int, default=DEFAULT_PORT,
                   help=f"Local bind port (default {DEFAULT_PORT}; matches firmware)")
    p.add_argument("--listen", action="store_true",
                   help="Listen-only mode (no sends)")
    p.add_argument("--listen-secs", type=float, default=10.0,
                   help="Listen-only duration in seconds (default 10)")
    p.add_argument("-q", "--quiet", action="store_true",
                   help="Suppress per-packet output")
    p.add_argument("--serial", default=None, metavar="PORT",
                   help="Also stream firmware serial in background (e.g. COM9). "
                        "Close any other serial monitor first.")
    p.add_argument("--baud", type=int, default=115200,
                   help="Serial baud (default 115200)")
    p.add_argument("--settle", type=float, default=0.0,
                   help="Seconds to wait after opening serial before sending "
                        "(useful if --serial resets the board on open)")

    args = p.parse_args(argv)

    monitor: Optional[SerialMonitor] = None
    if args.serial:
        monitor = SerialMonitor(args.serial, args.baud)
        monitor.__enter__()
        if args.settle > 0:
            time.sleep(args.settle)

    try:
        if args.listen:
            listen_only(args.bind, args.bind_port, args.listen_secs)
            return 0

        print(f"Probing {args.host}:{args.port} from {args.bind}:{args.bind_port} "
              f"({args.count} packets, {args.interval*1000:.0f}ms apart)")
        result = echo_probe(
            host=args.host, port=args.port,
            payload=args.payload.encode(),
            count=args.count, interval_s=args.interval, timeout_s=args.timeout,
            bind_addr=args.bind, bind_port=args.bind_port,
            verbose=not args.quiet,
        )
        print(result.summary())
        # Give the firmware a beat to print any post-RX log lines before
        # we tear down the monitor.
        if monitor is not None:
            time.sleep(0.3)
        return 0 if result.recv > 0 else 1
    finally:
        if monitor is not None:
            monitor.__exit__(None, None, None)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
