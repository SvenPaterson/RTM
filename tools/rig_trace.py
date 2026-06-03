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
    # Live to stdout + auto log under test/log/YYYY/MM/DD.
    python tools/rig_trace.py

    # Override log file location, also include XPB serial.
    python tools/rig_trace.py --xpb-port COM9 --log trace.log

    # Capture for a fixed duration (seconds).
    python tools/rig_trace.py --duration-s 30

    # Disable file logging entirely.
    python tools/rig_trace.py --no-log

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
import hashlib
import queue
import socket
import sys
import threading
import time
from datetime import datetime
from dataclasses import dataclass
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
DEFAULT_LOG_ROOT = Path(__file__).resolve().parents[1] / "test" / "log"


@dataclass
class TraceSummary:
    net_req_proto: int = 0
    net_pr_beg: int = 0
    net_pr_dat: int = 0
    net_pr_end: int = 0
    net_proto_rx_ok: int = 0
    net_cc_hb: int = 0
    net_xpb_stat: int = 0
    xpb_dbg_ccreq: int = 0
    xpb_dbg_link_down: int = 0
    xpb_dbg_link_up: int = 0
    xpb_prod_req_rx: int = 0
    xpb_prod_proto_timeout: int = 0
    ccdbg_lines: int = 0

    def ingest(self, tag: str, text: str) -> None:
        if tag == "NET":
            if "CC->XPB" in text and "REQ:PROTO" in text:
                self.net_req_proto += 1
            if "XPB->CC" in text and "PR_BEG;" in text:
                self.net_pr_beg += 1
            if "XPB->CC" in text and "PR_DAT;" in text:
                self.net_pr_dat += 1
            if "XPB->CC" in text and "PR_END;" in text:
                self.net_pr_end += 1
            if "CC->XPB" in text and "NOTICE;PROTO_RX=OK" in text:
                self.net_proto_rx_ok += 1
            if "CC->XPB" in text and "HB;" in text:
                self.net_cc_hb += 1
            if "XPB->CC" in text and "STAT;" in text:
                self.net_xpb_stat += 1
            return

        if tag == "XPB":
            if "XPBDBG;CC_REQ_PROTO_RX;COUNT=" in text:
                self.xpb_dbg_ccreq += 1
            if "XPBDBG;LINK=DOWN" in text:
                self.xpb_dbg_link_down += 1
            if "XPBDBG;LINK=UP" in text:
                self.xpb_dbg_link_up += 1
            if "[PROTO] REQ:PROTO rx" in text:
                self.xpb_prod_req_rx += 1
            if "PROTO_ACK_TIMEOUT" in text or "PROTO_SILENCE_TIMEOUT" in text:
                self.xpb_prod_proto_timeout += 1
            return

        if tag == "CC " and "CCDBG;" in text:
            self.ccdbg_lines += 1

    def classify_hint(self) -> str:
        # Branch hint for the intermittent proof-pack ladder. Treat as guidance,
        # not a final verdict.
        net_req_seen = self.net_req_proto > 0
        xpb_req_seen = (self.xpb_dbg_ccreq > 0) or (self.xpb_prod_req_rx > 0)
        link_flap_seen = self.xpb_dbg_link_down > 0

        if not net_req_seen and not xpb_req_seen and link_flap_seen:
            return "Hint=B2 (link/receiver instability likely)"
        if not net_req_seen and not xpb_req_seen and self.ccdbg_lines > 0:
            return "Hint=B1/B2 (CC debug active but no REQ observed; inspect CCDBG tx/bp/ep counters)"
        if net_req_seen and not xpb_req_seen and link_flap_seen:
            return "Hint=B2 (REQ visible on observer but never seen by XPB; link/receiver path unstable)"
        if net_req_seen and not xpb_req_seen:
            return "Hint=B2 (REQ visible on observer but no XPB-side REQ marker)"
        if xpb_req_seen and self.net_pr_beg == 0:
            return "Hint=B3-class (REQ seen but no PR_BEG on wire)"
        if self.net_pr_end > 0 and self.net_proto_rx_ok == 0:
            return "Hint=B3-class (upload reached PR_END but no PROTO_RX=OK)"
        if self.net_req_proto > 0 and self.net_pr_beg > 0 and self.net_proto_rx_ok > 0:
            return "Hint=handoff chain observed complete"
        return "Hint=insufficient evidence; run with fixed topology + synchronized captures"

    def render_lines(self) -> list[str]:
        return [
            "[rig_trace] Summary",
            f"  NET REQ:PROTO (CC->XPB): {self.net_req_proto}",
            f"  NET PR_BEG/PR_DAT/PR_END: {self.net_pr_beg}/{self.net_pr_dat}/{self.net_pr_end}",
            f"  NET NOTICE;PROTO_RX=OK: {self.net_proto_rx_ok}",
            f"  NET CC HB count: {self.net_cc_hb}",
            f"  NET XPB STAT count: {self.net_xpb_stat}",
            f"  XPB_DEBUG CC_REQ hits: {self.xpb_dbg_ccreq}",
            f"  XPB_DEBUG link DOWN/UP: {self.xpb_dbg_link_down}/{self.xpb_dbg_link_up}",
            f"  XPB_PROD_TRACE REQ rx markers: {self.xpb_prod_req_rx}",
            f"  XPB_PROD_TRACE proto timeouts: {self.xpb_prod_proto_timeout}",
            f"  CC_DEBUG lines seen: {self.ccdbg_lines}",
            f"  {self.classify_hint()}",
        ]


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


def format_udp_payload(data: bytes) -> str:
    """Render UDP payloads for logs without flooding on binary noise."""
    if not data:
        return ""

    printable = 0
    for b in data:
        if b in (0x09, 0x0A, 0x0D) or 0x20 <= b < 0x7F:
            printable += 1
    ratio = printable / float(len(data))

    # If payload looks binary or very large, emit a compact summary with a
    # short escaped prefix so traces remain readable and deterministic.
    if len(data) > 512 or ratio < 0.85:
        digest = hashlib.sha1(data).hexdigest()[:12]
        head = escape_payload(data[:64])
        if head.endswith("\\n"):
            head = head[:-2]
        return f"<binary len={len(data)} printable={ratio:.2f} sha1={digest} head={head}>"

    payload = escape_payload(data)
    if payload.endswith("\\n"):
        payload = payload[:-2]
    if len(payload) > 512:
        return payload[:512] + "...(truncated)"
    return payload


def format_raw_net_packet(ts: float, src_ip: str, direction: str, data: bytes) -> str:
    """Return one deterministic line for full raw UDP payload capture."""
    digest = hashlib.sha1(data).hexdigest()[:12]
    return (
        f"[+{ts*1000:9.1f} ms] src={src_ip} dir={direction} "
        f"len={len(data)} sha1={digest} hex={data.hex()}"
    )


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
    raw_net_log: Path | None,
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
    raw_fp: TextIO | None = None
    if raw_net_log is not None:
        raw_net_log.parent.mkdir(parents=True, exist_ok=True)
        raw_fp = raw_net_log.open("w", encoding="utf-8", newline="")
        raw_fp.write(f"# rig_trace raw NET started {datetime.now().isoformat(timespec='milliseconds')}\n")
        raw_fp.write(f"# udp_port={port} firmware_port={firmware_port}\n")
        raw_fp.flush()
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
            payload = format_udp_payload(data)
            out_q.put((ts, "NET", f"{direction:<8s} {src_ip:<11s} {payload}"))
            if raw_fp is not None:
                raw_fp.write(format_raw_net_packet(ts, src_ip, direction, data) + "\n")
                raw_fp.flush()
    finally:
        if raw_fp is not None:
            try:
                raw_fp.close()
            except Exception:
                pass
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
                   help="Optional override for the trace log path.")
    p.add_argument("--no-log", action="store_true",
                   help="Disable trace file logging (default is auto dated log path).")
    p.add_argument("--duration-s", type=float, default=None,
                   help="Stop after N seconds. Default: run until Ctrl-C.")
    p.add_argument("--no-udp", action="store_true",
                   help="Disable UDP capture (useful if port is already bound).")
    p.add_argument("--observer-interval-s", type=float, default=DEFAULT_OBSERVER_INTERVAL_S,
                   help="Seconds between observer beacons to CC/XPB (default 1.0).")
    p.add_argument("--no-observer-beacon", action="store_true",
                   help="Do not announce this PC as a debug observer.")
    p.add_argument("--no-summary", action="store_true",
                   help="Disable end-of-run summary counters and branch hint.")
    p.add_argument("--raw-net-log", type=Path, default=None,
                   help="Optional path to write full raw UDP payload bytes (hex) for forensic review.")
    return p.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    if args.log and args.no_log:
        print("ERROR: choose either --log or --no-log, not both", file=sys.stderr)
        return 2

    now = datetime.now()
    start_mono = time.monotonic()
    start_iso = now.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]
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
                args.raw_net_log,
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
    log_path: Optional[Path] = None
    if not args.no_log:
        if args.log is not None:
            log_path = args.log
        else:
            day_dir = DEFAULT_LOG_ROOT / f"{now.year:04d}" / f"{now.month:02d}" / f"{now.day:02d}"
            day_dir.mkdir(parents=True, exist_ok=True)
            log_path = day_dir / f"{now.strftime('%H%M%S')}_rig_trace.log"

    if log_path is not None:
        try:
            log_path.parent.mkdir(parents=True, exist_ok=True)
            log_fp = log_path.open("w", encoding="utf-8", newline="")
        except OSError as exc:
            print(f"ERROR: cannot open log file {log_path}: {exc}", file=sys.stderr)
            return 2
        log_fp.write(f"# rig_trace started {start_iso}\n")
        log_fp.flush()

    header = (
        f"[rig_trace] start {start_iso}  "
        f"cc={args.cc_port or '-'} xpb={args.xpb_port or '-'} "
        f"udp={'-' if args.no_udp else args.udp_port}"
    )
    if log_path is not None:
        print(f"[rig_trace] logging to {log_path}")
    print(header)
    if log_fp is not None:
        log_fp.write(header + "\n")
        log_fp.flush()

    deadline = (start_mono + args.duration_s) if args.duration_s else None
    summary = TraceSummary()
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
            summary.ingest(tag, text)
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
            summary.ingest(tag, text)
            if log_fp is not None:
                log_fp.write(line + "\n")
        if not args.no_summary:
            for sline in summary.render_lines():
                print(sline)
                if log_fp is not None:
                    log_fp.write(sline + "\n")
        if log_fp is not None:
            log_fp.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
