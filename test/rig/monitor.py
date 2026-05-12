"""UDP listener thread + queryable frame buffer for the RTM rig.

A `Monitor` binds the rig UDP port (default 8888) and decodes every
inbound datagram into a typed `Frame` (see `parser.py`). Frames are
appended to an in-memory list under a lock; tests pull from it via
`snapshot()` / `wait_for()` (see `assertions.py`).

Designed for pytest fixture use:

    with Monitor() as mon:
        ...                            # frames accumulate in background
        hbs = mon.snapshot(kind="HB")  # all HB frames so far

The reader thread is daemonized so a hung test won't keep the process
alive. `__exit__` always cleanly closes the socket and joins the
thread within a short bounded timeout.

This is intentionally minimal — no callbacks, no per-frame filters at
ingest, no async. Tests do their own filtering on snapshots, which
keeps the harness easy to reason about and debug.
"""

from __future__ import annotations

import socket
import threading
import time
from dataclasses import dataclass
from typing import Callable, List, Optional

from .parser import (
    BadChecksumError,
    Frame,
    MalformedFrameError,
    parse_frame,
)


DEFAULT_PORT = 8888
DEFAULT_BIND = "0.0.0.0"
RECV_BUFSIZE = 2048
SOCK_TIMEOUT_S = 0.25     # poll interval — also the worst-case stop latency


@dataclass
class ParseError:
    """Captured parse failure. Surfaced via `Monitor.parse_errors`."""
    t_ms: float
    src_ip: str
    payload: bytes
    error: str


class Monitor:
    """Background UDP capture into a typed `Frame` buffer.

    Thread-safety: `snapshot()`, `clear()`, `parse_errors`, and
    `frame_count` are safe to call from the test thread while the
    reader runs. All reads copy under the same lock used by the writer.
    """

    def __init__(
        self,
        *,
        bind: str = DEFAULT_BIND,
        port: int = DEFAULT_PORT,
        verify_checksum: bool = True,
    ) -> None:
        self._bind = bind
        self._port = port
        self._verify_checksum = verify_checksum

        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

        self._frames: List[Frame] = []
        self._errors: List[ParseError] = []
        self._t0: float = 0.0

    # -- lifecycle -------------------------------------------------------

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("Monitor already started")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self._bind, self._port))
        sock.settimeout(SOCK_TIMEOUT_S)
        self._sock = sock
        self._t0 = time.monotonic()
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="rig-monitor", daemon=True
        )
        self._thread.start()

    def stop(self, *, timeout_s: float = 2.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout_s)
            self._thread = None
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None

    def __enter__(self) -> "Monitor":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()

    # -- reader ----------------------------------------------------------

    def _run(self) -> None:
        sock = self._sock
        assert sock is not None
        while not self._stop.is_set():
            try:
                data, addr = sock.recvfrom(RECV_BUFSIZE)
            except socket.timeout:
                continue
            except OSError:
                # Socket closed under us during shutdown — exit cleanly.
                return
            t_ms = (time.monotonic() - self._t0) * 1000.0
            src_ip = addr[0]
            try:
                line = data.decode("ascii", errors="strict")
            except UnicodeDecodeError as exc:
                self._record_error(t_ms, src_ip, data, f"decode: {exc}")
                continue
            try:
                frame = parse_frame(
                    line, t_ms=t_ms, src_ip=src_ip,
                    verify_checksum=self._verify_checksum,
                )
            except (BadChecksumError, MalformedFrameError) as exc:
                self._record_error(t_ms, src_ip, data, str(exc))
                continue
            with self._lock:
                self._frames.append(frame)

    def _record_error(self, t_ms: float, src_ip: str, payload: bytes, msg: str) -> None:
        with self._lock:
            self._errors.append(ParseError(t_ms=t_ms, src_ip=src_ip,
                                           payload=payload, error=msg))

    # -- queries ---------------------------------------------------------

    def snapshot(
        self,
        *,
        kind: Optional[str] = None,
        src_ip: Optional[str] = None,
        since_ms: float = 0.0,
        predicate: Optional[Callable[[Frame], bool]] = None,
    ) -> List[Frame]:
        """Return a filtered copy of the frame buffer.

        All filters are AND-combined. `kind` matches `frame.kind`
        exactly. `src_ip` matches the dotted-quad. `since_ms` keeps
        frames with `t_ms >= since_ms`. `predicate` is a final
        catch-all callable.
        """
        with self._lock:
            frames = list(self._frames)
        out: List[Frame] = []
        for f in frames:
            if kind is not None and f.kind != kind:
                continue
            if src_ip is not None and f.src_ip != src_ip:
                continue
            if f.t_ms < since_ms:
                continue
            if predicate is not None and not predicate(f):
                continue
            out.append(f)
        return out

    @property
    def parse_errors(self) -> List[ParseError]:
        with self._lock:
            return list(self._errors)

    @property
    def frame_count(self) -> int:
        with self._lock:
            return len(self._frames)

    @property
    def elapsed_ms(self) -> float:
        if self._t0 == 0.0:
            return 0.0
        return (time.monotonic() - self._t0) * 1000.0

    def clear(self) -> None:
        """Drop all buffered frames and parse errors. The capture
        clock (`elapsed_ms`) is *not* reset — callers that need a
        baseline should record `mon.elapsed_ms` and use `since_ms`."""
        with self._lock:
            self._frames.clear()
            self._errors.clear()
