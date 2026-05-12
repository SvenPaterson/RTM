"""Minimal serial controller for the Teensy rig sniffer.

The Teensy is wired to drive the CC's RUN and RESET TTL inputs and to
switch the rig's PSU relay. Its USB-CDC firmware accepts line-based
commands:

    RUN=0    /  RUN=1            — drive RUN line low/high (held)
    PULSE RST=<ms>               — assert RST high for <ms>, then low
    PULSE RUN=<ms>               — assert RUN high for <ms>, then low
    PWR=0    /  PWR=1            — PSU relay off/on
    STATUS                       — query (responses ignored here)

VID:PID for auto-detect: 0x16C0:0x0483.

This module is intentionally thin: it owns the serial port, sends the
right strings, and emits structured log events so test transcripts read
naturally (``teensy -> RUN=1``, ``teensy -> RESET=1`` / ``=0`` for the
edges of a pulse).
"""

from __future__ import annotations

import logging
import os
import time
from typing import Optional

try:
    import serial  # type: ignore
    from serial.tools import list_ports  # type: ignore
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "pyserial is required. Install with `pip install pyserial`."
    ) from exc


_log = logging.getLogger("rig.teensy")

TEENSY_VID = 0x16C0
TEENSY_PID = 0x0483
DEFAULT_PORT = "COM7"
DEFAULT_BAUD = 115200
DEFAULT_OPEN_TIMEOUT_S = 0.1
DEFAULT_OPEN_WAIT_S = 10.0
DEFAULT_SETTLE_S = 0.5


def detect_port() -> Optional[str]:
    """Return the first port whose VID:PID matches the Teensy, else
    ``None``. Falls back to a description match for ``"teensy"``.
    """
    for p in list_ports.comports():
        if p.vid == TEENSY_VID and p.pid == TEENSY_PID:
            return p.device
        if "teensy" in (p.description or "").lower():
            return p.device
    return None


class Teensy:
    """Serial wrapper around the Teensy rig controller.

    Use as a context manager or call ``close()`` explicitly. All public
    methods log a single structured line per action so test logs stay
    readable.
    """

    def __init__(
        self,
        port: Optional[str] = None,
        baud: int = DEFAULT_BAUD,
        *,
        open_timeout_s: float = DEFAULT_OPEN_TIMEOUT_S,
        open_wait_s: float = DEFAULT_OPEN_WAIT_S,
        safe_park_on_close: bool = True,
    ) -> None:
        chosen = (
            port
            or os.environ.get("RTM_TEENSY_PORT")
            or detect_port()
            or DEFAULT_PORT
        )
        self.port = chosen
        self.baud = baud
        self._safe_park_on_close = safe_park_on_close
        self._ser = self._open_with_retry(
            chosen, baud, open_timeout_s, open_wait_s
        )
        time.sleep(DEFAULT_SETTLE_S)
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()

    # -- context manager ------------------------------------------------
    def __enter__(self) -> "Teensy":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()

    def close(self) -> None:
        if self._ser is not None and self._ser.is_open:
            if self._safe_park_on_close:
                try:
                    # Best-effort: park the rig in a known-safe state.
                    # Disabled for ad-hoc CLI usage where the caller wants
                    # the last-issued RUN/RST level to persist after exit.
                    self._send("RUN=0")
                except Exception:
                    pass
            self._ser.close()

    # -- low-level ------------------------------------------------------
    @staticmethod
    def _open_with_retry(
        port: str, baud: int, timeout_s: float, wait_s: float
    ) -> "serial.Serial":
        deadline = time.monotonic() + wait_s
        last_err: Optional[Exception] = None
        while time.monotonic() < deadline:
            try:
                return serial.Serial(port, baud, timeout=timeout_s)
            except (serial.SerialException, OSError) as exc:
                last_err = exc
                time.sleep(0.25)
        raise serial.SerialException(
            f"Timed out waiting for Teensy on {port} after {wait_s:.1f}s"
        ) from last_err

    def _send(self, line: str) -> None:
        payload = (line + "\n").encode("ascii")
        self._ser.write(payload)
        self._ser.flush()

    # -- public API -----------------------------------------------------
    def set_run(self, level: int) -> None:
        """Drive the RUN line to ``level`` (0 or 1) and hold it there."""
        lvl = 1 if level else 0
        _log.info("teensy -> RUN=%d", lvl)
        self._send(f"RUN={lvl}")

    def pulse_run(self, ms: int) -> None:
        """Pulse RUN high for ``ms`` milliseconds, then release."""
        _log.info("teensy -> RUN=1 (pulse %d ms)", ms)
        self._send(f"PULSE RUN={int(ms)}")
        time.sleep(ms / 1000.0)
        _log.info("teensy -> RUN=0 (pulse end)")

    def pulse_reset(self, ms: int) -> None:
        """Pulse RST high for ``ms`` milliseconds, then release.

        For ``ms >= 5000`` this drives a *logical* reset on CC; shorter
        pulses are a debounced cancel. The two log lines bracket the
        active window so transcripts show edge timing.
        """
        _log.info("teensy -> RESET=1 (pulse %d ms)", ms)
        self._send(f"PULSE RST={int(ms)}")
        time.sleep(ms / 1000.0)
        _log.info("teensy -> RESET=0 (pulse end)")

    def set_power(self, on: bool) -> None:
        """Switch the rig PSU relay."""
        lvl = 1 if on else 0
        _log.info("teensy -> POWER=%s", "ON" if on else "OFF")
        self._send(f"PWR={lvl}")
