"""Pytest fixtures for the RTM rig harness.

Importing `Monitor` here keeps every scenario file boilerplate-free —
each test just declares `def test_x(monitor): ...` and gets a started
listener bound to the rig UDP port.
"""

from __future__ import annotations

import pytest

from test.rig.monitor import DEFAULT_BIND, DEFAULT_PORT, Monitor
from test.rig.teensy import DEFAULT_PORT as TEENSY_DEFAULT_PORT
from test.rig.teensy import Teensy


def pytest_addoption(parser: pytest.Parser) -> None:
    parser.addoption(
        "--rig-port",
        action="store",
        type=int,
        default=DEFAULT_PORT,
        help="UDP port the rig boards send on (default: 8888).",
    )
    parser.addoption(
        "--rig-bind",
        action="store",
        default=DEFAULT_BIND,
        help="Local interface address to bind (default: 0.0.0.0).",
    )
    parser.addoption(
        "--teensy-port",
        action="store",
        default=None,
        help=(
            "Serial port for the Teensy rig controller. Default: "
            f"auto-detect by VID:PID, falling back to {TEENSY_DEFAULT_PORT}."
        ),
    )


@pytest.fixture(scope="function")
def monitor(request: pytest.FixtureRequest) -> Monitor:
    """Per-test UDP monitor. Started before the test, stopped after.

    Per-function scope (not session) is deliberate: each test gets a
    clean frame buffer and clock zero, which makes assertions on
    `t_ms` and `since_ms` straightforward.
    """
    port = request.config.getoption("--rig-port")
    bind = request.config.getoption("--rig-bind")
    mon = Monitor(bind=bind, port=port)
    mon.start()
    yield mon
    mon.stop()


@pytest.fixture(scope="function")
def teensy(request: pytest.FixtureRequest) -> Teensy:
    """Per-test Teensy serial controller for driving RUN/RST/PWR.

    Skips the test if the Teensy can't be opened (no rig attached).
    """
    port = request.config.getoption("--teensy-port")
    try:
        t = Teensy(port=port)
    except Exception as exc:  # serial.SerialException, OSError, etc.
        pytest.skip(f"Teensy controller not reachable: {exc}")
    yield t
    t.close()
