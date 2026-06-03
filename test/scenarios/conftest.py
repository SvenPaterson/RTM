"""Pytest fixtures for the RTM rig harness.

Importing `Monitor` here keeps every scenario file boilerplate-free —
each test just declares `def test_x(monitor): ...` and gets a started
listener bound to the rig observer UDP port.
"""

from __future__ import annotations

import pytest

from test.rig.monitor import DEFAULT_BIND, DEFAULT_FIRMWARE_PORT, DEFAULT_PORT, Monitor
from test.rig.teensy import DEFAULT_PORT as TEENSY_DEFAULT_PORT
from test.rig.teensy import Teensy


SCENARIO_ORDER = {
    "test_comms_health.py": 10,
    "test_reset_reloads_protocol.py": 20,
    "test_run_cycle.py": 30,
    "test_protocol_upload.py": 40,
    "test_protocol_execution.py": 50,
    "test_reset_cancel.py": 60,
    "test_reset_pulse_multi.py": 70,
    "test_run_gate.py": 80,
    "test_cold_boot_protocol_upload.py": 900,
    "test_manual_power_loss_resume.py": 910,
    "test_manual_protocol_swap.py": 920,
    "test_manual_thermal_preheat.py": 930,
}


def pytest_addoption(parser: pytest.Parser) -> None:
    parser.addoption(
        "--rig-port",
        action="store",
        type=int,
        default=DEFAULT_PORT,
        help="UDP observer port the rig boards tee to (default: 8889).",
    )
    parser.addoption(
        "--rig-firmware-port",
        action="store",
        type=int,
        default=DEFAULT_FIRMWARE_PORT,
        help="UDP firmware control port to beacon (default: 8888).",
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
    parser.addoption(
        "--manual-base-protocol",
        action="store",
        default="HLFB_QUICK",
        help=(
            "Expected baseline protocol name before manual swap/power tests "
            "(default: HLFB_QUICK)."
        ),
    )
    parser.addoption(
        "--manual-swap-protocol",
        action="store",
        default="HEAT_TEST",
        help=(
            "Expected protocol name after manual SD swap test "
            "(default: HEAT_TEST)."
        ),
    )
    parser.addoption(
        "--manual-thermal-mode",
        action="store",
        choices=("body", "external"),
        default="body",
        help=(
            "Manual thermal stimulus mode: 'body' for bench testing or "
            "'external' for controlled test-stand heat source."
        ),
    )
    parser.addoption(
        "--manual-thermal-body-timeout-s",
        action="store",
        type=float,
        default=60.0,
        help="Maximum wait for body-heat transition to resume/run (seconds).",
    )
    parser.addoption(
        "--manual-thermal-external-rise-timeout-s",
        action="store",
        type=float,
        default=180.0,
        help=(
            "Maximum wait for measurable sump temperature rise in external "
            "thermal mode (seconds)."
        ),
    )
    parser.addoption(
        "--manual-thermal-body-rise-delta-c",
        action="store",
        type=float,
        default=1.0,
        help="Required minimum sump rise in body-heat mode.",
    )
    parser.addoption(
        "--manual-thermal-rise-delta-c",
        action="store",
        type=float,
        default=2.0,
        help="Required minimum sump temperature rise to prove heating is active.",
    )
    parser.addoption(
        "--manual-thermal-hold-seconds",
        action="store",
        type=float,
        default=30.0,
        help="Duration to hold near setpoint in external thermal mode.",
    )
    parser.addoption(
        "--manual-thermal-hold-band-c",
        action="store",
        type=float,
        default=10.0,
        help="Allowed +/- band (deg C) around setpoint during hold window.",
    )


@pytest.hookimpl(tryfirst=True)
def pytest_collection_modifyitems(items: list[pytest.Item]) -> None:
    """Keep live-rig output in a safe, readable order.

    Scenario correctness must not depend on this ordering; stateful tests
    still normalize their own preconditions. The order simply makes full-suite
    transcripts easier to scan and keeps manual scenarios last.
    """
    manual_subgroups = ("manual_power", "manual_protocol", "manual_thermal")
    for item in items:
        if item.get_closest_marker("manual") is None:
            if any(item.get_closest_marker(name) for name in manual_subgroups):
                item.add_marker("manual")

    items.sort(
        key=lambda item: (
            SCENARIO_ORDER.get(item.path.name, 500),
            item.nodeid,
        )
    )


@pytest.fixture(scope="function")
def monitor(request: pytest.FixtureRequest) -> Monitor:
    """Per-test UDP monitor. Started before the test, stopped after.

    Per-function scope (not session) is deliberate: each test gets a
    clean frame buffer and clock zero, which makes assertions on
    `t_ms` and `since_ms` straightforward.
    """
    port = request.config.getoption("--rig-port")
    firmware_port = request.config.getoption("--rig-firmware-port")
    bind = request.config.getoption("--rig-bind")
    mon = Monitor(bind=bind, port=port, firmware_port=firmware_port)
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
