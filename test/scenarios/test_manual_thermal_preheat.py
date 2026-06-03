"""Manual S12: Thermal preheat behavior (body vs external stimulus modes).

Mode selection:
- body     : bench workflow using body heat; must transition within 60 s.
- external : controlled heat source on test stand; must prove heating rise,
             then hold near setpoint for a dwell window.

Run with:
    python -m pytest test/scenarios/test_manual_thermal_preheat.py -v -s

Override mode:
    python -m pytest test/scenarios/test_manual_thermal_preheat.py -v -s \
        --manual-thermal-mode external
"""

from __future__ import annotations

import logging
import time
from typing import Iterable

import pytest

from test.rig.monitor import Monitor
from test.rig.parser import Frame, GenericFrame, HbFrame, StatFrame
from test.rig.scenario import (
    dump_frames,
    operator_confirm,
    operator_prompt,
    reset_to_idle,
    wait_for_state,
)
from test.rig.teensy import Teensy


_log = logging.getLogger("rig.manual.thermal")

CC_IP = "10.0.0.10"
RUN_STATES = {"RUNNING", "RESUME"}
POLL_S = 0.25


def _first_resume(frames: Iterable[Frame]) -> HbFrame | None:
    hits = [
        frame for frame in frames
        if isinstance(frame, HbFrame)
        and frame.src_ip == CC_IP
        and frame.state in RUN_STATES
    ]
    if not hits:
        return None
    return min(hits, key=lambda frame: frame.t_ms)


def _latest_setpoint(frames: Iterable[Frame], current: float | None) -> float | None:
    sp = current
    for frame in frames:
        if not isinstance(frame, GenericFrame):
            continue
        if frame.src_ip != CC_IP or frame.kind != "CMD":
            continue
        raw = frame.fields.get("SP")
        if raw is None:
            continue
        try:
            sp = float(raw)
        except ValueError:
            continue
    return sp


@pytest.mark.manual
@pytest.mark.manual_thermal
@pytest.mark.live_rig
@pytest.mark.slow
@pytest.mark.stateful
def test_manual_thermal_preheat_transition(
    monitor: Monitor, teensy: Teensy, request: pytest.FixtureRequest
) -> None:
    """Validate thermal behavior in body/external operator modes."""

    mode = str(request.config.getoption("--manual-thermal-mode"))
    expected_proto = str(request.config.getoption("--manual-swap-protocol"))

    body_timeout_s = float(request.config.getoption("--manual-thermal-body-timeout-s"))
    body_rise_c = float(request.config.getoption("--manual-thermal-body-rise-delta-c"))

    ext_rise_timeout_s = float(
        request.config.getoption("--manual-thermal-external-rise-timeout-s")
    )
    ext_rise_c = float(request.config.getoption("--manual-thermal-rise-delta-c"))
    hold_seconds = float(request.config.getoption("--manual-thermal-hold-seconds"))
    hold_band_c = float(request.config.getoption("--manual-thermal-hold-band-c"))

    if mode not in ("body", "external"):
        pytest.fail(f"Unknown --manual-thermal-mode {mode!r}")

    if mode == "body":
        operator_prompt(
            "Thermal body mode: ensure bench-safe setup and protocol with heat "
            "setpoint is loaded. Press Enter to continue."
        )
    else:
        operator_prompt(
            "Thermal external mode: ensure controlled heat source + safety "
            "procedures are in place. Press Enter to continue."
        )

    operator_confirm(
        "Confirm SD is reinserted and protocol swap activity is complete before "
        "thermal normalization reset.",
        token="READY",
    )

    # Reload to a clean IDLE and verify we are on the expected thermal profile.
    result = reset_to_idle(monitor, teensy, _log, require_hlfb_quick=False)
    if expected_proto and result.proto.name.upper() != expected_proto.upper():
        pytest.fail(
            f"VERDICT: THERMAL_PROTOCOL_MISMATCH — loaded {result.proto.name!r}, "
            f"expected {expected_proto!r}. Run manual protocol swap first."
        )

    run_t = monitor.elapsed_ms
    teensy.set_run(1)

    preheat = wait_for_state(monitor, run_t, "PREHEAT", 15.0)
    if not preheat:
        teensy.set_run(0)
        snap = monitor.snapshot(since_ms=run_t)
        pytest.fail(
            "VERDICT: NO_PREHEAT_ENTRY — RUN asserted but CC never entered "
            "PREHEAT. Confirm protocol includes non-zero temperature setpoints.\n"
            f"  Transcript:\n  {dump_frames(snap)}"
        )

    try:
        operator_prompt(
            "Apply thermal stimulus now, then press Enter to start timed monitoring."
        )
        stim_t = monitor.elapsed_ms

        snap0 = monitor.snapshot(since_ms=run_t)
        baseline_stats = [f for f in snap0 if isinstance(f, StatFrame)]
        if not baseline_stats:
            pytest.fail("No STAT frames available to establish baseline sump temperature")

        baseline_sump = float(baseline_stats[-1].sump_c)
        max_sump = baseline_sump
        setpoint: float | None = _latest_setpoint(snap0, None)

        _log.info(
            "Thermal baseline: mode=%s baseline_sump=%.1fC setpoint=%s",
            mode,
            baseline_sump,
            "unknown" if setpoint is None else f"{setpoint:.1f}C",
        )

        if mode == "body":
            deadline = time.monotonic() + body_timeout_s
            resume: HbFrame | None = None
            rise_ok = False
            while time.monotonic() < deadline:
                snap = monitor.snapshot(since_ms=stim_t)
                setpoint = _latest_setpoint(snap, setpoint)
                stats = [f for f in snap if isinstance(f, StatFrame)]
                if stats:
                    max_sump = max(max_sump, float(stats[-1].sump_c))
                    rise_ok = (max_sump - baseline_sump) >= body_rise_c
                resume = _first_resume(snap)
                if rise_ok and resume is not None:
                    _log.info(
                        "Body mode PASS: rise=%.1fC resume_state=%s at %.1fms",
                        max_sump - baseline_sump,
                        resume.state,
                        resume.t_ms,
                    )
                    return
                time.sleep(POLL_S)

            snap = monitor.snapshot(since_ms=stim_t)
            pytest.fail(
                f"VERDICT: BODY_HEAT_TIMEOUT — no preheat transition within "
                f"{body_timeout_s:.0f}s (rise={max_sump - baseline_sump:.1f}C).\n"
                f"  Transcript:\n  {dump_frames(snap)}"
            )

        # external mode
        rise_deadline = time.monotonic() + ext_rise_timeout_s
        rise_time_ms: float | None = None
        while time.monotonic() < rise_deadline:
            snap = monitor.snapshot(since_ms=stim_t)
            setpoint = _latest_setpoint(snap, setpoint)
            stats = [f for f in snap if isinstance(f, StatFrame)]
            if stats:
                last = float(stats[-1].sump_c)
                max_sump = max(max_sump, last)
                if (max_sump - baseline_sump) >= ext_rise_c:
                    rise_time_ms = stats[-1].t_ms
                    break
            time.sleep(POLL_S)

        if rise_time_ms is None:
            snap = monitor.snapshot(since_ms=stim_t)
            pytest.fail(
                f"VERDICT: EXTERNAL_NO_RISE — sump did not rise by "
                f"{ext_rise_c:.1f}C within {ext_rise_timeout_s:.0f}s. "
                "Heater path may be non-functional.\n"
                f"  Transcript:\n  {dump_frames(snap)}"
            )

        if setpoint is None:
            snap = monitor.snapshot(since_ms=stim_t)
            pytest.fail(
                "VERDICT: EXTERNAL_NO_SETPOINT — no CMD;SP setpoint observed "
                "from CC during preheat.\n"
                f"  Transcript:\n  {dump_frames(snap)}"
            )

        _log.info(
            "External rise detected at %.1fms: rise=%.1fC setpoint=%.1fC",
            rise_time_ms,
            max_sump - baseline_sump,
            setpoint,
        )

        # Wait until we are near setpoint, then enforce hold band.
        near_deadline = time.monotonic() + ext_rise_timeout_s
        hold_start: float | None = None
        while time.monotonic() < near_deadline:
            snap = monitor.snapshot(since_ms=stim_t)
            stats = [f for f in snap if isinstance(f, StatFrame)]
            if stats:
                last = float(stats[-1].sump_c)
                if last >= (setpoint - hold_band_c):
                    hold_start = time.monotonic()
                    break
            time.sleep(POLL_S)

        if hold_start is None:
            snap = monitor.snapshot(since_ms=stim_t)
            pytest.fail(
                f"VERDICT: EXTERNAL_NEVER_NEAR_SETPOINT — sump never reached "
                f"setpoint band ({setpoint - hold_band_c:.1f}C) before timeout.\n"
                f"  Transcript:\n  {dump_frames(snap)}"
            )

        hold_deadline = hold_start + hold_seconds
        while time.monotonic() < hold_deadline:
            snap = monitor.snapshot(since_ms=stim_t)
            stats = [f for f in snap if isinstance(f, StatFrame)]
            if stats:
                last = float(stats[-1].sump_c)
                if not (setpoint - hold_band_c <= last <= setpoint + hold_band_c):
                    pytest.fail(
                        f"VERDICT: EXTERNAL_SETPOINT_NOT_HELD — sump={last:.1f}C "
                        f"outside [{setpoint - hold_band_c:.1f}, {setpoint + hold_band_c:.1f}] "
                        f"during {hold_seconds:.0f}s hold.\n"
                        f"  Transcript:\n  {dump_frames(snap)}"
                    )
            time.sleep(POLL_S)

        _log.info(
            "External mode PASS: rise=%.1fC and setpoint %.1fC held for %.0fs (+/-%.1fC)",
            max_sump - baseline_sump,
            setpoint,
            hold_seconds,
            hold_band_c,
        )

    finally:
        teensy.set_run(0)
