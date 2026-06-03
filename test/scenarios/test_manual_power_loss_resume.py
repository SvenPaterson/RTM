"""Manual S10: Power-loss and single-side failure resume policy.

Covers three real-world outage classes with operator actions:

1) Full lab outage (both boards lose power) -> automatic resume is allowed.
2) Single-side failure (CC-only, XPB-only, or one-side comms loss) ->
   fault must be flagged before any automatic restart.

Run with interactive stdin enabled:
    python -m pytest test/scenarios/test_manual_power_loss_resume.py -v -s
"""

from __future__ import annotations

import logging
import time
from typing import Iterable, List

import pytest

from test.rig.monitor import Monitor
from test.rig.parser import Frame, GenericFrame, HbFrame
from test.rig.scenario import (
    dump_frames,
    operator_prompt,
    reset_to_idle,
    wait_for_state,
)
from test.rig.teensy import Teensy


_log = logging.getLogger("rig.manual.power_resume")

CC_IP = "10.0.0.10"
RUN_STATES = {"RUNNING", "PREHEAT", "RESUME"}
RESUME_TIMEOUT_S = 30.0
POLL_S = 0.25


def _first_resume(frames: Iterable[Frame], *, after_t_ms: float = 0.0) -> HbFrame | None:
    hits = [
        frame for frame in frames
        if isinstance(frame, HbFrame)
        and frame.src_ip == CC_IP
        and frame.state in RUN_STATES
        and frame.t_ms >= after_t_ms
    ]
    if not hits:
        return None
    return min(hits, key=lambda frame: frame.t_ms)


def _first_fault_flag(frames: Iterable[Frame]) -> Frame | None:
    flags: List[Frame] = []
    for frame in frames:
        if isinstance(frame, HbFrame):
            if frame.src_ip == CC_IP and (frame.e_flag or frame.state == "E-STOP"):
                flags.append(frame)
            continue
        if not isinstance(frame, GenericFrame):
            continue
        if frame.kind == "NOTICE" and frame.fields.get("LINK") == "DOWN":
            flags.append(frame)
            continue
        if frame.kind in ("ALARM", "FAULT"):
            flags.append(frame)
    if not flags:
        return None
    return min(flags, key=lambda frame: frame.t_ms)


@pytest.mark.manual
@pytest.mark.manual_power
@pytest.mark.live_rig
@pytest.mark.slow
@pytest.mark.stateful
def test_manual_power_loss_and_single_side_failure_policy(
    monitor: Monitor, teensy: Teensy, request: pytest.FixtureRequest
) -> None:
    """Validate resume policy for full outage vs single-side failure."""

    base_proto = str(request.config.getoption("--manual-base-protocol"))

    # Normalize and start motion so the outage policy is exercised from an
    # in-flight run context.
    result = reset_to_idle(
        monitor,
        teensy,
        _log,
        require_hlfb_quick=(base_proto.upper() == "HLFB_QUICK"),
    )
    if base_proto and result.proto.name and result.proto.name.upper() != base_proto.upper():
        pytest.fail(
            f"Baseline protocol mismatch before manual outage test: "
            f"loaded {result.proto.name!r}, expected {base_proto!r}."
        )

    teensy.set_run(1)
    running = wait_for_state(monitor, monitor.elapsed_ms, "RUNNING", 12.0)
    if not running:
        teensy.set_run(0)
        snap = monitor.snapshot(since_ms=0)
        pytest.fail(
            "VERDICT: PRECONDITION_NO_RUNNING — could not enter RUNNING "
            "before outage validation.\n"
            f"  Transcript:\n  {dump_frames(snap)}"
        )

    try:
        # -----------------------------------------------------------------
        # Phase A: both boards lose power -> auto resume is allowed.
        # -----------------------------------------------------------------
        operator_prompt(
            "Phase A (both boards): keep RUN asserted. Cut power to BOTH CC "
            "and XPB (or full lab outage), restore power, then press Enter."
        )

        phase_a_start = monitor.elapsed_ms
        resume_a: HbFrame | None = None
        deadline = time.monotonic() + RESUME_TIMEOUT_S
        while time.monotonic() < deadline:
            snap = monitor.snapshot(since_ms=phase_a_start)
            resume_a = _first_resume(snap)
            if resume_a is not None:
                break
            time.sleep(POLL_S)

        if resume_a is None:
            snap = monitor.snapshot(since_ms=phase_a_start)
            pytest.fail(
                f"VERDICT: BOTH_OUTAGE_NO_AUTO_RESUME — no CC "
                f"RUNNING/PREHEAT/RESUME heartbeat within {RESUME_TIMEOUT_S:.0f}s "
                "after full outage restore.\n"
                f"  Transcript:\n  {dump_frames(snap)}"
            )

        _log.info(
            "Phase A PASS: auto resume observed at %.1fms in STATE=%s",
            resume_a.t_ms,
            resume_a.state,
        )

        # -----------------------------------------------------------------
        # Phase B: single-side failure -> must flag before auto restart.
        # -----------------------------------------------------------------
        operator_prompt(
            "Phase B arm: keep RUN asserted. Press Enter to arm capture, then "
            "perform single-side fault at next prompt."
        )
        phase_b_start = monitor.elapsed_ms

        operator_prompt(
            "Phase B execute: induce SINGLE-SIDE failure (CC-only power, "
            "XPB-only power, or one-side comms loss), restore, then press Enter."
        )

        deadline = time.monotonic() + RESUME_TIMEOUT_S
        while time.monotonic() < deadline:
            time.sleep(POLL_S)

        snap_b = monitor.snapshot(since_ms=phase_b_start)
        flag = _first_fault_flag(snap_b)
        if flag is None:
            pytest.fail(
                "VERDICT: SINGLE_SIDE_NO_FLAG — single-side failure did not "
                "produce a LINK=DOWN / ALARM / E-STOP indicator before timeout.\n"
                f"  Transcript:\n  {dump_frames(snap_b)}"
            )

        resume_b = _first_resume(snap_b, after_t_ms=flag.t_ms)
        if resume_b is not None and resume_b.t_ms < flag.t_ms:
            pytest.fail(
                "VERDICT: SINGLE_SIDE_RESTARTED_BEFORE_FLAG — auto restart "
                "was observed before any fault indicator.\n"
                f"  Transcript:\n  {dump_frames(snap_b)}"
            )

        if resume_b is None:
            _log.info(
                "Phase B PASS: fault flag observed at %.1fms; no auto restart "
                "seen in %.0fs window.",
                flag.t_ms,
                RESUME_TIMEOUT_S,
            )
        else:
            _log.info(
                "Phase B PASS: fault flag at %.1fms preceded auto restart at %.1fms.",
                flag.t_ms,
                resume_b.t_ms,
            )

    finally:
        teensy.set_run(0)
