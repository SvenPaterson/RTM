"""S6: Loaded HLFB_QUICK protocol actually executes step-by-step.

Drives RUN high and watches HBs as the CC walks the 12 steps of
HLFB_QUICK to completion. For each step we verify:

    * Step number appears in HBs in monotonically-increasing order
      (1 -> 2 -> ... -> 12, no skips, no regressions).
    * Steady-state |RPM| within tolerance of the step's |target RPM|
      (sampled from the LAST HBs of each step, after accel ramp).
    * Sign of RPM matches sign of target (negative-target steps verify
      the reverse-direction code path).
    * Test ends with STATE=COMPLETED within a grace window
      (LOOP_COUNT=1, so a single pass through all 12 steps completes
      the protocol; CC reports State::Completed which the LCD shows
      as "DONE").

Pytest skips if the loaded protocol PHASH doesn't match HLFB_QUICK
(operator should swap SD + RST first, or run the reset-reload test
which leaves HLFB_QUICK loaded).
"""

from __future__ import annotations

import logging
import time
from collections import Counter, defaultdict
from typing import Dict, Iterable, List, Tuple

import pytest

from test.rig.monitor import Monitor
from test.rig.parser import Frame, GenericFrame, HbFrame
from test.rig.scenario import reset_to_idle
from test.rig.teensy import Teensy


_log = logging.getLogger("rig.proto_exec")


CC_IP = "10.0.0.10"
XPB_IP = "10.0.0.11"

# Mirror of /protocols/HLFB_quick.csv. Tuple = (target_rpm, dwell_seconds).
# Accel is large enough (>=500 RPM/s) that ramp completes well before the
# dwell ends, so the LAST HB of each step is a steady-state sample.
EXPECTED_STEPS: List[Tuple[int, float]] = [
    ( 100, 5.0),
    (   0, 3.0),
    ( 500, 5.0),
    (   0, 3.0),
    (1500, 5.0),
    (   0, 3.0),
    (3000, 5.0),
    (   0, 3.0),
    (-500, 5.0),
    (   0, 3.0),
    (-3000, 5.0),
    (   0, 5.0),
]
EXPECTED_NAME = "HLFB_QUICK"
EXPECTED_PHASH = 1052422810   # informational; we trust PR_BEG, not csv math
EXPECTED_LOOPS = 1

# Tolerances. RPM samples are taken from the FINAL HB(s) of each step,
# which gives the controller plenty of time to settle after the accel
# ramp. Heartbeats arrive at 250 ms cadence so we always see at least
# one sample per step (smallest dwell is 3 s).
RPM_TOL_NONZERO   = 200      # |observed - target| must be within this
RPM_TOL_ZERO_DWELL = 100     # at-rest steps should be very close to 0
TAIL_HBS_PER_STEP = 2        # how many trailing HBs to average per step

# Total protocol runtime: sum(dwells) + a generous accel margin. CSV is
# ~50 s of dwell; add ~20 s slack for ramps and BOOTING/IDLE settle.
RUN_TIMEOUT_S = 90.0
IDLE_GRACE_S = 8.0
BASELINE_S = 3.0

# Reset-then-reload phase. Pulse RST > RESET_THRESHOLD_MS (5000 ms) so
# the CC actually executes the reset (not just ARM->cancel), then wait
# for the full protocol-reload chain to settle and CC to reach IDLE.
RST_PULSE_MS = 6000
RST_RELOAD_SETTLE_S = 22.0


def _fmt_frame(f: Frame) -> str:
    if isinstance(f, HbFrame):
        return (
            f"{f.t_ms:8.1f}ms {f.src_ip:>12s} HB SEQ={f.seq} "
            f"STATE={f.state} STEP={f.step} LOOP={f.loop_idx}/{f.loop_total} "
            f"RPM={f.rpm}"
        )
    if isinstance(f, GenericFrame):
        kv = ";".join(f"{k}={v}" for k, v in f.fields.items())
        return f"{f.t_ms:8.1f}ms {f.src_ip:>12s} {f.kind};{kv}"
    return f"{f.t_ms:8.1f}ms {f.src_ip:>12s} {f.kind}"


def _dump(frames: Iterable[Frame], limit: int = 80) -> str:
    rows = [_fmt_frame(f) for f in list(frames)[-limit:]]
    return "\n  ".join(rows) if rows else "(none)"


def _group_hbs_by_step(hbs: Iterable[HbFrame]) -> Dict[int, List[HbFrame]]:
    """Group RUNNING HBs by their STEP value, preserving order."""
    out: Dict[int, List[HbFrame]] = defaultdict(list)
    for hb in hbs:
        if hb.state == "RUNNING":
            out[hb.step].append(hb)
    return out


@pytest.mark.live_rig
@pytest.mark.slow
@pytest.mark.full
@pytest.mark.stateful
@pytest.mark.requires_hlfb_quick
def test_hlfb_quick_executes_each_step(
    monitor: Monitor, teensy: Teensy
) -> None:
    """RUN=1 -> CC walks all 12 steps with correct steady-state RPM."""

    # --- Phase 0: hard reset to guarantee a clean IDLE start ------------
    # Test may be re-run after a prior pass left CC in COMPLETED, or
    # mid-run in PAUSED. Normalize through the shared reset/reload helper
    # so the loaded protocol and final IDLE state are both explicit.
    reset_to_idle(
        monitor, teensy, _log,
        timeout_s=RST_RELOAD_SETTLE_S,
        require_hlfb_quick=True,
    )

    # --- Phase 1: baseline + protocol-loaded check ----------------------
    teensy.set_run(0)
    time.sleep(0.5)
    monitor.clear()
    base_t = monitor.elapsed_ms
    _log.info("Baseline: observing wire for %.1fs (RUN held low)", BASELINE_S)
    time.sleep(BASELINE_S)

    base_hbs = [
        f for f in monitor.snapshot(since_ms=base_t, src_ip=CC_IP)
        if isinstance(f, HbFrame)
    ]
    if not base_hbs:
        pytest.skip(f"No HBs from CC at {CC_IP}; rig not booted?")
    last = base_hbs[-1]
    _log.info(
        "Baseline OK: %d HBs, last STATE=%s STEP=%d LOOP=%d/%d",
        len(base_hbs), last.state, last.step, last.loop_idx, last.loop_total,
    )
    if last.state not in ("IDLE", "PAUSED"):
        pytest.skip(
            f"Pre-RUN STATE={last.state}; expected IDLE/PAUSED. Reset rig."
        )
    if last.loop_total != EXPECTED_LOOPS:
        pytest.skip(
            f"Loaded protocol LOOP_TOTAL={last.loop_total} != "
            f"{EXPECTED_LOOPS} expected for HLFB_QUICK. Swap SD + RST."
        )

    # --- Phase 2: drive RUN high, watch full protocol -------------------
    run_t = monitor.elapsed_ms
    teensy.set_run(1)
    _log.info("RUN raised; waiting up to %.1fs for protocol to complete",
              RUN_TIMEOUT_S)

    deadline = time.monotonic() + RUN_TIMEOUT_S
    completed = False
    last_progress_step = -1
    next_log_t = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        snap_hbs = [
            f for f in monitor.snapshot(since_ms=run_t, src_ip=CC_IP)
            if isinstance(f, HbFrame)
        ]
        if snap_hbs:
            tail = snap_hbs[-1]
            # Log step transitions immediately, plus a periodic heartbeat.
            if tail.step != last_progress_step or time.monotonic() >= next_log_t:
                _log.info(
                    "  progress: STATE=%s STEP=%d LOOP=%d/%d RPM=%d",
                    tail.state, tail.step, tail.loop_idx, tail.loop_total,
                    tail.rpm,
                )
                last_progress_step = tail.step
                next_log_t = time.monotonic() + 5.0
            # Completion: CC drops to State::Completed after the final
            # step's dwell expires. The LCD shows "DONE".
            saw_running = any(h.state == "RUNNING" for h in snap_hbs)
            if saw_running and tail.state == "COMPLETED":
                completed = True
                break
            # Early failure: error flag asserted.
            err_hbs = [h for h in snap_hbs if h.e_flag]
            if err_hbs:
                teensy.set_run(0)
                msg = (
                    f"VERDICT: ERROR_FLAG_DURING_RUN — CC raised E flag "
                    f"during execution."
                )
                _log.error(msg)
                _log.error("Transcript tail:\n  %s", _dump(snap_hbs))
                pytest.fail(msg)
        time.sleep(0.5)

    teensy.set_run(0)

    # Allow COMPLETED to register if loop closed right at the deadline.
    time.sleep(IDLE_GRACE_S)
    all_hbs = [
        f for f in monitor.snapshot(since_ms=run_t, src_ip=CC_IP)
        if isinstance(f, HbFrame)
    ]
    if not completed:
        last = all_hbs[-1] if all_hbs else None
        last_str = _fmt_frame(last) if last else "(none)"
        msg = (
            f"VERDICT: NEVER_COMPLETED — protocol did not reach "
            f"STATE=COMPLETED within {RUN_TIMEOUT_S:.0f}s. "
            f"Last HB: {last_str}"
        )
        _log.error(msg)
        _log.error("Transcript tail:\n  %s", _dump(all_hbs))
        pytest.fail(msg)

    # --- Phase 3: per-step verification ---------------------------------
    grouped = _group_hbs_by_step(all_hbs)
    seen_steps_in_order: List[int] = []
    last_step = -1
    for hb in all_hbs:
        if hb.state != "RUNNING":
            continue
        if hb.step != last_step:
            seen_steps_in_order.append(hb.step)
            last_step = hb.step

    expected_step_nums = list(range(1, len(EXPECTED_STEPS) + 1))

    # Order check: each expected step must appear in sequence (no skips,
    # no regressions). Allow extra HBs per step but the unique sequence
    # of step transitions must equal 1..N.
    if seen_steps_in_order != expected_step_nums:
        pytest.fail(
            f"VERDICT: STEP_SEQUENCE_WRONG — expected {expected_step_nums}, "
            f"saw {seen_steps_in_order}.\n"
            f"  Transcript tail:\n  {_dump(all_hbs)}"
        )

    # Per-step RPM check.
    failures: List[str] = []
    for idx, (target, dwell) in enumerate(EXPECTED_STEPS, start=1):
        step_hbs = grouped.get(idx, [])
        if not step_hbs:
            failures.append(f"step {idx}: NO HBs observed")
            continue
        tail = step_hbs[-TAIL_HBS_PER_STEP:]
        avg_rpm = sum(h.rpm for h in tail) / len(tail)
        tol = RPM_TOL_ZERO_DWELL if target == 0 else RPM_TOL_NONZERO
        delta = avg_rpm - target
        ok_mag = abs(delta) <= tol
        ok_sign = (target == 0) or ((avg_rpm >= 0) == (target >= 0))
        marker = "OK " if (ok_mag and ok_sign) else "BAD"
        _log.info(
            "  step %2d: target=%+5d dwell=%.1fs n_hbs=%2d tail_avg_rpm=%+7.1f "
            "delta=%+6.1f tol=%d %s",
            idx, target, dwell, len(step_hbs), avg_rpm, delta, tol, marker,
        )
        if not ok_mag:
            failures.append(
                f"step {idx}: tail-avg RPM {avg_rpm:+.1f} differs from "
                f"target {target:+d} by {delta:+.1f} (tol +/-{tol})"
            )
        elif not ok_sign:
            failures.append(
                f"step {idx}: RPM sign wrong (target {target:+d}, "
                f"observed {avg_rpm:+.1f})"
            )

    if failures:
        pytest.fail(
            "VERDICT: RPM_TARGET_MISMATCH — "
            + str(len(failures)) + " step(s) failed:\n  - "
            + "\n  - ".join(failures)
            + f"\n  Transcript tail:\n  {_dump(all_hbs)}"
        )

    # --- Phase 4: summary ----------------------------------------------
    full_snap = monitor.snapshot(since_ms=base_t)
    cc = [f for f in full_snap if f.src_ip == CC_IP]
    xpb = [f for f in full_snap if f.src_ip == XPB_IP]
    _log.info("CC  (%s) frames=%d kinds=%s",
              CC_IP, len(cc), dict(Counter(f.kind for f in cc)))
    _log.info("XPB (%s) frames=%d kinds=%s",
              XPB_IP, len(xpb), dict(Counter(f.kind for f in xpb)))
    _log.info(
        "VERDICT: PASS — HLFB_QUICK ran all %d steps in order; tail-avg RPM "
        "within tolerance for every step; CC reached STATE=COMPLETED.",
        len(EXPECTED_STEPS),
    )
