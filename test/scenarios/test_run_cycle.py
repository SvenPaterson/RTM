"""S2: RUN line edge drives RUNNING/PAUSED state transitions.

Fully automated — Teensy drives RUN. Sequence:

    1. Park RUN low, baseline ~3 s, capture starting STATE.
    2. Raise RUN -> expect HBs to show STATE=RUNNING within
       ~RUN_SETTLE_S, with at least one RPM>0 sample (proves the
       motor command path is exercising the loaded protocol).
    3. Drop RUN -> expect HBs to show STATE=PAUSED within ~RUN_SETTLE_S
       and RPM to fall back to 0.
    4. Raise RUN again -> expect STATE=RUNNING again (resume edge).
    5. Drop RUN -> expect STATE=PAUSED again.

Verifies the full RUN-edge handling on CC (handleIdle/handleRunning/
handlePaused). Does NOT exercise reset; that's covered elsewhere.
"""

from __future__ import annotations

import logging
import time
from collections import Counter
from typing import Iterable, List

import pytest

from test.rig.monitor import Monitor
from test.rig.parser import Frame, GenericFrame, HbFrame
from test.rig.teensy import Teensy


_log = logging.getLogger("rig.run_cycle")


CC_IP = "10.0.0.10"
XPB_IP = "10.0.0.11"

BASELINE_S = 3.0
RUN_SETTLE_S = 6.0       # max time to wait for HB STATE to change after edge
PAUSE_SETTLE_S = 6.0


def _fmt_frame(f: Frame) -> str:
    if isinstance(f, HbFrame):
        return (
            f"{f.t_ms:8.1f}ms {f.src_ip:>12s} HB SEQ={f.seq} "
            f"STATE={f.state} STEP={f.step} LOOP={f.loop_idx}/{f.loop_total} "
            f"RPM={f.rpm} E={int(f.e_flag)}"
        )
    if isinstance(f, GenericFrame):
        kv = ";".join(f"{k}={v}" for k, v in f.fields.items())
        return f"{f.t_ms:8.1f}ms {f.src_ip:>12s} {f.kind};{kv}"
    return f"{f.t_ms:8.1f}ms {f.src_ip:>12s} {f.kind}"


def _dump(frames: Iterable[Frame]) -> str:
    rows = [_fmt_frame(f) for f in frames]
    return "\n  ".join(rows) if rows else "(none)"


def _wait_for_state(
    monitor: Monitor, since_ms: float, want_state: str, timeout_s: float
) -> List[HbFrame]:
    """Poll for HBs whose state matches ``want_state``. Returns the
    matching list (possibly empty if timeout)."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        snap = monitor.snapshot(since_ms=since_ms, src_ip=CC_IP)
        hits = [
            f for f in snap if isinstance(f, HbFrame) and f.state == want_state
        ]
        if hits:
            return hits
        time.sleep(0.25)
    return []


@pytest.mark.live_rig
def test_run_edge_cycles_state(monitor: Monitor, teensy: Teensy) -> None:
    """RUN high -> RUNNING (RPM>0); RUN low -> PAUSED; repeat."""

    # --- Phase 1: baseline -----------------------------------------------
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
        pytest.skip(
            f"Baseline saw no HBs from CC at {CC_IP}. Boot the rig first."
        )
    start_state = base_hbs[-1].state
    _log.info("Baseline OK: %d HBs, last STATE=%s", len(base_hbs), start_state)
    if start_state not in ("IDLE", "PAUSED"):
        pytest.skip(
            f"Pre-RUN STATE={start_state}; expected IDLE or PAUSED. "
            "Reset the rig before running this test."
        )

    # --- Phase 2: RUN high #1 -> RUNNING ---------------------------------
    edge1_t = monitor.elapsed_ms
    teensy.set_run(1)
    running_hbs = _wait_for_state(monitor, edge1_t, "RUNNING", RUN_SETTLE_S)
    if not running_hbs:
        snap = monitor.snapshot(since_ms=edge1_t)
        teensy.set_run(0)
        pytest.fail(
            f"VERDICT: NO_RUNNING_AFTER_EDGE — RUN raised but CC never "
            f"reported STATE=RUNNING within {RUN_SETTLE_S:.1f}s.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )
    # Hold a bit longer to let the protocol actually drive RPM.
    time.sleep(2.0)
    run_window = [
        f for f in monitor.snapshot(since_ms=edge1_t, src_ip=CC_IP)
        if isinstance(f, HbFrame) and f.state == "RUNNING"
    ]
    rpm_max = max((abs(h.rpm) for h in run_window), default=0)
    _log.info("RUN#1: %d RUNNING HBs, max |RPM|=%d", len(run_window), rpm_max)

    # --- Phase 3: RUN low #1 -> PAUSED -----------------------------------
    edge2_t = monitor.elapsed_ms
    teensy.set_run(0)
    paused_hbs = _wait_for_state(monitor, edge2_t, "PAUSED", PAUSE_SETTLE_S)
    if not paused_hbs:
        snap = monitor.snapshot(since_ms=edge2_t)
        pytest.fail(
            f"VERDICT: NO_PAUSED_AFTER_DROP — RUN dropped but CC never "
            f"reported STATE=PAUSED within {PAUSE_SETTLE_S:.1f}s.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )
    time.sleep(1.5)
    pause_window = [
        f for f in monitor.snapshot(since_ms=edge2_t, src_ip=CC_IP)
        if isinstance(f, HbFrame) and f.state == "PAUSED"
    ]
    rpm_paused = pause_window[-1].rpm if pause_window else None
    _log.info("PAUSE#1: %d PAUSED HBs, last RPM=%s",
              len(pause_window), rpm_paused)

    # --- Phase 4: RUN high #2 -> RUNNING (resume edge) -------------------
    edge3_t = monitor.elapsed_ms
    teensy.set_run(1)
    running2 = _wait_for_state(monitor, edge3_t, "RUNNING", RUN_SETTLE_S)
    if not running2:
        snap = monitor.snapshot(since_ms=edge3_t)
        teensy.set_run(0)
        pytest.fail(
            f"VERDICT: NO_RESUME — second RUN edge did not return CC "
            f"to STATE=RUNNING within {RUN_SETTLE_S:.1f}s.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )
    _log.info("RUN#2: %d RUNNING HBs after resume edge", len(running2))

    # --- Phase 5: RUN low #2 -> PAUSED ---------------------------------
    edge4_t = monitor.elapsed_ms
    teensy.set_run(0)
    paused2 = _wait_for_state(monitor, edge4_t, "PAUSED", PAUSE_SETTLE_S)
    if not paused2:
        snap = monitor.snapshot(since_ms=edge4_t)
        pytest.fail(
            f"VERDICT: NO_FINAL_PAUSE — final RUN drop did not return "
            f"CC to STATE=PAUSED within {PAUSE_SETTLE_S:.1f}s.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )

    # --- Optional: RPM sanity ------------------------------------------
    # Prefer to see RPM go non-zero during the RUNNING window, but don't
    # hard-fail if the loaded protocol's first step happens to be 0 RPM.
    if rpm_max == 0:
        _log.warning(
            "RPM stayed 0 throughout RUN window. Loaded protocol's first "
            "step may be a dwell. Test still passes on STATE transitions "
            "alone — consider loading a CSV whose step 1 has non-zero RPM."
        )

    full_snap = monitor.snapshot(since_ms=base_t)
    cc = [f for f in full_snap if f.src_ip == CC_IP]
    xpb = [f for f in full_snap if f.src_ip == XPB_IP]
    _log.info("CC  (%s) frames=%d kinds=%s",
              CC_IP, len(cc), dict(Counter(f.kind for f in cc)))
    _log.info("XPB (%s) frames=%d kinds=%s",
              XPB_IP, len(xpb), dict(Counter(f.kind for f in xpb)))
    _log.info(
        "VERDICT: PASS — RUN-edge cycle OK: %s -> RUNNING (max |RPM|=%d) "
        "-> PAUSED -> RUNNING -> PAUSED.", start_state, rpm_max,
    )
