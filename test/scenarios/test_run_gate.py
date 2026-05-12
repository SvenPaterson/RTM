"""S7: RUN-edge gate semantics across reset boundaries.

This is the integration-style "gauntlet" that exercises CC's policy
that RUN is treated as an *edge* trigger to start (not a level), with
the gate re-armed by every reset. Concretely:

    1. Park RUN low. Pulse RST (>5 s). Expect CC to come back to
       STATE=IDLE *with RUN still low* — must NOT auto-start.
    2. Raise RUN (rising edge). Expect IDLE -> RUNNING.
    3. Drop RUN. Expect RUNNING -> PAUSED.
    4. RAISE RUN AGAIN (without dropping). Expect PAUSED -> RUNNING
       (resume edge) — verifies the rising edge from PAUSED is honored.
    5. Pulse RST again *while RUN is still high*. Expect CC to come
       back to IDLE — the held-high RUN must NOT be interpreted as a
       fresh start; the gate re-arms and waits for an edge.
    6. Drop RUN, then raise it. Expect IDLE -> RUNNING (proves edge
       re-arm worked).
    7. Final cleanup: drop RUN, expect RUNNING -> PAUSED.

This is the test that catches the worst class of regressions —
"hardware reset and the rig auto-starts even though the operator's
hand is on RUN." Per the legacy run_gate verdict, that case is a
hard FAIL.
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


_log = logging.getLogger("rig.run_gate")


CC_IP = "10.0.0.10"
XPB_IP = "10.0.0.11"

PRE_OBSERVE_S = 2.0
RST_PULSE_MS = 6000
POST_RST_CAPTURE_S = 25.0       # wait for full reload chain to complete
EDGE_SETTLE_S = 6.0             # max time to wait for an HB STATE change
RUN_HOLD_S = 2.0                # how long to dwell in each RUNNING window
PAUSE_HOLD_S = 1.5


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


def _wait_for_proto_rx_ok(
    monitor: Monitor, since_ms: float, timeout_s: float
) -> List[Frame]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        snap = monitor.snapshot(since_ms=since_ms, src_ip=CC_IP)
        hits = [
            f for f in snap
            if isinstance(f, GenericFrame) and f.kind == "NOTICE"
            and f.fields.get("PROTO_RX") == "OK"
        ]
        if hits:
            return hits
        time.sleep(0.25)
    return []


@pytest.mark.live_rig
@pytest.mark.slow
def test_run_gate_after_reset(monitor: Monitor, teensy: Teensy) -> None:
    """Full RUN-edge gate gauntlet across two resets."""

    # --- Step 0: baseline ----------------------------------------------
    teensy.set_run(0)
    time.sleep(0.5)
    monitor.clear()
    base_t = monitor.elapsed_ms
    _log.info("Step 0: baseline %.1fs (RUN low)", PRE_OBSERVE_S)
    time.sleep(PRE_OBSERVE_S)
    base_hbs = [
        f for f in monitor.snapshot(since_ms=base_t, src_ip=CC_IP)
        if isinstance(f, HbFrame)
    ]
    if not base_hbs:
        pytest.skip(
            f"Baseline saw no HBs from CC at {CC_IP}. Boot the rig first."
        )

    # --- Step 1: RST while RUN low; expect IDLE, no auto-start ---------
    _log.info("Step 1: RST while RUN held LOW; expect IDLE post-reload")
    rst1_t = monitor.elapsed_ms
    teensy.pulse_reset(RST_PULSE_MS)
    proto_rx = _wait_for_proto_rx_ok(monitor, rst1_t, POST_RST_CAPTURE_S)
    if not proto_rx:
        snap = monitor.snapshot(since_ms=rst1_t)
        pytest.fail(
            "VERDICT: STEP1_NO_RELOAD — first RST didn't complete reload "
            f"within {POST_RST_CAPTURE_S:.0f}s.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )
    # Give CC ~2 s to settle into IDLE and observe HBs.
    time.sleep(2.0)
    post_rst1 = monitor.snapshot(since_ms=rst1_t, src_ip=CC_IP)
    hbs_after_rst1 = [f for f in post_rst1 if isinstance(f, HbFrame)]
    # Take the *last few* HBs as the steady state.
    steady_hbs = hbs_after_rst1[-5:] if hbs_after_rst1 else []
    bad = [h for h in steady_hbs if h.state in ("RUNNING", "RESUME")]
    if bad:
        snap = monitor.snapshot(since_ms=rst1_t)
        pytest.fail(
            f"VERDICT: GATE_LEAK_AT_BOOT — after reset with RUN held "
            f"low, CC entered {bad[0].state} (would auto-start with no "
            f"operator edge). Last 5 HB states: "
            f"{[h.state for h in steady_hbs]}.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )
    if not steady_hbs or steady_hbs[-1].state != "IDLE":
        snap = monitor.snapshot(since_ms=rst1_t)
        pytest.fail(
            f"VERDICT: STEP1_NO_IDLE — after reset, CC's last HB was "
            f"STATE={steady_hbs[-1].state if steady_hbs else 'NONE'}; "
            f"expected IDLE.\n  Transcript:\n  {_dump(snap)}"
        )
    _log.info("Step 1 OK: CC at IDLE, RUN low, no auto-start")

    # --- Step 2: rising RUN edge -> RUNNING ----------------------------
    _log.info("Step 2: raise RUN; expect IDLE -> RUNNING")
    edge_t = monitor.elapsed_ms
    teensy.set_run(1)
    if not _wait_for_state(monitor, edge_t, "RUNNING", EDGE_SETTLE_S):
        snap = monitor.snapshot(since_ms=edge_t)
        teensy.set_run(0)
        pytest.fail(
            f"VERDICT: STEP2_NO_RUNNING — RUN raised but no STATE=RUNNING "
            f"within {EDGE_SETTLE_S:.1f}s.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )
    time.sleep(RUN_HOLD_S)

    # --- Step 3: drop RUN -> PAUSED ------------------------------------
    _log.info("Step 3: drop RUN; expect RUNNING -> PAUSED")
    edge_t = monitor.elapsed_ms
    teensy.set_run(0)
    if not _wait_for_state(monitor, edge_t, "PAUSED", EDGE_SETTLE_S):
        snap = monitor.snapshot(since_ms=edge_t)
        pytest.fail(
            f"VERDICT: STEP3_NO_PAUSED — RUN dropped but no STATE=PAUSED "
            f"within {EDGE_SETTLE_S:.1f}s.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )
    time.sleep(PAUSE_HOLD_S)

    # --- Step 4: rising edge from PAUSED -> RUNNING --------------------
    _log.info("Step 4: raise RUN; expect PAUSED -> RUNNING (resume)")
    edge_t = monitor.elapsed_ms
    teensy.set_run(1)
    if not _wait_for_state(monitor, edge_t, "RUNNING", EDGE_SETTLE_S):
        snap = monitor.snapshot(since_ms=edge_t)
        teensy.set_run(0)
        pytest.fail(
            f"VERDICT: STEP4_NO_RESUME — second RUN edge failed to "
            f"return CC to STATE=RUNNING within {EDGE_SETTLE_S:.1f}s.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )
    time.sleep(RUN_HOLD_S)

    # --- Step 5: RST while RUN is HIGH; expect IDLE, no auto-start ----
    _log.info("Step 5: RST while RUN held HIGH; expect IDLE, gate must "
              "re-arm and ignore the held-high level")
    rst2_t = monitor.elapsed_ms
    teensy.pulse_reset(RST_PULSE_MS)  # RUN stays high throughout
    proto_rx2 = _wait_for_proto_rx_ok(monitor, rst2_t, POST_RST_CAPTURE_S)
    if not proto_rx2:
        snap = monitor.snapshot(since_ms=rst2_t)
        teensy.set_run(0)
        pytest.fail(
            "VERDICT: STEP5_NO_RELOAD — second RST didn't complete "
            f"reload within {POST_RST_CAPTURE_S:.0f}s.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )
    time.sleep(2.0)
    post_rst2 = monitor.snapshot(since_ms=rst2_t, src_ip=CC_IP)
    hbs_after_rst2 = [f for f in post_rst2 if isinstance(f, HbFrame)]
    steady2 = hbs_after_rst2[-5:] if hbs_after_rst2 else []
    bad2 = [h for h in steady2 if h.state in ("RUNNING", "RESUME")]
    if bad2:
        snap = monitor.snapshot(since_ms=rst2_t)
        teensy.set_run(0)
        pytest.fail(
            f"VERDICT: GATE_LEAK_RUN_HELD_HIGH — after RST with RUN held "
            f"HIGH, CC entered {bad2[0].state} (auto-started off the "
            f"held level instead of waiting for a fresh edge). Last 5 "
            f"HB states: {[h.state for h in steady2]}.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )
    if not steady2 or steady2[-1].state != "IDLE":
        snap = monitor.snapshot(since_ms=rst2_t)
        teensy.set_run(0)
        pytest.fail(
            f"VERDICT: STEP5_NO_IDLE — after RST-with-RUN-high, CC last "
            f"HB STATE={steady2[-1].state if steady2 else 'NONE'}; "
            f"expected IDLE.\n  Transcript:\n  {_dump(snap)}"
        )
    _log.info("Step 5 OK: CC at IDLE despite RUN held high — gate re-armed")

    # --- Step 6: drop RUN, raise it -> RUNNING -------------------------
    _log.info("Step 6: drop+raise RUN; expect IDLE -> RUNNING (fresh edge)")
    teensy.set_run(0)
    time.sleep(0.75)
    edge_t = monitor.elapsed_ms
    teensy.set_run(1)
    if not _wait_for_state(monitor, edge_t, "RUNNING", EDGE_SETTLE_S):
        snap = monitor.snapshot(since_ms=edge_t)
        teensy.set_run(0)
        pytest.fail(
            f"VERDICT: STEP6_NO_RESTART — fresh RUN edge after gate "
            f"re-arm did not start RUNNING within {EDGE_SETTLE_S:.1f}s.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )
    time.sleep(RUN_HOLD_S)

    # --- Step 7: final pause ------------------------------------------
    _log.info("Step 7: drop RUN; expect RUNNING -> PAUSED (cleanup)")
    edge_t = monitor.elapsed_ms
    teensy.set_run(0)
    if not _wait_for_state(monitor, edge_t, "PAUSED", EDGE_SETTLE_S):
        snap = monitor.snapshot(since_ms=edge_t)
        pytest.fail(
            f"VERDICT: STEP7_NO_PAUSE — final RUN drop did not return "
            f"CC to STATE=PAUSED within {EDGE_SETTLE_S:.1f}s.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )

    full = monitor.snapshot(since_ms=base_t)
    cc = [f for f in full if f.src_ip == CC_IP]
    xpb = [f for f in full if f.src_ip == XPB_IP]
    _log.info("CC  (%s) frames=%d kinds=%s",
              CC_IP, len(cc), dict(Counter(f.kind for f in cc)))
    _log.info("XPB (%s) frames=%d kinds=%s",
              XPB_IP, len(xpb), dict(Counter(f.kind for f in xpb)))
    _log.info(
        "VERDICT: PASS — RUN-edge gate held across two resets: "
        "no auto-start with RUN-low after RST, no auto-start with "
        "RUN-high after RST, edges trigger start/pause/resume cleanly."
    )
