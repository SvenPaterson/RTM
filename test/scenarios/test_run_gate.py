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
    5. From RUNNING, release RUN to middle (low) first, then pulse RST.
       This matches the physical rocker path (RUN -> middle -> RESET).
       Expect CC to come back to IDLE with no auto-start.
    6. Raise RUN again. Expect IDLE -> RUNNING (proves edge re-arm
       worked after reset from the physical operator path).
    7. Final cleanup: drop RUN, expect RUNNING -> PAUSED.

This is the test that catches the worst class of regressions —
"hardware reset and the rig auto-starts without a fresh RUN edge."
Per the legacy run_gate verdict, that case is a hard FAIL.
"""

from __future__ import annotations

import logging
import time
from collections import Counter
from typing import Iterable, List, Tuple

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


def _wait_for_reload_idle(
    monitor: Monitor, since_ms: float, timeout_s: float
) -> Tuple[List[Frame], List[HbFrame], List[HbFrame]]:
    deadline = time.monotonic() + timeout_s
    proto_rx: List[Frame] = []
    idle_hbs: List[HbFrame] = []
    leak_hbs: List[HbFrame] = []
    while time.monotonic() < deadline:
        snap = monitor.snapshot(since_ms=since_ms, src_ip=CC_IP)
        hbs = [f for f in snap if isinstance(f, HbFrame)]
        if not proto_rx:
            proto_rx = [
                f for f in snap
                if isinstance(f, GenericFrame) and f.kind == "NOTICE"
                and f.fields.get("PROTO_RX") == "OK"
            ]
        leak_hbs = [h for h in hbs if h.state in ("RUNNING", "RESUME")]
        if leak_hbs:
            return proto_rx, idle_hbs, leak_hbs
        idle_hbs = [h for h in hbs if h.state == "IDLE"]
        if proto_rx and idle_hbs:
            return proto_rx, idle_hbs, leak_hbs
        time.sleep(0.25)
    return proto_rx, idle_hbs, leak_hbs


@pytest.mark.live_rig
@pytest.mark.slow
@pytest.mark.full
@pytest.mark.stateful
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
    proto_rx, idle_hbs, early_bad = _wait_for_reload_idle(
        monitor, rst1_t, POST_RST_CAPTURE_S)
    if not proto_rx:
        snap = monitor.snapshot(since_ms=rst1_t)
        pytest.fail(
            "VERDICT: STEP1_NO_RELOAD — first RST didn't complete reload "
            f"within {POST_RST_CAPTURE_S:.0f}s.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )
    if early_bad:
        snap = monitor.snapshot(since_ms=rst1_t)
        pytest.fail(
            f"VERDICT: GATE_LEAK_AT_BOOT — after reset with RUN held "
            f"low, CC entered {early_bad[0].state} before reaching IDLE "
            f"(would auto-start with no operator edge).\n"
            f"  Transcript:\n  {_dump(snap)}"
        )
    if not idle_hbs:
        snap = monitor.snapshot(since_ms=rst1_t)
        hbs = [f for f in snap if isinstance(f, HbFrame) and f.src_ip == CC_IP]
        last_state = hbs[-1].state if hbs else "NONE"
        pytest.fail(
            f"VERDICT: STEP1_NO_IDLE — after reset, CC's last HB was "
            f"STATE={last_state}; expected IDLE within "
            f"{POST_RST_CAPTURE_S:.0f}s.\n  Transcript:\n  {_dump(snap)}"
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

    # --- Step 5: physical reset path RUN->middle then RST --------------
    _log.info("Step 5: physical reset path: drop RUN to middle, then RST")
    edge_t = monitor.elapsed_ms
    teensy.set_run(0)
    if not _wait_for_state(monitor, edge_t, "PAUSED", EDGE_SETTLE_S):
        snap = monitor.snapshot(since_ms=edge_t)
        pytest.fail(
            f"VERDICT: STEP5_NO_PAUSED_FOR_RESET — RUN dropped before "
            f"reset path but no STATE=PAUSED within {EDGE_SETTLE_S:.1f}s.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )

    _log.info("Step 5: pulse RST from middle (RUN low); expect IDLE")
    rst2_t = monitor.elapsed_ms
    teensy.pulse_reset(RST_PULSE_MS)
    proto_rx2, idle_hbs2, early_bad2 = _wait_for_reload_idle(
        monitor, rst2_t, POST_RST_CAPTURE_S)
    if not proto_rx2:
        snap = monitor.snapshot(since_ms=rst2_t)
        pytest.fail(
            "VERDICT: STEP5_NO_RELOAD — second RST didn't complete "
            f"reload within {POST_RST_CAPTURE_S:.0f}s.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )
    if early_bad2:
        snap = monitor.snapshot(since_ms=rst2_t)
        pytest.fail(
            f"VERDICT: GATE_LEAK_AFTER_PHYSICAL_RESET — after physical "
            f"RUN->RST path, CC entered {early_bad2[0].state} before "
            f"reaching IDLE.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )
    if not idle_hbs2:
        snap = monitor.snapshot(since_ms=rst2_t)
        hbs = [f for f in snap if isinstance(f, HbFrame) and f.src_ip == CC_IP]
        last_state = hbs[-1].state if hbs else "NONE"
        pytest.fail(
            f"VERDICT: STEP5_NO_IDLE — after physical RUN->RST path, CC last "
            f"HB STATE={last_state}; expected IDLE within "
            f"{POST_RST_CAPTURE_S:.0f}s.\n  Transcript:\n  {_dump(snap)}"
        )
    time.sleep(2.0)
    post_rst2 = monitor.snapshot(since_ms=rst2_t, src_ip=CC_IP)
    hbs_after_rst2 = [f for f in post_rst2 if isinstance(f, HbFrame)]
    steady2 = hbs_after_rst2[-5:] if hbs_after_rst2 else []
    bad2 = [h for h in steady2 if h.state in ("RUNNING", "RESUME")]
    if bad2:
        snap = monitor.snapshot(since_ms=rst2_t)
        pytest.fail(
            f"VERDICT: GATE_LEAK_AFTER_PHYSICAL_RESET — after physical "
            f"RUN->RST path, CC entered {bad2[0].state}. Last 5 "
            f"HB states: {[h.state for h in steady2]}.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )
    if not steady2 or steady2[-1].state != "IDLE":
        snap = monitor.snapshot(since_ms=rst2_t)
        pytest.fail(
            f"VERDICT: STEP5_NO_IDLE — after physical RUN->RST path, CC last "
            f"HB STATE={steady2[-1].state if steady2 else 'NONE'}; "
            f"expected IDLE.\n  Transcript:\n  {_dump(snap)}"
        )
    _log.info("Step 5 OK: CC at IDLE after physical RUN->RST sequence")

    # --- Step 6: raise RUN -> RUNNING ----------------------------------
    _log.info("Step 6: raise RUN; expect IDLE -> RUNNING (fresh edge)")
    edge_t = monitor.elapsed_ms
    teensy.set_run(1)
    if not _wait_for_state(monitor, edge_t, "RUNNING", EDGE_SETTLE_S):
        snap = monitor.snapshot(since_ms=edge_t)
        teensy.set_run(0)
        pytest.fail(
            f"VERDICT: STEP6_NO_RESTART — fresh RUN edge after physical "
            f"reset path "
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
        "no auto-start after physical RUN->RST path, and "
        "edges trigger start/pause/resume cleanly."
    )
