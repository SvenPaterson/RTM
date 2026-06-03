"""S5: Sub-threshold RST pulses must NOT trigger a logical reset.

CC's RESET_THRESHOLD_MS is 5000 ms. Any RST pulse shorter than that
should be debounced/cancelled — the only on-wire evidence allowed is
``CMD;RESET=ARM`` (the operator-feedback countdown). We must NOT see:

    CMD;RESET=EXEC      (CC committed to reset)
    NOTICE;XPB_RESET    (XPB rebooted)
    REQ:PROTO           (post-boot protocol re-request)
    PR_BEG / PR_END     (XPB re-uploading the protocol)

Sequence:
    1. Park RUN low, baseline 2 s, capture starting STATE.
    2. For each pulse width in PULSE_WIDTHS_MS, drive RST high for
       that long, then sleep INTER_PULSE_S to let any spurious chain
       fire.
    3. Capture full transcript and assert no EXEC, no XPB_RESET, no
       REQ:PROTO, no PR_BEG/PR_END.
    4. Assert CC's STATE never left IDLE/PAUSED (no transient
       RUNNING/RESUME spurs).

If the test PASSES we know the debouncer holds across a representative
spread of widths from 500 ms (very short) to 4500 ms (just below
threshold).
"""

from __future__ import annotations

import logging
import time
from collections import Counter
from typing import Iterable

import pytest

from test.rig.monitor import Monitor
from test.rig.parser import Frame, GenericFrame, HbFrame
from test.rig.teensy import Teensy


_log = logging.getLogger("rig.reset_cancel")


CC_IP = "10.0.0.10"
XPB_IP = "10.0.0.11"

# Spread across the sub-threshold range. Threshold is 5000 ms.
PULSE_WIDTHS_MS = (500, 1500, 2500, 3500, 4500)
BASELINE_S = 2.0
INTER_PULSE_S = 2.0     # let any spurious chain fire before next pulse
SETTLE_AFTER_S = 4.0    # final settle before snapshotting


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


@pytest.mark.live_rig
@pytest.mark.slow
@pytest.mark.full
@pytest.mark.stateful
def test_subthreshold_rst_pulses_are_cancelled(
    monitor: Monitor, teensy: Teensy
) -> None:
    """N short RST pulses, all <5 s. Verify no logical reset fires."""

    # --- Phase 1: baseline ----------------------------------------------
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

    # --- Phase 2: pulse train -------------------------------------------
    pulse_t = monitor.elapsed_ms
    for ms in PULSE_WIDTHS_MS:
        _log.info("Pulsing RST for %d ms (sub-threshold; expect cancel)", ms)
        teensy.pulse_reset(ms)
        time.sleep(INTER_PULSE_S)

    _log.info("Settling for %.1fs after final pulse", SETTLE_AFTER_S)
    time.sleep(SETTLE_AFTER_S)

    snap = monitor.snapshot(since_ms=pulse_t)
    cc = [f for f in snap if f.src_ip == CC_IP]
    xpb = [f for f in snap if f.src_ip == XPB_IP]
    _log.info("CC  (%s) frames=%d kinds=%s",
              CC_IP, len(cc), dict(Counter(f.kind for f in cc)))
    _log.info("XPB (%s) frames=%d kinds=%s",
              XPB_IP, len(xpb), dict(Counter(f.kind for f in xpb)))

    # --- Mine forbidden markers -----------------------------------------
    cmd_arm = [
        f for f in cc
        if isinstance(f, GenericFrame) and f.kind == "CMD"
        and f.fields.get("RESET") == "ARM"
    ]
    cmd_exec = [
        f for f in cc
        if isinstance(f, GenericFrame) and f.kind == "CMD"
        and f.fields.get("RESET") == "EXEC"
    ]
    cmd_cancel = [
        f for f in cc
        if isinstance(f, GenericFrame) and f.kind == "CMD"
        and f.fields.get("RESET") == "CANCEL"
    ]
    xpb_reset = [
        f for f in (cc + xpb)
        if isinstance(f, GenericFrame) and f.kind == "NOTICE"
        and ("XPB_RESET" in f.fields or f.fields.get("XPB_RESET") == "NOW")
    ]
    req_proto = [f for f in cc if f.kind in ("REQ:PROTO", "REQ")]
    pr_beg = [f for f in xpb if f.kind == "PR_BEG"]
    pr_end = [f for f in xpb if f.kind == "PR_END"]

    sw_rst_edges = [
        f for f in xpb
        if isinstance(f, GenericFrame) and f.kind == "SW"
        and f.fields.get("RST") == "1"
    ]

    _log.info("RESET-CANCEL MARKERS:")
    _log.info("  Pulses sent           : %d (widths=%s)",
              len(PULSE_WIDTHS_MS), list(PULSE_WIDTHS_MS))
    _log.info("  SW;RST=1 from XPB     : %d", len(sw_rst_edges))
    _log.info("  CMD;RESET=ARM         : %d  (operator-feedback ok)",
              len(cmd_arm))
    _log.info("  CMD;RESET=CANCEL      : %d  (expected for sub-threshold)",
              len(cmd_cancel))
    _log.info("  CMD;RESET=EXEC        : %d  (FORBIDDEN)", len(cmd_exec))
    _log.info("  NOTICE;XPB_RESET=...  : %d  (FORBIDDEN)", len(xpb_reset))
    _log.info("  REQ:PROTO             : %d  (FORBIDDEN)", len(req_proto))
    _log.info("  PR_BEG / PR_END       : %d / %d  (FORBIDDEN)",
              len(pr_beg), len(pr_end))

    # XPB should at minimum have noticed the edge. If not, the test
    # didn't really exercise the path — surface as a skip not a pass.
    if not sw_rst_edges:
        pytest.skip(
            "XPB never emitted SW;RST=1 across the pulse train. Either "
            "the Teensy isn't wired to the RST input or XPB's edge "
            "detector dropped them all — investigate before trusting "
            "this test as a 'pass'."
        )

    # --- Verdict ladder -------------------------------------------------
    if cmd_exec:
        pytest.fail(
            f"VERDICT: SPURIOUS_RESET — sub-threshold pulse(s) triggered "
            f"{len(cmd_exec)} CMD;RESET=EXEC frames.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )
    if xpb_reset:
        pytest.fail(
            f"VERDICT: SPURIOUS_XPB_RESET — XPB rebooted "
            f"({len(xpb_reset)} NOTICE frames) on a sub-threshold pulse.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )
    if req_proto:
        pytest.fail(
            f"VERDICT: SPURIOUS_REQ_PROTO — CC re-asked for the protocol "
            f"({len(req_proto)} times) without a real reset.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )
    if pr_beg or pr_end:
        pytest.fail(
            f"VERDICT: SPURIOUS_REUPLOAD — XPB re-uploaded the protocol "
            f"(PR_BEG={len(pr_beg)} PR_END={len(pr_end)}) without a real "
            f"reset.\n  Transcript:\n  {_dump(snap)}"
        )

    # Every ARM must cancel before the next ARM and before any EXEC.
    reset_events = [
        f for f in cc
        if isinstance(f, GenericFrame) and f.kind == "CMD"
        and f.fields.get("RESET") in {"ARM", "CANCEL", "EXEC"}
    ]
    pending_arm = None
    for evt in reset_events:
        marker = evt.fields.get("RESET")
        if marker == "ARM":
            if pending_arm is not None:
                pytest.fail(
                    "VERDICT: ARM_WITHOUT_CANCEL — saw CMD;RESET=ARM before "
                    "the prior ARM was cancelled.\n"
                    f"  Transcript:\n  {_dump(snap)}"
                )
            pending_arm = evt
        elif marker == "EXEC" and pending_arm is not None:
            pytest.fail(
                "VERDICT: ARM_ESCALATED_TO_EXEC — sub-threshold ARM escalated "
                "to EXEC before cancel.\n"
                f"  Transcript:\n  {_dump(snap)}"
            )
        elif marker == "CANCEL" and pending_arm is not None:
            pending_arm = None

    if pending_arm is not None:
        pytest.fail(
            "VERDICT: MISSING_CANCEL — saw CMD;RESET=ARM with no matching "
            "CMD;RESET=CANCEL in capture window.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )

    # State sanity: CC must not have spuriously transitioned out of
    # IDLE/PAUSED into RUNNING/RESUME during the pulse train.
    forbidden_states = {
        "RUNNING", "RESUME", "BOOT", "BOOTING", "PROTO_LOADING",
    }
    bad_state_hbs = [
        f for f in cc
        if isinstance(f, HbFrame) and f.state.upper() in forbidden_states
    ]
    if bad_state_hbs:
        states = Counter(h.state for h in bad_state_hbs)
        pytest.fail(
            f"VERDICT: SPURIOUS_STATE_CHANGE — CC entered forbidden "
            f"state(s) {dict(states)} during sub-threshold pulse train.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )

    _log.info(
        "VERDICT: PASS — %d sub-threshold RST pulses (%d..%d ms) all "
        "cancelled. ARM=%d EXEC=%d. CC stayed in %s throughout.",
        len(PULSE_WIDTHS_MS), min(PULSE_WIDTHS_MS), max(PULSE_WIDTHS_MS),
        len(cmd_arm), len(cmd_exec), start_state,
    )
