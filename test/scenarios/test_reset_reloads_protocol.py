"""S4c: RST (logical reset) drives full protocol-reload chain.

Fully automated — the Teensy drives the RST line. Sequence:

    1. Baseline: confirm rig is alive (CC HBs + XPB STATs). Pre-reset
       STATE doesn't matter — RUN is parked low and RST will reboot
       both boards anyway.
    2. Teensy pulses RST high for >=5 s (logical reset threshold).
    3. Capture wire transcript and verify the full reset+reload chain:
         SW;RST=1            from XPB
         CMD;RESET=ARM       from CC  (countdown to user)
         CMD;RESET=EXEC      from CC
         (XPB logicalReset → CC logicalReset → BOOT)
         REQ:PROTO           from CC
         PR_BEG/PR_DAT/PR_END from XPB
         NOTICE;PROTO_RX=OK  from CC
         CC HB STATE=IDLE
    4. Verify the reloaded PR_BEG fields are non-empty and the PHASH
       echoed in PROTO_RX=OK matches PR_BEG.

The companion ``test_cold_boot_protocol_upload.py`` covers the
human-in-the-loop power-cycle path. This test exercises the
production-equivalent reset path that an operator triggers from the
front-panel toggle (and that XPB's logicalReset() must service).
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


_log = logging.getLogger("rig.reset_reload")


PRE_OBSERVE_S = 5.0          # baseline window
RST_PULSE_MS = 6000          # CC RESET_THRESHOLD_MS is 5000; 6 s clears it
POST_PULSE_CAPTURE_S = 25.0  # capture window after the pulse starts
CC_IP = "10.0.0.10"
XPB_IP = "10.0.0.11"


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
@pytest.mark.smoke
@pytest.mark.full
@pytest.mark.stateful
def test_rst_pulse_reloads_protocol(monitor: Monitor, teensy: Teensy) -> None:
    """Teensy-driven RST pulse triggers full reload; verify on the wire."""

    # --- Phase 1: park RUN low, then baseline ----------------------------
    teensy.set_run(0)
    monitor.clear()
    base_t = monitor.elapsed_ms
    _log.info("Baseline: observing wire for %.1fs to confirm rig is alive",
              PRE_OBSERVE_S)
    time.sleep(PRE_OBSERVE_S)
    base_snap = monitor.snapshot(since_ms=base_t)
    base_cc = [f for f in base_snap if f.src_ip == CC_IP]
    base_xpb = [f for f in base_snap if f.src_ip == XPB_IP]
    base_hbs = [f for f in base_cc if isinstance(f, HbFrame)]
    base_stats = [f for f in base_xpb if f.kind == "STAT"]

    if not base_hbs:
        pytest.skip(
            f"Baseline saw no HBs from CC at {CC_IP}. Boot the rig "
            "(run the cold-boot test first) before running this test."
        )
    if not base_stats:
        pytest.skip(
            f"Baseline saw no STATs from XPB at {XPB_IP}. Boot the rig "
            "before running this test."
        )

    last_base_hb = base_hbs[-1]
    _log.info("Baseline OK: CC=%d HBs (last STATE=%s), XPB=%d STATs",
              len(base_hbs), last_base_hb.state, len(base_stats))

    # --- Phase 2: drive the RST pulse ------------------------------------
    monitor.clear()
    pulse_t = monitor.elapsed_ms
    _log.info("Driving RST high for %d ms via Teensy", RST_PULSE_MS)
    teensy.pulse_reset(RST_PULSE_MS)
    # pulse_reset() blocks for the pulse width, so the rig has already
    # entered the reset phase by the time we start the capture window.

    # --- Phase 3: capture the reload chain -------------------------------
    elapsed_since_pulse_start = (monitor.elapsed_ms - pulse_t) / 1000.0
    remaining = max(0.0, POST_PULSE_CAPTURE_S - elapsed_since_pulse_start)
    if remaining > 0:
        _log.info("Capturing for %.1fs more after pulse end", remaining)
        time.sleep(remaining)

    snap = monitor.snapshot(since_ms=pulse_t)
    cc = [f for f in snap if f.src_ip == CC_IP]
    xpb = [f for f in snap if f.src_ip == XPB_IP]

    _log.info("CC  (%s) frames=%d kinds=%s",
              CC_IP, len(cc), dict(Counter(f.kind for f in cc)))
    _log.info("XPB (%s) frames=%d kinds=%s",
              XPB_IP, len(xpb), dict(Counter(f.kind for f in xpb)))
    _log.info("Full transcript:\n  %s", _dump(snap))

    # --- Mine the reset chain markers -----------------------------------
    sw_rst = [
        f for f in xpb
        if f.kind == "SW" and isinstance(f, GenericFrame)
        and f.fields.get("RST") == "1"
    ]
    cmd_arm = [
        f for f in cc
        if f.kind == "CMD" and isinstance(f, GenericFrame)
        and f.fields.get("RESET") == "ARM"
    ]
    cmd_exec = [
        f for f in cc
        if f.kind == "CMD" and isinstance(f, GenericFrame)
        and f.fields.get("RESET") == "EXEC"
    ]
    req_proto = [f for f in cc if f.kind in ("REQ:PROTO", "REQ")]
    pr_beg = [f for f in xpb if f.kind == "PR_BEG"]
    pr_dat = [f for f in xpb if f.kind == "PR_DAT"]
    pr_end = [f for f in xpb if f.kind == "PR_END"]
    proto_rx_ok = [
        f for f in cc
        if isinstance(f, GenericFrame)
        and f.kind == "NOTICE"
        and f.fields.get("PROTO_RX") == "OK"
    ]
    hbs = [f for f in cc if isinstance(f, HbFrame)]
    idle_hbs = [h for h in hbs if h.state == "IDLE"]

    new_name = ""
    new_phash = ""
    new_steps = 0
    new_loops = 0
    if pr_beg and isinstance(pr_beg[0], GenericFrame):
        f0 = pr_beg[0]
        new_name = f0.fields.get("NAME", "")
        new_phash = f0.fields.get("PHASH", "")
        try:
            new_steps = int(f0.fields.get("STEPS", "0"))
        except ValueError:
            pass
        try:
            new_loops = int(f0.fields.get("LOOPS", "0"))
        except ValueError:
            pass

    _log.info("RESET CHAIN MARKERS:")
    _log.info("  SW;RST=1   from XPB : %d", len(sw_rst))
    _log.info("  CMD;RESET=ARM       : %d", len(cmd_arm))
    _log.info("  CMD;RESET=EXEC      : %d", len(cmd_exec))
    _log.info("  REQ:PROTO  from CC  : %d", len(req_proto))
    _log.info("  PR_BEG     from XPB : %d  (NAME=%r STEPS=%d LOOPS=%d "
              "PHASH=%s)", len(pr_beg), new_name, new_steps, new_loops,
              new_phash)
    _log.info("  PR_DAT     from XPB : %d", len(pr_dat))
    _log.info("  PR_END     from XPB : %d", len(pr_end))
    _log.info("  PROTO_RX=OK from CC : %d", len(proto_rx_ok))
    _log.info("  HBs total / IDLE    : %d / %d", len(hbs), len(idle_hbs))

    # --- Verdict ladder -------------------------------------------------
    if not sw_rst:
        pytest.fail(
            "VERDICT: NO_SW_RST — Teensy pulsed RST but XPB never "
            "reported SW;RST=1. Wiring or XPB debouncer issue.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )
    if not cmd_arm and not cmd_exec:
        pytest.fail(
            "VERDICT: STALL_NO_RESET_CMD — XPB saw SW;RST=1 but CC "
            "never emitted CMD;RESET=ARM or =EXEC.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )
    if not cmd_exec:
        pytest.fail(
            f"VERDICT: STALL_NO_EXEC — got {len(cmd_arm)} CMD;RESET=ARM "
            "but no CMD;RESET=EXEC. CC stuck in the ARM countdown.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )
    if not req_proto:
        pytest.fail(
            "VERDICT: STALL_NO_REREQ — reset chain fired but CC never "
            "re-sent REQ:PROTO. logicalReset() did not re-arm the "
            "protocol-request loop.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )
    if not pr_beg:
        pytest.fail(
            "VERDICT: STALL_NO_PR_BEG — CC re-asked but XPB never "
            "replied with PR_BEG after its own logicalReset.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )
    if not pr_end:
        pytest.fail(
            f"VERDICT: STALL_MID_UPLOAD — got PR_BEG and {len(pr_dat)} "
            "PR_DAT but no PR_END after reset.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )
    if not proto_rx_ok:
        pytest.fail(
            "VERDICT: STALL_NO_PROTO_RX_OK — full PR_BEG…PR_END seen "
            "post-reset, but CC never emitted NOTICE;PROTO_RX=OK.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )
    if new_steps <= 0 or new_loops <= 0:
        pytest.fail(
            f"VERDICT: EMPTY_PROTOCOL — reload reported STEPS={new_steps} "
            f"LOOPS={new_loops}.\n  Transcript:\n  {_dump(snap)}"
        )

    rx = proto_rx_ok[0]
    if isinstance(rx, GenericFrame):
        rx_phash = rx.fields.get("PHASH", "")
        if new_phash and rx_phash and rx_phash != new_phash:
            pytest.fail(
                f"VERDICT: PHASH_MISMATCH — XPB sent PHASH={new_phash} "
                f"but CC ack'd PHASH={rx_phash}.\n"
                f"  Transcript:\n  {_dump(snap)}"
            )

    if not idle_hbs:
        pytest.fail(
            f"VERDICT: STALL_NO_IDLE — reset+reload completed but CC "
            f"never reached STATE=IDLE in {POST_PULSE_CAPTURE_S:.0f}s. "
            f"Last HB STATE={hbs[-1].state}.\n  Transcript:\n  {_dump(snap)}"
        )

    last = idle_hbs[-1]
    _log.info(
        "VERDICT: PASS — Teensy RST pulse drove full reload of %r "
        "(STEPS=%d LOOPS=%d PHASH=%s); CC reached IDLE. Last HB "
        "STATE=%s STEP=%d LOOP=%d/%d", new_name, new_steps, new_loops,
        new_phash, last.state, last.step, last.loop_idx, last.loop_total,
    )
