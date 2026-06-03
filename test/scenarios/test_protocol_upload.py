"""S4: Protocol upload (XPB → CC) — passive diagnostic.

Bug-finder for "I've never seen REQ:PROTO from CC or PR_DAT from XPB".

Because the bench rig has no remote power-cut (the 24 V relay only kills
the motor/heater bus, not the controllers themselves), we cannot trigger
a fresh cold boot autonomously. So this test is *passive*: it listens
for ``OBSERVE_S`` seconds and renders a verdict based on what's on the
wire.

Three verdicts:

    * **PROTOCOL LOADED** — at least one HB shows STEP > 1 or
      LOOP_TOT > 1, or a ``NOTICE;PROTO_RX=OK`` was seen in the window.
        * **PROTOCOL ALREADY EXECUTED** — CC is already RUNNING, PAUSED, or
            COMPLETED when this passive test runs later in the suite.
    * **EMPTY WEDGE** — all HBs show ``STEP=1 LOOP=1/1`` and no
      protocol-upload frames flew. This is the "never loaded" failure
      mode the user reported.

If you want to capture a fresh boot transcript, manually power-cycle the
PSU within ``OBSERVE_S`` of test start.
"""

from __future__ import annotations

import logging
import time
from collections import Counter
from typing import Iterable

import pytest

from test.rig.monitor import Monitor
from test.rig.parser import Frame, GenericFrame, HbFrame


_log = logging.getLogger("rig.proto_upload")


OBSERVE_S = 8.0  # ≥ 1 STAT period + 30+ HBs; 8 s is plenty for a verdict
CC_IP = "10.0.0.10"
XPB_IP = "10.0.0.11"


def _summarize_frames(frames: Iterable[Frame], limit: int = 12) -> str:
    rows = list(frames)[-limit:]
    out: list[str] = []
    for f in rows:
        if isinstance(f, HbFrame):
            out.append(
                f"{f.t_ms:8.1f}ms {f.src_ip:>12s} HB SEQ={f.seq} "
                f"STATE={f.state} STEP={f.step} LOOP={f.loop_idx}/{f.loop_total} "
                f"RPM={f.rpm} E={int(f.e_flag)}"
            )
        elif isinstance(f, GenericFrame):
            kv = ";".join(f"{k}={v}" for k, v in f.fields.items())
            out.append(f"{f.t_ms:8.1f}ms {f.src_ip:>12s} {f.kind};{kv}")
        else:
            out.append(f"{f.t_ms:8.1f}ms {f.src_ip:>12s} {f.kind}")
    return "\n  ".join(out) if out else "(none)"


@pytest.mark.live_rig
@pytest.mark.full
def test_protocol_actually_loaded(monitor: Monitor) -> None:
    """Passive observation: is a real protocol actually loaded on CC?"""
    monitor.clear()
    t0 = monitor.elapsed_ms

    _log.info("Observing wire for %.1fs (passive)", OBSERVE_S)
    time.sleep(OBSERVE_S)

    snap = monitor.snapshot(since_ms=t0)
    cc_frames = [f for f in snap if f.src_ip == CC_IP]
    xpb_frames = [f for f in snap if f.src_ip == XPB_IP]

    cc_kinds = Counter(f.kind for f in cc_frames)
    xpb_kinds = Counter(f.kind for f in xpb_frames)

    _log.info("CC  (%s) frames=%d kinds=%s", CC_IP, len(cc_frames), dict(cc_kinds))
    _log.info("XPB (%s) frames=%d kinds=%s", XPB_IP, len(xpb_frames), dict(xpb_kinds))

    # --- Liveness gate: must see at least one HB from CC.
    hbs = [f for f in cc_frames if isinstance(f, HbFrame)]
    if not hbs:
        pytest.fail(
            f"No HBs from CC at {CC_IP} during {OBSERVE_S:.1f}s window. "
            f"CC is offline, Ethernet link is down, or CC firmware is wedged.\n"
            f"  All frames seen:\n  {_summarize_frames(snap, limit=20)}"
        )

    # --- Protocol-upload-related frames in the window.
    proto_frames = [
        f for f in snap
        if (
            f.kind.startswith("PR_")
            or (f.kind == "NOTICE" and isinstance(f, GenericFrame)
                and any(k.startswith("PROTO") for k in f.fields))
        )
    ]
    _log.info("Protocol-related frames in window: %d", len(proto_frames))
    for pf in proto_frames:
        _log.info("  %s", _summarize_frames([pf], limit=1))

    # --- Verdict from HB fingerprint.
    last_hb = hbs[-1]
    real_proto_hbs = [
        h for h in hbs if h.step > 1 or h.loop_total > 1 or h.loop_idx > 1
    ]
    execution_state_hbs = [
        h for h in hbs if h.state in ("RUNNING", "PAUSED", "COMPLETED")
    ]
    proto_rx_ok = [
        f for f in cc_frames
        if isinstance(f, GenericFrame)
        and f.kind == "NOTICE"
        and f.fields.get("PROTO_RX") == "OK"
    ]

    _log.info(
        "Last HB: STATE=%s STEP=%d LOOP=%d/%d RPM=%d E=%d E_CODE=%s",
        last_hb.state, last_hb.step, last_hb.loop_idx,
        last_hb.loop_total, last_hb.rpm, int(last_hb.e_flag),
        last_hb.e_code,
    )
    _log.info("HBs with STEP>1 or LOOP_TOT>1: %d / %d",
              len(real_proto_hbs), len(hbs))
    _log.info("HBs in execution/completion states: %d / %d",
              len(execution_state_hbs), len(hbs))
    _log.info("NOTICE;PROTO_RX=OK frames: %d", len(proto_rx_ok))

    if real_proto_hbs or proto_rx_ok:
        _log.info("VERDICT: PROTOCOL LOADED")
        return

    if execution_state_hbs:
        loaded_hb = execution_state_hbs[-1]
        _log.info(
            "VERDICT: PROTOCOL ALREADY EXECUTED — observed STATE=%s "
            "STEP=%d LOOP=%d/%d",
            loaded_hb.state, loaded_hb.step, loaded_hb.loop_idx,
            loaded_hb.loop_total,
        )
        return

    # --- Empty wedge. Build a sharp diagnostic.
    diagnosis = []
    if not proto_frames:
        diagnosis.append(
            "No PR_*/NOTICE;PROTO_* traffic in window — either no upload "
            "is being attempted, OR the upload happened before this "
            "observation window. To capture the boot transcript, "
            f"manually power-cycle the PSU and re-run within {OBSERVE_S:.0f}s."
        )
    else:
        diagnosis.append(
            "Upload-related frames flew but CC HBs still show the empty "
            "wedge (STEP=1 LOOP=1/1). Inspect the frames above to see "
            "where the chain broke (PR_BEG ack? PR_DAT seq? PR_END?)."
        )

    if last_hb.e_flag:
        diagnosis.append(
            f"CC is in E-STOP (E_CODE={last_hb.e_code}). The front-panel "
            f"RESET button cannot fire a logical reset from E-STOP — clear "
            f"E-STOP first (e.g. by power-cycling the PSU)."
        )

    pytest.fail(
        "VERDICT: EMPTY WEDGE — no real protocol loaded on CC.\n"
        f"  Last HB: STATE={last_hb.state} STEP={last_hb.step} "
        f"LOOP={last_hb.loop_idx}/{last_hb.loop_total} RPM={last_hb.rpm} "
        f"E={int(last_hb.e_flag)} E_CODE={last_hb.e_code}\n"
        f"  CC kinds:  {dict(cc_kinds)}\n"
        f"  XPB kinds: {dict(xpb_kinds)}\n"
        f"  Proto-related frames in window: {len(proto_frames)}\n"
        f"  Diagnosis: {' | '.join(diagnosis)}\n"
        f"  Last frames:\n  {_summarize_frames(snap, limit=20)}"
    )
