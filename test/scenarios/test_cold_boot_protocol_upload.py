"""S4b: Operator-triggered reset capture.

Captures the boot transcript of the CC↔XPB protocol-upload handshake.
The operator resets both boards (CC barrel jack reseat + Nano RST
button — or just both RST buttons), and the test auto-detects the boot
on the wire and grabs the next ``CAPTURE_S`` of traffic.

Sequence:
    1. Listener starts.
    2. Test prints a single prompt: "RESET both boards now, then press
       Enter (you can press Enter first, then reset)."
    3. After Enter, the test waits up to ``BOOT_TIMEOUT_S`` for the
       first post-prompt frame from either CC or XPB. That frame
       anchors the capture window (t0 = first boot frame).
    4. Captures ``CAPTURE_S`` of UDP from that anchor and renders a
       verdict, classifying which step (if any) of the handshake
       broke.

Verdict ladder:
    PASS                 — full chain seen, PR_BEG reports a non-empty
                            protocol, PHASH echo matches, CC reaches
                            STATE=IDLE within the capture window.
    NO_BOOT              — nothing on the wire after the prompt.
    NO_CC_HB             — XPB alive, CC silent.
    STALL_NO_REQ         — CC alive but never sent REQ:PROTO.
    STALL_NO_PR_BEG      — CC asked, XPB never replied (XPB wedged
                            mid-init: SD or W5500 hang).
    STALL_MID_UPLOAD     — got PR_BEG and some PR_DAT but no PR_END.
    STALL_NO_PROTO_RX_OK — full upload, but CC never sent PROTO_RX=OK.
    PHASH_MISMATCH       — CC ack'd a different PHASH than XPB sent.
    EMPTY_PROTOCOL       — PR_BEG reports STEPS=0 or LOOPS=0.
    STALL_NO_IDLE        — handshake done but CC never reached IDLE.
"""

from __future__ import annotations

import logging
import sys
import time
from collections import Counter
from typing import Iterable

import pytest

from test.rig.monitor import Monitor
from test.rig.parser import Frame, GenericFrame, HbFrame


_log = logging.getLogger("rig.proto_upload.cold_boot")


CAPTURE_S = 30.0          # transcript window after first boot frame
BOOT_TIMEOUT_S = 20.0     # how long to wait for the first post-Enter frame
POLL_S = 0.1
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


def _operator_prompt(message: str) -> None:
    banner = f"\n{'=' * 60}\n>>> {message}\n{'=' * 60}\n"
    sys.stderr.write(banner)
    sys.stderr.flush()
    try:
        input()
    except EOFError:
        pytest.skip("stdin closed — interactive test cannot proceed "
                    "(use `pytest -s` to enable input)")


@pytest.mark.manual
@pytest.mark.live_rig
@pytest.mark.slow
def test_cold_boot_protocol_upload(monitor: Monitor) -> None:
    """Operator-triggered reset. Auto-detects boot, captures handshake."""
    _operator_prompt(
        "RESET both boards now (CC barrel-jack reseat OR power-cycle, "
        "and Nano RST button), then press Enter. Order does not matter."
    )

    # Drain anything that flew before/during the prompt.
    monitor.clear()
    arm_t = monitor.elapsed_ms

    # Wait for the first post-Enter frame to anchor the capture window.
    _log.info("Armed at t=%.1fms — waiting up to %.1fs for first boot frame",
              arm_t, BOOT_TIMEOUT_S)
    deadline = time.monotonic() + BOOT_TIMEOUT_S
    anchor_t: float | None = None
    while time.monotonic() < deadline:
        snap = monitor.snapshot(since_ms=arm_t)
        if snap:
            anchor_t = snap[0].t_ms
            _log.info(
                "First post-reset frame at t=%.1fms from %s: %s",
                anchor_t, snap[0].src_ip, _fmt_frame(snap[0]),
            )
            break
        time.sleep(POLL_S)

    if anchor_t is None:
        pytest.fail(
            f"VERDICT: NO_BOOT — no traffic from either board within "
            f"{BOOT_TIMEOUT_S:.0f}s after Enter. Did the reset actually "
            "happen? Check power, Ethernet link LEDs, and that you "
            "pressed Enter AFTER cutting power (not before)."
        )

    # Capture the next CAPTURE_S from the anchor.
    _log.info("Capturing %.1fs from anchor", CAPTURE_S)
    elapsed_since_anchor = (monitor.elapsed_ms - anchor_t) / 1000.0
    remaining = max(0.0, CAPTURE_S - elapsed_since_anchor)
    if remaining > 0:
        time.sleep(remaining)

    snap = monitor.snapshot(since_ms=anchor_t)
    cc = [f for f in snap if f.src_ip == CC_IP]
    xpb = [f for f in snap if f.src_ip == XPB_IP]

    _log.info("CC  (%s) frames=%d kinds=%s",
              CC_IP, len(cc), dict(Counter(f.kind for f in cc)))
    _log.info("XPB (%s) frames=%d kinds=%s",
              XPB_IP, len(xpb), dict(Counter(f.kind for f in xpb)))
    _log.info("Full transcript:\n  %s", _dump(snap))

    # --- Mine the handshake markers -------------------------------------

    req_proto = [
        f for f in cc
        if f.kind in ("REQ:PROTO", "REQ")
        or (f.kind == "CMD" and isinstance(f, GenericFrame)
            and any("PROTO" in k or "PROTO" in v for k, v in f.fields.items()))
    ]
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

    # Pull NAME / STEPS / LOOPS / PHASH out of the first PR_BEG so the log
    # tells us *what* protocol got loaded, not just that one did.
    proto_name = ""
    proto_steps = 0
    proto_loops = 0
    proto_phash = ""
    if pr_beg and isinstance(pr_beg[0], GenericFrame):
        f0 = pr_beg[0]
        proto_name = f0.fields.get("NAME", "")
        try:
            proto_steps = int(f0.fields.get("STEPS", "0"))
        except ValueError:
            proto_steps = 0
        try:
            proto_loops = int(f0.fields.get("LOOPS", "0"))
        except ValueError:
            proto_loops = 0
        proto_phash = f0.fields.get("PHASH", "")

    _log.info("HANDSHAKE MARKERS:")
    _log.info("  REQ:PROTO from CC  : %d", len(req_proto))
    _log.info("  PR_BEG    from XPB : %d  (NAME=%r STEPS=%d LOOPS=%d PHASH=%s)",
              len(pr_beg), proto_name, proto_steps, proto_loops, proto_phash)
    _log.info("  PR_DAT    from XPB : %d", len(pr_dat))
    _log.info("  PR_END    from XPB : %d", len(pr_end))
    _log.info("  PROTO_RX=OK from CC: %d", len(proto_rx_ok))
    _log.info("  HBs total / IDLE   : %d / %d", len(hbs), len(idle_hbs))

    # --- Verdict ladder -------------------------------------------------
    if not hbs and not xpb:
        pytest.fail(
            f"VERDICT: NO_BOOT — anchor frame seen but no follow-up "
            f"traffic in {CAPTURE_S:.0f}s.\n  Transcript:\n  {_dump(snap)}"
        )

    if not hbs:
        pytest.fail(
            f"VERDICT: NO_CC_HB — XPB={len(xpb)} frames, but CC sent "
            f"zero HBs in {CAPTURE_S:.0f}s.\n  Transcript:\n  {_dump(snap)}"
        )

    if not req_proto:
        pytest.fail(
            "VERDICT: STALL_NO_REQ — CC is alive (HBs flowing) but "
            "never sent REQ:PROTO during the boot window.\n"
            f"  CC kinds: {dict(Counter(f.kind for f in cc))}\n"
            f"  Transcript:\n  {_dump(snap)}"
        )

    if not pr_beg:
        pytest.fail(
            f"VERDICT: STALL_NO_PR_BEG — CC sent {len(req_proto)} "
            "REQ:PROTO but XPB never replied with PR_BEG. XPB is wedged "
            "in early init (SD or W5500 hang); LCD usually shows '||' "
            "garbage in this case.\n"
            f"  XPB kinds: {dict(Counter(f.kind for f in xpb))}\n"
            f"  Transcript:\n  {_dump(snap)}"
        )

    if not pr_end:
        pytest.fail(
            f"VERDICT: STALL_MID_UPLOAD — got PR_BEG and {len(pr_dat)} "
            "PR_DAT but no PR_END. Upload aborted partway through.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )

    if not proto_rx_ok:
        pytest.fail(
            "VERDICT: STALL_NO_PROTO_RX_OK — full PR_BEG…PR_END seen, "
            "but CC never emitted NOTICE;PROTO_RX=OK. CC is parked in "
            "PROTO_LOADING.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )

    # PHASH echo check: CC's PROTO_RX=OK should carry the same PHASH that
    # XPB advertised in PR_BEG. Any mismatch means CC accepted a different
    # protocol than XPB sent — silent corruption.
    rx = proto_rx_ok[0]
    if isinstance(rx, GenericFrame):
        rx_phash = rx.fields.get("PHASH", "")
        if proto_phash and rx_phash and rx_phash != proto_phash:
            pytest.fail(
                f"VERDICT: PHASH_MISMATCH — XPB sent PHASH={proto_phash} "
                f"but CC acknowledged PHASH={rx_phash}.\n"
                f"  Transcript:\n  {_dump(snap)}"
            )

    # Sanity: the protocol that loaded should actually have content. A
    # zero-step / zero-loop CSV would also produce a successful handshake
    # but is operationally useless.
    if proto_steps <= 0 or proto_loops <= 0:
        pytest.fail(
            f"VERDICT: EMPTY_PROTOCOL — handshake completed but PR_BEG "
            f"reported STEPS={proto_steps} LOOPS={proto_loops}. The CSV "
            "on the SD card is empty or unparseable.\n"
            f"  Transcript:\n  {_dump(snap)}"
        )

    # CC is expected to settle into IDLE within the capture window once
    # the protocol is loaded (it stays in BOOTING until the upload-done
    # path runs). No IDLE HB means CC is wedged in PROTO_LOADING/BOOTING.
    if not idle_hbs:
        pytest.fail(
            f"VERDICT: STALL_NO_IDLE — handshake completed but CC never "
            f"reached STATE=IDLE in {CAPTURE_S:.0f}s. Last HB STATE="
            f"{hbs[-1].state}.\n  Transcript:\n  {_dump(snap)}"
        )

    last = idle_hbs[-1]
    _log.info(
        "VERDICT: PASS — protocol %r loaded (STEPS=%d LOOPS=%d "
        "PHASH=%s); CC reached IDLE. Last HB STATE=%s STEP=%d LOOP=%d/%d",
        proto_name, proto_steps, proto_loops, proto_phash,
        last.state, last.step, last.loop_idx, last.loop_total,
    )
