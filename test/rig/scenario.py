"""Shared live-rig scenario helpers.

These helpers keep pytest scenarios self-normalizing without hiding the
observable contract: all verdicts still come from UDP frames and Teensy
RUN/RST actions.
"""

from __future__ import annotations

import logging
import sys
import time
from collections import Counter
from dataclasses import dataclass
from typing import Iterable, List, Mapping, Sequence

import pytest

from test.rig.monitor import Monitor
from test.rig.parser import Frame, GenericFrame, HbFrame
from test.rig.teensy import Teensy


CC_IP = "10.0.0.10"
XPB_IP = "10.0.0.11"

RST_PULSE_MS = 6000
RESET_RELOAD_TIMEOUT_S = 25.0

HLFB_QUICK_NAME = "HLFB_QUICK"
HLFB_QUICK_STEPS = 12
HLFB_QUICK_LOOPS = 1
HLFB_QUICK_PHASH = "1052422810"


@dataclass(frozen=True)
class ProtocolInfo:
    name: str = ""
    steps: int = 0
    loops: int = 0
    phash: str = ""


@dataclass(frozen=True)
class ReloadResult:
    snap: List[Frame]
    cc: List[Frame]
    xpb: List[Frame]
    sw_rst: List[Frame]
    cmd_arm: List[Frame]
    cmd_exec: List[Frame]
    req_proto: List[Frame]
    pr_beg: List[Frame]
    pr_dat: List[Frame]
    pr_end: List[Frame]
    proto_rx_ok: List[Frame]
    hbs: List[HbFrame]
    idle_hbs: List[HbFrame]
    proto: ProtocolInfo

    @property
    def counts(self) -> Mapping[str, int]:
        return {
            "SW;RST=1": len(self.sw_rst),
            "CMD;RESET=ARM": len(self.cmd_arm),
            "CMD;RESET=EXEC": len(self.cmd_exec),
            "REQ:PROTO": len(self.req_proto),
            "PR_BEG": len(self.pr_beg),
            "PR_DAT": len(self.pr_dat),
            "PR_END": len(self.pr_end),
            "PROTO_RX=OK": len(self.proto_rx_ok),
            "IDLE HBs": len(self.idle_hbs),
        }


def fmt_frame(frame: Frame) -> str:
    if isinstance(frame, HbFrame):
        return (
            f"{frame.t_ms:8.1f}ms {frame.src_ip:>12s} HB SEQ={frame.seq} "
            f"STATE={frame.state} STEP={frame.step} "
            f"LOOP={frame.loop_idx}/{frame.loop_total} RPM={frame.rpm} "
            f"E={int(frame.e_flag)}"
        )
    if isinstance(frame, GenericFrame):
        kv = ";".join(f"{k}={v}" for k, v in frame.fields.items())
        return f"{frame.t_ms:8.1f}ms {frame.src_ip:>12s} {frame.kind};{kv}"
    return f"{frame.t_ms:8.1f}ms {frame.src_ip:>12s} {frame.kind}"


def dump_frames(frames: Iterable[Frame]) -> str:
    rows = [fmt_frame(frame) for frame in frames]
    return "\n  ".join(rows) if rows else "(none)"


def operator_prompt(message: str) -> None:
    banner = f"\n{'=' * 60}\n>>> {message}\n{'=' * 60}\n"
    sys.stderr.write(banner)
    sys.stderr.flush()
    try:
        input()
    except EOFError:
        pytest.fail(
            "manual scenario requires interactive stdin; rerun with '-s'"
        )


def operator_confirm(message: str, token: str = "READY") -> None:
    """Pause until operator types the expected confirmation token."""

    expected = token.strip()
    if not expected:
        raise ValueError("confirmation token must be non-empty")

    banner = (
        f"\n{'=' * 60}\n>>> {message}\n"
        f">>> Type {expected!r} then press Enter to continue.\n"
        f"{'=' * 60}\n"
    )
    while True:
        sys.stderr.write(banner)
        sys.stderr.flush()
        try:
            reply = input().strip()
        except EOFError:
            pytest.fail(
                "manual scenario requires interactive stdin; rerun with '-s'"
            )
        if reply == expected:
            return
        sys.stderr.write(
            f">>> Waiting for exact token {expected!r}; got {reply!r}.\n"
        )
        sys.stderr.flush()


def wait_for_state(
    monitor: Monitor,
    since_ms: float,
    want_state: str,
    timeout_s: float,
    *,
    src_ip: str = CC_IP,
    poll_s: float = 0.25,
) -> List[HbFrame]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        hits = [
            frame for frame in monitor.snapshot(since_ms=since_ms, src_ip=src_ip)
            if isinstance(frame, HbFrame) and frame.state == want_state
        ]
        if hits:
            return hits
        time.sleep(poll_s)
    return []


def park_run(teensy: Teensy, *, settle_s: float = 0.5) -> None:
    teensy.set_run(0)
    if settle_s > 0:
        time.sleep(settle_s)


def split_by_endpoint(frames: Sequence[Frame]) -> tuple[List[Frame], List[Frame]]:
    cc = [frame for frame in frames if frame.src_ip == CC_IP]
    xpb = [frame for frame in frames if frame.src_ip == XPB_IP]
    return cc, xpb


def summarize_kinds(frames: Sequence[Frame]) -> dict[str, int]:
    return dict(Counter(frame.kind for frame in frames))


def classify_reload(frames: Sequence[Frame]) -> ReloadResult:
    snap = list(frames)
    cc, xpb = split_by_endpoint(snap)
    sw_rst = [
        frame for frame in xpb
        if isinstance(frame, GenericFrame) and frame.kind == "SW"
        and frame.fields.get("RST") == "1"
    ]
    cmd_arm = [
        frame for frame in cc
        if isinstance(frame, GenericFrame) and frame.kind == "CMD"
        and frame.fields.get("RESET") == "ARM"
    ]
    cmd_exec = [
        frame for frame in cc
        if isinstance(frame, GenericFrame) and frame.kind == "CMD"
        and frame.fields.get("RESET") == "EXEC"
    ]
    req_proto = [frame for frame in cc if frame.kind in ("REQ:PROTO", "REQ")]
    pr_beg = [frame for frame in xpb if frame.kind == "PR_BEG"]
    pr_dat = [frame for frame in xpb if frame.kind == "PR_DAT"]
    pr_end = [frame for frame in xpb if frame.kind == "PR_END"]
    proto_rx_ok = [
        frame for frame in cc
        if isinstance(frame, GenericFrame) and frame.kind == "NOTICE"
        and frame.fields.get("PROTO_RX") == "OK"
    ]
    hbs = [frame for frame in cc if isinstance(frame, HbFrame)]
    idle_hbs = [frame for frame in hbs if frame.state == "IDLE"]
    proto = protocol_from_pr_beg(pr_beg[0] if pr_beg else None)
    return ReloadResult(
        snap=snap,
        cc=cc,
        xpb=xpb,
        sw_rst=sw_rst,
        cmd_arm=cmd_arm,
        cmd_exec=cmd_exec,
        req_proto=req_proto,
        pr_beg=pr_beg,
        pr_dat=pr_dat,
        pr_end=pr_end,
        proto_rx_ok=proto_rx_ok,
        hbs=hbs,
        idle_hbs=idle_hbs,
        proto=proto,
    )


def protocol_from_pr_beg(frame: Frame | None) -> ProtocolInfo:
    if not isinstance(frame, GenericFrame):
        return ProtocolInfo()
    return ProtocolInfo(
        name=frame.fields.get("NAME", ""),
        steps=_field_int(frame.fields, "STEPS"),
        loops=_field_int(frame.fields, "LOOPS"),
        phash=frame.fields.get("PHASH", ""),
    )


def _field_int(fields: Mapping[str, str], key: str) -> int:
    try:
        return int(fields.get(key, "0"))
    except ValueError:
        return 0


def reload_verdict(result: ReloadResult) -> str:
    if not result.sw_rst:
        return "NO_SW_RST"
    if not result.cmd_arm and not result.cmd_exec:
        return "STALL_NO_RESET_CMD"
    if not result.cmd_exec:
        return "STALL_NO_EXEC"
    if not result.req_proto:
        return "STALL_NO_REREQ"
    if not result.pr_beg:
        return "STALL_NO_PR_BEG"
    if not result.pr_end:
        return "STALL_NO_PR_END"
    if not result.proto_rx_ok:
        return "STALL_NO_PROTO_RX_OK"
    if result.proto.steps <= 0 or result.proto.loops <= 0:
        return "EMPTY_PROTOCOL"
    if _proto_rx_phash(result) not in ("", result.proto.phash):
        return "PHASH_MISMATCH"
    if not result.idle_hbs:
        return "STALL_NO_IDLE"
    return "PASS"


def _proto_rx_phash(result: ReloadResult) -> str:
    rx = result.proto_rx_ok[0] if result.proto_rx_ok else None
    if isinstance(rx, GenericFrame):
        return rx.fields.get("PHASH", "")
    return ""


def require_reload_pass(result: ReloadResult, *, context: str = "reload") -> None:
    verdict = reload_verdict(result)
    if verdict == "PASS":
        return
    last_hb = fmt_frame(result.hbs[-1]) if result.hbs else "(none)"
    raise AssertionError(
        f"VERDICT: {verdict} — {context} did not complete cleanly. "
        f"Markers={dict(result.counts)} Last HB={last_hb}\n"
        f"  Transcript:\n  {dump_frames(result.snap)}"
    )


def expect_hlfb_quick(result: ReloadResult) -> None:
    proto = result.proto
    mismatches: list[str] = []
    if proto.name != HLFB_QUICK_NAME:
        mismatches.append(f"NAME={proto.name!r}")
    if proto.steps != HLFB_QUICK_STEPS:
        mismatches.append(f"STEPS={proto.steps}")
    if proto.loops != HLFB_QUICK_LOOPS:
        mismatches.append(f"LOOPS={proto.loops}")
    if proto.phash and proto.phash != HLFB_QUICK_PHASH:
        mismatches.append(f"PHASH={proto.phash}")
    if mismatches:
        expected = (
            f"NAME={HLFB_QUICK_NAME!r} STEPS={HLFB_QUICK_STEPS} "
            f"LOOPS={HLFB_QUICK_LOOPS} PHASH={HLFB_QUICK_PHASH}"
        )
        raise AssertionError(
            "Loaded protocol is not the automated HLFB_QUICK baseline: "
            f"{', '.join(mismatches)}; expected {expected}."
        )


def reset_to_idle(
    monitor: Monitor,
    teensy: Teensy,
    logger: logging.Logger,
    *,
    timeout_s: float = RESET_RELOAD_TIMEOUT_S,
    require_hlfb_quick: bool = False,
) -> ReloadResult:
    park_run(teensy)
    monitor.clear()
    reset_t = monitor.elapsed_ms
    logger.info("Pulsing RST for %d ms to normalize to IDLE", RST_PULSE_MS)
    teensy.pulse_reset(RST_PULSE_MS)
    elapsed = (monitor.elapsed_ms - reset_t) / 1000.0
    remaining = max(0.0, timeout_s - elapsed)
    if remaining > 0:
        logger.info("Waiting %.1fs for reset reload + IDLE", remaining)
        time.sleep(remaining)
    result = classify_reload(monitor.snapshot(since_ms=reset_t))
    log_reload_summary(logger, result)
    require_reload_pass(result, context="reset-to-IDLE normalization")
    if require_hlfb_quick:
        expect_hlfb_quick(result)
    last = result.idle_hbs[-1]
    logger.info(
        "Normalization OK: CC IDLE after loading %r STEPS=%d LOOPS=%d PHASH=%s",
        result.proto.name, result.proto.steps, result.proto.loops,
        result.proto.phash,
    )
    logger.debug("Last IDLE HB: %s", fmt_frame(last))
    return result


def log_reload_summary(logger: logging.Logger, result: ReloadResult) -> None:
    logger.info("CC  (%s) frames=%d kinds=%s", CC_IP, len(result.cc),
                summarize_kinds(result.cc))
    logger.info("XPB (%s) frames=%d kinds=%s", XPB_IP, len(result.xpb),
                summarize_kinds(result.xpb))
    logger.info("RESET CHAIN MARKERS: %s", dict(result.counts))
    logger.info(
        "Reload protocol: NAME=%r STEPS=%d LOOPS=%d PHASH=%s",
        result.proto.name, result.proto.steps, result.proto.loops,
        result.proto.phash,
    )


def stable_hbs(
    monitor: Monitor,
    since_ms: float,
    *,
    states: set[str] | None = None,
) -> List[HbFrame]:
    hbs = [
        frame for frame in monitor.snapshot(since_ms=since_ms, src_ip=CC_IP)
        if isinstance(frame, HbFrame)
    ]
    if states is None:
        return hbs
    return [frame for frame in hbs if frame.state in states]
