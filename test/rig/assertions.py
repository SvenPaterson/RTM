"""Test assertions for the RTM rig harness.

These helpers operate on `Monitor` snapshots (or any `Frame` list)
and raise `AssertionError` with a clear, single-line failure message
suitable for pytest output.

Two flavors:
  * `wait_for(...)` polls a `Monitor` until a predicate matches or a
    timeout elapses. Use for "the next frame that does X" semantics.
  * `expect_*` validates an entire pre-collected list. Use after a
    fixed-duration capture window.

All cadence checks are interval-based (max gap between consecutive
matching frames), not count-based — counts are sensitive to capture
window placement, intervals are not.
"""

from __future__ import annotations

import logging
import time
from typing import Callable, Iterable, List, Optional

from .monitor import Monitor
from .parser import Frame, HbFrame, StatFrame


_log = logging.getLogger("rig.assert")


# Nominal cadences. Single source of truth for the harness.
HB_NOMINAL_MS = 250.0
STAT_NOMINAL_MS = 1000.0


def wait_for(
    mon: Monitor,
    predicate: Callable[[Frame], bool],
    *,
    timeout_s: float,
    poll_s: float = 0.05,
    since_ms: Optional[float] = None,
) -> Frame:
    """Block until `predicate` matches a buffered frame or the
    timeout elapses. Returns the matching frame.

    `since_ms` defaults to the monitor's current `elapsed_ms` at
    call time, so the helper finds frames that arrive *after* the
    call — pass `0.0` to also accept earlier frames.
    """
    if since_ms is None:
        since_ms = mon.elapsed_ms
    deadline = time.monotonic() + timeout_s
    while True:
        for f in mon.snapshot(since_ms=since_ms):
            if predicate(f):
                _log.info("wait_for matched at t=%.1fms: %s",
                          f.t_ms, _summarize_frame(f))
                return f
        if time.monotonic() >= deadline:
            # Dump recent HB context so the failure tells us *what*
            # the wire said, not just "nothing matched". 8 frames at
            # 250 ms HB cadence covers ~2 s — enough to diagnose
            # state-machine behavior without flooding the log.
            recent = mon.snapshot(since_ms=since_ms)[-8:]
            tail = "\n  ".join(_summarize_frame(f) for f in recent) or "(no frames)"
            _log.error("wait_for TIMEOUT after %.2fs (frames=%d):\n  %s",
                       timeout_s, mon.frame_count, tail)
            raise AssertionError(
                f"wait_for: predicate not satisfied within {timeout_s:.2f}s "
                f"(since_ms={since_ms:.0f}, frames seen={mon.frame_count})"
                f"\n  recent frames:\n  {tail}"
            )
        time.sleep(poll_s)


def _summarize_frame(f: Frame) -> str:
    """One-line frame summary for failure diagnostics."""
    base = f"[{f.t_ms:8.1f}ms] {f.src_ip:>11s}"
    if isinstance(f, HbFrame):
        return (f"{base} HB SEQ={f.seq} STATE={f.state} STEP={f.step} "
                f"LOOP={f.loop_idx}/{f.loop_total} RPM={f.rpm} "
                f"E={int(f.e_flag)} E_CODE={f.e_code}")
    if isinstance(f, StatFrame):
        return f"{base} STAT SEQ={f.seq} OUT={f.out} SUMP={f.sump_c} SEAL={f.seal_c}"
    return f"{base} {f.kind} {getattr(f, 'fields', {})}"


def expect_seq_monotonic(frames: Iterable[Frame], *, kind: str) -> None:
    """Every `kind` frame's SEQ must be strictly greater than the prior.

    SEQ wrap is not handled here — at HB 250ms cadence with a 16-bit
    SEQ that's >4 hours of continuous run; flag wrap as a real defect
    in long-running tests rather than silently accommodating it.
    """
    last: Optional[int] = None
    last_t: float = 0.0
    for f in frames:
        if f.kind != kind:
            continue
        seq = getattr(f, "seq", None)
        if seq is None:
            raise AssertionError(f"{kind} frame has no SEQ field: {f}")
        if last is not None and seq <= last:
            raise AssertionError(
                f"{kind} SEQ not monotonic: prev={last}@{last_t:.1f}ms "
                f"now={seq}@{f.t_ms:.1f}ms"
            )
        last, last_t = seq, f.t_ms


def expect_no_seq_gaps(frames: Iterable[Frame], *, kind: str) -> None:
    """Adjacent `kind` frames must have SEQ differing by exactly 1.

    Stricter than `expect_seq_monotonic` — use when the test is run
    on a known-good link and any drop is a regression.
    """
    last: Optional[int] = None
    last_t: float = 0.0
    for f in frames:
        if f.kind != kind:
            continue
        seq = getattr(f, "seq")
        if last is not None and seq != last + 1:
            raise AssertionError(
                f"{kind} SEQ gap: prev={last}@{last_t:.1f}ms "
                f"now={seq}@{f.t_ms:.1f}ms (expected {last + 1})"
            )
        last, last_t = seq, f.t_ms


def expect_cadence(
    frames: Iterable[Frame],
    *,
    kind: str,
    nominal_ms: float,
    tolerance_ms: float,
    min_count: int = 2,
) -> None:
    """Every interval between consecutive `kind` frames must be
    within ±`tolerance_ms` of `nominal_ms`. Requires at least
    `min_count` matching frames.
    """
    matches = [f for f in frames if f.kind == kind]
    if len(matches) < min_count:
        raise AssertionError(
            f"{kind} cadence: only {len(matches)} frames seen "
            f"(need >= {min_count})"
        )
    lo, hi = nominal_ms - tolerance_ms, nominal_ms + tolerance_ms
    for prev, cur in zip(matches, matches[1:]):
        gap = cur.t_ms - prev.t_ms
        if not (lo <= gap <= hi):
            raise AssertionError(
                f"{kind} cadence: interval {gap:.1f}ms outside "
                f"[{lo:.1f}, {hi:.1f}] at t={cur.t_ms:.1f}ms "
                f"(SEQ {getattr(prev, 'seq', '?')} -> {getattr(cur, 'seq', '?')})"
            )


def expect_no_estop(frames: Iterable[HbFrame | Frame]) -> None:
    """No HB frame may carry E=1."""
    for f in frames:
        if isinstance(f, HbFrame) and f.e_flag:
            raise AssertionError(
                f"E-STOP asserted at t={f.t_ms:.1f}ms "
                f"(SEQ={f.seq}, E_CODE={f.e_code}, STATE={f.state})"
            )


def expect_no_parse_errors(mon: Monitor) -> None:
    """The monitor must have decoded every datagram cleanly."""
    errs = mon.parse_errors
    if errs:
        first = errs[0]
        raise AssertionError(
            f"{len(errs)} parse error(s); first at t={first.t_ms:.1f}ms "
            f"src={first.src_ip}: {first.error} payload={first.payload!r}"
        )


def states_seen(frames: Iterable[Frame]) -> List[str]:
    """Distinct STATE values from HB frames, in first-seen order."""
    seen: List[str] = []
    for f in frames:
        if isinstance(f, HbFrame) and f.state not in seen:
            seen.append(f.state)
    return seen
