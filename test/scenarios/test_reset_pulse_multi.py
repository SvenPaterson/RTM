"""S6: N back-to-back logical RST pulses must each fully reload.

Stress version of ``test_reset_reloads_protocol`` — instead of a
single 6 s pulse, drive ``COUNT`` consecutive supra-threshold pulses
and assert each one drives a full reload chain ending in
``NOTICE;PROTO_RX=OK`` and CC HB STATE=IDLE.

Per-iteration verdict ladder mirrors the single-pulse test; we collect
all failures and report the first one with full transcript context.
"""

from __future__ import annotations

import logging
import time
from collections import Counter
from typing import Iterable, List, Tuple

import pytest

from test.rig.monitor import Monitor
from test.rig.parser import Frame, GenericFrame, HbFrame
from test.rig.scenario import classify_reload, reload_verdict
from test.rig.teensy import Teensy


_log = logging.getLogger("rig.reset_pulse_multi")


CC_IP = "10.0.0.10"
XPB_IP = "10.0.0.11"

COUNT = 3
RST_PULSE_MS = 6000
POST_PULSE_CAPTURE_S = 25.0
INTER_ITER_SETTLE_S = 3.0
PRE_OBSERVE_S = 3.0


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


def _classify_iteration(
    snap: List[Frame],
) -> Tuple[str, dict]:
    """Return (verdict, marker_counts) for a single iteration's snapshot."""
    result = classify_reload(snap)
    return reload_verdict(result), dict(result.counts)


@pytest.mark.live_rig
@pytest.mark.slow
@pytest.mark.full
@pytest.mark.stateful
def test_back_to_back_rst_pulses_each_reload(
    monitor: Monitor, teensy: Teensy
) -> None:
    """COUNT consecutive supra-threshold RST pulses; all must reload."""

    teensy.set_run(0)
    time.sleep(0.5)
    monitor.clear()
    base_t = monitor.elapsed_ms
    _log.info("Pre-iter baseline: %.1fs", PRE_OBSERVE_S)
    time.sleep(PRE_OBSERVE_S)
    base_hbs = [
        f for f in monitor.snapshot(since_ms=base_t, src_ip=CC_IP)
        if isinstance(f, HbFrame)
    ]
    if not base_hbs:
        pytest.skip(
            f"Baseline saw no HBs from CC at {CC_IP}. Boot the rig first."
        )

    failures: List[Tuple[int, str, dict]] = []
    full_log = []

    for i in range(1, COUNT + 1):
        _log.info("=" * 60)
        _log.info("Iteration %d/%d: pulsing RST for %d ms",
                  i, COUNT, RST_PULSE_MS)
        monitor.clear()
        iter_t = monitor.elapsed_ms
        teensy.pulse_reset(RST_PULSE_MS)
        # pulse_reset() blocks for the pulse width; capture the rest.
        elapsed = (monitor.elapsed_ms - iter_t) / 1000.0
        remaining = max(0.0, POST_PULSE_CAPTURE_S - elapsed)
        if remaining > 0:
            time.sleep(remaining)

        snap = monitor.snapshot(since_ms=iter_t)
        verdict, counts = _classify_iteration(snap)
        cc = [f for f in snap if f.src_ip == CC_IP]
        xpb = [f for f in snap if f.src_ip == XPB_IP]
        _log.info("Iter %d markers: %s", i, counts)
        _log.info("Iter %d CC kinds : %s",
                  i, dict(Counter(f.kind for f in cc)))
        _log.info("Iter %d XPB kinds: %s",
                  i, dict(Counter(f.kind for f in xpb)))
        _log.info("Iter %d verdict : %s", i, verdict)

        if verdict != "PASS":
            failures.append((i, verdict, counts))
            full_log.append((i, snap))

        # Settle before next iteration so HB STATE=IDLE is rock-solid.
        time.sleep(INTER_ITER_SETTLE_S)

    _log.info("=" * 60)
    _log.info("Suite summary: %d/%d iterations PASS",
              COUNT - len(failures), COUNT)

    if failures:
        first_iter, first_verdict, first_counts = failures[0]
        snap_for_first = next(s for (i, s) in full_log if i == first_iter)
        pytest.fail(
            f"VERDICT: {first_verdict} on iter {first_iter}/{COUNT}. "
            f"Markers={first_counts}. {len(failures)} of {COUNT} "
            f"iterations failed.\n"
            f"  First-failure transcript:\n  {_dump(snap_for_first)}"
        )

    _log.info(
        "VERDICT: PASS — %d/%d back-to-back RST pulses each drove a "
        "full reload chain.", COUNT, COUNT,
    )
