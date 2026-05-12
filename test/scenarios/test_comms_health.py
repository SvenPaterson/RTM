"""S1: Baseline comms health.

Verifies that with both boards in IDLE on a healthy link the harness
sees:
  * HB frames from CC (10.0.0.10) at ~250 ms cadence
  * STAT frames from XPB (10.0.0.11) at ~1000 ms cadence
  * SEQ monotonically increasing on both streams
  * E=0 for every HB (no E-STOP)
  * Zero parse / checksum errors

This is the first executable spec for the new harness — if this
fails, no other scenario is meaningful.

Replaces the legacy `rig_control.py comms-health` (T8) for the UDP
transport. The legacy test still applies for the TTL physical layer
on rigs that haven't cut over.
"""

from __future__ import annotations

import time

import pytest

from test.rig.assertions import (
    HB_NOMINAL_MS,
    STAT_NOMINAL_MS,
    expect_cadence,
    expect_no_estop,
    expect_no_parse_errors,
    expect_seq_monotonic,
)
from test.rig.monitor import Monitor


# Source IPs — single source of truth here is `include/RtmNet.h`. If
# you change the rig addressing there, mirror it.
CC_IP = "10.0.0.10"
XPB_IP = "10.0.0.11"

# Capture window. Long enough to see ~40 HB and ~10 STAT samples,
# short enough to keep the suite fast.
CAPTURE_S = 10.0

# Tolerances. Chosen well above observed jitter (<1 ms in the
# reference capture) but tight enough to catch a 1-frame stall.
# A single missed UDP datagram would show as ~2× nominal interval.
HB_TOL_MS = 100.0       # 250 ± 100 ms — flags any drop > 1 frame
STAT_TOL_MS = 200.0     # 1000 ± 200 ms — same logic at 1 Hz


@pytest.mark.live_rig
def test_baseline_link_healthy(monitor: Monitor) -> None:
    """Capture for CAPTURE_S, then assert the wire shape is nominal."""
    time.sleep(CAPTURE_S)

    frames = monitor.snapshot()
    assert frames, (
        f"no frames received in {CAPTURE_S:.0f}s — check that both boards "
        f"are powered, on the rig LAN, and the PC NIC is on 10.0.0.100/24"
    )

    # No corrupted/garbled frames at all.
    expect_no_parse_errors(monitor)

    # CC heartbeat: source-tagged + cadence + monotonic SEQ + no E-STOP.
    cc_hbs = monitor.snapshot(kind="HB", src_ip=CC_IP)
    assert cc_hbs, f"no HB frames from CC ({CC_IP})"
    expect_seq_monotonic(cc_hbs, kind="HB")
    expect_cadence(cc_hbs, kind="HB",
                   nominal_ms=HB_NOMINAL_MS, tolerance_ms=HB_TOL_MS)
    expect_no_estop(cc_hbs)

    # XPB status: same shape at 1 Hz.
    xpb_stats = monitor.snapshot(kind="STAT", src_ip=XPB_IP)
    assert xpb_stats, f"no STAT frames from XPB ({XPB_IP})"
    expect_seq_monotonic(xpb_stats, kind="STAT")
    expect_cadence(xpb_stats, kind="STAT",
                   nominal_ms=STAT_NOMINAL_MS, tolerance_ms=STAT_TOL_MS)
