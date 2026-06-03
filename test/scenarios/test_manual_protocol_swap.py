"""Manual S11: SD protocol swap + reload verification.

Operator swaps the SD protocol, then the test drives a deterministic reset
and verifies that the newly loaded protocol identity is what was requested.

Run with:
    python -m pytest test/scenarios/test_manual_protocol_swap.py -v -s
"""

from __future__ import annotations

import logging

import pytest

from test.rig.monitor import Monitor
from test.rig.scenario import operator_confirm, reset_to_idle
from test.rig.teensy import Teensy


_log = logging.getLogger("rig.manual.protocol_swap")


@pytest.mark.manual
@pytest.mark.manual_protocol
@pytest.mark.live_rig
@pytest.mark.slow
@pytest.mark.stateful
def test_manual_protocol_swap_and_reload(
    monitor: Monitor, teensy: Teensy, request: pytest.FixtureRequest
) -> None:
    """Validate protocol identity after manual SD swap."""

    base_proto = str(request.config.getoption("--manual-base-protocol"))
    swap_proto = str(request.config.getoption("--manual-swap-protocol"))

    if not swap_proto:
        pytest.fail("--manual-swap-protocol must be a non-empty protocol name")

    # Baseline sanity: prove what is currently loaded before requesting swap.
    before = reset_to_idle(
        monitor,
        teensy,
        _log,
        require_hlfb_quick=(base_proto.upper() == "HLFB_QUICK"),
    )
    _log.info(
        "Baseline protocol: NAME=%r STEPS=%d LOOPS=%d PHASH=%s",
        before.proto.name,
        before.proto.steps,
        before.proto.loops,
        before.proto.phash,
    )
    if base_proto and before.proto.name and before.proto.name.upper() != base_proto.upper():
        pytest.fail(
            f"VERDICT: BASE_PROTOCOL_MISMATCH — loaded {before.proto.name!r} "
            f"but expected baseline {base_proto!r} before swap."
        )

    operator_confirm(
        f"Swap SD protocol to {swap_proto!r}, reinsert SD, and ensure LCD no "
        "longer shows 'Protocol Missing on SD Card!'."
        " If you reset XPB during swap, wait until it is back up before continuing.",
        token="SWAPPED",
    )

    after = reset_to_idle(monitor, teensy, _log, require_hlfb_quick=False)
    _log.info(
        "Post-swap protocol: NAME=%r STEPS=%d LOOPS=%d PHASH=%s",
        after.proto.name,
        after.proto.steps,
        after.proto.loops,
        after.proto.phash,
    )

    if after.proto.name.upper() != swap_proto.upper():
        pytest.fail(
            f"VERDICT: SWAP_PROTOCOL_MISMATCH — loaded {after.proto.name!r}, "
            f"expected {swap_proto!r}."
        )
    if after.proto.steps <= 0 or after.proto.loops <= 0:
        pytest.fail(
            f"VERDICT: SWAP_PROTOCOL_EMPTY — loaded {after.proto.name!r} "
            f"with STEPS={after.proto.steps} LOOPS={after.proto.loops}."
        )

    _log.info(
        "VERDICT: PASS — manual swap loaded %r (STEPS=%d LOOPS=%d PHASH=%s)",
        after.proto.name,
        after.proto.steps,
        after.proto.loops,
        after.proto.phash,
    )
