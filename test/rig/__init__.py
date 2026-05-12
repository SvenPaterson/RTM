"""Test rig harness for the RTM CC↔XPB Ethernet link.

Pure passive UDP capture — the PC binds the rig port, decodes frames,
and lets scenarios assert on what the boards actually transmit.

Layout:
    parser.py      — wire frame → typed dataclass
    monitor.py     — UDP listener thread + queryable Frame queue
    assertions.py  — wait_for/expect helpers built on the queue

Scenarios consume these via the `monitor` pytest fixture in
`test/scenarios/conftest.py`.
"""
