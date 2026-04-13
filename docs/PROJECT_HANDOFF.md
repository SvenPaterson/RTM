# RTM Project Handoff

## Purpose
This document is the fast resume map for firmware, host tooling, and validation
work in this repository.

## Repository map
- Firmware targets
  - ClearCore controller: [src/clearcore/main.cpp](../src/clearcore/main.cpp)
  - Expansion board (Nano Every): [src/exp-board/main.cpp](../src/exp-board/main.cpp)
  - TTL sniffer controller (Teensy): [src/sniffer/main.cpp](../src/sniffer/main.cpp)
- Shared transport
  - TTL framing and retries: [src/shared/TTLComms.cpp](../src/shared/TTLComms.cpp)
- Test harness
  - Unified CLI: [test/rig_control.py](../test/rig_control.py)
  - Suite runner: [test/run_suite.py](../test/run_suite.py) — `--quick` (T2/T5/T8) or `--full` (adds T1/T4/T6)
  - Compatibility wrappers: [test/run_pulse_test.py](../test/run_pulse_test.py), [test/reset_pulse_test.py](../test/reset_pulse_test.py), [test/reset_cancel_test.py](../test/reset_cancel_test.py), [test/protocol_upload_test.py](../test/protocol_upload_test.py)
  - RUN-gate validation: `python test/rig_control.py run-gate --port COM7` (three-step automated test from CODE_REVIEW.md §7, **all 3 steps PASS** as of 2026-04-13)
  - Captured logs: [test/log/](../test/log/)
- Debug tooling
  - Live stream capture utility: [tools/ttl_stream_capture.py](../tools/ttl_stream_capture.py)
  - Tool dependencies: [tools/requirements.txt](../tools/requirements.txt)

## Current state checkpoint
- **RUN-gate validation: PASS (all 3 steps)** — validated 2026-04-13 with all firmware fixes, including stale-resume hardening (bugs #7 and #8).
- Eight firmware bugs fixed (PHASH/SW_AGE overflow, resume slot wipe, heartbeat skip, RUN gate bypass, resume ACK REF mismatch, stale resume after COMPLETED, RESET=EXEC stale save). See CODE_REVIEW.md §0 for details.
- Resume snapshot hardening complete: retry loops, abort-on-failure, COMPLETED clear, and `everRan_` guard on RESET=EXEC.
- Investigation status and active findings are tracked in
  [CODE_REVIEW.md](../CODE_REVIEW.md).
- The latest resume checkpoint is section 0 in [CODE_REVIEW.md](../CODE_REVIEW.md).
- Remaining open items: LCD latch state UX, TTL transport instrumentation.

## Build and upload commands
Run from repository root. COM ports: COM8 = ClearCore, COM9 = Expansion Board (Nano Every), COM7 = Sniffer (Teensy).

```bash
platformio run -e clearcore -t upload          # uploads to COM8
platformio run -e exp-board -t upload          # uploads to COM9
platformio run -e ttl-sniffer -t upload
```

**Important:** Power on the rig via sniffer before uploading firmware (bootloader reset can fail if boards are unpowered):
```bash
python test/rig_control.py --port COM7 power --state on
```

## Sniffer runtime controls
Sniffer firmware accepts USB commands for RUN/RESET and board power control.

- RUN latch: `RUN=1` / `RUN=0`
- RESET latch: `RST=1` / `RST=0`
- Power relay: `PWR=1` / `PWR=0`
- Momentary pulse: `PULSE RUN=<ms>` or `PULSE RST=<ms>`
- Status summary: `STATUS`

Use the unified test CLI for scripted scenarios:

```bash
python test/rig_control.py --help
```

## Hard reset workflow
For a full power-cycle reset from host tooling:

0. Verify the active sniffer COM target first (COM assignments can change):

```bash
C:/Users/Stephen.Garden/RTM/.venv/Scripts/python.exe -c "from serial.tools import list_ports; [print(f'{p.device} | {p.description} | {p.hwid}') for p in list_ports.comports()]"
```

1. Turn power off through the sniffer relay.
2. Wait for rails to collapse and serial output to stop.
3. Turn power back on and capture boot traffic.

Example:

```bash
python test/rig_control.py power --state off --capture-s 4
python test/rig_control.py power --state on --capture-s 8 --drop-first-line
```

## Documentation map
- System overview and architecture: [README.md](../README.md)
- Sniffer wiring and command reference: [docs/ttl_sniffer_debug.md](ttl_sniffer_debug.md)
- Test harness usage and modes: [test/README](../test/README)
- Findings and validation plan: [CODE_REVIEW.md](../CODE_REVIEW.md)
- Workspace agent policy: [.github/copilot_instructions.md](../.github/copilot_instructions.md)
- Path-specific agent policy: [.github/instructions/](../.github/instructions/)
- Docs-specific policy: [.github/instructions/docs.instructions.md](../.github/instructions/docs.instructions.md)

## Documentation policy for future changes
- Functional changes require matching docs updates in the same change.
- Test changes require matching docs updates in the same change.
- If no docs changed, include an explicit rationale in review notes.
