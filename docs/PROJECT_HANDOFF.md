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
  - Suite runner: `python test/rig_control.py run-suite --port COM7` — runs cold-boot, comms-health, reset-cancel, reset-pulse, run-cycle, run-gate sequentially and writes a `{timestamp}_suite.log` master log with combined output and summary table
  - Compatibility wrappers: [test/run_pulse_test.py](../test/run_pulse_test.py), [test/reset_pulse_test.py](../test/reset_pulse_test.py), [test/reset_cancel_test.py](../test/reset_cancel_test.py), [test/protocol_upload_test.py](../test/protocol_upload_test.py)
  - RUN-gate validation: `python test/rig_control.py run-gate --port COM7` (four-step automated test from CODE_REVIEW.md §7)
  - Captured logs: [test/log/](../test/log/)
- Debug tooling
  - Live stream capture utility: [tools/ttl_stream_capture.py](../tools/ttl_stream_capture.py)
  - Tool dependencies: [tools/requirements.txt](../tools/requirements.txt)

## Current state checkpoint
- **HLFB speed measurement: PASS (±0.1% at 3000 RPM)** — validated 2026-04-14. Span-based frequency measurement, distributed HLFB polling, USB debug echo disabled.
- **RUN-gate validation: four-step test** — Steps 1-2 PASS (2026-04-15). Step 3 (power-loss resume) INCONCLUSIVE (resume ACK accepted but no RUNNING HB in capture window). Step 4 (manual reset clears resume) SKIPPED. Previously all 3 steps PASS (2026-04-13 with old three-step design).
- Twelve firmware bugs fixed (PHASH/SW_AGE overflow, resume slot wipe, heartbeat skip, RUN gate bypass, resume ACK REF mismatch, stale resume after COMPLETED, RESET=EXEC stale save, span-based HLFB, distributed HLFB polling, USB echo disable, manual reset clears resume). See CODE_REVIEW.md §0 for details.
- Resume snapshot hardening complete: retry loops, abort-on-failure, COMPLETED clear, and `everRan_` guard on RESET=EXEC. Manual RESET now unconditionally clears resume slots.
- Build sizes: ClearCore 94,508 B flash (18.6%), 8,244 B RAM (4.2%); XPB 47,812 B flash (**98.3% — 828 B free**), 2,342 B RAM (38.1%).
- Investigation status and active findings are tracked in
  [CODE_REVIEW.md](../CODE_REVIEW.md).
- The latest resume checkpoint is section 0 in [CODE_REVIEW.md](../CODE_REVIEW.md).
- Remaining open items: LCD latch state UX, TTL transport instrumentation, regression suite run.

## HLFB speed measurement architecture
- **Motor**: ClearPath CPM-SDSK-2321S-RLN, NEMA 23, max 3170 RPM.
- **HLFB mode**: `HLFB_MODE_STATIC` with `HlfbFilterLength(1)` (200µs minimum filter).
- **PPR**: 16 pulses per revolution (configured in MSP, constant `kHlfbPPR` in firmware).
- **Measurement**: Span-based frequency counting over the 250ms heartbeat window. Edges are counted via `HlfbHasRisen()` (clear-on-read flag from 5 kHz ISR). RPM computed at HB send time: `(edges-1) × 60,000,000 / (spanUs × PPR)`. Sign derived from `VelocityRefCommanded()`. Stale timeout: 400ms with no edges → RPM=0.
- **Distributed polling**: `pollHlfbEdge_()` called from 5 points in `tick()` to prevent missed edges during serial I/O and STAT processing.
- **USB debug echo**: Disabled on both CC and XPB. Sniffer captures all TTL traffic on the wire.

## Reset behavior
- **Manual reset** (`RESET=EXEC`): Unconditionally clears resume slots (RA.BIN/RB.BIN). Protocol starts fresh on next boot. No resume snapshot is saved.
- **Power-loss recovery**: Periodic saves during RUNNING at step/loop boundaries. On next boot, XPB finds valid resume data and sends `CMD;RESUME=AUTO` — with `AUTOSTART=1` if RUN is engaged (auto-start from resume point) or `AUTOSTART=0` if RUN is off (CC loads position, stays IDLE until operator toggles RUN).
- **Protocol completion**: Resume slots cleared, `everRan_` reset.

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
