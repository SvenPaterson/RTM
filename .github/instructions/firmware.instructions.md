---
applyTo: "src/**/*.cpp,src/**/*.h,include/**/*.h"
description: "Use when: editing ClearCore, expansion board, sniffer, or shared embedded C/C++ firmware."
---

# Firmware Instructions

## Scope
Applies to embedded firmware under src and include.

## Rules
- Preserve runtime safety invariants for active-low RUN, RESET, and power control lines.
- Prefer explicit state transitions over side effects; avoid hidden state coupling between boot, idle, run, and reset flows.
- Keep serial protocol behavior backward compatible unless a protocol change is explicitly requested and documented.
- For timing-sensitive changes, log enough context to validate transitions in captures.
- Keep comments concise and focused on intent where control flow is non-obvious.

## Required documentation updates
- Update [docs/ttl_sniffer_debug.md](../../docs/ttl_sniffer_debug.md) when sniffer command behavior, wiring assumptions, or control semantics change.
- Update [README.md](../../README.md) and [docs/PROJECT_HANDOFF.md](../../docs/PROJECT_HANDOFF.md) when architecture, workflows, or operator procedures change.
- Update [CODE_REVIEW.md](../../CODE_REVIEW.md) when new findings, mitigations, or validation evidence are produced.

## Upload / flash policy
- **NEVER** attempt to upload or flash firmware to any board (ClearCore, Expansion Board, Sniffer) from the terminal or via PlatformIO CLI (`pio run --target upload`). Always let the user perform uploads manually through the PlatformIO IDE task runner.
- **NEVER** open a serial monitor or DeviceMonitor on any COM port. Holding a COM port can brick a Nano Every and prevent re-enumeration.
- Build (`pio run -e <env>`) is permitted; upload and monitor are not.

## Validation expectations
- Include concrete bench validation steps for RUN, RESET, and boot/resume interactions when relevant.
- If tests were not run on hardware, state that clearly and identify the highest-risk unvalidated path.
