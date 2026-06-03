# RTM Project Handoff

## Purpose
Fast resume map for firmware, host tooling, and validation work in this
repository.

## Repository map
- Firmware targets
  - ClearCore controller: [src/clearcore/main.cpp](../src/clearcore/main.cpp)
  - Expansion board (Nano Every): [src/exp-board/main.cpp](../src/exp-board/main.cpp)
- Shared transport
  - UDP framing and retries: [src/shared/RtmComms.cpp](../src/shared/RtmComms.cpp)
  - Network constants (IPs, port): [include/RtmNet.h](../include/RtmNet.h)
- Test harness (pytest, UDP-passive)
  - Layout: `test/rig/{parser,monitor,assertions}.py` + `test/scenarios/test_*.py`
  - Procedure: [docs/TEST_PROCEDURE.md](TEST_PROCEDURE.md)
  - Run full automated: `python -m pytest test/scenarios/ -v -m "not manual"`
    (host PC must be on 10.0.0.100)
  - Run full manual-assisted: `python -m pytest test/scenarios/ -v -s -m "manual"`
    (`manual` includes `manual_power`, `manual_protocol`, and `manual_thermal`)
  - Manual SD-swap guardrails: protocol/thermal scenarios now block on typed
    confirmation tokens before reset normalization (`SWAPPED` for protocol swap,
    `READY` for thermal preheat) to avoid advancing while SD is out or XPB is
    still rebooting.
  - Manual thermal mode toggle:
    `python -m pytest test/scenarios/test_manual_thermal_preheat.py -v -s --manual-thermal-mode body|external`
  - Run smoke: `python -m pytest test/scenarios/ -v -m smoke`
  - Smoke-marked scenarios run first and gate the suite. Any smoke failure or
    skip stops the session unless `--no-smoke-gate` is supplied.
  - Per-session log: `test/log/<YYYY>/<MM>/<DD>/<HHMMSS>_pytest.log`
  - Shared scenario helpers: [test/rig/scenario.py](../test/rig/scenario.py)
  - Passive protocol-upload diagnostics accept RUNNING/PAUSED/COMPLETED HBs as
    proof that a protocol was already loaded by an earlier scenario.
  - Reset-gate diagnostics wait for `PROTO_RX=OK` and the later `IDLE` HB;
    `PROTO_RX=OK` can precede final IDLE by several seconds.
- Host tooling
  - UDP capture: [tools/udp_capture.py](../tools/udp_capture.py)
  - UDP probe: [tools/udp_probe.py](../tools/udp_probe.py)
  - Unified tri-capture tracer: [tools/rig_trace.py](../tools/rig_trace.py)
  - Tool dependencies: [tools/requirements.txt](../tools/requirements.txt)

## Inter-board transport
- UDP over Ethernet (W5500 modules on both boards).
- Static IPs: ClearCore = 10.0.0.10, XPB = 10.0.0.11, host PC = 10.0.0.100.
- Production port: 8888.
- PC observer capture port: 8889.
- Frame format: line-oriented ASCII, `KIND;K1=V1;...:CC\n`, where `CC` is XOR
  checksum of every byte before `:`.
- Cadences: HB from CC every 250 ms, STAT from XPB every 1000 ms.
- `RTM_TEE_TO_PC=1` compiles in PC observation support. Host tools send
  `OBS;PC=1` to CC/XPB once per second on UDP 8888; each board tees debug
  copies to directed broadcast `10.0.0.255:8889` while the beacon is fresh
  (3s TTL). Without a beacon, only the production CC↔XPB unicast path runs.
- XPB protocol handoff resilience: while `ProtoTxState::WaitingReq`, XPB now
  re-sends `NOTICE;PROTO_READY` every 2s until CC requests upload; this covers
  dropped startup notices.
- XPB TX path no longer drops frames based on `Ethernet.link()` status. This
  avoids observer-silent handoff failures when PHY status reads are transiently
  incorrect during bench bring-up.

## Build and upload
COM ports: COM8 = ClearCore, COM9 = Expansion Board (Nano Every).

Build from a PowerShell terminal:
```
cmd /c "C:\Users\stephen\.platformio\penv\Scripts\platformio.exe run -e XPB -e clearcore > %TEMP%\pio.log 2>&1"
Get-Content $env:TEMP\pio.log -Tail 30
```

XPB UDP sanity debug build (Nano Every + W5500 wiring/transport only):
```
cmd /c "C:\Users\stephen\.platformio\penv\Scripts\platformio.exe run -e XPB_DEBUG > %TEMP%\pio_xpb_debug.log 2>&1"
Get-Content $env:TEMP\pio_xpb_debug.log -Tail 30
```
The `XPB_DEBUG` image is a minimal W5500 validation target. It does not run
the production protocol stack, SD parsing, or LCD runtime UI. It provides:
- static XPB IP bring-up (`10.0.0.11`)
- UDP listen on `8888`
- UDP heartbeat to observer port `8889` at 1 Hz
- packet echo replies for inbound traffic on 8888
- serial diagnostics at `115200` (`BOOT`, `LINK`, `RX`, `SEQ` counters)

Use `XPB_DEBUG` specifically to isolate W5500 wiring/hardware from production
firmware behavior. If `XPB_DEBUG` cannot be observed by PC tooling on UDP,
the issue is below the production protocol layer.

Hardware note: these telemetry additions are build-validated but require bench
validation to confirm tick monotonicity and expected NET fault transitions under
cable pull/replug scenarios.

Uploads are performed manually from the PlatformIO IDE task runner — never
from the terminal.

Tri-capture workflow (CC serial + XPB serial + UDP observer) with end-of-run
handoff summary counters:
```
python tools/rig_trace.py --cc-port COM8 --xpb-port COM9 --duration-s 30
```
By default, `rig_trace.py` now auto-writes a dated log at
`test/log/<YYYY>/<MM>/<DD>/<HHMMSS>_rig_trace.log` and prints the resolved
path at startup.

If binary/noisy NET payloads are suspected, capture full raw UDP bytes in a
separate sidecar file while keeping the main trace readable:
```
python tools/rig_trace.py --cc-port COM8 --xpb-port COM9 --duration-s 30 --raw-net-log test/log/trace_attempt.raw_net.log
```
At completion, `rig_trace.py` now prints a deterministic summary block with:
- observed `REQ:PROTO`, `PR_BEG/PR_DAT/PR_END`, and `NOTICE;PROTO_RX=OK` counts
- XPB debug link flap counters (`LINK=DOWN/UP`)
- XPB debug/trace request markers (`CC_REQ_PROTO_RX`, `[PROTO] REQ:PROTO rx`)
- a branch hint (`B1/B2/B3`) for the current root-cause proof-pack workflow

If you need raw streaming output only (no post-run summary), add `--no-summary`.

## HLFB speed measurement architecture
- Motor: ClearPath CPM-SDSK-2321S-RLN, NEMA 23, max 3170 RPM.
- HLFB mode: `HLFB_MODE_STATIC` with `HlfbFilterLength(1)` (200 µs minimum).
- PPR: 16 (configured in MSP, constant `kHlfbPPR` in firmware).
- Measurement: span-based frequency counting over the 250 ms heartbeat window.
  RPM at HB send time: `(edges-1) × 60,000,000 / (spanUs × PPR)`. Sign from
  `VelocityRefCommanded()`. Stale timeout: 400 ms with no edges → RPM = 0.
- `pollHlfbEdge_()` is called from 5 points in `tick()` to prevent missed
  edges during I/O.

## Reset behavior
- **Manual reset** (`RESET=EXEC`): unconditionally clears resume slots
  (RA.BIN/RB.BIN). Protocol starts fresh on next boot. No resume snapshot
  is saved.
- **Reset timing**: after `PROTO_RX=OK`, CC may remain in `BOOTING` for several
  seconds before emitting steady `IDLE` HBs. Tests should assert the final
  `IDLE` heartbeat, not assume protocol receipt is the end of reset recovery.
- **Power-loss recovery**: periodic saves during RUNNING at step/loop
  boundaries. On next boot, XPB finds valid resume data and sends
  `CMD;RESUME=AUTO` — with `AUTOSTART=1` if RUN is engaged or
  `AUTOSTART=0` otherwise.
- **Protocol completion**: resume slots cleared, `everRan_` reset.

## Documentation map
- System overview and architecture: [README.md](../README.md)
- Test harness usage: [test/README](../test/README)
- Full test procedure: [docs/TEST_PROCEDURE.md](TEST_PROCEDURE.md)
- Findings and validation plan: [CODE_REVIEW.md](../CODE_REVIEW.md)
- Workspace agent policy: [.github/copilot_instructions.md](../.github/copilot_instructions.md)
- Path-specific agent policy: [.github/instructions/](../.github/instructions/)

## Documentation policy
- Functional changes require matching docs updates in the same change set.
- Test changes require matching docs updates in the same change set.
