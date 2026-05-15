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
  - Run: `python -m pytest test/scenarios/ -v` (host PC must be on 10.0.0.100)
  - Per-session log: `test/log/<YYYY>/<MM>/<DD>/<HHMMSS>_pytest.log`
- Host tooling
  - UDP capture: [tools/udp_capture.py](../tools/udp_capture.py)
  - UDP probe: [tools/udp_probe.py](../tools/udp_probe.py)
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

## Build and upload
COM ports: COM8 = ClearCore, COM9 = Expansion Board (Nano Every).

Build from a PowerShell terminal:
```
cmd /c "C:\Users\stephen\.platformio\penv\Scripts\platformio.exe run -e XPB -e clearcore > %TEMP%\pio.log 2>&1"
Get-Content $env:TEMP\pio.log -Tail 30
```

Uploads are performed manually from the PlatformIO IDE task runner — never
from the terminal.

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
- **Power-loss recovery**: periodic saves during RUNNING at step/loop
  boundaries. On next boot, XPB finds valid resume data and sends
  `CMD;RESUME=AUTO` — with `AUTOSTART=1` if RUN is engaged or
  `AUTOSTART=0` otherwise.
- **Protocol completion**: resume slots cleared, `everRan_` reset.

## Documentation map
- System overview and architecture: [README.md](../README.md)
- Test harness usage: [test/README](../test/README)
- Findings and validation plan: [CODE_REVIEW.md](../CODE_REVIEW.md)
- Workspace agent policy: [.github/copilot_instructions.md](../.github/copilot_instructions.md)
- Path-specific agent policy: [.github/instructions/](../.github/instructions/)

## Documentation policy
- Functional changes require matching docs updates in the same change set.
- Test changes require matching docs updates in the same change set.
