# RTM Test Procedure

This document is the operator guide for validating the RTM ClearCore and
Expansion Board Ethernet workflow. It covers the normal automated suite,
manual-assisted tests, expected rig inputs, and the common recovery steps for
false negatives.

## Scope

The pytest harness observes the CC-to-XPB UDP traffic from the host PC and, for
automated tests, uses the Teensy rig controller to drive RUN and RST. Test
verdicts come from observable wire traffic: HB, STAT, SW, CMD, ACK, PR_*, and
NOTICE frames.

The harness does not read private firmware state, does not upload firmware, and
does not power-cycle the CC or XPB controllers automatically. Controller cold
boot and real power-loss validation remain manual-assisted.

## Rig Requirements

- ClearCore firmware uploaded and running.
- XPB firmware uploaded and running.
- Teensy rig controller available on USB, normally COM7.
- ClearCore upload port is normally COM8. Do not open it during tests unless
  intentionally uploading from the PlatformIO task runner.
- XPB upload port is normally COM9. Do not open it during tests unless
  intentionally uploading from the PlatformIO task runner.
- Host PC Ethernet interface configured as `10.0.0.100/24` on the isolated rig
  LAN.
- ClearCore static IP: `10.0.0.10`.
- XPB static IP: `10.0.0.11`.
- Production firmware UDP port: `8888`.
- PC observer capture port: `8889`.
- Ethernet cable connected to the rig LAN before running live tests.
- `protocol.csv` on the XPB SD card should normally be `HLFB_QUICK` for the
  automated full suite.

## Observer Behavior

The PC monitor binds UDP `8889` and sends `OBS;PC=1` beacons once per second to
both boards on UDP `8888`. While the beacon is fresh, both boards tee debug
copies to directed broadcast `10.0.0.255:8889`.

If the PC or harness is absent, the boards continue using only the production
CC-to-XPB unicast path. The PC observer must not be required for RUN/RST control
or production board communication.

## Safety Setup

Before live tests:

1. Verify the rig is mechanically safe for repeated RUN, PAUSE, and RESET
   transitions.
2. Verify the expected protocol on SD is safe to run.
3. Confirm motor/heater bus power state is appropriate for the selected test.
4. Keep hands clear of moving parts before running automated motion tests.
5. Confirm the Ethernet link is connected. A missing cable causes all-zero UDP
   captures and misleading failures.

## Test Tiers

### Collection Only

Use this after harness edits. It does not open live fixtures or move hardware.

```powershell
python -m pytest test/scenarios --collect-only -q
```

### Smoke

Short live-rig confidence path. Intended to prove observer UDP, reset reload,
and RUN/RST basics.

```powershell
python -m pytest test/scenarios/ -v -m smoke
```

Expected scenarios:

- `test_comms_health.py`
- `test_reset_reloads_protocol.py`
- `test_run_cycle.py`

### Full Automated

Runs all non-manual live-rig coverage. Tests should self-normalize their own
state before making assertions.

```powershell
python -m pytest test/scenarios/ -v -m "not manual"
```

The full automated suite assumes `HLFB_QUICK` is loaded for the protocol
execution scenario. If another protocol is loaded, protocol-specific tests may
skip with a message naming the mismatch.

Smoke-marked scenarios run first as a gate. If any smoke scenario fails or
skips, pytest stops the session before running the rest of the automated suite.
Use `--no-smoke-gate` only when intentionally collecting additional failure
evidence after a known bad baseline.

### Manual-Assisted

Manual-assisted tests require operator action and should be run individually
with `-s` so prompts are visible and stdin is available.

```powershell
python -m pytest test/scenarios/test_cold_boot_protocol_upload.py -v -s
```

Run the full manual-assisted suite with:

```powershell
python -m pytest test/scenarios/ -v -s -m "manual"
```

The `manual` marker is the umbrella selector and includes any subgroup tests
marked `manual_power`, `manual_protocol`, or `manual_thermal`.

Current manual scenarios:

- `test_cold_boot_protocol_upload.py`
- `test_manual_power_loss_resume.py`
- `test_manual_protocol_swap.py`
- `test_manual_thermal_preheat.py`

Use marker expressions to select future manual groups:

```powershell
python -m pytest test/scenarios/ -v -s -m manual_power
python -m pytest test/scenarios/ -v -s -m manual_protocol
python -m pytest test/scenarios/ -v -s -m manual_thermal
```

## Command Options

- `--rig-port`: UDP observer capture port. Default: `8889`.
- `--rig-firmware-port`: board production/beacon port. Default: `8888`.
- `--rig-bind`: local bind interface. Default: `0.0.0.0`.
- `--teensy-port`: Teensy COM port. Default: auto-detect, then COM7.
- `--log-file`: explicit pytest log file. If omitted, the harness creates
  `test/log/<YYYY>/<MM>/<DD>/<HHMMSS>_pytest.log`.
- `--no-smoke-gate`: continue after a smoke-marked failure or skip. Default is
  to stop the selected suite immediately after a failed smoke gate.
- `--manual-base-protocol`: expected baseline protocol before manual swap/power
  checks. Default: `HLFB_QUICK`.
- `--manual-swap-protocol`: expected protocol after manual SD swap and for
  thermal manual tests. Default: `HEAT_TEST`.
- `--manual-thermal-mode`: `body` or `external` thermal scenario behavior.
  Default: `body`.

## Markers

- `live_rig`: requires powered CC and XPB on the rig LAN.
- `smoke`: short confidence path.
- `full`: automated full-regression scenario.
- `stateful`: mutates physical rig state and must self-normalize.
- `requires_hlfb_quick`: requires the `HLFB_QUICK` baseline protocol.
- `slow`: nominal duration greater than 30 seconds.
- `manual`: any operator-assisted scenario.
- `manual_power`: manual controller power-cycle or unplug/replug.
- `manual_protocol`: manual SD protocol swap or edit.
- `manual_thermal`: manual thermal stimulus or supervision.

## Normal Scenario Order

Pytest collection is ordered for readable transcripts, but each automated test
must still establish its own preconditions.

1. `test_comms_health.py`
2. `test_reset_reloads_protocol.py`
3. `test_run_cycle.py`
4. `test_protocol_upload.py`
5. `test_protocol_execution.py`
6. `test_reset_cancel.py`
7. `test_reset_pulse_multi.py`
8. `test_run_gate.py`
9. `test_cold_boot_protocol_upload.py`
10. `test_manual_power_loss_resume.py`
11. `test_manual_protocol_swap.py`
12. `test_manual_thermal_preheat.py`

## Scenario Reference

### Comms Health

File: `test/scenarios/test_comms_health.py`

- Purpose: prove the observer sees nominal CC HB and XPB STAT traffic.
- Inputs: powered boards, Ethernet connected, PC at `10.0.0.100`.
- Self-normalizing: no; passive liveness gate.
- Pass signals: HB cadence near 250 ms, STAT cadence near 1000 ms, monotonic
  sequences, no parse errors, no E-STOP HBs.
- False negatives: Ethernet unplugged, wrong PC IP, wrong observer port, boards
  not powered, stale firmware without observer support.

### Passive Protocol Upload Diagnostic

File: `test/scenarios/test_protocol_upload.py`

- Purpose: passive diagnostic for whether a real protocol is loaded or has
  already executed.
- Inputs: powered boards and observer capture.
- Self-normalizing: no; intentionally passive.
- Pass signals: PR_* or `NOTICE;PROTO_RX=OK` in window, or HB state already
  `RUNNING`, `PAUSED`, or `COMPLETED`.
- Failure means: no proof of protocol load/execution was observed in that
  passive window.
- Use deterministic reset/reload tests for the authoritative upload proof.

### Reset Reload

File: `test/scenarios/test_reset_reloads_protocol.py`

- Purpose: drive a supra-threshold RST pulse and prove the full reload chain.
- Inputs: Teensy RST wiring, CC/XPB powered, Ethernet connected.
- Self-normalizing: yes; parks RUN and drives RST.
- Pass signals: `SW;RST=1`, `CMD;RESET=ARM`, `CMD;RESET=EXEC`, `REQ:PROTO`,
  `PR_BEG`, `PR_DAT`, `PR_END`, `NOTICE;PROTO_RX=OK`, and final CC `IDLE` HB.
- Common false negatives: test timeout too short for post-`PROTO_RX=OK`
  BOOTING tail, missing RST wiring, SD protocol unreadable.

### Run Cycle

File: `test/scenarios/test_run_cycle.py`

- Purpose: prove RUN edge behavior: IDLE to RUNNING, drop to PAUSED, resume to
  RUNNING, final PAUSED.
- Inputs: safe motion setup and a loaded protocol.
- Self-normalizing: yes; resets/reloads to IDLE before asserting edges.
- Pass signals: HBs transition through RUNNING and PAUSED as commanded.
- Common false negatives: no loaded protocol, motor path disabled, RUN wiring
  fault, rig not safe to move.

### Protocol Execution

File: `test/scenarios/test_protocol_execution.py`

- Purpose: run `HLFB_QUICK` through all 12 steps and verify step order and RPM
  tolerance.
- Inputs: `HLFB_QUICK` on SD, safe motion setup, Teensy RUN/RST.
- Self-normalizing: yes; reset/reload helper verifies `HLFB_QUICK` before RUN.
- Pass signals: CC reaches `COMPLETED`, steps 1 through 12 appear in order, and
  tail-average RPM is within tolerance for each step.
- Common false negatives: wrong SD protocol, motor disabled, HLFB signal absent,
  capture lost during run.

### Reset Cancel

File: `test/scenarios/test_reset_cancel.py`

- Purpose: prove sub-threshold RST pulses arm/cancel but never execute reset.
- Inputs: Teensy RST wiring.
- Self-normalizing: parks RUN and observes a stable baseline.
- Pass signals: `SW;RST=1` and `CMD;RESET=ARM` are allowed; no
  `CMD;RESET=EXEC`, no XPB reset notice, no protocol re-request/re-upload, and
  no forbidden RUNNING/BOOTING/RESETTING transitions.
- Common false negatives: RST wiring not seen by XPB, baseline already unstable.

### Reset Pulse Multi

File: `test/scenarios/test_reset_pulse_multi.py`

- Purpose: stress repeated reset/reload chains.
- Inputs: Teensy RST wiring, SD protocol, Ethernet observer.
- Self-normalizing: each iteration clears the monitor and drives its own RST.
- Pass signals: every iteration reaches the reset reload PASS verdict.
- Common false negatives: slow SD init exceeding timeout, intermittent Ethernet
  capture gaps, repeated RST stress exposing a real firmware race.

### Run Gate

File: `test/scenarios/test_run_gate.py`

- Purpose: prove held RUN levels do not auto-start after reset; only fresh RUN
  edges start or resume motion.
- Inputs: safe motion setup, Teensy RUN/RST wiring, loaded protocol.
- Self-normalizing: drives resets inside the scenario and waits for reload IDLE.
- Pass signals: reset with RUN low returns to IDLE, RUN edge starts, RUN drop
  pauses, resume edge starts, reset with RUN high still returns to IDLE, fresh
  edge starts again.
- Common false negatives: expecting IDLE immediately after `PROTO_RX=OK`; CC can
  remain BOOTING for several seconds before final IDLE.

### Cold Boot Protocol Upload

File: `test/scenarios/test_cold_boot_protocol_upload.py`

- Purpose: capture a true controller cold boot or physical reset transcript.
- Inputs: manual controller reset/power-cycle; pytest run with `-s`.
- Self-normalizing: no; operator action is the stimulus.
- Pass signals: boot traffic appears, CC requests protocol, XPB uploads it, CC
  emits `PROTO_RX=OK`, and CC reaches IDLE.
- Common false negatives: pressing Enter before the actual reset, no Ethernet,
  stdin closed because `-s` was omitted.

### Manual Power-Loss / Single-Side Failure

File: `test/scenarios/test_manual_power_loss_resume.py`

- Purpose: validate outage policy for both-board outages vs single-side
  failures/comms-loss events.
- Inputs: operator-performed outage actions with RUN asserted.
- Pass signals: both-board outage auto-resumes within 30 s; single-side outage
  emits a fault flag (`LINK=DOWN`, alarm, or E-STOP) before any automatic
  restart.

### Manual Protocol Swap

File: `test/scenarios/test_manual_protocol_swap.py`

- Purpose: verify SD swap to a target protocol and deterministic reset reload.
- Inputs: operator swaps SD protocol file, test drives reset pulse.
- Pass signals: post-reset loaded protocol NAME matches
  `--manual-swap-protocol`, with non-zero steps/loops and normal reload chain.

### Manual Thermal Preheat

File: `test/scenarios/test_manual_thermal_preheat.py`

- Purpose: validate thermal preheat behavior under bench body-heat mode and
  external test-stand mode.
- Inputs: thermal profile loaded (typically `HEAT_TEST`) plus operator stimulus.
- Pass signals (body): rise + transition within 60 s.
- Pass signals (external): measurable rise first, then hold near setpoint
  through the configured hold window.

## Manual Protocol Swap Procedure

Use this when validating a non-`HLFB_QUICK` protocol.

1. Stop the automated suite.
2. Put the desired `protocol.csv` on the XPB SD card.
3. Reinsert SD and reset/power-cycle the XPB as required.
4. Run a manual protocol scenario or `test_reset_reloads_protocol.py` to verify
   `PR_BEG NAME/STEPS/LOOPS/PHASH` and `PROTO_RX=OK`.
5. Restore `HLFB_QUICK` before running the full automated suite.

## Manual Thermal Procedure

Use this only with a low-temperature protocol intended for bench validation.
Example candidate: a `HEAT_TEST` variant with setpoints near lab ambient.

1. Confirm the thermal protocol is safe for the connected hardware.
2. Run the manual thermal pytest scenario with `-s` when implemented.
3. Let the test observe below-setpoint STAT frames.
4. Warm the thermocouple by breath or hand heat when prompted.
5. Pass criteria should include CC waiting below setpoint, STAT crossing the
   setpoint, and CC leaving preheat only after the threshold is met.

## Manual Power-Loss / Resume Procedure

Controller power-loss recovery cannot be fully automated unless the rig can
power-cycle CC and XPB controller power. For now:

1. Use an automated or guided step to start a protocol and create a resume point.
2. When prompted, physically power-cycle or unplug/replug the controllers.
3. Hold RUN in the requested state during boot.
4. Verify `CMD;RESUME=AUTO`, matching PHASH, correct AUTOSTART value, and the
   expected post-boot state or resumed motion.

## Log Review

Each run writes a timestamped log under `test/log/<YYYY>/<MM>/<DD>/`. The log
includes Python logging records and, after the harness hardening, a compact
pytest summary with result counts and failed node IDs.

When reviewing a run:

1. Start with the pytest summary at the bottom.
2. For failures, inspect the scenario-specific `VERDICT:` line.
3. Check the transcript for the required marker chain.
4. Separate harness/precondition failures from firmware behavior failures.
5. If frames are zero from both boards, check Ethernet, PC IP, observer port,
   and that the boards were powered.

## Recovery Checklist

- No frames from either board: check Ethernet cable, switch/router power, PC IP,
  observer port, and firmware observer support.
- CC frames but no XPB frames: check XPB power, W5500 link, SD/LCD SPI bus, and
  XPB firmware.
- XPB frames but no CC frames: check ClearCore power, Ethernet link, and CC
  firmware.
- Reset tests fail before `SW;RST=1`: check Teensy RST wiring to XPB input.
- Reset tests see `PROTO_RX=OK` but no IDLE: check timeout first; CC may remain
  BOOTING for several seconds after protocol receipt.
- Protocol execution skips: restore `HLFB_QUICK` on the SD card.
- Motion state transitions fail: check RUN wiring, motor enable path, HLFB, and
  whether the rig is in E-STOP.

## Expected Final States

- Smoke suite: rig may end in PAUSED after run-cycle cleanup.
- Full automated suite: rig may end in PAUSED or IDLE depending on the final
  scenario path. Tests should park RUN low on teardown through the Teensy
  fixture best-effort close behavior.
- Manual tests: final state depends on the operator action and scenario prompt;
  park RUN low before leaving the bench.
