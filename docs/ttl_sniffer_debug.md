## Overview
The Teensy 4.0 sniffer watches the ClearCore and XPB UART traffic on dedicated receive pins and
forwards tagged lines to the host over USB CDC. Recent firmware updates add startup synchronization,
line buffering, and an optional hex-dump diagnostic window to reduce boot-time corruption.

## Wiring
- Teensy 4.0 pin 0 (RX1) -> ClearCore TX tap (through any required level shifting)
- Teensy 4.0 pin 7 (RX2) -> XPB TX tap
- Teensy 4.0 pin 2 (open-drain) -> Active-low RUN line under test (add a level shifter if the
  remote pull-up exceeds 3.3 V)
- Teensy 4.0 pin 3 (open-drain) -> Active-low RESET line under test (same level-shifting caveats)
- Teensy GND -> ClearCore/XPB ground reference
- Verify that both tapped signals stay within the Teensy 3.3 V input range; add dividers or
  buffers if necessary
- Disconnect any other USB or power sources that could back-feed the bus when sniffing

The RUN/RESET pins idle in a high-impedance state; the Teensy only sinks current when you assert a
command. If you are bridging into a 5 V pull-up, place a MOSFET, optocoupler, or level shifter so
the Teensy pin never sees more than 3.3 V.

## Build and Upload
The sniffer firmware lives under [src/sniffer/](src/sniffer/) and is built by PlatformIO environment
ttl-sniffer (see [platformio.ini](platformio.ini)). Build and upload from the repo root with:

```
platformio run -e ttl-sniffer -t upload
```

The environment targets `teensy40` and uses `sniffer/*.cpp` sources only. Confirm the correct upload
port in `platformio.ini` or supply `--upload-port` when running the command.

## Run/Reset Control

With the USB console open the firmware prints a helper banner. Available commands:

```
RUN=1           # assert RUN (drive line LOW)
RUN=0           # release RUN (line floats HIGH)
RST=1           # assert RESET (drive line LOW)
RST=0           # release RESET (line floats HIGH)
PWR=1           # turn board power relay ON
PWR=0           # turn board power relay OFF
PULSE RUN=250   # sink RUN for 250 ms, then release unless latched
PULSE RST=250   # sink RESET for 250 ms, then release unless latched
STATUS          # print current line levels and latch/pulse state
HELP            # show command summary
```

Aliases such as `RESET=` and textual values (`ON`, `OFF`, `ASSERT`, `RELEASE`) are also accepted.
Latch commands override pulses; if you need a momentary press ensure the line is released before
issuing another `PULSE`.

### Hard reset via power relay
Use host commands or the test CLI power mode to force a cold boot when RUN/RESET pulses are not
sufficient. Typical sequence:

```
PWR=0
PWR=1
```

From host automation:

```
python test/rig_control.py power --state off --capture-s 4
python test/rig_control.py power --state on --capture-s 8 --drop-first-line
```

## LED Status
- Standby/Idle: slow blink (~0.5 s toggle)
  - USB not connected/open yet, or
  - USB connected but no UART traffic seen for ~2 s
- Active Traffic: fast blink (~0.125 s toggle) when recent UART bytes are observed

## Host Capture Script
Use [tools/ttl_stream_capture.py](tools/ttl_stream_capture.py) to monitor USB output:

```
python tools/ttl_stream_capture.py --port COM7 --baud 460800 --drop-first-line
```

Key options:
- --drop-first-line skips one partial line after connect if the target is mid-boot
- --timestamp prepends ISO timestamps to emitted lines when you need trace correlation
- --raw renders each captured line as hex byte pairs for side-by-side comparison when you need to
  inspect raw framing issues
- --log <path> appends the capture to a UTF-8 text file while optionally echoing to console

## Tool Selection Guide
Use the right host tool for the task:

| Goal | Preferred tool | Why |
|------|----------------|-----|
| Repeatable validation run (reset, run, protocol, power cycle) | [test/rig_control.py](../test/rig_control.py) | Scripted scenarios, stable defaults, and named logs in `test/log/`. |
| Open-ended live monitoring while reproducing an unknown issue | [tools/ttl_stream_capture.py](../tools/ttl_stream_capture.py) | Lightweight continuous stream viewer/logger without test orchestration. |
| Inspect framing corruption or byte-level startup noise | [tools/ttl_stream_capture.py](../tools/ttl_stream_capture.py) with `--raw` | Hex output helps distinguish encoding issues from transport corruption. |
| Generate evidence for findings and regressions | [test/rig_control.py](../test/rig_control.py) first, then optional stream capture | Structured logs are easier to compare across runs. |

## Instrument-and-Observe Debugging

When sniffer logs alone don't reveal the root cause, add a temporary debug field to
an existing periodic message, re-capture with the test harness, and compare the field
across state transitions.  This keeps the investigation on real hardware and produces
a deterministic log you can diff against the post-fix run.

### Workflow

1. **Hypothesise** — identify which internal variable is suspect (e.g. a countdown,
   timer offset, or state flag).
2. **Instrument** — append a short key=value field to an existing XPB or CC periodic
   message so the sniffer captures it every cycle.  Keep the field compact to avoid
   breaking message budgets.
   ```cpp
   // Example: add T=<stepRemainingSeconds> to the STAT line
   dbgkv("T", stepRemainingMs_ / 1000);
   ```
3. **Build & deploy** — `pio run -e XPB` (or whichever target), then flash via
   the Upload task or manual upload.  Do **not** remove any existing fields; only add.
4. **Capture** — run a scripted scenario that exercises the suspect transition:
   ```
   python test/rig_control.py --port COM7 run-cycle --pulse 10000 --observe 15
   ```
   The `--pulse` duration controls how long RUN is held before the pause; `--observe`
   sets how many seconds the sniffer records after the final state change.
5. **Analyse** — grep the log for the new field across phase boundaries:
   ```
   grep -E "STAT;.*T=|Phase|RUN=" test/log/2026/04/17/<logfile>.log
   ```
   Confirm the value matches expectations before the transition, goes to the expected
   interim value during the pause, and either continues or resets after resume.
6. **Fix** — apply the code change, rebuild, and re-run the same scenario.  The new
   log should show the corrected values at the same phase boundaries.
7. **Clean up** — remove the debug field, rebuild, and run a full regression suite to
   ensure the production message format is unchanged.

### Worked example — Bug #15 (dwell countdown reset on pause/resume)

| Phase | Expected `T=` | Observed `T=` | Verdict |
|-------|---------------|---------------|---------|
| IDLE baseline | 0 | 0 | OK |
| RUNNING (14 s elapsed) | 300 → 286 | 300 → 286 | OK |
| PAUSED | 0 (display cleared) | 0 | OK |
| RESUME | ~286 (continue) | **300** (reset) | **BUG** — lost 14 s |
| RUNNING post-fix | ~286 (continue) | **285** (continue) | **FIXED** |

The `T=` field made the 14-second loss immediately visible in a single grep pass,
eliminating guesswork about SW_AGE resets and XPB-internal timer state.

### Tips

- Prefer appending to an existing periodic message over adding a new message; this
  avoids changing bus timing or saturating the link.
- Use the same `run-cycle` parameters for the pre-fix and post-fix captures so the
  logs are directly comparable.
- Keep the debug field name short (one or two characters) to stay within the XPB flash
  budget — the ATmega4809 is >90 % full.
- If the field you need is on the CC side, instrument the HB line instead of STAT;
  the sniffer captures both channels.
- Save pre-fix and post-fix logs; they serve as regression evidence in CODE_REVIEW.md.

## Diagnostic Checklist
- Test each UART independently by temporarily commenting out the opposite channel in firmware to
  isolate cross-talk issues
- Confirm the sniffer reports `CC sniff baud=9600` and `XPB sniff baud=9600` at boot; mismatch or
  legacy settings often cause replacement characters
- Enable the firmware hex window (toggle via constant kEnableHexBootWindow) when you need to inspect
  raw framing bytes during startup
- Ensure ClearCore and XPB are not powering the Teensy through signal lines; disconnect extra USB
  links to avoid phantom power paths
- Validate wiring continuity and that shield grounds are common before monitoring live systems
- Recheck voltage levels if corruption persists; Teensy inputs are not 5 V tolerant without level
  shifting
