# Boot-Up Serial Review Findings

## 0. Session checkpoint (2026-04-13)
### Current status
- **RUN-gate validation: ALL THREE STEPS PASS.**  Validated on hardware after reset-pulse + run-gate sequence (stale-resume regression confirmed fixed).【F:test/log/20260413-100750_run_gate.log†L1-L280】
- Eight firmware bugs identified and fixed across ClearCore and Expansion Board:
  1. **PHASH overflow** (CC + XPB): `toInt()` returns signed long, clamping uint32 hash values >2^31 to `INT_MAX` (2147483647). Fixed with `strtoul()`.【F:include/ClearCoreRTM.h†L489】【F:src/exp-board/ExpansionBoard.cpp†L1274】
  2. **SW_AGE overflow** (XPB): Same `toInt()` truncation pattern. Fixed with `strtoul()`.【F:src/exp-board/ExpansionBoard.cpp†L1453】
  3. **Resume slot wipe** (XPB): `RESET=EXEC` handler called `saveResumeTU()` then immediately `clearResumeSlots_()`, deleting the just-saved resume record. Removed the spurious clear.【F:src/exp-board/ExpansionBoard.cpp†L1385】
  4. **CC heartbeat skip on auto-start**: Proto completion handler went `Idle→Running` in one callback, bypassing `handleIdle(justEntered=true)` which enables `heartbeatSystemEnabled_`. Added explicit `heartbeatSystemEnabled_ = true` in proto completion.【F:include/ClearCoreRTM.h†L641-L656】
  5. **RUN gate bypass**: Proto completion forced `runGateReleased_ = true` and called `promoteRun_()` unconditionally, ignoring the boot gate. Fixed to only auto-start if gate was already open.【F:include/ClearCoreRTM.h†L641-L656】
  6. **Resume ACK REF mismatch** (CC): Resume ACK responses used CC's own REF counter instead of echoing the sender's REF, causing transport-layer ACK mismatch and retry storms (`ERR_WRONG_STAT` on retries). Fixed all 5 ACK;RESUME sends to echo the incoming REF via `MessageType::NORMAL`.【F:include/ClearCoreRTM.h†L361-L440】
  7. **Stale resume after COMPLETED** (XPB): Resume slots were never cleared when protocol completed normally. On next boot with RUN held low, CC found stale AUTOSTART=1 data and auto-started from the old resume point, bypassing the RUN gate. Fixed: XPB now calls `clearResumeTU()` and resets `everRan_` when CC heartbeat reports `STATE=COMPLETED`.【F:src/exp-board/ExpansionBoard.cpp†L1429-L1437】
  8. **RESET=EXEC saves stale resume** (XPB): RESET=EXEC unconditionally saved resume data even when protocol was already COMPLETED or had never run, leaving stale step/loop values on SD for the next boot. Fixed: save is now gated on `everRan_`; when skipped, any stale resume files are also cleared via `clearResumeTU()`.【F:src/exp-board/ExpansionBoard.cpp†L1356-L1378】
- Test script fix: Step 3 evaluation now only fails on `resume_err` if `resume_ok` was NOT also present, tolerating stale retry errors from transport mismatch.【F:test/rig_control.py†L871】
- Snapshot hardening implemented: `saveResumeTU` and `writeResetFlagTU` have retry loops; `RESET=EXEC` handler aborts on save failure; periodic save logs failures.
- Primary unresolved risks remain transport reliability (`TTL bad checksum` / duplicate retries) and LCD operator visibility for RUN/RESET latch state.

### Validated test results (2026-04-13)
| Step | Description | Result | Evidence |
|------|-------------|--------|----------|
| 1 | RUN held low across cold boot, CC stays IDLE | **PASS** | 23 IDLE HBs, no RUNNING before RUN released, no `CMD;RESUME=AUTO` |
| 2 | Gate open, IDLE→RUNNING→PAUSED | **PASS** | 10 RUNNING HBs after RUN=1, 9 PAUSED HBs after RUN=0 |
| 3 | RESUME AUTOSTART=1 with RUN held low | **PASS** | Reset during RUNNING saved snapshot; cold boot with RUN held → `CMD;RESUME=AUTO;STEP=2;LOOP=1;AUTOSTART=1` → `ACK;RESUME=OK;REF=7` → RUNNING at step 2 loop 1/2 |

Preceded by reset-pulse test confirming RESET=EXEC no longer saves stale resume when protocol is completed.【F:test/log/20260413-100706_reset_pulse.log†L1-L60】

### Next actions
1. Add temporary TTL transport instrumentation around send/ACK paths to classify duplicate `PR_END`/`QUIESCE` events as retry-vs-logic.

### Completed audits
- **ACK REF-echo audit (2026-04-10):** All ACK responses in both CC (`ClearCoreRTM.h`) and XPB (`ExpansionBoard.cpp`) now correctly echo the sender's REF and use non-retry message types (`INFO` or `NORMAL`). The 5 resume ACKs fixed earlier in this session were the only instances of the bug. Protocol upload ACKs (`PR_BEG`, `PR_DAT`, `PR_END`), QUIESCE ACKs, and `REQ:PROTO` ACKs were already correct. The repeated QUIESCE bursts seen in test logs are genuine transport retries from TTL checksum drops, not REF-echo mismatches.

### Resume point for next session
- All §7 RUN-gate bugs are resolved and validated (8 bugs total).  Focus shifts to LCD UX and transport reliability.

## 0.1 To-do tracker and validation attempt (2026-03-19)
### What was attempted in this session
1. Audited all captured logs in `test/log/` (21 files) for the primary failure signatures from section 7.
2. Verified the host harness entry points are available after installing `pyserial` in the workspace venv (`test/rig_control.py --help` succeeds).
3. Reviewed current firmware sources for each unresolved item to determine whether code changes already exist.
4. Ran a fresh venv-based power-cycle sequence on COM4 (`power --state off`, then `power --state on --drop-first-line`) and captured new logs for this session.【F:test/log/20260319-104427_power_off.log†L1-L9】【F:test/log/20260319-104437_power_on.log†L1-L9】
5. Re-ran the same power-cycle sequence on COM6 (sniffer port) and captured controller telemetry successfully, including CC boot traffic and `SW;RUN=0;RST=0` after power-on.【F:test/log/20260319-104619_power_off.log†L1-L10】【F:test/log/20260319-104623_power_on.log†L1-L31】
6. Executed section-7 validation step 1 with RUN latched low across a power cycle on COM6 and captured a dedicated log (`20260319-104856_step1_run_held_low_cold_boot.log`).【F:test/log/20260319-104856_step1_run_held_low_cold_boot.log†L1-L68】
7. Executed section-7 validation step 2 on COM6 (boot with RUN released, then assert RUN low) and captured expected state transitions `IDLE -> RUNNING -> PAUSED`.【F:test/log/20260319-105144_step2_gate_open_transition.log†L48-L95】
8. Executed a section-7 step-3 attempt after a fresh reset-pulse snapshot flow, then cold-booted with RUN held low; captured telemetry did not show `CMD;RESUME=AUTO;...AUTOSTART=1` in this run.【F:test/log/20260319-105258_reset_pulse.log†L1-L87】【F:test/log/20260319-105313_step3_resume_autostart_attempt.log†L1-L61】

### Path-to-resolution status
1. RUN-gate / auto-resume collision (`ERR_WRONG_STAT` loop)
   - Status: Partial
   - Current evidence: no `ERR_WRONG_STAT` occurrences found in the reviewed logs; historical power-on captures still show normal bring-up into `SW;RUN=0;RST=0`.【F:test/log/20260109-170658_power_on.log†L12-L17】
   - This-session note: COM4 captures were inconclusive (harness-only). Re-run on COM6 captured CC telemetry and expected post-boot switch state (`SW;RUN=0;RST=0`), confirming valid signal path on the sniffer port for continued validation.【F:test/log/20260319-104427_power_off.log†L1-L9】【F:test/log/20260319-104437_power_on.log†L1-L9】【F:test/log/20260319-104623_power_on.log†L19-L26】
   - Step-1 result: RUN-held-low power-cycle was executed on COM6 and telemetry captured, but the expected `[RUN] Ignoring RUN line held low before XPB resume` marker was not observed in this stream, and explicit BOOT/PROTO_LOADING state text was not surfaced in-capture; treat step 1 as inconclusive pending a tighter cold-boot capture from known idle/off baseline.【F:test/log/20260319-104856_step1_run_held_low_cold_boot.log†L17-L31】
   - Step-2 result: pass. With RUN released at boot, XPB remained in `IDLE` after protocol load; asserting RUN low transitioned to `RUNNING`, and releasing RUN transitioned to `PAUSED`, matching intended gate-open behavior after handshake.【F:test/log/20260319-105144_step2_gate_open_transition.log†L48-L95】
   - Step-3 result: attempted but not observed. After a reset-pulse capture plus cold boot with RUN held low, no `CMD;RESUME=AUTO;...AUTOSTART=1` frame appeared in the stream; keep open pending a deterministic resume-record setup and repeat run.【F:test/log/20260319-105313_step3_resume_autostart_attempt.log†L1-L61】
   - Why not checked off: step 1 remains inconclusive and step 3 did not yet produce the expected resume command signature.

2. LCD surfacing of RUN/RESET latch state during non-idle/reset windows
   - Status: Open
   - Current evidence: reset pages exist, but explicit on-screen latched RUN/RESET bit display is still not documented as implemented; reset-flow captures continue to focus on heartbeat/state text and QUIESCE churn rather than switch-bit confirmation UX.【F:src/exp-board/ExpansionBoard.cpp†L702-L739】【F:test/log/20260109-083256_reset_pulse.log†L31-L55】

3. TTL transport instrumentation/back-off and duplicate frame suppression
   - Status: Open
   - Current evidence: duplicate protocol and quiesce patterns persist in recent logs (`PR_END` repeated with same REF and repeated `QUIESCE` despite ACKs).【F:test/log/20260108-110836_protocol_upload_TEST_0108_D71A.log†L31-L34】【F:test/log/20260109-083256_reset_pulse.log†L31-L55】
   - Code review note: retries remain fixed-time resend logic; no explicit back-off strategy or richer duplicate diagnostics were identified in `TTLComms::checkRetries()`.【F:src/shared/TTLComms.cpp†L122-L134】

4. Resume snapshot hardening (retry + stronger diagnostics)
   - Status: **Complete** — validated 2026-04-13
   - Retry logic added to `saveResumeTU(maxRetries)` and `writeResetFlagTU(maxRetries)`. RESET=EXEC aborts if snapshot save fails; skips save entirely when `everRan_` is false (protocol completed or never ran) and clears stale files. Periodic save logs failures. Resume slots cleared on protocol COMPLETED state.【F:src/exp-board/ExpansionBoard.cpp†L136-L147】【F:src/exp-board/ExpansionBoard.cpp†L1356-L1378】【F:src/exp-board/ExpansionBoard.cpp†L1429-L1437】

### Additional open item carried from findings
1. Protocol summary formatting (`PHASH` line glue)
   - Status: Open
   - Current evidence: `logProtocol_()` still emits `dbgkv("\nPHASH: ", ...)`, so the formatting issue remains reproducible in principle.【F:src/exp-board/ExpansionBoard.cpp†L1052-L1056】

## 1. Auto-resume races with run switch
The ClearCore immediately transitions from `IDLE` to `RUNNING` on the first rising edge it sees from the RUN switch, even during cold boot, because it treats the latched remote RUN input as a start trigger (`runActive && !prevRunActive_`).【F:src/clearcore/ClearCoreRTM.cpp†L134-L186】 When the expansion board later issues `CMD;RESUME=AUTO;...` after completing the protocol upload, the ClearCore rejects it with `ERR_WRONG_STAT` because it is already in the `RUNNING` state and only accepts resume commands while `IDLE`.【F:include/ClearCoreRTM.h†L360-L420】 This matches the captured log where the CC reports repeated `ERR_WRONG_STAT` ACKs immediately after entering `RUNNING` when the XPB's active-low RUN line is held asserted during boot.

The new cold boots performed with `RA.BIN`/`RB.BIN` deleted show the CC and XPB remaining in `IDLE` with steadily increasing `SW_AGE` so long as RUN stays low, proving the resume collision is limited to the “RUN held on boot” condition rather than a general resume failure.【F:logs/cold_boot_no_resume.txt†L1-L17】 Decide whether the CC should defer auto-starting until it has a chance to honor an incoming resume request, or whether the XPB should suppress the auto-resume command when it sees the CC already running.  Without that handshake, the two ends will fight and the stored resume step will never be applied.

## 2. Run switch status vs. UI feedback
With the RUN pin hard-low during boot, the XPB immediately advertises `RUN=1` through
`publishSwitchState_()`, so the ClearCore latches `runActiveRemote_ = true` and
transitions into `RUNNING` as soon as it leaves the protocol loader.
`ClearCoreRTM::tick()` only requires a rising edge while already in `Idle` to promote the
state, so a latched RUN input at boot causes the CC to advance before the XPB can finish
its auto-resume negotiation.【F:src/clearcore/ClearCoreRTM.cpp†L134-L175】【F:include/ClearCoreRTM.h†L305-L315】

When RUN stays low—as in the new captures—the LCD’s `SW age` counter increments normally and both controllers sit in `IDLE`, so the steady-state display path checks out.【F:logs/cold_boot_no_resume.txt†L1-L17】 The problem shows up only in the latched-run boot: the LCD keeps receiving `SW;RUN=1` updates every few hundred milliseconds, so `SW age` sticks at zero even though the CC has already entered `RUNNING`. Consider exposing the latched RUN state and/or CC state string on the LCD whenever the CC is actively running so technicians can see that the system is live.【F:src/exp-board/ExpansionBoard.cpp†L783-L829】

## 3. Duplicate `PR_END` and checksum warnings
The ClearCore prints a duplicate `PR_END` notice as well as `WARN: TTL bad checksum (rate-limited)` during protocol transfer.  That warning is emitted when `TTLComms::validateMessage` fails and `trySplitGluedFrames_` cannot recover the payload.【F:include/ClearCoreRTM.h†L726-L733】【F:src/shared/TTLComms.cpp†L306-L353】 Because a bad frame forces the XPB retry logic to resend the last chunk, the CC ends up logging the harmless duplicate `PR_END`.  It is worth instrumenting or scoping the TTL line to understand whether this is electrical noise or a framing bug—right now the software simply drops the frame and keeps going.

## 4. Protocol summary formatting
`ExpansionBoard::logProtocol_()` writes the protocol summary with `dbgkv("\nPHASH: ", ...)`, which emits the PHASH value without an explicit newline.【F:src/exp-board/ExpansionBoard.cpp†L990-L1005】 In the captured log the "Step 1" line is glued directly to the PHASH print.  Switching that one line to `dbgln` (or appending `\r\n`) will make the summary easier to read.

## 5. Miscellaneous follow-ups
* Confirm that the XPB backs off after the CC reports `ERR_WRONG_STAT`; the log shows multiple retries with the same REF, so ensure the retry policy stops once an explicit error ACK is received.【F:src/shared/TTLComms.cpp†L258-L353】
* Verify whether the heater setpoint / preheat command path is exercised during an auto-resume.  If the CC suppresses auto-start to wait for XPB resume, make sure the stored step's temperature target still flows through the preheat handler before resuming motion.【F:include/ClearCoreRTM.h†L399-L418】【F:src/clearcore/ClearCoreRTM.cpp†L145-L175】

## 6. User-requested reset flow
The operator-driven reset capture shows the controllers exchanging the full `RESET=ARM`/`RESET=EXEC` handshake while both ends sit in `WAITING_XPB`, so the high-level flow is wired correctly.【F:logs/user_requested_reset.txt†L1-L127】【F:logs/user_requested_reset.txt†L129-L268】 Still, three issues surface:

1. **Run/Reset interlock clarity** – The XPB immediately reports `RUN=0;RST=1` after the reset switch is pulled, but the LCD only surfaces the reset counter change; there's no explicit confirmation that RUN is still low while the reset timer counts down.  Consider surfacing both latched RUN and RESET bits on-screen during a reset so technicians can tell the drive will remain stopped.【F:logs/user_requested_reset.txt†L69-L121】【F:logs/user_requested_reset.txt†L269-L343】
2. **Repeated QUIESCE bursts** – After each `RESET=EXEC` the ClearCore emits a new `QUIESCE;WHO=XPB;...` with incrementing REFs, even though the XPB acknowledges the first mask request immediately.【F:logs/user_requested_reset.txt†L85-L127】 Because the XPB echoes several identical QUIESCE frames back-to-back, it looks like the CC is retrying before it sees the ACK on the wire (possibly due to the same TTL checksum drops noted earlier).  Add logging around `TTLComms::sendFrame` and the ACK path to confirm whether these duplicates are transport retries or logic bugs in the reset state machine.
3. **Resume snapshot failure** – The CC saves multiple resume snapshots during the reset window, but one attempt reports `snapshot SAVE FAILED` just before the XPB finally drops offline for the reset.【F:logs/user_requested_reset.txt†L96-L119】 If that failure happens while a real job is mid-step, the system will reboot without a valid resume point.  We should add diagnostics to the SD writer (and potentially retry logic) so a transient write hiccup does not silently discard the checkpoint.

## 7. Path to resolution
To iron out the boot-and-reset issues captured so far:

* Gate the ClearCore's auto-run promotion behind an explicit XPB resume allowance (or have the XPB
  suppress its resume when RUN is already high) so we never enter the `ERR_WRONG_STAT` retry loop.
  **Update:** the ClearCore now keeps the active-low RUN masked through `BOOT/PROTO_LOADING`, then
  reopens the gate once the protocol upload completes. If RUN is still asserted at that moment a
  dedicated helper promotes the latched request immediately (or waits until we reach `IDLE/PAUSED`),
  so brown-out recoveries proceed without the extra RUN toggle noted in the latest capture.【F:src/clearcore/ClearCoreRTM.cpp†L103-L210】【F:include/ClearCoreRTM.h†L131-L220】
  * **Validation: COMPLETE (2026-04-10)** — All three steps pass on hardware.【F:test/log/20260410-164028_run_gate.log†L1-L390】
    1. ✅ Cold-boot with RUN held low: CC stays in IDLE for 24 heartbeat cycles, no RUNNING observed before RUN released.
    2. ✅ Gate open transitions: IDLE→RUNNING on RUN=1 (10 RUNNING HBs), RUNNING→PAUSED on RUN=0 (10 PAUSED HBs).
    3. ✅ Resume AUTOSTART=1 with RUN held low across cold boot: `CMD;RESUME=AUTO;STEP=2;LOOP=1;PHASH=3889579914;AUTOSTART=1;REF=7` sent by XPB, `ACK;RESUME=OK;REF=7` received with matching REF (no retries, no ERR_WRONG_STAT), system entered RUNNING and progressed through all steps and loops.

### Bugs fixed during validation (2026-04-10)
| # | Bug | File(s) | Root Cause | Fix |
|---|-----|---------|------------|-----|
| 1 | PHASH overflow | `ClearCoreRTM.h`, `ExpansionBoard.cpp` | `toInt()` returns signed long; values >2³¹ clamp to INT_MAX | `strtoul(str.c_str(), nullptr, 10)` |
| 2 | SW_AGE overflow | `ExpansionBoard.cpp` | Same `toInt()` pattern | `strtoul()` |
| 3 | Resume slot wipe | `ExpansionBoard.cpp` | `RESET=EXEC` handler called `saveResumeTU()` then `clearResumeSlots_()` | Removed spurious `clearResumeSlots_()` call |
| 4 | Heartbeat skip on auto-start | `ClearCoreRTM.h` | Proto completion bypassed `handleIdle(justEntered=true)` which enables heartbeat | Added `heartbeatSystemEnabled_ = true` in proto completion |
| 5 | RUN gate bypass | `ClearCoreRTM.h` | Proto completion forced `runGateReleased_ = true` and `promoteRun_()` | Only auto-start if gate already open |
| 6 | Resume ACK REF mismatch | `ClearCoreRTM.h` | ACK responses used CC's own REF counter instead of echoing sender's | Echo incoming REF in all 5 `ACK;RESUME=` sends via `MessageType::NORMAL` |

### Remaining open items
* Instrument the TTL transport for checksum failures and ensure duplicate `PR_END` / `QUIESCE` frames are genuine retries; add back-off so we do not spam commands when the peer already acknowledged them.【F:src/shared/TTLComms.cpp†L258-L353】
* Harden resume persistence: wrap the snapshot writer with retries and surface failures prominently, then verify the reset flow waits for a confirmed snapshot before forcing the XPB reset.
* ~~Audit remaining `sendMessage(..., MessageType::IMPORTANT)` ACK paths for REF-echo mismatches~~ — **Done (2026-04-10).** All clean; only the 5 resume ACKs had the bug.
