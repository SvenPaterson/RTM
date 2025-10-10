# Boot-Up Serial Review Findings

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
  **Update:** the ClearCore now treats the XPB's active-low RUN input as gated until it either sees
  the line released high or the XPB issues a resume/autostart, so a latched low no longer promotes
  the state machine while the XPB is still uploading the protocol.【F:src/clearcore/ClearCoreRTM.cpp†L103-L205】【F:include/ClearCoreRTM.h†L131-L200】【F:logs/cold_boot_no_resume.txt†L1-L17】
  * **Validation plan:**
    1. Cold-boot both controllers with the XPB RUN pin held low (call-for-run) and confirm the CC
       stays in `BOOT/PROTO_LOADING` while logging `[RUN] Ignoring RUN line held low before XPB
       resume` until the line is released high or a `RESUME AUTOSTART=1` arrives; this exercises the
       `runGateReleased_` guard reset in `handleBoot()`.【F:src/clearcore/ClearCoreRTM.cpp†L116-L205】
    2. After the XPB handshake completes, momentarily release RUN high and drive it low again to
       verify the controller transitions from `IDLE` into `PREHEAT/RUN`, proving the gate opens once
       the active-low line has been seen high.【F:src/clearcore/ClearCoreRTM.cpp†L134-L175】
    3. From `IDLE`, send `RESUME AUTOSTART=1` while keeping RUN asserted low and confirm the
       ClearCore accepts the resume and advances only after the XPB command; this covers the resume
       handler overriding the gate for coordinated auto-starts.【F:include/ClearCoreRTM.h†L360-L413】
* Surface RUN/RESET latch state on the LCD whenever the controller is not idle, and make the switch-age timer freeze explicitly signal "RUN held" so operators know why the system started without interaction.【F:src/exp-board/ExpansionBoard.cpp†L783-L829】【F:logs/user_requested_reset.txt†L69-L127】
* Instrument the TTL transport for checksum failures and ensure duplicate `PR_END` / `QUIESCE` frames are genuine retries; add back-off so we do not spam commands when the peer already acknowledged them.【F:src/shared/TTLComms.cpp†L258-L353】【F:logs/user_requested_reset.txt†L85-L127】
* Harden resume persistence: wrap the snapshot writer with retries and surface failures prominently, then verify the reset flow waits for a confirmed snapshot before forcing the XPB reset.【F:logs/user_requested_reset.txt†L96-L119】

These are the main items that stood out when comparing the two boot logs with the current firmware.
