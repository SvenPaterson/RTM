


// ---------------------------------------------------------------------------
// Send protocol ready notice to ClearCore (PHASH, steps, loops)
// ---------------------------------------------------------------------------


#include "ExpansionBoard.h"
#include <stdio.h>

void ExpansionBoard::sendProtoReady_() {
    if (stepCount_ == 0) return;
    char line[96];
    snprintf(line, sizeof(line),
        "NOTICE;PROTO_READY;NAME=%s;PHASH=%lu;STEPS=%u;LOOPS=%u",
        protocolName_, (unsigned long)progHash_, (unsigned)stepCount_, (unsigned)loopCount_);
    comms_.sendMessage(line, MessageType::NORMAL);
}
#include "ExpansionBoard.h"
#include <Arduino.h>
#include <SPI.h>

#define XPB_INJECT_FROM_USB 0

namespace {  // anonymous namespace: TU-private helpers for Resume logic
    // 8.3-safe filenames for Arduino SD
    static constexpr const char *kSlotA = "/RA.BIN";
    static constexpr const char *kSlotB = "/RB.BIN";
    static constexpr const char *kTmpA  = "/RA.TMP";
    static constexpr const char *kTmpB  = "/RB.TMP";

    struct ResumeRec {
    uint32_t magic;      // 'XPBR'
    uint16_t version;    // 1
    uint16_t reserved;
    uint32_t seq;
    uint32_t phash;
    uint16_t step;
    uint16_t loopCur;
    uint16_t loopTot;
    uint16_t flags;
    uint32_t crc32;      // computed with this field = 0
    };

    /**
     * @brief Compute CRC-32 (Ethernet/ZIP) over a buffer.
     * @param data Pointer to input bytes.
     * @param len  Number of bytes.
     * @return CRC-32 of the buffer.
     * @note Used by resume slot integrity and PHASH computation.
     */
    uint32_t crc32_calc(const void *data, size_t len) {
        const uint8_t *p = static_cast<const uint8_t*>(data);
        uint32_t crc = 0xFFFFFFFFu;
        for (size_t i = 0; i < len; ++i) {
            crc ^= p[i];
            for (int k = 0; k < 8; ++k)
            crc = (crc >> 1) ^ (0xEDB88320u & (-(int)(crc & 1)));
        }
        return ~crc;
    }

    /**
     * @brief Atomically write a resume record to a slot file.
     * @param path Destination path (e.g., "/resumeA.bin").
     * @param rec  Resume record (crc32 field will be filled in).
     * @return true on success.
     * @details Writes to a temporary file then replaces the destination to avoid
     *          torn writes. Validates sizes and flushes.
     */
    bool writeSlot(const char *finalPath, const char *tmpPath, ResumeRec rec) {
        rec.crc32 = 0;
        rec.crc32 = crc32_calc(&rec, sizeof(rec));

        File f = SD.open(tmpPath, FILE_WRITE);       // create temp (8.3)
        if (!f) return false;
        bool ok = (f.write(reinterpret_cast<const uint8_t*>(&rec), sizeof(rec)) == sizeof(rec));
        f.flush(); f.close();
        if (!ok) { SD.remove(tmpPath); return false; }

        SD.remove(finalPath);                         // remove old final (if any)

        File src = SD.open(tmpPath, FILE_READ);
        if (!src) { SD.remove(tmpPath); return false; }
        File dst = SD.open(finalPath, FILE_WRITE);    // create final (append mode but file is new)
        if (!dst) { src.close(); SD.remove(tmpPath); return false; }

        uint8_t buf[64];
        int n;
        while ((n = src.read(buf, sizeof(buf))) > 0) dst.write(buf, n);
        dst.flush(); dst.close(); src.close();
        SD.remove(tmpPath);
        return true;
    }


    /**
     * @brief Read and verify a resume record from a slot.
     * @param path Source path (e.g., "/resumeA.bin").
     * @param out  Output resume record (set only if valid).
     * @return true if a record exists and CRC is valid.
     */
    bool readSlot(const char *path, ResumeRec &out) {
        File f = SD.open(path, FILE_READ);
        if (!f) return false;
        if (f.size() != (int)sizeof(ResumeRec)) { f.close(); return false; }
        ResumeRec r;
        bool ok = (f.read(reinterpret_cast<uint8_t*>(&r), sizeof(r)) == sizeof(r));
        f.close(); if (!ok) return false;
        uint32_t saved = r.crc32; r.crc32 = 0;
        if (saved != crc32_calc(&r, sizeof(r))) return false;
        out = r; return true;
    }

    /**
     * @brief Save the newest resume record, alternating between A/B slots.
     * @param phash   Program hash for identity checking.
     * @param step    Current step index (0-based).
     * @param loopCur Current loop (0-based or 1-based per protocol; mirror of CC field).
     * @param loopTot Total loops.
     * @return true on success.
     * @details Increments a sequence number and chooses the older slot to overwrite.
     */
    bool saveResumeTU(uint32_t phash, uint16_t step, uint16_t loopCur, uint16_t loopTot,
                       uint8_t maxRetries = 1) {
        ResumeRec a{}, b{};
        bool ha = readSlot(kSlotA, a), hb = readSlot(kSlotB, b);

        ResumeRec rec{}; rec.magic=0x58504252u; rec.version=1;
        rec.seq = (ha||hb) ? ( (ha?a.seq:0) > (hb?b.seq:0) ? (a.seq+1) : (b.seq+1) ) : 1;
        rec.phash = phash; rec.step = step; rec.loopCur = loopCur; rec.loopTot = loopTot; rec.flags = 0;

        // Overwrite the older slot; write via matching temp name
        const bool writeB = (ha && (!hb || a.seq <= b.seq));
        const char *finalPath = writeB ? kSlotB : kSlotA;
        const char *tmpPath   = writeB ? kTmpB  : kTmpA;

        for (uint8_t attempt = 0; attempt <= maxRetries; ++attempt) {
            if (writeSlot(finalPath, tmpPath, rec)) return true;
            if (attempt < maxRetries) delay(10);
        }
        return false;
    }

    /**
     * @brief Load the latest valid resume record from A/B slots.
     * @param phash   Out: program hash from record.
     * @param step    Out: step index.
     * @param loopCur Out: loop current.
     * @param loopTot Out: loop total.
     * @return true if any valid slot exists.
     */
    bool loadResumeLatestTU(uint32_t &phash, uint16_t &step, uint16_t &loopCur, uint16_t &loopTot) {
        ResumeRec a{}, b{};
        bool ha = readSlot(kSlotA, a), hb = readSlot(kSlotB, b);
        if (!ha && !hb) return false;
        const ResumeRec &r = (!hb || (ha && a.seq >= b.seq)) ? a : b;
        phash = r.phash; step = r.step; loopCur = r.loopCur; loopTot = r.loopTot;
        return true;
    }

    /**
     * @brief Clear both resume slots from SD.
     */
    void clearResumeTU() {
        SD.remove(kSlotA);
        SD.remove(kSlotB);
        SD.remove(kTmpA);
        SD.remove(kTmpB);
    }

    // ============================================================================
    // Post-reset UI mask flag (survives reboot; 8.3 filename)
    // ============================================================================

    /**
     * @brief 8.3 path for a one-shot "intentional XPB reset" flag.
     * @details When present at boot, the UI will suppress the E-STOP page for a
     *          short grace window and show "Resetting" instead. The flag is
     *          consumed (deleted) on first check to behave as one-shot.
     */
    static constexpr const char *kXpbResetFlag = "/XR.BIN";

    /**
     * @brief Create the one-shot "intentional XPB reset" flag on SD.
     * @return true on success, false if SD write failed.
     * @details The content is irrelevant; existence is the signal.
     */
    bool writeResetFlagTU(uint8_t maxRetries = 2) {
        for (uint8_t attempt = 0; attempt <= maxRetries; ++attempt) {
            File f = SD.open(kXpbResetFlag, FILE_WRITE);
            if (!f) { if (attempt < maxRetries) delay(10); continue; }
            uint32_t tag = 0x21505842u; // "!XPB"
            bool ok = (f.write(reinterpret_cast<const uint8_t*>(&tag), sizeof(tag)) == sizeof(tag));
            f.flush();
            f.close();
            if (ok) { delay(12); return true; }
            if (attempt < maxRetries) delay(10);
        }
        return false;
    }


    /**
     * @brief Consume (detect and delete) the one-shot reset flag.
     * @return true if the flag existed and was removed; false otherwise.
     */
    bool consumeResetFlagTU() {
    File f = SD.open(kXpbResetFlag, FILE_READ);
    if (!f) return false;
    f.close();
    SD.remove(kXpbResetFlag);
    return true;
    }
}

const char *ExpansionBoard::protoStateName_(ProtoTxState s) const {
    switch (s) {
        case ProtoTxState::Idle:        return "Idle";
        case ProtoTxState::WaitingReq:  return "WaitingReq";
        case ProtoTxState::SDFail:      return "SDFail";
        case ProtoTxState::BegSent:     return "BegSent";
        case ProtoTxState::Sending:     return "Sending";
        case ProtoTxState::EndSent:     return "EndSent";
        case ProtoTxState::AwaitResult: return "AwaitResult";
        case ProtoTxState::Complete:    return "Complete";
        case ProtoTxState::Failed:      return "Failed";
        case ProtoTxState::Timeout:     return "Timeout";
    }
    return "Unknown";
}

void ExpansionBoard::setProtoState_(ProtoTxState next, const char *reason) {
    if (protoState_ == next) {
        return;
    }
#if XPB_PROTO_TRACE
    const ProtoTxState prev = protoState_;
    char line[128];
    snprintf(line, sizeof(line),
             "[PROTO] %s -> %s reason=%s",
             protoStateName_(prev),
             protoStateName_(next),
             reason ? reason : "n/a");
    if (Serial) {
        Serial.println(line);
    }
#endif
    protoState_ = next;
    if (next == ProtoTxState::WaitingReq) {
        protoReadyTmr_ = 0;
    }
}

/**
 * @brief Initialize all board subsystems and start comms/UI.
 * @copydetails ExpansionBoard::begin()
 */
bool ExpansionBoard::begin() {
    // 115200 keeps XPB serial consistent with XPB_DEBUG and rig_trace's
    // default --xpb-baud, so observer captures don't silently garble.
    Serial.begin(115200);

    // ---- USB console attach policy ----
    // Default: no wait. Nano Every auto-resets on USB/DTR anyway.
    // If you use a terminal that does NOT toggle DTR and want early logs, build with -DXPB_WAIT_USB_MS=2000.
    #ifndef XPB_WAIT_USB_MS
    #define XPB_WAIT_USB_MS 0
    #endif
    if (XPB_WAIT_USB_MS > 0) {
        unsigned long t0 = millis();
        while (!Serial && (millis() - t0 < XPB_WAIT_USB_MS)) { /* spin */ }
    }
#if XPB_PROTO_TRACE
    if (Serial) Serial.println("[BOOT] serial ready");
#endif
    dbgln("\nUSB Serial Monitor Connected!");

    // UART to ClearCore (not on SPI) – safe to bring up early
#if XPB_PROTO_TRACE
    if (Serial) Serial.println("[COMMS] begin enter");
#endif
    comms_.begin();
#if XPB_PROTO_TRACE
    if (Serial) Serial.println("[COMMS] begin exit");
#endif
    delay(200);
    dbgln("Connecting with CC..");
    // Defer QUIESCE until tick() sees CC traffic. Sending here has
    // occasionally blocked during early boot while link/socket settle.
#if XPB_PROTO_TRACE
    if (Serial) Serial.println("[BOOT] quiesce queued");
#endif
    quiesceSecsPending_ = 15;
    quiesceSent_ = false;

    // --- SPI bus & SD first (prevents other devices from holding MISO) ---
    spiQuiesceAll_();
#if XPB_PROTO_TRACE
    if (Serial) Serial.println("[BOOT] sd init start");
#endif
    dbgln("[XPB] Starting SD init (cold boot)");
    bool sdOk = sdInitWithRetry_();  // default: 10 tries, 100ms backoff (header)
    dbgln(sdOk ? "[XPB] SD init OK (cold boot)" : "[XPB] SD init FAIL (cold boot)");
#if XPB_PROTO_TRACE
    if (Serial) Serial.println(sdOk ? "[BOOT] sd init ok" : "[BOOT] sd init fail");
#endif

    // --- E-STOP UI mask on boot and after intentional XPB reset ---
    if ((int32_t)(millis() - estopUiMaskUntilMs_) >= 0) {
        estopUiMaskUntilMs_ = millis() + 2500UL;
        dbgln("[UI] E-STOP boot mask 2.5s");
    }
    if (consumeResetFlagTU()) {
        estopUiMaskUntilMs_ = millis() + 8000UL;
        dbgln("[UI] E-STOP mask active (post-XPB reset)");
    }

    // LCD AFTER SD so the LCD CS can't hold MISO low during SD init
    if (!lcd_.begin()) {
#if XPB_PROTO_TRACE
        if (Serial) Serial.println("[BOOT] lcd init fail");
#endif
        dbgln("FATAL: LCD initialization failed!");
        return false;
    }
#if XPB_PROTO_TRACE
    if (Serial) Serial.println("[BOOT] lcd init ok");
#endif
    lastUi_ = static_cast<UiPage>(0xFF);

    // Switches
    runSw_.attach(RUN_SW_PIN_, INPUT_PULLUP);   runSw_.interval(25);
    resetSw_.attach(RESET_SW_PIN_, INPUT_PULLUP); resetSw_.interval(25);

    // Sensors
    dbg("Initializing MAX31855 sensor - TC1...");
    delay(500); // stabilize
    tc1_.begin();
    dbgln("DONE");

    dbg("Initializing MAX31855 sensor - TC2...");
    delay(250);
    tc2_.begin();
    dbgln("DONE");

    // Protocol + Resume (only if SD OK)
    if (sdOk) {
#if XPB_PROTO_TRACE
        if (Serial) Serial.println("[BOOT] proto load start");
#endif
        successfulProtoLoadFromSD_ = loadProtocolFromSD_("/protocol.csv");
        if (!successfulProtoLoadFromSD_) {
            dbgln("Protocol load FAILED");
#if XPB_PROTO_TRACE
            if (Serial) Serial.println("[BOOT] proto load fail");
#endif
        } else {
            dbgln("Protocol loaded OK");
#if XPB_PROTO_TRACE
            char pl[96];
            snprintf(pl, sizeof(pl),
                     "[BOOT] proto ok steps=%u loops=%lu",
                     (unsigned)stepCount_,
                     (unsigned long)loopCount_);
            if (Serial) Serial.println(pl);
#endif
            logProtocol_();   // optional debug dump
        }

        // Optional slot dump for visibility
        auto dumpSlots = [&](){
            File fa = SD.open(kSlotA), fb = SD.open(kSlotB);
            if (fa) { dbgkv("[SD] RA.BIN bytes=", (unsigned long)fa.size()); fa.close(); }
            else     dbgln("[SD] RA.BIN missing");
            if (fb) { dbgkv("[SD] RB.BIN bytes=", (unsigned long)fb.size()); fb.close(); }
            else     dbgln("[SD] RB.BIN missing");
        };
        dumpSlots();

        // Load resume data if it exists
        haveStoredResume_ = loadResumeLatestTU(storedPhash_, storedStep_, storedLoopCur_, storedLoopTot_);
        if (haveStoredResume_) {
            if (storedPhash_ != progHash_) {
                dbgln("Resume record PHASH mismatch -> clearing");
                clearResumeTU();
                haveStoredResume_ = false;
            } else {
                char rb[64];
                snprintf(rb, sizeof(rb), "Resume available: step=%u loop=%u",
                        (unsigned)storedStep_, (unsigned)storedLoopCur_);
                dbgln(rb);
            }
        }

        // NOTE: virgin-resume scrub removed.  Resume is only saved when
        // everRan_ is true (STATE=RUNNING seen), so a step=1/loop=1
        // record IS legitimate if the power was lost during step 1.
        // Stale records from prior runs are already cleared on COMPLETED
        // or manual RESET;EXEC.

        // === ALWAYS UPLOAD PROTOCOL TO CC AFTER BOOT ===
        // === DO NOT AUTO-UPLOAD FROM HERE ===
        if (stepCount_ > 0) {
            dbgln("[PROTOCOL] Standing by for CC REQ:PROTO");
            setProtoState_(ProtoTxState::WaitingReq, "BOOT_READY_WAIT_REQ");
#if XPB_PROTO_TRACE
            if (Serial) Serial.println("[BOOT] wait REQ:PROTO");
#endif
            //isProtoLoadedOntoCC_ = false;

            // Decide now whether we should auto-resume AFTER CC requests + receives protocol
            if (haveStoredResume_ && storedPhash_ == progHash_) {
                // scrub virgin resume as you already do, then:
                needResumeAfterProto_ = true;

                // optional: log intent
                char rb[96];
                snprintf(rb, sizeof(rb),
                        "[RESUME] Will auto-resume after CC completes protocol load: step=%u loop=%u",
                        (unsigned)storedStep_, (unsigned)storedLoopCur_);
                dbgln(rb);
            } else {
                dbgln("[RESUME] No valid stored resume or PHASH mismatch");
                needResumeAfterProto_ = false;
            }

            // Announce protocol to CC for PHASH drift detection
            sendProtoReady_();
#if XPB_PROTO_TRACE
            if (Serial) Serial.println("[BOOT] proto ready sent");
#endif
        } else {
            dbgln("No protocol loaded from SD - nothing to upload");
            setProtoState_(ProtoTxState::SDFail, "BOOT_NO_PROTOCOL");
            needResumeAfterProto_ = false;
#if XPB_PROTO_TRACE
            if (Serial) Serial.println("[BOOT] no protocol cached");
#endif
        }


    } else {
        dbgln("SD init failed - no protocol available");
#if XPB_PROTO_TRACE
        if (Serial) Serial.println("[BOOT] skip proto (sd fail)");
#endif
    }

    ccAnySeen_ = false;
    quiesceSent_ = false;
    warnedNoLink_ = false;
    linkState_ = LinkState::NoLink;
    linkDownActive_ = false;
    linkDownSinceMs_ = 0;
    linkReinitCount_ = 0;
    sinceBoot = 0;
    dbgln("Awaiting CC traffic...");
#if XPB_PROTO_TRACE
    if (Serial) Serial.println("[BOOT] begin done");
#endif

    // PID lives on CC now; XPB only reports temps and caches setpoint for LCD.
    lastSpC_ = 0;

    publishSwitchState_(true);

    sinceBoot = 0;

    return true;
}

// ---------------------------------------------------------------------------
// logicalReset — in-place state reset (replaces MCU hard reset)
// ---------------------------------------------------------------------------
void ExpansionBoard::logicalReset() {
    dbgln("[RESET] Logical reset starting");

    // --- Boot & reset UI ---
    resetUiActive_    = false;
    resetUiSecs_      = 0;
    resetUiTmr_       = 0;
    resetUiRemaining_ = 0;
    ccReady_          = false;
    ccAnySeen_        = false;
    sinceBoot         = 0;
    warnedNoLink_     = false;
    linkState_        = LinkState::NoLink;
    linkDownActive_   = false;
    linkDownSinceMs_  = 0;
    linkReinitCount_  = 0;

    // --- CC heartbeat mirror ---
    ccHbAgeTmr_    = 0;
    ccHbSeen_      = false;
    hbSeq_         = 0;
    ccAlarmActive_ = false;
    memset(ccAlarmMsg_, 0, sizeof(ccAlarmMsg_));
    strcpy(ccState_, "IDLE");
    ccStep_      = 0;
    ccLoopCur_   = 0;
    ccLoopTot_   = 0;
    ccSwAgeMs_   = 0;
    ccRpm_       = 0;
    ccEstop_     = false;
    ccEstopCode_ = 0;

    // --- Protocol TX state -------------------------------------------
    // Resume contract:
    //   * Cold boot (power loss / first plug-in)  -> RA/RB consulted to
    //     resume the in-flight step+loop; CSV re-read happens in begin().
    //   * Manual RST (this path)                  -> RA/RB are wiped by
    //     the EXEC handler before this function runs, AND we re-read
    //     /protocol.csv here so an operator who swapped the SD card
    //     before pressing RST gets the new protocol on the next boot
    //     cycle without needing to fully power-cycle the rig.
    setProtoState_(ProtoTxState::WaitingReq, "LOGICAL_RESET");
    protoSince_    = 0;
    lastProtoRef_  = 0;
    protoStepSent_ = 0;
    ccProtoReq_    = false;
    needResumeAfterProto_ = false;
    targetMet_     = false;
    everRan_       = false;

    {
        // Re-init SD before re-reading: the cold-boot path quiesces SPI
        // and re-runs SD.begin() first; without this, loadProtocolFromSD_
        // can fail spuriously after the post-reset SPI churn (LCD, TCs,
        // Ethernet), surfacing as "Protocol Missing on SD Card".
        spiQuiesceAll_();
        const bool sdOk = sdInitWithRetry_(3, 40);
        const bool reloaded = sdOk && loadProtocolFromSD_("/protocol.csv");
        successfulProtoLoadFromSD_ = reloaded;
        if (reloaded) {
            dbgln("[RESET] Protocol re-loaded from SD");
            logProtocol_();
        } else {
            dbgln(sdOk
                  ? "[RESET] Protocol RE-LOAD FAILED (CSV parse) — SD recovery loop will retry"
                  : "[RESET] SD re-init FAILED — SD recovery loop will retry");
            // stepCount_/protocolName_/progHash_ have been zeroed by the
            // failed parse; the recovery timer in tick() will retry.
        }
    }

    // --- Resume tracking (already cleared by EXEC handler, reinforce) ---
    haveStoredResume_ = false;
    storedPhash_      = 0;
    storedStep_       = 0;
    storedLoopCur_    = 0;
    storedLoopTot_    = 0;

    // --- Preheat / heater ---
    preheatActive_ = false;
    preheatSpC_    = 0;
    preheatTmr_    = 0;
    lastSpC_       = 0;

    // --- USB sim ---
    usbSimHold_        = false;
    usbSimHoldUntilMs_ = 0;
    usbInjecting_      = false;

    // --- Step countdown ---
    countdownStepSnapshot_ = 0;
    countdownLoopSnapshot_ = 0;
    stepStartAgeMs_        = 0;
    stepTotalMs_           = 0;
    stepRemainingMs_       = 0;

    // --- Sensors (keep TC handles, reset readings) ---
    dataTmr_     = 0;
    latestSealC_ = NAN;
    latestSumpC_ = NAN;

    // --- UI ---
    lcdToggle_        = false;
    modeTorqueToggle_ = false;
    lcdTmr_           = 0;
    lastUi_           = static_cast<UiPage>(0xFF);   // force redraw
    bootPhase_        = BootPhase::Done;
    bootMsgSince_     = 0;

    // --- Switch ---
    lastSwPublishMs_ = 0;

    // --- Timers ---
    heartbeatTmr_ = 0;
    sdRecoveryTmr_ = 0;

    // --- Comms ---
    comms_.resetState();

    // --- Post-reset actions ---
    // E-STOP UI mask (same as post-XPB-reset-flag path in begin())
    estopUiMaskUntilMs_ = millis() + 8000UL;

    // Defer reset QUIESCE until we observe CC traffic in tick().
    quiesceSecsPending_ = 10;
    quiesceSent_ = false;

    // Immediately publish switch state so CC gets fresh RUN/RST
    publishSwitchState_(true);

    dbgln("[RESET] Logical reset complete — awaiting CC");
}

/**
 * @brief Periodic task driving comms, IO, sensors, PID, heartbeat, and UI.
 * @copydetails ExpansionBoard::tick()
 * @note Supports test injection via USB when XPB_INJECT_FROM_USB is defined.
 */
void ExpansionBoard::tick() {
    // ----- Comms housekeeping -----
#if XPB_PROTO_TRACE
    if (Serial) Serial.print('T');
#endif
    comms_.checkForMessages();
#if XPB_PROTO_TRACE
    if (Serial) Serial.print('M');
#endif
    comms_.checkRetries();
#if XPB_PROTO_TRACE
    if (Serial) Serial.println('R');
#endif

    // Soft "no link yet" note after 10s with no CC traffic at all
    if (!ccAnySeen_ && !warnedNoLink_ && sinceBoot > 10000) {
        linkState_ = LinkState::NoLink;
        dbgln("INFO: No CC traffic yet (>10s). Continuing without link.");
#if XPB_PROTO_TRACE
        if (Serial) Serial.println("[LINK] no CC traffic >10s");
#endif
    }
    linkState_ = ccAnySeen_ ? LinkState::Alive : LinkState::NoLink;

        // Send deferred QUIESCE only after CC traffic is visible.
        // This avoids early-boot TX blocking while still preserving the
        // intent of suppressing stale-STAT E-STOP once link is active.
        if (quiesceSecsPending_ > 0 && ccAnySeen_ && !quiesceSent_) {
        char line[32];
        snprintf(line, sizeof(line), "QUIESCE;SECS=%u", (unsigned)quiesceSecsPending_);
    #if XPB_PROTO_TRACE
        if (Serial) Serial.println("[BOOT] quiesce deferred send");
    #endif
        comms_.sendCommand(line, MessageType::IMPORTANT);
        quiesceSent_ = true;
        quiesceSecsPending_ = 0;
    #if XPB_PROTO_TRACE
        if (Serial) Serial.println("[BOOT] quiesce deferred sent");
    #endif
        }

#if XPB_PROTO_TRACE
    static elapsedMillis traceBeatMs;
    if (traceBeatMs > 2000) {
        traceBeatMs = 0;
        char hb[112];
        snprintf(hb, sizeof(hb),
                 "[TICK] sb=%lu st=%s cc=%u rx=%lu gate=%u",
                 (unsigned long)sinceBoot,
                 protoStateName_(protoState_),
                 (unsigned)(ccAnySeen_ ? 1U : 0U),
                 (unsigned long)comms_.rxAgeMs(),
                 (unsigned)(ccProtoReq_ ? 1U : 0U));
        if (Serial) Serial.println(hb);
    }
#endif

    // While waiting for CC's REQ:PROTO, periodically re-announce protocol
    // metadata so a dropped startup notice cannot leave the handoff silent.
    if (protoState_ == ProtoTxState::WaitingReq && stepCount_ > 0 &&
        protoReadyTmr_ >= kProtoReadyResendMs) {
        protoReadyTmr_ = 0;
        sendProtoReady_();
#if XPB_PROTO_TRACE
        if (Serial) Serial.println("[PROTO] periodic PROTO_READY");
#endif
    }

    // ----- USB injection / commands -----
    #if XPB_INJECT_FROM_USB
    {
        static String usbLine; // accumulates a full line
        while (Serial && Serial.available() > 0) {
            char ch = (char)Serial.read();
            if (ch == '\r') continue;                // ignore CR
            if (ch == 8 || ch == 127) {              // backspace / DEL
                if (usbLine.length()) usbLine.remove(usbLine.length()-1);
                continue;
            }
            if (ch != '\n') { usbLine += ch; continue; } // collect until LF

            // Got a full line
            usbLine.trim();
            if (usbLine.length()) {
                if (usbLine.startsWith("SIM=")) {
                    String arg = usbLine.substring(4); arg.trim();
                    if (arg.equalsIgnoreCase("0") || arg.equalsIgnoreCase("OFF")) {
                        usbSimHold_ = false;
                        dbg("[USB] SIM hold OFF");
                    } else if (arg.equalsIgnoreCase("INF") || arg.equalsIgnoreCase("*") || arg.equalsIgnoreCase("ON")) {
                        usbSimHold_ = true;
                        usbSimHoldUntilMs_ = 0; // infinite
                        dbg("[USB] SIM hold ON (infinite)");
                    } else {
                        long sec = arg.toInt();
                        if (sec > 0) {
                            usbSimHold_ = true;
                            usbSimHoldUntilMs_ = millis() + (uint32_t)sec * 1000UL;
                            dbgkv("[USB] SIM hold ON for secs=", sec);
                        } else {
                            dbg("[USB] SIM usage: SIM=OFF | SIM=INF | SIM=<seconds>");
                        }
                    }
                }
                else if (usbLine.charAt(0) == '>') {
                    // TEMPORARY HANDLING OF PR_BEG etc. TO TEST SYSTEM
                    String frame = usbLine.substring(1);  // drop '>'
                    
                    // PR_* messages go TO ClearCore, not processed locally
                    if (frame.startsWith("PR_")) {
                        // Send over the wire to ClearCore
                        comms_.sendCommand(frame.c_str(), MessageType::CRITICAL);
                        dbgkv("[USB->CC] Sending: ", frame);
                    } 
                    // Everything else is injected locally for testing
                    else {
                        usbInjecting_ = true;
                        comms_.onMessageReceived(frame);   // call directly so we can mark it as injected
                        usbInjecting_ = false;
                        dbgkv("[USB INJECT] ", frame);
                    }
                }
                else if (usbLine.equalsIgnoreCase("SIM?")) {
                    if (Serial) {
                        dbg("[USB] SIM hold ");
                        dbg(usbSimHold_ ? "ON" : "OFF");
                        dbg(", until=");
                        Serial.println(usbSimHoldUntilMs_, HEX);
                    }
                }
                else if (usbLine.equalsIgnoreCase("UPLOAD")) {
                    dbgln("[USB] Triggering protocol upload...");
                    if (uploadProtocolToCC_()) {
                        dbgln("[USB] Upload completed");
                    } else {
                        dbgln("[USB] Upload failed");
                    }
                }
                else {
                    dbgkv("[USB] ignored: ", usbLine);
                }
            }
            usbLine = ""; // reset for next line
        }
    }
    #endif

    // ----- SWITCH PUBLISH w/ 60s keep-alive -----
    runSw_.update(); resetSw_.update();
    if (runSw_.changed() || resetSw_.changed()) publishSwitchState_();
    else if (millis() - lastSwPublishMs_ > 60000UL) publishSwitchState_(true);

    // ----- LINK-LOSS RECOVERY -----
    // While CC has been silent (>3s, same threshold as the LostComms LCD),
    // periodically tear down + re-init the W5500/UDP socket. WIZnet sockets
    // can soft-lock when the L2 link drops underneath an open socket
    // (router reboot, cable bounce). The reinit is the only way to recover
    // without power-cycling the board.
    const uint32_t rxAgeMs = comms_.rxAgeMs();
    if (!linkDownActive_ && rxAgeMs > 3000U) {
        linkDownActive_ = true;
        linkDownSinceMs_ = millis();
        linkReinitCount_ = 0;

        char line[112];
        snprintf(line, sizeof(line),
                 "NOTICE;LINK=DOWN;WHO=XPB;AGE=%lu;OBS=%u",
                 (unsigned long)rxAgeMs,
                 (unsigned)(comms_.observerActive() ? 1U : 0U));
        comms_.sendMessage(line, MessageType::IMPORTANT);
        dbgln("[LINK] DOWN (XPB): CC RX stale >3s");
    } else if (linkDownActive_ && rxAgeMs <= 3000U) {
        const uint32_t downMs = millis() - linkDownSinceMs_;
        char line[112];
        snprintf(line, sizeof(line),
                 "NOTICE;LINK=UP;WHO=XPB;DOWN_MS=%lu;REINITS=%u",
                 (unsigned long)downMs,
                 (unsigned)linkReinitCount_);
        comms_.sendMessage(line, MessageType::IMPORTANT);
        dbgln("[LINK] UP (XPB): CC RX restored");

        linkDownActive_ = false;
        linkDownSinceMs_ = 0;
        linkReinitCount_ = 0;
    }

    if (rxAgeMs > 3000U) {
        if (linkRecoveryTmr_ > 5000U) {
            linkRecoveryTmr_ = 0;
            if (linkDownActive_) {
                ++linkReinitCount_;
            }
            dbgln("[LINK] Recovery watchdog -- reinit UDP");
            comms_.reinitUdp();
        }
    } else {
        linkRecoveryTmr_ = 0;
    }
    // Operator escape hatch: pressing RESET while the link is silent forces
    // an immediate reinit. Useful when the watchdog cadence is too slow or
    // the operator needs to confirm intent. No effect when link is healthy
    // (CC handles RESET via the published switch state as normal).
    if (resetSw_.fell() && rxAgeMs > 3000U) {
        if (linkDownActive_) {
            ++linkReinitCount_;
        }
        dbgln("[LINK] RESET button -- manual UDP reinit");
        comms_.reinitUdp();
        linkRecoveryTmr_ = 0;
    }
    
    // ----- SENSORS / CONTROL -----
    updateData();  // MAX31855, etc.

    // Reset countdown ETA at 1 Hz (non-blocking)
    if (resetUiActive_ && resetUiTmr_ >= 1000) {
        resetUiTmr_ = 0;
        if (resetUiRemaining_ > 0) --resetUiRemaining_;
    }

    // STAT heartbeat to ClearCore
    if (heartbeatTmr_ >= 1000) {
        heartbeatTmr_ = 0;
        char line[64];
        const int tempC = isnan(latestSumpC_) ? 0 : (int)latestSumpC_;
        const int sealC = isnan(latestSealC_) ? 0 : (int)latestSealC_;
        snprintf(line, sizeof(line), "STAT;SEQ=%u;SUMP=%d;SEAL=%d",
                 hbSeq_++, tempC, sealC);
        comms_.sendMessage(line, MessageType::INFO);
    }

    // --- Non-blocking SD recovery (runs only when boot SD init failed) ---
    if (!successfulProtoLoadFromSD_ && stepCount_ == 0 && sdRecoveryTmr_ >= 2000) {
        sdRecoveryTmr_ = 0;
        dbgln("[SD] Recovery attempt (soft reset)...");
        spiQuiesceAll_();
        dbgln("[XPB] Starting SD init (soft reset)");
        bool sdOk = sdInitWithRetry_(3, 40); // much faster for soft reset recovery
        dbgln(sdOk ? "[XPB] SD init OK (soft reset)" : "[XPB] SD init FAIL (soft reset)");
        if (sdOk) {
            dbgln("[SD] Recovery: card init OK");
            if (loadProtocolFromSD_("/protocol.csv")) {
                successfulProtoLoadFromSD_ = true;
                setProtoState_(ProtoTxState::WaitingReq, "SD_RECOVERY_OK");
                ccProtoReq_ = false;  // allow next REQ:PROTO to trigger upload
                dbgln("[SD] Recovery: protocol loaded");
            } else {
                dbgln("[SD] Recovery: CSV parse failed");
            }
        }
    }

    // --- Protocol timeouts ---
    if (protoState_ == ProtoTxState::BegSent ||
        protoState_ == ProtoTxState::EndSent ||
        protoState_ == ProtoTxState::AwaitResult) {
        if (protoSince_ > kProtoAckTimeoutMs_) {
            setProtoState_(ProtoTxState::Timeout, "PROTO_ACK_TIMEOUT");
        }
    }
    else if (protoState_ == ProtoTxState::Sending) {
        if (protoSince_ > kProtoSilenceTimeoutMs) {
            setProtoState_(ProtoTxState::Timeout, "PROTO_SILENCE_TIMEOUT");
        }
    }

    // ----- UI DECISION (exactly one page per tick) -----
    UiPage page = UiPage::Normal;
    const bool protoBusy =
        (protoState_ == ProtoTxState::BegSent)  ||
        (protoState_ == ProtoTxState::Sending)  ||
        (protoState_ == ProtoTxState::EndSent)  ||
        (protoState_ == ProtoTxState::AwaitResult);

    if (protoState_ == ProtoTxState::Failed || protoState_ == ProtoTxState::Timeout) {
        page = UiPage::ProtoTxFail;
    }
    else if (ccEstop_) {
        // Prefer "Resetting" during (a) our post-XPB-reset mask window, or
        // (b) when CC HBs have gone stale (>1s) during an E-STOP reboot.
        if ((int32_t)(millis() - estopUiMaskUntilMs_) < 0 || 
            (ccHbSeen_ && ccHbAgeTmr_ > 1000U)) {
            page = UiPage::Resetting;
        } else {
            page = UiPage::EStop;
        }
    }
    else if (resetUiActive_) {
        page = UiPage::ResetCountdown;
    }
    else if (!successfulProtoLoadFromSD_) {
        page = UiPage::ProtoMissingSD;
    } 
    else if (ccHbSeen_ && ccHbAgeTmr_ > 3000U &&
             (int32_t)(millis() - estopUiMaskUntilMs_) >= 0) {
        // Suppress LostComms during the post-reset mask window — CC's
        // heartbeat system is restarting and will resume shortly.
        page = UiPage::LostComms;
    } else {
        page = UiPage::Normal;
    }

    // LCD toggle (used by Normal page)
    if (lcdTmr_ >= lcdToggle_ms_) { lcdTmr_ = 0; lcdToggle_ = !lcdToggle_; }

    // Render exactly one page and flush once
    renderUi_(page);

    // Final retry pump
    comms_.checkRetries();

    // expire SIM hold
    if (usbSimHold_ && usbSimHoldUntilMs_ != 0 &&
        (int32_t)(millis() - usbSimHoldUntilMs_) >= 0) {
        usbSimHold_ = false;
        dbgln("[USB] SIM hold expired");
    }

}

void ExpansionBoard::setDataInterval(uint16_t milli_secs) {
    kDataIntervalMs_ = milli_secs;
}

/**
 * @brief Read MAX31855 in °C; NAN on fault.
 */
double ExpansionBoard::readTC(Max31855Min &TC, const char *label) {
    float c = TC.readCelsius();
    if (isnan(c)) {
        dbg(label); dbgln(" fault");
        return NAN;
    }
    return (double)c;
}

/**
 * @brief Update sensor readings on a throttled cadence.
 * @copydetails ExpansionBoard::updateData()
 */
void ExpansionBoard::updateData() {
    /** Reads all onboard sensors **/
    if (dataTmr_ < kDataIntervalMs_) return;
    dataTmr_ = 0;

    latestSealC_ = readTC(tc1_, "TC1");
    latestSumpC_ = readTC(tc2_, "TC2");
}

/**
 * @brief Render a specific UI page and flush once.
 * @copydetails ExpansionBoard::renderUi_()
 */
void ExpansionBoard::renderUi_(UiPage page) {
    const bool entering = (page != lastUi_);
    if (entering) {
        lcd_.clearScreen();   // clear only when changing pages (prevents artifacts/flicker)
    }

    switch (page) {
        case UiPage::ProtoMissingSD:
            lcd_.setLineCenter(1, "Protocol Missing");
            lcd_.setLineCenter(2, "on SD Card!");
            break;

        case UiPage::ProtoTxFail:
            lcd_.setLineCenter(2, "Protocol Tx to CC");
            lcd_.setLineCenter(3, "FAILED!");
            break;

        case UiPage::Resetting:
            lcd_.setLineCenter(0, "Controller resetting");
            lcd_.setLineCenter(1, "Please wait...");
            break;

        case UiPage::LostComms:
            lcd_.setLineCenter(0, "Lost ClearCore Comms");
            lcd_.setLineCenter(1, "Link lost for >3s");
            lcd_.setLineCenter(2, "Check cable/power");
            break;

        case UiPage::EStop:
            lcd_.setLineCenter(0, ccAlarmActive_ ? "ALARM:" : "!!! E-STOP !!!");
            if (ccAlarmActive_ && ccAlarmMsg_[0]) {
                lcd_.setLineCenter(1, ccAlarmMsg_);
            } else {
                char line[21];
                ecodeToText(ccEstop_, line);
                lcd_.setLineCenter(1, line);
            }
            lcd_.setLineCenter(2, "Reset on controller");
            lcd_.setLineCenter(3, "to clear alarm");
            break;

        case UiPage::ResetCountdown:
            char buff[LCDDriver::kNumCols+1];
            lcd_.setLineCenter(0, "RESETTING...");
            snprintf(buff, sizeof(buff), "in %us", (unsigned)resetUiRemaining_);
            lcd_.setLineCenter(1, buff);
            lcd_.setLineCenter(2, "Return switch to");
            lcd_.setLineCenter(3, "center to cancel.");
            break;

        case UiPage::Normal:
        default:
            renderNormal_();   // draws all normal info (no flush here)
            break;
    }

    if (entering) {
        const char* name =
            page == UiPage::Boot           ? "Boot"        :
            page == UiPage::Resetting      ? "Resetting"   :
            page == UiPage::LostComms      ? "LostComms"   :
            page == UiPage::EStop          ? "EStop"       :
            page == UiPage::ResetCountdown ? "ResetCount"  :
                                             "Normal";
        char pg[16]; snprintf(pg, sizeof(pg), "[UI]%s", name);
        dbgln(pg);
    }

    lcd_.flush();
    lastUi_ = page;
}

/**
 * @brief Draw the “Normal” page contents (no flush).
 * @copydetails ExpansionBoard::renderNormal_()
 */
void ExpansionBoard::renderNormal_() {
    char buff[LCDDriver::kNumCols+1];

    // Line 0: left = protocol name OR runtime; right = CC state
    if (lcdToggle_ || stepTotalMs_ == 0) {
        // Show protocol name when toggle is active OR no countdown running.
        // Two ways the rig can be "without a runnable protocol":
        //   (a) XPB never parsed one off the SD card (stepCount_ == 0)
        //   (b) XPB parsed one but CC hasn't acknowledged receiving it
        //       this boot (protoState_ != Complete)
        // Either way the operator should see "NO PROTOCOL" instead of
        // a stale name that implies a runnable protocol exists.
        if (stepCount_ == 0 || protoState_ != ProtoTxState::Complete) {
            strncpy(buff, "NO PROTOCOL", LCDDriver::kNumCols);
        } else {
            strncpy(buff, protocolName_, LCDDriver::kNumCols);
        }
        buff[LCDDriver::kNumCols] = '\0';
    } else {
        char countdown[16];
        formatStepCountdown_(countdown, sizeof(countdown));
        snprintf(buff, sizeof(buff), "T: %s", countdown);
    }
    lcd_.setLineLR(0, buff, ccState_);

    // Line 1: step & loop
    if (ccLoopTot_ > 0) {
        snprintf(buff, sizeof(buff), "STEP:%2u  Loop:%lu/%lu",
                 (unsigned)ccStep_,
                 (unsigned long)ccLoopCur_,
                 (unsigned long)ccLoopTot_);
    } else {
        snprintf(buff, sizeof(buff), "STEP:%2u  Loop:%lu",
                 (unsigned)ccStep_,
                 (unsigned long)ccLoopCur_);
    }
    lcd_.setLineLeft(1, buff);

    // Line 2 & 3 content (two views)
    if (modeTorqueToggle_) {
        // Torque-stand view (placeholder values for now)
        lcd_.setLineLR(2, "RPM/s    RPM", "Dwell");
        uint16_t acc = 500;   // TODO: ccAccel_
        int16_t  rpm = 2123;  // TODO: ccRpm_
        char     dwellRight[8] = "";
        char left[21];
        snprintf(left, sizeof(left), "%5u  %5d", (unsigned)acc, (int)rpm);
        lcd_.setLineLR(3, left, dwellRight);
    } else {
        // RTM view
        int sp  = (int)lastSpC_;
        if (sp > 0) {
            snprintf(buff, sizeof(buff), "Heat:%3d\xDF""C RPM:%4d", sp, (int)ccRpm_);
        } else {
            snprintf(buff, sizeof(buff), "HEAT:none  RPM:%4d", (int)ccRpm_);
        }
        lcd_.setLineLeft(2, buff);

        uint16_t sealInt = isnan(latestSealC_) ? 0U : (uint16_t)(latestSealC_ + 0.5f);
        uint16_t sumpInt = isnan(latestSumpC_) ? 0U : (uint16_t)(latestSumpC_ + 0.5f);
        if (sumpInt < 100) {
            snprintf(buff, sizeof(buff), "Seal:%3u\xDF""C Sump:%2u\xDF""C",
                    (unsigned)sealInt, (unsigned)sumpInt);
        } else {
            snprintf(buff, sizeof(buff), "Seal:%3u\xDF""C Sump:%3uC",
                    (unsigned)sealInt, (unsigned)sumpInt);
        }
        lcd_.setLineLeft(3, buff);
    }
}

void ExpansionBoard::refreshStepCountdown_(bool stepOrLoopChanged) {
    if (stepOrLoopChanged) {
        countdownStepSnapshot_ = ccStep_;
        countdownLoopSnapshot_ = ccLoopCur_;
        stepStartAgeMs_ = ccSwAgeMs_;

        if (ccStep_ > 0) {
            uint8_t idx = (ccStep_ > 0) ? static_cast<uint8_t>(ccStep_ - 1) : 0;
            if (idx < stepCount_) {
                stepTotalMs_ = steps_[idx].dwellS_ * 1000UL;
            } else {
                stepTotalMs_ = 0;
            }
        } else {
            stepTotalMs_ = 0;
        }
    }

    if (stepTotalMs_ == 0) {
        stepRemainingMs_ = 0;
        return;
    }

    uint32_t elapsed = (ccSwAgeMs_ >= stepStartAgeMs_) ? (ccSwAgeMs_ - stepStartAgeMs_) : 0;
    stepRemainingMs_ = (elapsed >= stepTotalMs_) ? 0 : (stepTotalMs_ - elapsed);
}

void ExpansionBoard::formatStepCountdown_(char *dst, size_t len) const {
    if (len == 0) {
        return;
    }

    if (stepTotalMs_ == 0) {
        strncpy(dst, ccStep_ ? "--" : "Idle", len);
        dst[len - 1] = '\0';
        return;
    }

    uint32_t msRemaining = stepRemainingMs_;
    uint32_t secs = (msRemaining + 999U) / 1000U;
    if (secs == 0U) {
        strncpy(dst, "0s", len);
        dst[len - 1] = '\0';
        return;
    }

    uint32_t hours = secs / 3600U;
    uint32_t minutes = (secs % 3600U) / 60U;
    uint32_t seconds = secs % 60U;

    if (hours > 0U) {
        snprintf(dst, len, "%luh %02lum %02lus", (unsigned long)hours,
                 (unsigned long)minutes, (unsigned long)seconds);
    } else if (minutes > 0U) {
        snprintf(dst, len, "%lum %02lus", (unsigned long)minutes,
                 (unsigned long)seconds);
    } else {
        snprintf(dst, len, "%lus", (unsigned long)seconds);
    }
}

/**
 * @brief Publish RUN/RESET state with optional forcing.
 * @copydetails ExpansionBoard::publishSwitchState_()
 */
void ExpansionBoard::publishSwitchState_(bool force, int ref = -1) {
    int runActive   = (runSw_.read()   == LOW) ? 1 : 0;
    int resetActive = (resetSw_.read() == LOW) ? 1 : 0;

    static int lastRun = -1, lastReset = -1;
    if (!force && runActive == lastRun && resetActive == lastReset) return;

    lastRun = runActive;
    lastReset = resetActive;

    char msg[40];
    if (ref >= 0) {
        snprintf(msg, sizeof(msg), "SW;RUN=%d;RST=%d;REF=%d", runActive, resetActive, ref);
    } else {
        snprintf(msg, sizeof(msg), "SW;RUN=%d;RST=%d", runActive, resetActive);
    }
    comms_.sendMessage(msg, MessageType::INFO);   // telemetry / response; no ACK expected

    lastSwPublishMs_ = millis();
}

/**
 * @brief Update CRC-32 with additional data.
 * @copydetails ExpansionBoard::crc32_update_()
 */
uint32_t ExpansionBoard::crc32_update_(uint32_t crc, const uint8_t *data, size_t len) {
  crc = ~crc;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; ++i)
      crc = (crc >> 1) ^ (0xEDB88320UL & (-(int32_t)(crc & 1)));
  }
  return ~crc;
}

void ExpansionBoard::spiQuiesceAll_() {
    pinMode(LCD_CS_, OUTPUT);  digitalWrite(LCD_CS_, HIGH);
    pinMode(TC1_CS_, OUTPUT);  digitalWrite(TC1_CS_, HIGH);
    pinMode(TC2_CS_, OUTPUT);  digitalWrite(TC2_CS_, HIGH);
    pinMode(SD_CS_,  OUTPUT);  digitalWrite(SD_CS_,  HIGH);
    SPI.begin();
}

bool ExpansionBoard::sdInitWithRetry_(uint8_t tries, uint16_t backoffMs) {
    for (uint8_t i = 1; i <= tries; ++i) {
        spiQuiesceAll_();
        delay(10);  // let CS lines settle
        // 80 dummy clocks (10 × 8 bits) with all CS HIGH resets the
        // SD card's internal SPI state machine after a dirty MCU reset
        // while the card stayed powered.
        for (uint8_t j = 0; j < 10; ++j) SPI.transfer(0xFF);
        if (SD.begin(SD_CS_)) { dbgln("SD ready"); return true; }
        dbgkv("[SD] init attempt ", (unsigned long)i);
        delay(backoffMs);
    }
    dbgln("SD init FAILED (check 3.3V, CS, level shifting).");
    return false;
}

/**
 * @brief Parse protocol CSV into internal steps and compute PHASH.
 * @copydetails ExpansionBoard::loadProtocolFromSD_()
 */
bool ExpansionBoard::loadProtocolFromSD_(const char *path) {
    File csv = SD.open(path, FILE_READ);
    if (!csv) { dbgln("Open CSV failed"); return false; }

    auto stripCommas = [&](String &s) {
    s.trim();
        while (s.startsWith(",")) s.remove(0,1), s.trim();
        while (s.endsWith(","))   s.remove(s.length()-1), s.trim();
    };
    auto validSigned = [&](const String &s) {
        if (!s.length()) return false;
        for (uint16_t i=0;i<s.length();++i){ char c=s.charAt(i);
            if (i==0 && c=='-') continue; if (!isDigit(c)) return false; }
        return true;
    };
    auto isDigits = [&](const String &s) {
        if (!s.length()) return false;
            for (uint16_t i=0;i<s.length();++i) if (!isDigit(s.charAt(i))) return false;
            return true;
    };

    stepCount_ = 0;
    strncpy(protocolName_, "NA", sizeof(protocolName_) - 1);
    protocolName_[sizeof(protocolName_) - 1] = '\0';
    loopCount_ = 1;
    countdownStepSnapshot_ = 0;
    countdownLoopSnapshot_ = 0;
    stepStartAgeMs_ = 0;
    stepTotalMs_ = 0;
    stepRemainingMs_ = 0;

    // 1) PROTOCOL_NAME=...
    String line = csv.readStringUntil('\n');
    if (!line.startsWith("PROTOCOL_NAME=")) {
    #if XPB_PROTO_TRACE
        if (Serial) Serial.println("[PROTO] CSV parse fail: expected PROTOCOL_NAME on line 1");
    #endif
        csv.close();
        return false;
    }
    String nameVal = line.substring(strlen("PROTOCOL_NAME=")); stripCommas(nameVal);
    strncpy(protocolName_, nameVal.c_str(), sizeof(protocolName_) - 1);
    protocolName_[sizeof(protocolName_) - 1] = '\0';

    // 2) LOOP_COUNT=...
    line = csv.readStringUntil('\n');
    if (!line.startsWith("LOOP_COUNT=")) { dbgln("CSV: no LOOPS"); csv.close(); return false; }
    String lc = line.substring(strlen("LOOP_COUNT=")); stripCommas(lc);
    if (!isDigits(lc)) { dbgln("CSV: bad LOOPS"); csv.close(); return false; }
    loopCount_ = lc.toInt(); if (loopCount_ == 0) loopCount_ = 1;

    // 3) Skip header row
    csv.readStringUntil('\n');

    // 4) Parse rows
    // inside loadProtocolFromSD_()
    while (csv.available() && stepCount_ < kMaxProtocolSteps_) {
        String row = csv.readStringUntil('\n'); row.trim();
        if (!row.length()) continue;

        int c1 = row.indexOf(',');
        if (c1 < 0) {
#if XPB_PROTO_TRACE
            if (Serial) Serial.println("[PROTO] CSV parse fail: missing comma #1 in data row");
#endif
            csv.close();
            return false;
        }
        int c2 = row.indexOf(',', c1 + 1);
        if (c2 < 0) {
#if XPB_PROTO_TRACE
            if (Serial) Serial.println("[PROTO] CSV parse fail: missing comma #2 in data row");
#endif
            csv.close();
            return false;
        }
        int c3 = row.indexOf(',', c2 + 1);         // may be -1

        String s1 = row.substring(0, c1);          s1.trim();   // RPM
        String s2 = row.substring(c1+1, c2);       s2.trim();   // RPM/s
        String s3 = (c3 < 0) ? row.substring(c2+1)
                            : row.substring(c2+1, c3);          // dwell(s)
        s3.trim();
        String s4 = (c3 < 0) ? String() : row.substring(c3+1);  // TEMP_C (optional)
        s4.trim();

        if (!validSigned(s1) || !isDigits(s2) || !isDigits(s3)) {
#if XPB_PROTO_TRACE
            if (Serial) Serial.println("[PROTO] CSV parse fail: invalid RPM/ACCEL/DWELL in data row");
#endif
            csv.close();
            return false;
        }
        if (s4.length() && !isDigits(s4)) {
#if XPB_PROTO_TRACE
            if (Serial) Serial.println("[PROTO] CSV parse fail: invalid TEMP_C in data row");
#endif
            csv.close();
            return false;
        }

        Step &st = steps_[stepCount_++];
        st.rpmTarget_ = s1.toInt();
        st.rpmAccel_  = (uint32_t)s2.toInt();
        st.dwellS_    = (uint32_t)s3.toInt();
        st.tempC_     = s4.length() ? (uint16_t)constrain(s4.toInt(), 0, 200) : 0; // clamped to 200degC
    }
    csv.close();

  // 5) Compute PHASH
  uint32_t h = 0;
  h = crc32_update_(h, (const uint8_t*)protocolName_, strlen(protocolName_));
  h = crc32_update_(h, (const uint8_t*)&loopCount_, sizeof(loopCount_));
  h = crc32_update_(h, (const uint8_t*)&stepCount_, sizeof(stepCount_));
  for (uint8_t i=0;i<stepCount_;++i)
    h = crc32_update_(h, (const uint8_t*)&steps_[i], sizeof(Step));
  progHash_ = h;

  return (stepCount_ > 0);
}

static bool deleteIfExists_(const char *path) {
    if (!SD.exists(path)) return true;
    return SD.remove(path);
}

void ExpansionBoard::clearResumeSlots_() {
    (void)deleteIfExists_("/RA.BIN");
    (void)deleteIfExists_("/RB.BIN");
}

/**
 * @brief Print a human-readable summary of the loaded protocol.
 * @copydetails ExpansionBoard::logProtocol_()
 */
void ExpansionBoard::logProtocol_() const {
#if XPB_DEBUG
  dbgln("==== Protocol (XPB) ====");
  dbgkv("\nName: ", protocolName_);
  dbgkv("\nLoops: ", (unsigned long)loopCount_);
  dbgkv("\nSteps: ", (unsigned long)stepCount_);
  dbgkv("\nPHASH: ", (unsigned long)progHash_);
  dbgln("");
  for (uint8_t i=0;i<stepCount_;++i) {
    static constexpr uint16_t kStepsPerRev = 3200;
    long rpm   = steps_[i].rpmTarget_;
    long accel = steps_[i].rpmAccel_;
    unsigned dwell = steps_[i].dwellS_;
    uint16_t temp = steps_[i].tempC_;
    char buf[64];
    snprintf(buf, sizeof(buf), "Step %2u: %6ld RPM  %4ld RPM/s  %3us %3udegC", i+1, rpm, accel, dwell, temp);
    dbgln(buf);
  }
#endif
}



bool ExpansionBoard::uploadProtocolToCC_() {
    if (!stepCount_) {
        // No protocol in RAM.  Don't block here — tick() handles
        // periodic SD recovery so the next REQ:PROTO will succeed.
        dbgln("[PROTO] No protocol cached — waiting for SD recovery");
        return false;
    }
    
    dbgln("[PROTO] Starting upload to CC...");
    bootPhase_ = BootPhase::TxInProgress;

    // 1. Send PR_BEG
    char msg[96];
    snprintf(msg, sizeof(msg), "PR_BEG;NAME=%s;LOOPS=%lu;STEPS=%u;PHASH=%lu",
             protocolName_, 
             (unsigned long)loopCount_, 
             (unsigned)stepCount_, 
             (unsigned long)progHash_);
    
    comms_.sendMessage(msg, MessageType::CRITICAL);
    setProtoState_(ProtoTxState::BegSent, "PR_BEG_SENT");
    protoStepSent_ = 0;
    protoSince_ = 0;

    // Poll for CC to ACK PR_BEG (up to 500ms, driving retries).
    // At 9600 baud the ~65-char PR_BEG takes ~68ms on the wire,
    // plus CC parse + response time ≈ 150-200ms total.
    {
        const unsigned long deadline = millis() + 500;
        while (millis() < deadline && comms_.isWaitingForAck()) {
            delay(10);
            comms_.checkForMessages();
            comms_.checkRetries();
        }
    }

    // Verify CC acknowledged PR_BEG before flooding PR_DAT chunks
    if (comms_.isWaitingForAck()) {
        dbgln("[PROTO] PR_BEG not ACKed by CC — aborting upload");
        comms_.cancelPending();   // prevent orphaned retries
        setProtoState_(ProtoTxState::WaitingReq, "PR_BEG_NO_ACK");
        return false;
    }
    
    // 2. Send PR_DAT chunks (dummy for now)
    dbgln("[PROTO] Sending data chunks...");

    for (uint8_t i = 0; i < stepCount_; i++) {
        // Format: "SEQ=n;DATA=rpm,accel,dwell"
        snprintf(msg, sizeof(msg), "PR_DAT;SEQ=%u;DATA=%ld,%lu,%lu,%u", 
                i,
                steps_[i].rpmTarget_,
                steps_[i].rpmAccel_, 
                steps_[i].dwellS_,
                steps_[i].tempC_);
        comms_.sendMessage(msg, MessageType::IMPORTANT);
        protoStepSent_ = i + 1;
        delay(50);
        comms_.checkForMessages();
    }
    
    // 3. Send PR_END  
    snprintf(msg, sizeof(msg), "PR_END;CRC=%lu", (unsigned long)0);
    comms_.sendMessage(msg, MessageType::CRITICAL);
    setProtoState_(ProtoTxState::EndSent, "PR_END_SENT");
    protoSince_ = 0;
    //delay(100);
    //comms_.checkForMessages();

    return true;
}

/**
 * @brief Handle decoded frames from ClearCore and update owner state.
 */
void ExpansionBoard::ExpansionBoardComms::onMessageReceived(const String& data) {
    // === centralize REF detection; use presence, not value ===
    const int refPos = data.indexOf(F(";REF="));              
    const bool hasRef = (refPos > 0);                         
    const uint16_t refVal = hasRef ?                         
        (uint16_t)data.substring(refPos + 5).toInt() : 0;     

    // Consider any ACK;...;REF= as a valid ack for retry bookkeeping
    if (data.startsWith("ACK;")) {
        if (owner_) owner_->lastProtoRef_ = refVal;

        if (owner_ && owner_->protoState_ == ProtoTxState::BegSent) {
            owner_->setProtoState_(ProtoTxState::Sending, "ACK_AFTER_PR_BEG");
            owner_->protoSince_ = 0;
            // (optional) owner_->dbgln("[PROTO] PR_BEG ACK → Sending");
        }

        // If we already sent PR_END, any ACK is our cue to await final NOTICE
        if (owner_ && owner_->protoState_ == ProtoTxState::EndSent) {
            owner_->setProtoState_(ProtoTxState::AwaitResult, "ACK_AFTER_PR_END");
            owner_->protoSince_ = 0;
            // (optional) owner_->dbgln("[PROTO] PR_END ACK → AwaitResult");
        }
}

    // Mark that we've seen any CC traffic (for "no link" note)
    if (owner_ && !owner_->usbInjecting_) {            
        owner_->ccAnySeen_ = true;
        owner_->linkState_ = LinkState::Alive;
    }

    // ===== Policy: never ACK telemetry/one-way notices here =====
    // We fall through to dedicated handlers below.
    // HB/STAT/NOTICE/READY are processed later; no ACK emitted here.

    // Swallow REAL CC HB during SIM-hold (USB injection drives UI)
    if (owner_ && owner_->usbSimHold_ && !owner_->usbInjecting_ && data.startsWith("HB;")) {
        owner_->ccHbSeen_ = true; owner_->ccHbAgeTmr_ = 0;
        owner_->dbgln("[SIM] swallowed REAL CC HB");
        return;
    }

    // ===== READY from CC =====
    if (data.startsWith("READY;ID=CC")) {
        if (owner_) {
            owner_->ccReady_ = true;

            // Send current heater state
            char line[64];
            snprintf(line, sizeof(line), "STAT;SEQ=%u;SUMP=0;SEAL=0",
                    owner_->hbSeq_++);
            sendMessage(line, MessageType::INFO);

            // Publish current switch state
            owner_->publishSwitchState_(true);
            
            // Don't offer resume here - it's handled after protocol upload
        }
        return;
    }

    // ===== Request/Response: REQ:SW → reply with SW;... (the reply *is* the ACK) =====
    if (data.startsWith("REQ:SW")) { 
        if (owner_) owner_->publishSwitchState_(true, hasRef ? (int)refVal : -1); 
        return;  // important: do NOT send a separate ACK    
    }

    if (data.startsWith("REQ:PROTO")) {
        if (hasRef) {
            char ack[32];
            snprintf(ack, sizeof(ack), "ACK;OK;REF=%u", refVal);
            sendMessage(ack, MessageType::INFO);
        } else {
            sendMessage("ACK;OK", MessageType::INFO);
        }

        if (owner_) {
#if XPB_PROTO_TRACE
            char trace[144];
            snprintf(trace, sizeof(trace),
                     "[PROTO] REQ:PROTO rx ref=%u state=%s steps=%u gate=%u",
                     (unsigned)refVal,
                     owner_->protoStateName_(owner_->protoState_),
                     (unsigned)owner_->stepCount_,
                     (unsigned)(owner_->ccProtoReq_ ? 1U : 0U));
            if (Serial) Serial.println(trace);
#endif

            // Always attempt upload: CC only sends REQ:PROTO when it
            // doesn't have a protocol loaded, so repeating is correct.
            // ccProtoReq_ guards against reentrant calls from
            // checkForMessages() inside uploadProtocolToCC_().
            if (!owner_->ccProtoReq_) {
                owner_->ccProtoReq_ = true;
                if (!owner_->uploadProtocolToCC_()) {
                    owner_->ccProtoReq_ = false;
                } else {
                    owner_->bootPhase_ = BootPhase::TxInProgress;
                    owner_->protoSince_ = 0;
                }
                // If CC never confirms (NOTICE;PROTO_RX=OK), clear the
                // gate so the next REQ:PROTO triggers a fresh upload.
                if (owner_->protoState_ != ProtoTxState::Complete) {
                    owner_->ccProtoReq_ = false;
                }
            }
        }
        return;
    }

    // ===== NOTICE from CC: protocol receive completed =====
    // Format we expect: "NOTICE;PROTO_RX=OK;PHASH=<uint32>"
    if (data.startsWith("NOTICE;")) {
        const String rx = kvGet(data, "PROTO_RX=");
        if (rx == "OK") {
            if (owner_) {
                owner_->ccProtoReq_ = false;
                owner_->setProtoState_(ProtoTxState::Complete, "NOTICE_PROTO_RX_OK");

                // PHASH check  (strtoul: toInt() overflows for hashes > INT_MAX)
                const String sPH = kvGet(data, "PHASH=");
                const uint32_t rxPhash = sPH.length() ? strtoul(sPH.c_str(), nullptr, 10) : 0UL;

                // === cold-boot resume policy ===
                if (owner_->needResumeAfterProto_
                    && rxPhash != 0UL
                    && rxPhash == owner_->storedPhash_) {

                    // last-moment RUN check
                    owner_->runSw_.update();
                    const bool runEngaged = (owner_->runSw_.read() == LOW);

                    // Always send CMD;RESUME=AUTO so CC loads the
                    // saved position.  AUTOSTART=1 when RUN is engaged
                    // (auto-start immediately), AUTOSTART=0 when RUN is
                    // off (load position, stay IDLE until operator toggles
                    // RUN).
                    {
                        const int autoStart = runEngaged ? 1 : 0;
                        char line[96];
                        snprintf(line, sizeof(line),
                                "CMD;RESUME=AUTO;STEP=%u;LOOP=%u;PHASH=%lu;AUTOSTART=%d",
                                (unsigned)owner_->storedStep_,
                                (unsigned)owner_->storedLoopCur_,
                                (unsigned long)owner_->storedPhash_,
                                autoStart);
                        sendCommand(line, MessageType::CRITICAL);
                        if (runEngaged) {
                            owner_->dbgln("[RESUME] Sent AUTOSTART=1 (RUN switch engaged)");
                        } else {
                            owner_->dbgln("[RESUME] Sent AUTOSTART=0 (RUN switch OFF, position loaded)");
                        }
                    }
                } else {
                    owner_->dbgln("[RESUME] No valid resume (PHASH mismatch or none)");
                }
            }
            return;
        }

        if (rx == "FAIL") {
            if (owner_) {
                //owner_->isProtoLoadedOntoCC_ = false;
                owner_->ccProtoReq_ = false;
                owner_->setProtoState_(ProtoTxState::Failed, "NOTICE_PROTO_RX_FAIL");
            }
            // (Optional) store an error reason for the UI
            return;
        }

        return; // ignore other NOTICEs
}


    // ===== Commands from CC → send exactly one ACK; mirror REF if present =====
    // (We keep commands here—NOT at the top—so ACK happens once, then we run handlers.)
    if (data.startsWith("CMD;") || data.startsWith("QUIESCE;")) {
        if (hasRef) {
            char ack[28]; snprintf(ack, sizeof(ack), "ACK;OK;REF=%u", refVal);
            sendMessage(ack, MessageType::INFO);
        } else {
            sendMessage("ACK;OK", MessageType::INFO);
        }

        // --- CMD;... handling ---
        if (data.startsWith("CMD;") && owner_) {
            String reset = kvGet(data, "RESET=");
            if (reset.length()) {
                if (reset == "ARM") {
                    int secs = kvGetIntClamped(data, "SECS=", 5, 1, 30);
                    owner_->resetUiActive_    = true;
                    owner_->resetUiSecs_      = (uint8_t)secs;
                    owner_->resetUiRemaining_ = (uint8_t)secs;
                    owner_->resetUiTmr_       = 0;
                }
                else if (reset == "CANCEL") {
                    owner_->resetUiActive_ = false;
                }
                else if (reset == "EXEC") {
                    // 1) Persist resume snapshot only if protocol was actively
                    //    running (everRan_).  After COMPLETED, everRan_ is
                    //    false and there is nothing useful to resume.
                    const uint16_t stepSnap    = owner_->ccStep_;
                    const uint16_t loopCurSnap = (uint16_t)owner_->ccLoopCur_;
                    const uint16_t loopTotSnap = (uint16_t)owner_->ccLoopTot_;
                    const uint32_t ph          = owner_->progHash_;

                    // Manual reset: always clear resume state so the
                    // protocol starts fresh on next boot.  Resume-from-
                    // power-loss relies on the periodic saves already on
                    // SD; an intentional operator reset should not
                    // preserve them.
                    clearResumeTU();
                    owner_->haveStoredResume_ = false;
                    owner_->everRan_          = false;
                    owner_->dbgln("[RESET] resume slots cleared (manual reset)");

                    // 2) Ask CC to mask XPB-stale for ~10s (bounded to 3..15s on CC)
                    //    NOTE: Do NOT pump checkForMessages() here — we are
                    //    already inside onMessageReceived → checkForMessages.
                    //    Reentrant calls corrupt incomingMsg_ and can re-enter
                    //    this EXEC handler recursively, causing stack overflow.
                    //    The ~300ms of delays below give CC ample time to
                    //    process the QUIESCE before we proceed.
                    {
                        char q[96];
                        snprintf(q, sizeof(q),
                                 "QUIESCE;WHO=XPB;SECS=%d;STEP=%u;LOOP=%u/%u;PHASH=%lu",
                                 10,
                                 (unsigned)stepSnap,
                                 (unsigned)loopCurSnap,
                                 (unsigned)loopTotSnap,
                                 (unsigned long)ph);
                        sendCommand(q, MessageType::CRITICAL);
                        delay(100);  // settle — CC processes within one tick (~µs)
                    }

                    // 3) Mark intentional XPB reset, give SD a moment
                    bool fOK = writeResetFlagTU();
                    owner_->dbgln(fOK ? "[RESET] flag write OK" : "[RESET] flag write FAIL");
                    delay(50);  // 50ms settle — SD internal controller needs time after writes

                    // 4) Show reboot splash, notify CC, then reset
                    owner_->lcd_.clearScreen();
                    owner_->lcd_.setLineCenter(1, "SYSTEM");
                    owner_->lcd_.setLineCenter(2, "RESETTING...");
                    owner_->lcd_.flush();
                    delay(250);

                    sendMessage("NOTICE;XPB_RESET=NOW", MessageType::NORMAL);
                    delay(5);

                    // Logical reset: reinitialize all state in-place
                    // (no MCU reset — peripherals stay active)
                    owner_->logicalReset();
                }
            }

            String sSP = kvGet(data, "SP=");
            if (sSP.length()) {
                owner_->setHeaterTarget(sSP.toFloat());
            }

            String mode = kvGet(data, "MODE=");
            if (mode.length()) {
                owner_->modeTorqueToggle_ = (mode == "TORQUE");
            }
            return;
        }

        // --- QUIESCE;... from CC (if you later want semantics, add here) ---
        return;                                                            
    }

    // ===== Heartbeat from ClearCore =====
    if (data.startsWith("HB;") && owner_) {
        const uint8_t prevStep = owner_->ccStep_;
        const uint32_t prevLoop = owner_->ccLoopCur_;
        // Parse HB
        String sSTATE = kvGet(data, "STATE=");
        if (sSTATE == "RUNNING") owner_->everRan_ = true;
        if (sSTATE == "COMPLETED" && owner_->everRan_) {
            // Protocol finished normally — clear stale resume data so the
            // next boot won't auto-resume a completed run.
            clearResumeTU();
            owner_->haveStoredResume_ = false;
            owner_->everRan_          = false;
            owner_->dbgln("[RESUME] cleared (protocol COMPLETED)");
        }
        if (sSTATE.length()) {
            // Shorten COMPLETED → DONE for LCD display
            if (sSTATE == "COMPLETED") {
                strcpy(owner_->ccState_, "DONE");
            } else {
                sSTATE.toCharArray(owner_->ccState_, sizeof(owner_->ccState_));
            }
        }

        // E_CODE-based estop
        String sECODE  = kvGet(data, "E_CODE=");
        if (sECODE.length()) {
            long v = sECODE.toInt();
            if (v < 0) v = 0; if (v > 255) v = 255;
            owner_->ccEstopCode_ = (uint8_t)v;
        } else {
            owner_->ccEstopCode_ = 0;  // backward-compat if field absent
        }
        const bool estopNow = (owner_->ccEstopCode_ != 0);
        owner_->ccEstop_ = estopNow;
        if (!estopNow) {
            owner_->ccAlarmActive_ = false;
            owner_->ccAlarmMsg_[0] = '\0';
        }

        String sSTEP  = kvGet(data, "STEP=");
        if (sSTEP.length()) {
            long v = sSTEP.toInt();
            if (v < 0) v = 0; if (v > 255) v = 255;
            owner_->ccStep_ = (uint8_t)v;         // CC publishes 1-based; show as-is
        }

        String sLOOP  = kvGet(data, "LOOP=");
        if (sLOOP.length()) {
            int slash = sLOOP.indexOf('/');
            if (slash > 0) {
                const char *cstr = sLOOP.c_str();
                char *endp = nullptr;
                unsigned long cur = strtoul(cstr, &endp, 10);
                unsigned long tot = 0;
                if (endp && *endp == '/') tot = strtoul(endp + 1, nullptr, 10);
                owner_->ccLoopCur_ = (uint32_t)cur;
                owner_->ccLoopTot_ = (uint32_t)tot;
            }
        }

        String sAGE   = kvGet(data, "SW_AGE=");
        if (sAGE.length()) owner_->ccSwAgeMs_ = strtoul(sAGE.c_str(), nullptr, 10);

        String sRPM   = kvGet(data, "RPM=");
        if (sRPM.length()) owner_->ccRpm_ = (int16_t)sRPM.toInt();

        const bool stepChanged = (owner_->ccStep_ != prevStep) || (owner_->ccLoopCur_ != prevLoop);
        // Only count down during RUNNING — no countdown in IDLE, PREHEAT, etc.
        static bool wasRunning = false;
        static bool wasPaused  = false;
        if (sSTATE == "RUNNING") {
            const bool resumingFromPause = !wasRunning && wasPaused && !stepChanged;
            if (resumingFromPause) {
                // Re-populate stepTotalMs_ from protocol (zeroed during PAUSED)
                // then shrink it by the already-elapsed dwell so the countdown
                // continues where it left off.
                owner_->refreshStepCountdown_(true);   // sets stepTotalMs_ & stepStartAgeMs_
                if (owner_->pausedElapsedMs_ < owner_->stepTotalMs_) {
                    owner_->stepTotalMs_ -= owner_->pausedElapsedMs_;
                } else {
                    owner_->stepTotalMs_ = 0;
                }
                owner_->refreshStepCountdown_(false);  // recompute stepRemainingMs_
            } else {
                // Normal init on new step or first entry into RUNNING.
                owner_->refreshStepCountdown_(stepChanged || !wasRunning);
            }
            wasRunning = true;
            wasPaused  = false;
        } else {
            const bool isPausedLike = (sSTATE == "PAUSED" || sSTATE == "RESUME");
            if (wasRunning && isPausedLike) {
                // Save dwell progress before clearing countdown.
                // Use stepTotalMs_/stepRemainingMs_ from the last RUNNING tick
                // (SW_AGE has already reset by the time we see PAUSED).
                owner_->pausedElapsedMs_ = (owner_->stepTotalMs_ > owner_->stepRemainingMs_)
                    ? (owner_->stepTotalMs_ - owner_->stepRemainingMs_) : 0;
            }
            owner_->stepTotalMs_     = 0;
            owner_->stepRemainingMs_ = 0;
            wasRunning = false;
            wasPaused  = isPausedLike;
        }

        // One-time "ready" if HB arrived before READY (common on some boots)
        if (!owner_->ccReady_) {
            owner_->ccReady_ = true;

            // send initial STAT so CC immediately sees fresh telemetry (no ACK)
            char line[64];
            snprintf(line, sizeof(line), "STAT;SEQ=%u;SUMP=0;SEAL=0",
                     owner_->hbSeq_++);
            sendMessage(line, MessageType::INFO);

            // Offer resume if we truly have one (notice; no ACK)
/*             if (owner_->haveStoredResume_ && !owner_->suppressResumePrompt_ && !owner_->uiPendingResume_) {
                snprintf(line, sizeof(line), "RESUME?;STEP=%u;LOOP=%u;PHASH=%lu",
                         (unsigned)owner_->storedStep_,
                         (unsigned)owner_->storedLoopCur_,
                         (unsigned long)owner_->storedPhash_);
                sendMessage(line, MessageType::NORMAL);
                owner_->uiPendingResume_ = true;
            } */
        }

        owner_->ccHbSeen_   = true;
        owner_->ccHbAgeTmr_ = 0;

        // Persist resume point only when STEP/LOOP change
        static uint16_t lastStep = 0xFFFF, lastLoop = 0xFFFF;
        const uint16_t stepNow = owner_->ccStep_;
        const uint16_t loopNow = (uint16_t)owner_->ccLoopCur_;
        if (owner_->everRan_ && (stepNow != lastStep || loopNow != lastLoop)) {
            lastStep = stepNow; lastLoop = loopNow;
            const uint32_t ph = owner_->storedPhash_ ? owner_->storedPhash_ : owner_->progHash_;
            if (!saveResumeTU(ph, stepNow, loopNow, (uint16_t)owner_->ccLoopTot_)) {
                owner_->dbgln("[RESUME] periodic save FAILED");
            }
        }

        return;
    }

    // ===== Alarm from ClearCore =====
    if (data.startsWith("ALARM;") && owner_) {
        String t = kvGet(data, "TYPE=");
        String m = kvGet(data, "MSG=");
        if (t == "ESTOP") {
            owner_->ccAlarmActive_ = true;
            if (m.length()) {
                m.toCharArray(owner_->ccAlarmMsg_, sizeof(owner_->ccAlarmMsg_));
            } else {
                strncpy(owner_->ccAlarmMsg_, "E-STOP asserted", sizeof(owner_->ccAlarmMsg_) - 1);
            }
        }
        return;
    }

    // else ignore quietly
}

/**
 * @brief Count and occasionally warn about bad checksums.
 * @copydetails ExpansionBoard::ExpansionBoardComms::onBadChecksum()
 */
void ExpansionBoard::ExpansionBoardComms::onBadChecksum(const String&) {
    ++badCrcCount_;
    if (badCrcCount_ % 10 == 1 && owner_) {
        owner_->dbgln("WARN: bad checksum (rate-limited)");
    }
}

/// HeatingController removed: PID moved to CC. setHeaterTarget() now caches
/// lastSpC_ for LCD display only.
