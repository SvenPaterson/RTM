#include "ExpansionBoard.h"
#include <Arduino.h>
#include <SPI.h>

#define XPB_INJECT_FROM_USB 0


void xpbSoftResetNow() {
    #if defined(__AVR_ATmega4809__) || defined(ARDUINO_AVR_NANO_EVERY)
    // megaAVR-0 (Nano Every): use software reset register
    // Some cores name it SWRST, some SWRR – guard both.
    #if defined(RSTCTRL_SWRST)
        _PROTECTED_WRITE(RSTCTRL.SWRST, 1);
    #elif defined(RSTCTRL_SWRR)
        _PROTECTED_WRITE(RSTCTRL.SWRR, 1);
    #else
        // Fallback to WDT if symbol names differ
        wdt_enable(WDTO_15MS);
        for (;;) {}
    #endif
    #else
    // Classic AVRs (e.g., ATmega328P): WDT nuke
    wdt_enable(WDTO_15MS);
    for (;;) {}
    #endif
    }

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
    bool saveResumeTU(uint32_t phash, uint16_t step, uint16_t loopCur, uint16_t loopTot) {
        ResumeRec a{}, b{};
        bool ha = readSlot(kSlotA, a), hb = readSlot(kSlotB, b);

        ResumeRec rec{}; rec.magic=0x58504252u; rec.version=1;
        rec.seq = (ha||hb) ? ( (ha?a.seq:0) > (hb?b.seq:0) ? (a.seq+1) : (b.seq+1) ) : 1;
        rec.phash = phash; rec.step = step; rec.loopCur = loopCur; rec.loopTot = loopTot; rec.flags = 0;

        // Overwrite the older slot; write via matching temp name
        const bool writeB = (ha && (!hb || a.seq <= b.seq));
        const char *finalPath = writeB ? kSlotB : kSlotA;
        const char *tmpPath   = writeB ? kTmpB  : kTmpA;
        return writeSlot(finalPath, tmpPath, rec);
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
    bool writeResetFlagTU() {
        File f = SD.open(kXpbResetFlag, FILE_WRITE);
        if (!f) return false;
        uint32_t tag = 0x21505842u; // "!XPB"
        bool ok = (f.write(reinterpret_cast<const uint8_t*>(&tag), sizeof(tag)) == sizeof(tag));
        f.flush();
        f.close();
        delay(12);  // allow the card to commit the sector after soft reset
        return ok;
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

/**
 * @brief Initialize all board subsystems and start comms/UI.
 * @copydetails ExpansionBoard::begin()
 */
bool ExpansionBoard::begin() {
    Serial.begin(9600);

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
    dbgln("\nUSB Serial Monitor Connected!");

    // UART to ClearCore (not on SPI) – safe to bring up early
    ttlComms_.begin();
    ttlComms_.setRxUsbLogging(true, "CC"); // toggle by sending LOG=0 / LOG=1
    delay(200);
    dbgln("Connecting with CC..");
    // Ask CC to suppress stale-STAT E-STOP while XPB finishes boot work
    ttlComms_.sendCommand("QUIESCE;SECS=10");

    // --- SPI bus & SD first (prevents other devices from holding MISO) ---
    spiQuiesceAll_();
    bool sdOk = sdInitWithRetry_();  // prints "SD ready" or one FAIL summary

    // --- E-STOP UI mask on boot and after intentional XPB reset ---
    if ((int32_t)(millis() - estopUiMaskUntilMs_) >= 0) {
        estopUiMaskUntilMs_ = millis() + 2500UL;
        dbgln("[UI] E-STOP boot mask 2.5s");
    }
    if (consumeResetFlagTU()) {
        estopUiMaskUntilMs_ = millis() + 8000UL;
        suppressResumePrompt_ = true;                   // NEW: don’t prompt to resume
        dbgln("[UI] E-STOP mask active (post-XPB reset)");
    }

    // LCD AFTER SD so the LCD CS can't hold MISO low during SD init
    if (!lcd_.begin()) { dbgln("FATAL: LCD initialization failed!"); return false; }
    lastUi_ = static_cast<UiPage>(0xFF);

    // Switches
    runSw_.attach(RUN_SW_PIN_, INPUT_PULLUP);   runSw_.interval(25);
    resetSw_.attach(RESET_SW_PIN_, INPUT_PULLUP); resetSw_.interval(25);

    // Sensors
    dbg("Initializing MAX31855 sensor - TC1...");
    delay(500); // stabilize
    if (!tc1_.begin()) { dbgln("ERROR."); while (1) delay(10); }
    else dbgln("DONE");
    tc1_.setFaultChecks(MAX31855_FAULT_ALL);

    /*
    dbg("Initializing MAX31855 sensor - TC2...");
    delay(500); // stabilize
    if (!tc2_.begin()) { dbgln("ERROR."); while (1) delay(10); }
    else dbgln("DONE");
    tc2_.setFaultChecks(MAX31855_FAULT_ALL);
    */

    // Protocol + Resume (only if SD OK)
    if (sdOk) {
        if (!loadProtocolFromSD_("/protocol.csv")) {
            dbgln("Protocol load FAILED");
        } else {
            dbgln("Protocol loaded OK");
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
        // Scrub "virgin" resume (created before any real run)
        // i.e., step=1 loop=1 for the same PHASH
        if (haveStoredResume_ && storedPhash_ == progHash_) {
            if (storedStep_ <= 1 && storedLoopCur_ <= 1) {
                dbgln("Resume looks virgin (step=1 loop=1) -> clearing");
                clearResumeTU();
                haveStoredResume_ = false;
            }
        }

        // === SMART PROTOCOL UPLOAD DECISION ===
        // Upload to CC only when safe/necessary
        if (stepCount_ > 0) {  // Only if we loaded a protocol successfully
            bool shouldUpload = false;

            dbgln("[UPLOAD DECISION]");
            dbgkv("  haveStoredResume_: ", haveStoredResume_ ? "true" : "false");
            dbgkv("  suppressResumePrompt_: ", suppressResumePrompt_ ? "true" : "false");

            if (!haveStoredResume_) {
                // No resume = fresh start, need to upload
                dbgln("  Decision: No resume state - WILL upload");
                shouldUpload = true;
            } 
            else if (suppressResumePrompt_) {
                // After intentional XPB-only reset, CC already has protocol
                dbgln("  Decision: Post-XPB reset - WON'T upload");
                shouldUpload = false;
            }
            else {
                // Have resume for this exact protocol - preserve CC state
                dbgln("  Decision: Valid resume exists - WON'T upload");
                shouldUpload = false;
            }
            
            if (shouldUpload) {
                dbgln("Waiting for CC ready signal...");
                uint32_t uploadWait = millis();
                while (!ccReady_ && millis() - uploadWait < 2000) {
                    ttlComms_.checkForMessages();
                    ttlComms_.checkRetries();
                    delay(10);
                }

                dbgkv("  ccReady_: ", ccReady_ ? "true" : "false");
                dbgkv("  Wait time ms: ", (unsigned long)(millis() - uploadWait));
                
                if (ccReady_ || millis() - uploadWait >= 2000) {
                    dbgln("Uploading protocol to CC...");
                    if (uploadProtocolToCC_()) {
                        dbgln("Protocol upload successful");
                    } else {
                        dbgln("Protocol upload failed");
                    }
                } else {
                    dbgln("CC not ready - skipping upload");
                }
            }
        }

    } else {
        dbgln("No protocol loaded from SD - nothing to upload");
    }

    // Don’t warn here. We’ll show an info message only if there’s truly no CC traffic after 10s.
    ccAnySeen_ = false;
    warnedNoLink_ = false;
    sinceBoot = 0;
    dbgln("Awaiting CC traffic...");

    // DEBUGGING ONLY
    heater_.begin();                 // maybe only when a test / preheat starts?
    heater_.setTargetTemp(32.0);     // temp will come from CC later

    publishSwitchState_(true);

    sinceBoot = 0;
    warnedNoLink_ = false;

    return true;
}

/**
 * @brief Periodic task driving comms, IO, sensors, PID, heartbeat, and UI.
 * @copydetails ExpansionBoard::tick()
 * @note Supports test injection via USB when XPB_INJECT_FROM_USB is defined.
 */
void ExpansionBoard::tick() {
    // ----- Comms housekeeping -----
    ttlComms_.checkForMessages();
    ttlComms_.checkRetries();

    // Soft "no link yet" note after 10s with no CC traffic at all
    if (!ccAnySeen_ && !warnedNoLink_ && sinceBoot > 10000) {
        warnedNoLink_ = true;
        dbgln("INFO: No CC traffic yet (>10s). Continuing without link.");
        // UI stays on the normal "Connecting" page you already render.
    }

    // ----- USB injection / commands -----
    #ifdef XPB_INJECT_FROM_USB
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
                if (usbLine.equalsIgnoreCase("LOG=0")) {
                    ttlComms_.setRxUsbLogging(false, "CC");
                    dbg("[USB] RX log OFF");
                }
                else if (usbLine.equalsIgnoreCase("LOG=1")) {
                    ttlComms_.setRxUsbLogging(true, "CC");
                    dbg("[USB] RX log ON");
                }
                else if (usbLine.startsWith("SIM=")) {
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
                        // Send over TTL to ClearCore
                        ttlComms_.sendCommand(frame.c_str(), MessageType::CRITICAL);
                        dbgkv("[USB->CC] Sending: ", frame);
                    } 
                    // Everything else is injected locally for testing
                    else {
                        usbInjecting_ = true;
                        ttlComms_.onMessageReceived(frame);   // call directly so we can mark it as injected
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

    /* // Keep background discovery alive while not ready
    static uint32_t lastHello = 0;
    if (!ccReady_ && (millis() - lastHello >= 1000)) {
        ttlComms_.sendMessage("HELLO;ID=XPB", MessageType::INFO);
        lastHello = millis();
    } */

    // ----- SWITCH SCAN (debounced edges) -----
    bool runEdgeDown  = false, runEdgeUp  = false;
    bool rstEdgeDown  = false, rstEdgeUp  = false;

    if (runSw_.update()) {
        runEdgeDown = runSw_.fell();   // INPUT_PULLUP: HIGH->LOW = pressed
        runEdgeUp   = runSw_.rose();   // LOW->HIGH    = released
    }
    if (resetSw_.update()) {
        rstEdgeDown = resetSw_.fell();
        rstEdgeUp   = resetSw_.rose();
    }

    // ----- HANDLE RESUME PROMPT FIRST (consume edges, suppress normal publishing) -----
    if (uiPendingResume_) {
        if (runEdgeDown) {
            // User chose RESUME
            char line[64];
            snprintf(line, sizeof(line),
                     "CMD;RESUME=YES;STEP=%u;LOOP=%u;PHASH=%lu",
                     (unsigned)storedStep_, (unsigned)storedLoopCur_, (unsigned long)storedPhash_);
            ttlComms_.sendMessage(line, MessageType::CRITICAL);

            clearResumeTU();
            haveStoredResume_ = false;
            uiPendingResume_  = false;

            // Publish current switch state so CC sees RUN=1 immediately
            publishSwitchState_(true);
        } else if (rstEdgeDown) {
            // User chose RESTART
            ttlComms_.sendMessage("CMD;RESUME=NO", MessageType::CRITICAL);

            clearResumeTU();
            haveStoredResume_ = false;
            uiPendingResume_  = false;

            // Ensure CC gets the current switch state (likely RUN=0)
            publishSwitchState_(true);
        }
        // NOTE: While the prompt is displayed, skip normal switch publishing & keep-alive below.
    } else {
        // ----- NORMAL SWITCH PUBLISH (no prompt active) -----
        if (runSw_.changed() || resetSw_.changed()) {
            publishSwitchState_();  // reads debounced levels inside
        } else if (millis() - lastSwPublishMs_ > 60000UL) {
            publishSwitchState_(true); // 60s keep-alive
        }
    }

    // ----- SENSORS / CONTROL -----
    updateData();  // MAX31855, etc.

    // Reset countdown ETA at 1 Hz (non-blocking)
    if (resetUiActive_ && resetUiTmr_ >= 1000) {
        resetUiTmr_ = 0;
        if (resetUiRemaining_ > 0) --resetUiRemaining_;
    }

    // Heater PID cadence
    if (pidTmr_ >= 500) {
        pidTmr_ = 0;
        int outVal;
        double pv = isnan(latestSealC_) ? 0.0 : latestSealC_; // TEMP until CC drives SP
        (void)heater_.compute(pv, outVal);
    }

    // STAT heartbeat to ClearCore
    if (heartbeatTmr_ >= 1000) {
        heartbeatTmr_ = 0;
        char line[80];
        const int out = heater_.lastOut();
        snprintf(line, sizeof(line), "STAT;SEQ=%u;OUT=%03d", hbSeq_++, out);
        ttlComms_.sendMessage(line, MessageType::INFO);
    }

    // ----- UI DECISION (exactly one page per tick) -----
    UiPage page;
    if (!ccReady_) {
        page = UiPage::Connecting;
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
    else if (uiPendingResume_) {
        page = UiPage::ResumePrompt;
    } 
    else if (!resetUiActive_ && ccHbSeen_ && ccHbAgeTmr_ > 3000U) {
        page = UiPage::LostComms;
    } 
    else if (resetUiActive_) {
        page = UiPage::ResetCountdown;
    } 
    else {
        page = UiPage::Normal;
    }

    // LCD toggle (used by Normal page)
    if (lcdTmr_ >= lcdToggle_ms_) { lcdTmr_ = 0; lcdToggle_ = !lcdToggle_; }

    // Render exactly one page and flush once
    renderUi_(page);

    // Final retry pump
    ttlComms_.checkRetries();

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
 * @brief Read MAX31855 in °C with fault reporting.
 * @copydetails ExpansionBoard::readTC()
 */
double ExpansionBoard::readTC(Adafruit_MAX31855 &TC, const char *label) {
    double c = TC.readCelsius();
    if (isnan(c)) {
        uint8_t e = TC.readError();
        dbg(label); dbgln(" fault(s):");
        if (e & MAX31855_FAULT_OPEN)      dbgln("  • open circuit");
        if (e & MAX31855_FAULT_SHORT_GND) dbgln("  • short to GND");
        if (e & MAX31855_FAULT_SHORT_VCC) dbgln("  • short to VCC");
        return NAN;
    }
    return c;
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
    latestSumpC_ = 120; //readTC(tc2_, "TC2"); // PLACEHOLDER
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
        case UiPage::Connecting:
            lcd_.setLineCenter(2, "Connecting with CC...");
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

        case UiPage::ResumePrompt:
            lcd_.setLineCenter(0, "Resume previous test?");
            lcd_.setLineCenter(1, "RUN = Resume");
            lcd_.setLineCenter(2, "RESET = Start over");
            lcd_.setLineCenter(3, "");
            break;

        case UiPage::Normal:
        default:
            renderNormal_();   // draws all normal info (no flush here)
            break;
    }

    if (entering) {
        const char* name =
            page == UiPage::Connecting     ? "Connecting"  :
            page == UiPage::Resetting      ? "Resetting"   :
            page == UiPage::LostComms      ? "LostComms"   :
            page == UiPage::EStop          ? "EStop"       :
            page == UiPage::ResumePrompt   ? "Resume?"     :
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
    if (lcdToggle_) {
        strncpy(buff, protocolName_.c_str(), LCDDriver::kNumCols);
        buff[LCDDriver::kNumCols] = '\0';
    } else {
        if (runMins_ < 60) {
            snprintf(buff, sizeof(buff), "%2lu mins", (unsigned long)runMins_);
        } else {
            snprintf(buff, sizeof(buff), "%4.1f hrs", (float)runMins_ / 60.0f);
        }
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
        if (lcdToggle_) {
            int sp  = (int)lround(heater_.setpoint());
            int out = heater_.lastOut();
            snprintf(buff, sizeof(buff), "Heat:%3d\xDF""C OUT:%03d", sp, out);
        } else {
            unsigned long ageSec = (unsigned long)(ccSwAgeMs_ / 1000UL);
            snprintf(buff, sizeof(buff), "SW age:%lus", ageSec);
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
    ttlComms_.sendMessage(msg, MessageType::INFO);   // telemetry / response; no ACK expected

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
    protocolName_ = "NA";
    loopCount_ = 1;

    // 1) PROTOCOL_NAME=...
    String line = csv.readStringUntil('\n');
    if (!line.startsWith("PROTOCOL_NAME=")) { csv.close(); return false; }
    String nameVal = line.substring(strlen("PROTOCOL_NAME=")); stripCommas(nameVal);
    protocolName_ = nameVal;

    // 2) LOOP_COUNT=...
    line = csv.readStringUntil('\n');
    if (!line.startsWith("LOOP_COUNT=")) { csv.close(); return false; }
    String lc = line.substring(strlen("LOOP_COUNT=")); stripCommas(lc);
    if (!isDigits(lc)) { csv.close(); return false; }
    loopCount_ = lc.toInt(); if (loopCount_ == 0) loopCount_ = 1;

    // 3) Skip header row
    csv.readStringUntil('\n');

    // 4) Parse rows
    // inside loadProtocolFromSD_()
    while (csv.available() && stepCount_ < kMaxProtocolSteps_) {
        String row = csv.readStringUntil('\n'); row.trim();
        if (!row.length()) continue;

        int c1 = row.indexOf(',');                 if (c1 < 0) { csv.close(); return false; }
        int c2 = row.indexOf(',', c1 + 1);         if (c2 < 0) { csv.close(); return false; }
        int c3 = row.indexOf(',', c2 + 1);         // may be -1

        String s1 = row.substring(0, c1);          s1.trim();   // RPM
        String s2 = row.substring(c1+1, c2);       s2.trim();   // RPM/s
        String s3 = (c3 < 0) ? row.substring(c2+1)
                            : row.substring(c2+1, c3);          // dwell(s)
        s3.trim();
        String s4 = (c3 < 0) ? String() : row.substring(c3+1);  // TEMP_C (optional)
        s4.trim();

        if (!validSigned(s1) || !isDigits(s2) || !isDigits(s3)) { csv.close(); return false; }
        if (s4.length() && !isDigits(s4)) { csv.close(); return false; }

        Step &st = steps_[stepCount_++];
        st.rpmTarget_ = s1.toInt();
        st.rpmAccel_  = (uint32_t)s2.toInt();
        st.dwellS_    = (uint32_t)s3.toInt();
        st.tempC_     = s4.length() ? (uint16_t)constrain(s4.toInt(), 0, 200) : 0; // clamped to 200degC
    }
    csv.close();

  // 5) Compute PHASH
  uint32_t h = 0;
  h = crc32_update_(h, (const uint8_t*)protocolName_.c_str(), protocolName_.length());
  h = crc32_update_(h, (const uint8_t*)&loopCount_, sizeof(loopCount_));
  h = crc32_update_(h, (const uint8_t*)&stepCount_, sizeof(stepCount_));
  for (uint8_t i=0;i<stepCount_;++i)
    h = crc32_update_(h, (const uint8_t*)&steps_[i], sizeof(Step));
  progHash_ = h;

  return (stepCount_ > 0);
}

/**
 * @brief Print a human-readable summary of the loaded protocol.
 * @copydetails ExpansionBoard::logProtocol_()
 */
void ExpansionBoard::logProtocol_() const {
  dbgln("==== Protocol (XPB) ====");
  dbgkv("Name: ", protocolName_.c_str());
  dbgkv("Loops: ", (unsigned long)loopCount_);
  dbgkv("Steps: ", (unsigned long)stepCount_);
  dbgkv("PHASH: ", (unsigned long)progHash_);
  for (uint8_t i=0;i<stepCount_;++i) {
    // Convert back to human RPM/RPMs for print
    static constexpr uint16_t kStepsPerRev = 3200;
    long rpm   = steps_[i].rpmTarget_;
    long accel = steps_[i].rpmAccel_;
    unsigned dwell = steps_[i].dwellS_;
    uint16_t temp = steps_[i].tempC_;
    char buf[64];
    snprintf(buf, sizeof(buf), "Step %2u: %6ld RPM  %4ld RPM/s  %3us %3udegC", i+1, rpm, accel, dwell, temp);
    dbgln(buf);
  }
}

bool ExpansionBoard::uploadProtocolToCC_() {
    if (stepCount_ == 0) {
        dbgln("[PROTO] No protocol loaded");
        return false;
    }
    
    dbgln("[PROTO] Starting upload to CC...");
    
    // 1. Send PR_BEG
    char msg[96];
    snprintf(msg, sizeof(msg), "PR_BEG;NAME=%s;LOOPS=%lu;STEPS=%u;PHASH=%lu",
             protocolName_.c_str(), 
             (unsigned long)loopCount_, 
             (unsigned)stepCount_, 
             (unsigned long)progHash_);
    
    ttlComms_.sendMessage(msg, MessageType::CRITICAL);
    delay(100);
    ttlComms_.checkForMessages();
    
    // 2. Send PR_DAT chunks (dummy for now)
    dbgln("[PROTO] Sending data chunks...");

    // In XPB uploadProtocolToCC_():
    for (uint8_t i = 0; i < stepCount_; i++) {
        // Format: "SEQ=n;DATA=rpm,accel,dwell"
        snprintf(msg, sizeof(msg), "PR_DAT;SEQ=%u;DATA=%ld,%lu,%lu,%u", 
                i,
                steps_[i].rpmTarget_,
                steps_[i].rpmAccel_, 
                steps_[i].dwellS_,
                steps_[i].tempC_);
        ttlComms_.sendMessage(msg, MessageType::IMPORTANT);
        delay(50);
        ttlComms_.checkForMessages();
    }
    
    // 3. Send PR_END  
    snprintf(msg, sizeof(msg), "PR_END;CRC=%lu", (unsigned long)0);
    ttlComms_.sendMessage(msg, MessageType::CRITICAL);
    delay(100);
    ttlComms_.checkForMessages();
    
    return true;
}
/// ExpansionBoardTTL
/**
 * @brief Handle decoded TTL frames from ClearCore and update owner state.
 * @copydetails ExpansionBoard::ExpansionBoardTTL::onMessageReceived()
 */
/* void ExpansionBoard::ExpansionBoardTTL::onMessageReceived(const String& data) {
    // ---- XPB RX ACK policy (strict) ----
    const int refPos = data.indexOf(F(";REF="));
    const uint16_t ref = refPos > 0 ? (uint16_t)data.substring(refPos + 5).toInt() : 0;

    // 0) Telemetry / one-way notices: NEVER ACK here
    if (data.startsWith("HB;") || data.startsWith("STAT;") ||
        data.startsWith("NOTICE;") || data.startsWith("READY;")) {
        // fall through to existing parsing
    }

    // 1) Request/Response: REQ:SW → reply SW;... (reply = the ACK)
    else if (data.startsWith("REQ:SW")) {
        int ref = -1;
        const int pos = data.indexOf(F(";REF="));
        if (pos > 0) ref = data.substring(pos + 5).toInt();
        if (owner_) owner_->publishSwitchState_(true, ref);
        return;  // important: don't also send an ACK
    }

    // 2) Commands from CC → ACK once (mirror REF if present)
    else if (data.startsWith("CMD;") || data.startsWith("QUIESCE;") || data.startsWith("RESUME?")) {
        if (ref) {
            char ack[28]; snprintf(ack, sizeof(ack), "ACK;OK;REF=%u", ref);
            sendMessage(ack, MessageType::INFO);  // ACKs are not ACKed
        } else {
            sendMessage("ACK;OK", MessageType::INFO);
        }
        // continue into your existing command handling...
    }

    // trace
    if (owner_) {
        owner_->dbg("[RX->XPB] ");
        owner_->dbgkv("", data);
        owner_->dbg("   flags: inject="); owner_->dbgkv("", (unsigned long)(owner_->usbInjecting_ ? 1 : 0));
        owner_->dbg(" sim=");              owner_->dbgkv("", (unsigned long)(owner_->usbSimHold_  ? 1 : 0));
    }

    // Swallow REAL CC HB during SIM-hold
    if (owner_ && !owner_->usbInjecting_) {
        owner_->ccAnySeen_ = true;
    }
    if (owner_ && owner_->usbSimHold_ && !owner_->usbInjecting_ && data.startsWith("HB;")) {
        owner_->ccHbSeen_ = true; owner_->ccHbAgeTmr_ = 0;
        owner_->dbgln("[SIM] swallowed REAL CC HB");
        return;
    }

    // Discovery / readiness: CC announces READY once.
    if (data.startsWith("READY;ID=CC")) {
        if (owner_) {
            owner_->ccReady_ = true;

            // tell CC our current heater OUT immediately
            char line[64];
            snprintf(line, sizeof(line), "STAT;SEQ=%u;OUT=%03d",
                    owner_->hbSeq_++, owner_->heater_.lastOut());
            sendMessage(line, MessageType::INFO);

            // also publish switches so CC sees the current RUN/RST right away
            owner_->publishSwitchState_(true);

            // Optional resume prompt
            if (owner_->haveStoredResume_ && !owner_->suppressResumePrompt_) {
                snprintf(line, sizeof(line), "RESUME?;STEP=%u;LOOP=%u;PHASH=%lu",
                        (unsigned)owner_->storedStep_,
                        (unsigned)owner_->storedLoopCur_,
                        (unsigned long)owner_->storedPhash_);
                sendMessage(line, MessageType::CRITICAL);
                owner_->uiPendingResume_ = true;
            }
        }
        return;
    }

    // ----- Unified command block -----
    if (data.startsWith("CMD;") || data.startsWith("QUIESCE;") || data.startsWith("RESUME?")) {
        // ACK once here (mirror REF if present)
        int ref = -1;
        const int pos = data.indexOf(F(";REF="));
        if (pos > 0) ref = data.substring(pos + 5).toInt();

        if (ref >= 0) {
            char ack[28]; snprintf(ack, sizeof(ack), "ACK;OK;REF=%d", ref);
            sendMessage(ack, MessageType::INFO);
        } else {
            sendMessage("ACK;OK", MessageType::INFO);
        }
    }

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
                // 1) Persist resume snapshot
                const uint16_t stepSnap    = owner_->ccStep_;
                const uint16_t loopCurSnap = (uint16_t)owner_->ccLoopCur_;
                const uint16_t loopTotSnap = (uint16_t)owner_->ccLoopTot_;
                const uint32_t ph          = owner_->progHash_;
                bool ok = saveResumeTU(ph, stepSnap, loopCurSnap, loopTotSnap);
                owner_->dbgln(ok ? "[RESUME] snapshot saved" : "[RESUME] snapshot SAVE FAILED");

                // 2) Ask CC to mask XPB-stale for ~10s (bounded to 3 to 15s on CC)
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

                    // Best-effort: pump for an ACK for ~200 ms
                    uint32_t tWait = millis();
                    while ((uint32_t)(millis() - tWait) < 200U) {
                        owner_->ttlComms_.checkForMessages();
                        owner_->ttlComms_.checkRetries();
                        delay(2);
                    }
                }

                // 3) Mark intentional XPB reset, give SD a moment
                bool fOK = writeResetFlagTU();
                owner_->dbgln(fOK ? "[RESET] flag write OK" : "[RESET] flag write FAIL");
                delay(12);

                // 4) Notify CC, then reset us
                sendMessage("NOTICE;XPB_RESET=NOW", MessageType::NORMAL);
                delay(5);
                xpbSoftResetNow();
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

    // ----- Heartbeat from ClearCore -----
    if (data.startsWith("HB;") && owner_) {

        // If SIM-hold is active and this is a REAL CC HB (not injected),
        // swallow it so USB-injected frames drive the UI.
        if (owner_->usbSimHold_ && !owner_->usbInjecting_) {
            owner_->ccHbSeen_   = true;
            owner_->ccHbAgeTmr_ = 0;
            owner_->dbgln("[SIM] swallowed REAL CC HB");
            return;
        }

        // Parse HB
        String sSTATE = kvGet(data, "STATE=");
        if (sSTATE == "RUNNING") owner_->everRan_ = true;
        if (sSTATE.length()) sSTATE.toCharArray(owner_->ccState_, sizeof(owner_->ccState_));
        
        String sECODE  = kvGet(data, "E_CODE=");
        if (sECODE.length()) {
            long v = sECODE.toInt();
            if (v < 0) v = 0; if (v > 255) v = 255;
            owner_->ccEstopCode_ = (uint8_t)v;
        } else {
            owner_->ccEstopCode_ = 0;  // backward-compat if field absent
        }
        // Derive estop from E_CODE
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
        if (sAGE.length()) owner_->ccSwAgeMs_ = (uint32_t)sAGE.toInt();
        // END parse HB

        // One-time "ready" if HB arrived before READY (common on some boots)
        if (!owner_->ccReady_) {
            owner_->ccReady_ = true;

            // send initial STAT so CC immediately sees our OUT
            char line[64];
            snprintf(line, sizeof(line), "STAT;SEQ=%u;OUT=%03d", owner_->hbSeq_++, owner_->heater_.lastOut());
            sendMessage(line, MessageType::INFO);

            // Offer resume if we truly have one (and haven't already asked)
            if (owner_->haveStoredResume_ && !owner_->suppressResumePrompt_ && !owner_->uiPendingResume_) {
                snprintf(line, sizeof(line), "RESUME?;STEP=%u;LOOP=%u;PHASH=%lu",
                        (unsigned)owner_->storedStep_,
                        (unsigned)owner_->storedLoopCur_,
                        (unsigned long)owner_->storedPhash_);
                sendMessage(line, MessageType::CRITICAL);
                owner_->uiPendingResume_ = true;
            }
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
            (void)saveResumeTU(ph, stepNow, loopNow, (uint16_t)owner_->ccLoopTot_);
        }

        return;
    } 

    // ----- Alarm from ClearCore -----
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
}*/

/**
 * @brief Handle decoded TTL frames from ClearCore and update owner state.
 */
void ExpansionBoard::ExpansionBoardTTL::onMessageReceived(const String& data) {
    // === centralize REF detection; use presence, not value ===
    const int refPos = data.indexOf(F(";REF="));              
    const bool hasRef = (refPos > 0);                         
    const uint16_t refVal = hasRef ?                         
        (uint16_t)data.substring(refPos + 5).toInt() : 0;     

    // Mark that we've seen any CC traffic (for "no link" note)
    if (owner_ && !owner_->usbInjecting_) {            
        owner_->ccAnySeen_ = true;
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

            // Tell CC our current heater OUT immediately (telemetry; no ACK)
            char line[64];
            snprintf(line, sizeof(line), "STAT;SEQ=%u;OUT=%03d",
                     owner_->hbSeq_++, owner_->heater_.lastOut());
            sendMessage(line, MessageType::INFO);             

            // Also publish switches so CC sees the current RUN/RST right away (unsolicited)
            owner_->publishSwitchState_(true);                

            // Optional resume prompt to CC (notice; no ACK expected)
            if (owner_->haveStoredResume_ && !owner_->suppressResumePrompt_) {
                snprintf(line, sizeof(line), "RESUME?;STEP=%u;LOOP=%u;PHASH=%lu",
                         (unsigned)owner_->storedStep_,
                         (unsigned)owner_->storedLoopCur_,
                         (unsigned long)owner_->storedPhash_);
                sendMessage(line, MessageType::NORMAL);
                owner_->uiPendingResume_ = true;
            }
        }
        return;
    }

    // ===== Request/Response: REQ:SW → reply with SW;... (the reply *is* the ACK) =====
    if (data.startsWith("REQ:SW")) { 
        if (owner_) owner_->publishSwitchState_(true, hasRef ? (int)refVal : -1); 
        return;  // important: do NOT send a separate ACK    
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
                    // 1) Persist resume snapshot
                    const uint16_t stepSnap    = owner_->ccStep_;
                    const uint16_t loopCurSnap = (uint16_t)owner_->ccLoopCur_;
                    const uint16_t loopTotSnap = (uint16_t)owner_->ccLoopTot_;
                    const uint32_t ph          = owner_->progHash_;
                    bool ok = saveResumeTU(ph, stepSnap, loopCurSnap, loopTotSnap);
                    owner_->dbgln(ok ? "[RESUME] snapshot saved" : "[RESUME] snapshot SAVE FAILED");

                    // 2) Ask CC to mask XPB-stale for ~10s (bounded to 3..15s on CC)
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

                        // Best-effort: pump for an ACK for ~200 ms
                        uint32_t tWait = millis();
                        while ((uint32_t)(millis() - tWait) < 200U) {
                            owner_->ttlComms_.checkForMessages();
                            owner_->ttlComms_.checkRetries();
                            delay(2);
                        }
                    }

                    // 3) Mark intentional XPB reset, give SD a moment
                    bool fOK = writeResetFlagTU();
                    owner_->dbgln(fOK ? "[RESET] flag write OK" : "[RESET] flag write FAIL");
                    delay(12);

                    // 4) Notify CC, then reset us (notice; no ACK expected)
                    sendMessage("NOTICE;XPB_RESET=NOW", MessageType::NORMAL);
                    delay(5);
                    xpbSoftResetNow();
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
        // Parse HB
        String sSTATE = kvGet(data, "STATE=");
        if (sSTATE == "RUNNING") owner_->everRan_ = true;
        if (sSTATE.length()) sSTATE.toCharArray(owner_->ccState_, sizeof(owner_->ccState_));

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
        if (sAGE.length()) owner_->ccSwAgeMs_ = (uint32_t)sAGE.toInt();

        // One-time "ready" if HB arrived before READY (common on some boots)
        if (!owner_->ccReady_) {
            owner_->ccReady_ = true;

            // send initial STAT so CC immediately sees our OUT (telemetry; no ACK)
            char line[64];
            snprintf(line, sizeof(line), "STAT;SEQ=%u;OUT=%03d",
                     owner_->hbSeq_++, owner_->heater_.lastOut());
            sendMessage(line, MessageType::INFO);

            // Offer resume if we truly have one (notice; no ACK)
            if (owner_->haveStoredResume_ && !owner_->suppressResumePrompt_ && !owner_->uiPendingResume_) {
                snprintf(line, sizeof(line), "RESUME?;STEP=%u;LOOP=%u;PHASH=%lu",
                         (unsigned)owner_->storedStep_,
                         (unsigned)owner_->storedLoopCur_,
                         (unsigned long)owner_->storedPhash_);
                sendMessage(line, MessageType::NORMAL);
                owner_->uiPendingResume_ = true;
            }
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
            (void)saveResumeTU(ph, stepNow, loopNow, (uint16_t)owner_->ccLoopTot_);
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
 * @copydetails ExpansionBoard::ExpansionBoardTTL::onBadChecksum()
 */
void ExpansionBoard::ExpansionBoardTTL::onBadChecksum(const String&) {
    ++badCrcCount_;
    if (badCrcCount_ % 10 == 1 && owner_) {
        owner_->dbgln("WARN: TTL bad checksum (rate-limited)");
    }
}

/// HeatingController
/**
 * @brief Set heater setpoint.
 * @copydetails ExpansionBoard::HeatingController::setTargetTemp()
 */
void ExpansionBoard::HeatingController::setTargetTemp(double celsius) {
    sp_ = celsius;
    active_ = (celsius > 0);
}

/**
 * @brief Execute one PID compute and output integer result.
 * @copydetails ExpansionBoard::HeatingController::compute()
 */
bool ExpansionBoard::HeatingController::compute(double processValue, int &outInt) {
    pv_ = processValue;
    if (!active_) { out_ = 0; outInt = 0; return true; }
    bool did = pid_.Compute();
    if (did) outInt = (int)lround(out_);
    return did;
}