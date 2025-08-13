#include "ExpansionBoard.h"
#include <Arduino.h>
#include <SPI.h>


// public:
bool ExpansionBoard::begin() {
    Serial.begin(9600);

    unsigned long t0 = millis();
    while (!Serial && (millis() - t0 < 5000)) { /* spin */}

    dbgln("\nUSB Serial Monitor Connected!");

    runSw_.attach(RUN_SW_PIN, INPUT_PULLUP);
    runSw_.interval(25);
    resetSw_.attach(RESET_SW_PIN, INPUT_PULLUP);
    resetSw_.interval(25);

    if (!lcd_.begin()) {
        dbgln("FATAL: LCD initialization failed!");
        return false;
    }

    dbg("Initializing MAX31855 sensor - TC1...");
    delay(500); // stabilize
    if (!tc1_.begin()) {
        dbgln("ERROR.");
        while (1) delay(10);
    } else dbgln("DONE");
    tc1_.setFaultChecks(MAX31855_FAULT_ALL);

    // uncomment out when the 2nd sensor is installed
    /*
    Serial.print("Initializing MAX31855 sensor - TC2...");
    if (!tc2_.begin()) {
        Serial.println("ERROR.");
        while (1) delay(10);
    } else Serial.println("DONE");
    tc2_.setFaultChecks(MAX31855_FAULT_ALL);
    */

    ttlComms_.begin();
    ttlComms_.setRxUsbLogging(true, "CC");
    delay(200);
    dbgln("Connecting with CC..");
    lcd_.setLineCenter(0, "Connecting with CC..");
    lcd_.flush();

    uint32_t lastTx = 0;
    t0 = millis();
    while (!ccReady_ && millis() - t0 < 12000UL) {        // 12s window
        ttlComms_.checkForMessages();
        ttlComms_.checkRetries();
        if (millis() - lastTx >= 200) {                   // 200 ms cadence
            ttlComms_.sendMessage("HELLO;ID=XPB", MessageType::INFO);
            lastTx = millis();
        }
    }
    // short grace period to catch an in-flight READY
    uint32_t tGrace = millis();
    while (!ccReady_ && millis() - tGrace < 300) {
        ttlComms_.checkForMessages();
        ttlComms_.checkRetries();
    }

    if (!ccReady_) {
        dbgln("WARN: ClearCore not ready; continuing without link.");  // no hard halt
        lcd_.setLineCenter(2, "No CC link!!");
    }
    lcd_.setLineCenter(2, "ClearCore READY");
    lcd_.flush();
    delay(1000);
    dbgln("ClearCore READY");

    // DEBUGGING ONLY //
    heater_.begin(); // maybe only do this when a test is started or pre-heating begins?
    heater_.setTargetTemp(32.0); // debug only, will come from ClearCore heartbeat / step updates

    publishSwitchState_(true);
    return true;
}

void ExpansionBoard::tick() {
    // Comms housekeeping
    ttlComms_.checkForMessages();
    ttlComms_.checkRetries();

    // Keep background discovery alive while not ready
    static uint32_t lastHello = 0;
    if (!ccReady_ && (millis() - lastHello >= 1000)) {
        ttlComms_.sendMessage("HELLO;ID=XPB", MessageType::INFO);
        lastHello = millis();
    }

    // ---- INPUTS / SENSORS / CONTROL ----
    // Debounce switches and publish if changed (also periodic keep-alive)
    bool changed = false;
    if (runSw_.update())   changed |= runSw_.changed();
    if (resetSw_.update()) changed |= resetSw_.changed();
    if (changed) {
        publishSwitchState_();
    }
    if (millis() - lastSwPublishMs_ > 60000UL) {
        publishSwitchState_(true);
    }

    // Sensor sampling (MAX31855 etc.)
    updateData();

    // Reset countdown UI ETA at 1 Hz (non-blocking)
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
    if (heartbeatTmr_ >= 2000) {
        heartbeatTmr_ = 0;
        char line[80];
        const int out = heater_.lastOut();
        snprintf(line, sizeof(line), "STAT;SEQ=%u;OUT=%03d", hbSeq_++, out);
        ttlComms_.sendMessage(line, MessageType::IMPORTANT);
        ttlComms_.checkForMessages(); // receive fast ACK
    }

    // ---- UI DECISION (one page per tick) ----
    UiPage page;
    if (!ccReady_) {
        page = UiPage::Connecting;
    } else if (ccEstop_) {
        page = (ccHbSeen_ && ccHbAgeTmr_ > 1000U) ? UiPage::Resetting : UiPage::EStop;
    } else if (!resetUiActive_ && ccHbSeen_ && ccHbAgeTmr_ > 3000U) {
        page = UiPage::LostComms;
    } else if (resetUiActive_) {
        page = UiPage::ResetCountdown;
    } else {
        page = UiPage::Normal;
    }

    // LCD toggle (used by Normal page)
    if (lcdTmr_ >= lcdToggle_ms_) { lcdTmr_ = 0; lcdToggle_ = !lcdToggle_; }

    // Render exactly one page and flush once
    renderUi_(page);

    // Optional final retry pump
    ttlComms_.checkRetries();
}

void ExpansionBoard::setDataInterval(uint16_t milli_secs) {
    kDataIntervalMs_ = milli_secs;
}

/*********************/

// private:

double ExpansionBoard::readTC(Adafruit_MAX31855 &TC, const char *label) {
  double c = TC.readCelsius();
  if (isnan(c)) {
    uint8_t e = TC.readError();
    if (Serial) {
        Serial.print(label);
        Serial.println(" fault(s):");
        if (e & MAX31855_FAULT_OPEN)      Serial.println("  • open circuit");
        if (e & MAX31855_FAULT_SHORT_GND) Serial.println("  • short to GND");
        if (e & MAX31855_FAULT_SHORT_VCC) Serial.println("  • short to VCC");
    }
    return NAN;
  }
  return c;
}

void ExpansionBoard::updateData() {
    /** Reads all onboard sensors **/
    if (dataTmr_ < kDataIntervalMs_) return;
    dataTmr_ = 0;

    latestSealC_ = readTC(tc1_, "TC1");
    latestSumpC_ = 120; //readTC(tc2_, "TC2"); // PLACEHOLDER
}

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
            lcd_.setLineCenter(0, "!!! E-STOP !!!");
            lcd_.setLineCenter(1, ccAlarmActive_ ? ccAlarmMsg_ : (char*)"Fault asserted");
            lcd_.setLineCenter(2, "Reset on controller");
            lcd_.setLineCenter(3, "to clear alarm");
            break;

        case UiPage::ResetCountdown: {
            char buff[LCDDriver::kNumCols+1];
            lcd_.setLineCenter(0, "RESETTING...");
            snprintf(buff, sizeof(buff), "in %us", (unsigned)resetUiRemaining_);
            lcd_.setLineCenter(1, buff);
            lcd_.setLineCenter(2, "Return switch to");
            lcd_.setLineCenter(3, "center to cancel.");
            break;
        }

        case UiPage::Normal:
        default:
            renderNormal_();   // draws all normal info (no flush here)
            break;
    }

    lcd_.flush();
    lastUi_ = page;
}

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

void ExpansionBoard::publishSwitchState_(bool force) {
    int runActive   = (runSw_.read()   == LOW) ? 1 : 0;
    int resetActive = (resetSw_.read() == LOW) ? 1 : 0;

    static int lastRun = -1, lastReset = -1;
    if (!force && runActive == lastRun && resetActive == lastReset) return;

    lastRun = runActive;
    lastReset = resetActive;

    char msg[32];
    snprintf(msg, sizeof(msg), "SW;RUN=%d;RST=%d", runActive, resetActive);
    ttlComms_.sendMessage(msg, MessageType::CRITICAL);

    lastSwPublishMs_ = millis();
}