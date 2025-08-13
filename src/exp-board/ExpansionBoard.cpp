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
    lcd_.setLineCenter(2, "Connecting with CC..");
    lcd_.flush();

    uint32_t lastTx = 0;
    t0 = 0;
    while (!ccReady_ && millis() - t0 < 8000) {
        ttlComms_.checkForMessages();
        ttlComms_.checkRetries();
        if (millis() - lastTx >= 500) {
            ttlComms_.sendMessage("HELLO;ID=XPB", MessageType::INFO);
            lastTx = millis();
        }
    }
    // tiny grace spin to catch just-sent READY frames
    uint32_t tGrace = millis();
    while (!ccReady_ && millis() - tGrace < 200) {
        ttlComms_.checkForMessages();
    }

    if (!ccReady_) {
        dbgln("WARN: ClearCore not ready; halting.");
        while (1) { /* show error or blink */ }
    }
    dbgln("ClearCore READY");

    // DEBUGGING ONLY //
    heater_.begin(); // maybe only do this when a test is started or pre-heating begins?
    heater_.setTargetTemp(32.0); // debug only, will come from ClearCore heartbeat / step updates

    publishSwitchState_(true);
    return true;
}

void ExpansionBoard::tick() {
    ttlComms_.checkForMessages();

    bool updated = false;
    if (runSw_.update())    updated = true;
    if (resetSw_.update())  updated = true;
    if (updated && (runSw_.changed() || resetSw_.changed())) {
        publishSwitchState_(); // only sends if state actually changed
    }
    if (millis() - lastSwPublishMs_ > 60000UL) {
        publishSwitchState_(true); // periodic keep-alive for switch state (1 min)
    }

    updateData(); // reads all on-board sensors

    // Update reset UI ETA at 1 Hz (non-blocking)
    if (resetUiActive_ && resetUiTmr_ >= 1000) {
        resetUiTmr_ = 0;
        if (resetUiRemaining_ > 0) {
            --resetUiRemaining_;
        }
    }

    if (lcdTmr_ >= lcdToggle_ms_) {
        lcdTmr_ = 0;
        lcdToggle_ = !lcdToggle_;
    } renderScreen();

    if (pidTmr_ >= 500) {
        pidTmr_ = 0;
        int outVal;
        // double pv = isnan(latestSumpC_) ? 0 : latestSumpC_;
        double pv = isnan(latestSealC_) ? 0 : latestSealC_; // DEBUGGING ONLY!!!
        (void)heater_.compute(pv, outVal);
    }

    if (heartbeatTmr_ >= 2000) {
        heartbeatTmr_ = 0;

        // build up heatbeat data for clearcore
        char line[80];
        const int out = heater_.lastOut();
        snprintf(line, sizeof(line),
                 "STAT;SEQ=%u;OUT=%03d", hbSeq_++, out);
        ttlComms_.sendMessage(line, MessageType::IMPORTANT);
        ttlComms_.checkForMessages(); // recieve fast ACK
    }
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

void ExpansionBoard::renderScreen() {
    char buff[LCDDriver::kNumCols+1];

    if (resetUiActive_) {
        lcd_.setLineCenter(0, "RESETTING...");
        snprintf(buff, sizeof(buff), "in %us", (unsigned)resetUiRemaining_);
        lcd_.setLineCenter(1, buff);
        lcd_.setLineCenter(2, "Return switch to");
        lcd_.setLineCenter(3, "center to cancel.");
        lcd_.flush();
        return;
    }

    // Line 0: left = protocol name OR runtime; right = CC state (or E-STOP)
    if (lcdToggle_) {
        // protocol name
        strncpy(buff, protocolName_.c_str(), LCDDriver::kNumCols);
        buff[LCDDriver::kNumCols] = '\0';
    } else {
        // runtime minutes/hours
        if (runMins_ < 60) {
            snprintf(buff, sizeof(buff), "%2lu mins", (unsigned long)runMins_);
        } else {
            snprintf(buff, sizeof(buff), "%4.1f hrs", (float)runMins_ / 60.0f);
        }
    }
    lcd_.setLineLR(0, buff, ccEstop_ ? "E-STOP" : ccState_);

    // Line 1: step & loop
    if (ccLoopTot_ > 0) {
        // "STEP:xx  Loop:cur/tot"
        snprintf(buff, sizeof(buff), "STEP:%2u  Loop:%lu/%lu",
                 (unsigned)ccStep_,
                 (unsigned long)ccLoopCur_,
                 (unsigned long)ccLoopTot_);
    } else {
        // "STEP:xx  Loop:cur"
        snprintf(buff, sizeof(buff), "STEP:%2u  Loop:%lu",
                 (unsigned)ccStep_,
                 (unsigned long)ccLoopCur_);
    }
    lcd_.setLineLeft(1, buff);
    
    /// dummy data ///
    float rpm = 2560;
    int16_t v = 2123;
    uint16_t a = 500;
    targetMet_ = true;

    /* float rpm = static_cast<float>(targetSpeed_) * 60.0f / kStepsPerRev;
    int16_t v = (rpm >= 0.0f) ? static_cast<int16_t>(rpm + 0.5f) : static_cast<int16_t>(rpm - 0.5f);
    uint16_t a = static_cast<uint16_t>((static_cast<float>(targetAccel_) * 60.0f / kStepsPerRev) + 0.5f); */

    // Line 2 & 3 content
    if (modeTorqueToggle_) {
        // Torque-stand view
        lcd_.setLineLR(2, "RPM/s    RPM", "Dwell");

        // TODO: replace these with HB-fed values once CC publishes them.
        // For now, placeholders so the layout is correct.
        uint16_t acc = 500;  // e.g., ccAccel_
        int16_t  rpm = 2123; // e.g., ccRpm_
        char     dwellRight[8] = ""; // e.g., snprintf(dwellRight, sizeof, "%4us", ccDwellRemS_);

        char left[21];
        snprintf(left, sizeof(left), "%5u  %5d", (unsigned)acc, (int)rpm);
        lcd_.setLineLR(3, left, dwellRight);
    }
        // deal with this LATER!
        /* if (!targetMet_) {
            snprintf(dwell_buf, sizeof(dwell_buf), "ramp");
        } else if (state_ == State::Paused) {
            uint16_t t = (steps_[currentStep_].dwellMs - pause_time_) / 1000;
            snprintf(dwell_buf, sizeof(dwell_buf), "%4us", t);
        } else {
            uint16_t t = (steps_[currentStep_].dwellMs - dwellTmr_) / 1000;
            snprintf(dwell_buf, sizeof(dwell_buf), "%4us", t);
        } */

    else {
        // RTM view (HB-driven)
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
            snprintf(buff, sizeof(buff), "Seal:%3u\xDF""C Sump:%2u\xDF""C", (unsigned)sealInt, (unsigned)sumpInt);
        } else {
            snprintf(buff, sizeof(buff), "Seal:%3u\xDF""C Sump:%3uC", (unsigned)sealInt, (unsigned)sumpInt);
        }
        lcd_.setLineLeft(3, buff);
    }
    
    // commit
    lcd_.flush();
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