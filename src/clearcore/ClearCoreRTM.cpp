#include "ClearCore-RTM.h"

bool ClearCoreRTM::begin() {
    /* USB Serial Comms for Debugging */
    SerialPort.Mode(Connector::USB_CDC);
    SerialPort.Speed(9600);
    SerialPort.PortOpen();

    uint32_t t0 = Milliseconds();
    while (!SerialPort && Milliseconds() - t0 < 5000) {}

    dbgln("SerialReady");

    /* GPIO */
    PRGM_RUN_BUS_PIN.Mode(Connector::INPUT_DIGITAL);
    PRGM_RESET_BUS_PIN.Mode(Connector::INPUT_DIGITAL);
    SAFETY_PIN.Mode(Connector::INPUT_DIGITAL);
    LED_PIN.Mode(Connector::OUTPUT_DIGITAL);
    LED_PIN.State(true);
    dbgln("GPIO ready");

    /* MOTOR */
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
    MotorMgr.MotorModeSet(MotorManager::MOTOR_M0M1, Connector::CPM_MODE_STEP_AND_DIR);
    motor.HlfbMode(MotorDriver::HLFB_MODE_STATIC);
    motor.VelMax(kMotorMaxRpm * kStepsPerRev / 60);
    motor.AccelMax(kMotorMaxRpm * kStepsPerRev / 60);
    motor.EStopDecelMax(kMotorMaxRpm * kStepsPerRev / 60);
    dbgln("Motor ready");

    /* SD CARD */
    if (!SD.begin()) {
        dbgln("SD begin failed");
        return false;
    }
    dbgln("SD ready");

    Delay_ms(250);

    // ----------- LOAD PROTOCOL ---------
    Delay_ms(250);

    File csv = SD.open("protocol.csv", FILE_READ);
    if (!loadProtocol(csv)) {
        dbgln("Load config failed");
        return false;
    }
    csv.close();
    dbgln("Load config done");
    Delay_ms(250);

    /* TTL Comms */
    ttlComms_.begin();
    ttlComms_.setRxUsbLogging(true, "XPB");

    // --- NEW: Proactively announce readiness and request switch state ---
    {
        char line[64];
        snprintf(line, sizeof(line), "READY;ID=CC;VER=1.0;UPT=%lu",
                 (unsigned long)Milliseconds());
        ttlComms_.sendMessage(line, MessageType::CRITICAL);
        ttlComms_.sendMessage("REQ:SW", MessageType::CRITICAL);
    }
    // Briefly service RX so the XPB sees this immediately (avoids boot races)
    {
        uint32_t tReady = Milliseconds();
        while (Milliseconds() - tReady < 150) {
            ttlComms_.checkForMessages();
            ttlComms_.checkRetries();
        }
    }
    dbgln("TTL Ready");

    dwellTmr_ = 0;
    return true;
}

void ClearCoreRTM::tick() {
    bool eStopActive  = !SAFETY_PIN.State();
    bool runActive    = runActiveRemote_;
    bool resetActive  = resetActiveRemote_;

    // heartbeat to exp-board here, not sure of minimum interval needed

    // first check for E-Stop
    if (eStopActive && state_ != State::EStop) {
        state_ = State::EStop;
    }

    // check for reset request
    if (resetActive && !prevResetActive_ && state_ != State::EStop) {
        if (state_ != State::Running) {
            // capture re-reset state so it can be restored later
            preReset_ = state_;
            state_ = State::ResetRequested;
            //renderScreen();
            resetTmr_ = 0;
        }
    }
    prevResetActive_ = resetActive;

    // transition logic for pause / resume / start
    if (!runActive && !resetActive && state_ == State::Running) {
            // middle‐position ⇒ Pause
            state_ = State::Paused;
    }
    else if (runActive && state_ == State::Paused) {
            // User selects Run position again ⇒ Resume
            state_ = State::Resume;
    }
    else if (runActive && state_ == State::Idle) {
            // User selects Run position for first time ⇒ Running
            state_ = State::Running;
    }

    bool justEntered_ = (state_ != prevState_); // did we just state change?
    prevState_        = state_; // capture previous state
    // justEntered_ allows us to do things once upon first entering a state handler
    // this prevents needlessly firing screen updates or other logic every tick.
    // It also allows us to immediately update a screen the instant we change a state.

    // dispatch to state handlers
    switch (state_) {
        case State::Idle:
            handleIdle(runActive, justEntered_);
            break;
        case State::Running:
            handleRunning(runActive, justEntered_);
            break;
        case State::Paused:
            handlePaused(runActive, justEntered_);
            break;
        case State::ResetRequested:
            handleReset(resetActive, justEntered_);
            break;
        case State::EStop:
            handleEStop(resetActive, justEntered_);
            break;
        case State::Resume:
            handleResume(runActive, justEntered_);
            break;
        case State::Completed:
            handleCompleted(resetActive, justEntered_);
            break;
        default:
            break;
    }

    if (heartbeatTmr_ >= 1000) {
        heartbeatTmr_ = 0;
        // ttcComms_.sendMessag(SEND HEARTBEAT INFO HERE)
        const char *stateStr = stateToString(state_);
        char msg[96];
        snprintf(msg, sizeof(msg),
                 "HB;SEQ=%u;STATE=%s;E=%d;STEP=%u;LOOP=%lu/%lu;SW_AGE=%lu",
                 hbSeq_++,
                 stateStr,
                 eStopActive ? 1 : 0,
                 (unsigned)(currentStep_ + 1),
                 (unsigned long)(totalLoops_ - loopCount_ + 1),
                 (unsigned long)totalLoops_,
                 (unsigned long)(Milliseconds() - swLastUpdateMs_));
        ttlComms_.sendMessage(msg, MessageType::INFO);
        ttlComms_.checkForMessages(); // receive fast ACK
        //dbgln(msg);
    }
    ttlComms_.checkForMessages();
    ttlComms_.checkRetries();
    
}

// this will drastically change once exp-board is reading protocol
bool ClearCoreRTM::loadProtocol(File &csv) {
    if (!csv) return false;

    // 0) validation helpers
    // speed can be negative (CW or CCW)
    auto validSigned = [&](const String &s) {
        if (s.length() < 1) return false;
        for (uint16_t i = 0; i < s.length(); ++i) {
        char c = s.charAt(i);
        if (i == 0 && c == '-') continue;
        if (!isDigit(c)) return false;
        }
        return true;
    };

    // acceleration and dwell time must be positive integers
    auto isDigits = [&](const String &s) {
        if (s.length() == 0) return false;
        for (uint16_t i = 0; i < s.length(); ++i) if (!isDigit(s.charAt(i))) return false;
        return true;
    };

    // For stripping any leading or trailing commas (and then trim spaces), excel adds them to CSVs
    auto stripCommas = [&](String &s){
        s.trim();
        while (s.startsWith(",")) s = s.substring(1), s.trim();
        while (s.endsWith(","))   s.remove(s.length() - 1), s.trim();
    };
    
    // 1) Protocol name line – PROTOCOL_NAME=XXX
    String line = csv.readStringUntil('\n');
    const char *pfxName = "PROTOCOL_NAME=";
    if (!line.startsWith(pfxName)) return false;
    String nameVal = line.substring(strlen(pfxName));
    stripCommas(nameVal);
    protocolName_ = nameVal;

    // 2) Loop count – LOOP_COUNT=N
    line = csv.readStringUntil('\n');
    const char *pfxLoop = "LOOP_COUNT=";
    if (!line.startsWith(pfxLoop)) return false;
    String lc = line.substring(strlen(pfxLoop));
    stripCommas(lc);
    if (!isDigits(lc)) return false;
    loopCount_ = lc.toInt();
    if (loopCount_ == 0) loopCount_ = 1; // safeguard

    // 3) Skip header row
    csv.readStringUntil('\n');

    // 4) Iterate Protocol Steps found in CSV file
    stepCount_ = 0;
    while (csv.available() && stepCount_ < kMaxProtocolSteps) {
        String row = csv.readStringUntil('\n');
        row.trim();
        if (row.length() == 0) continue; // skip blanks

        int c1 = row.indexOf(',');
        int c2 = row.indexOf(',', c1 + 1);
        if (c1 < 0 || c2 < 0) return false; // malformed line

        // extract fields
        String s1 = row.substring(0, c1); // target speed
        String s2 = row.substring(c1 + 1, c2); // acceleration
        String s3 = row.substring(c2 + 1); // dwell time
        s1.trim(); s2.trim(); s3.trim();
  
        // validate inputs
        if (!validSigned(s1) || !isDigits(s2) || !isDigits(s3)) return false;

        // convert
        int32_t  rpmTarget = s1.toInt();
        uint32_t rpmAccel  = s2.toInt();
        uint32_t dwellS    = s3.toInt();

        Step &s = steps_[stepCount_++];
        s.speedSteps_s = (rpmTarget >= 0) // round‐nearest
                       ? (rpmTarget * kStepsPerRev + 30) / 60
                       : (rpmTarget * kStepsPerRev - 30) / 60;
        s.accelSteps_s2 = ((rpmAccel  * kStepsPerRev + 30) / 60);
        s.dwellMs       = dwellS     * 1000UL;
    }

    // 5) Print out protocol to Serial
    dbgln("==== Loaded Protocol ====");
    dbgkv("Protocol Name: ", protocolName_.c_str());
    dbgkv("Loop Count: ", loopCount_);
    dbgkv("Step Count: ", stepCount_);


    totalLoops_ = loopCount_; // to help display current test state

    // 6) Prints entire protocol to Terminal for debugging purposes
    for (uint8_t i = 0; i < stepCount_; ++i) {
        int32_t rpm = (steps_[i].speedSteps_s * 60 + (steps_[i].speedSteps_s >= 0 ? kStepsPerRev / 2 : -kStepsPerRev / 2)) / kStepsPerRev;
        uint32_t accel = (steps_[i].accelSteps_s2 * 60 + kStepsPerRev / 2) / kStepsPerRev;
        uint32_t dwell = steps_[i].dwellMs / 1000;

        snprintf(debugBuf_, sizeof(debugBuf_), "Step %2u: %6ld RPM  %4lu RPM/s²  %3lu s",
                 i + 1, rpm, accel, dwell);
        dbgln(debugBuf_);
    }

    dbgln("=========================");
    
    return (stepCount_ > 0);
}

/* ——— State Handlers ——— */
void ClearCoreRTM::handleIdle(bool, bool justEntered_) {
    if (justEntered_) {
        //renderScreen(); // no need to update the screen before a test starts
    }
    
    // flash LED slowly
    if (ledTmr_ > 500) {
        ledTmr_ = 0;
        LED_PIN.State(!LED_PIN.State());
    }

    // toggle display every 3S
    if (lcdTmr_ > lcdToggle_ms_ && state_ != State::EStop) {
        lcdTmr_ = 0;
        lcdToggle_ = !lcdToggle_;
        //renderScreen();
        testRunTmr_ = 0; // prevent run timer from ticking
    }
    return;
}

void ClearCoreRTM::handleRunning(bool runActive, bool justEntered_) {
    if (justEntered_) {
        // solid LED
        LED_PIN.State(true);
        targetMet_ = false; // false during a ramp to target speed
        //renderScreen();
    }
    
    if (testRunTmr_ >= 60000) {
        testRunTmr_ = 0;
        ++runMins_; // we are tracking total minutes of test runtime
    }

    // If rocker switch moved to 'pause'
    if (!runActive) {
        state_ = State::Paused;
        currentSpeed_ = motor.VelocityRefCommanded();
        pause_time_ = dwellTmr_; // to store when we left the dwell
        //renderScreen();
        return;
    }

    // Initialize new step (only once)
    if (!stepInit_) {
        motor.EnableRequest(true);
        targetAccel_ = steps_[currentStep_].accelSteps_s2;
        targetSpeed_ = steps_[currentStep_].speedSteps_s;
        motor.AccelMax(targetAccel_);
        motor.MoveVelocity(targetSpeed_);
        if (targetSpeed_ == 0) motor.MoveStopDecel(targetAccel_);
        
        stepInit_ = true;
        targetMet_ = false;
    }

    // ramp up to target speed
    if (!targetMet_) {
        if (steps_[currentStep_].speedSteps_s != 0) {
            // handle direction changes, the are thresholding speed above 99% to ensure we catch it.
            int32_t target = steps_[currentStep_].speedSteps_s;
                if (target > 0) {
                    if (motor.VelocityRefCommanded() >=  target * 0.99f) {
                        targetMet_ = true;
                        dwellTmr_  = 0;
                    }
                } else if (target < 0) {
                    if (motor.VelocityRefCommanded() <= target * 0.99f) {
                        targetMet_ = true;
                        dwellTmr_  = 0;
                    }
                }
        } else {
            // special case: target is zero -> wait for full stop
            if (motor.StepsComplete()) {
                targetMet_ = true;
                dwellTmr_ = 0;
            }
        }
    }

    // hold that speed (or stop) for dwellMs
    else {
        if ((uint32_t)dwellTmr_ >= steps_[currentStep_].dwellMs) {
            // done dwelling -> go to next step
            stepInit_ = false; // reinitialize next step
            uint8_t prev = currentStep_;
            currentStep_ = (currentStep_ + 1) % stepCount_;
            targetMet_ = false;
            dwellTmr_ = 0;

            // if we just wrapped past the last step
            if (prev == stepCount_ - 1) { // check for end of protocol steps
                if (--loopCount_ == 0) { // then decrement loops and see if test is done
                    state_ = State::Completed;
                    return;
                }
            }
            //renderScreen(); // ensure we render the start of the dwell timer
            return;
        }
    }

    if (lcdTmr_ > lcdToggle_ms_ && state_ != State::EStop) {
        // while running the LCD will toggle info every lcdToggle_ms_ milliseconds
        lcdTmr_ = 0;
        lcdToggle_ = !lcdToggle_;
        //renderScreen();
    }
}

void ClearCoreRTM::handlePaused(bool runActive, bool justEntered_) {
    if (justEntered_) {
        //renderScreen();
        motor.MoveStopDecel((1000 * kStepsPerRev) / 60); // decel to 0 RPM
    }

    // flash LED slowly
    if (ledTmr_ > 500) {
        ledTmr_ = 0;
        LED_PIN.State(!LED_PIN.State());
    }

    // toggle display info
    if (lcdTmr_ > lcdToggle_ms_ && state_ != State::EStop) {
        lcdTmr_ = 0;
        lcdToggle_ = !lcdToggle_;
        //renderScreen();
    }

    if (motor.StepsComplete()) {
        // spin motor down to full stop
        if (runActive) {
            state_ = State::Resume;
            //renderScreen();
        } else {
            motor.EnableRequest(false);
        }
    }
    return;
}

void ClearCoreRTM::handleReset(bool resetActive, bool justEntered_) {
    if (!resetActive) {
        ttlComms_.sendMessage("CMD;RESET=CANCEL", MessageType::CRITICAL);
        motor.EnableRequest(true);
        if (state_ != preReset_) {
            prevState_ = State::Debug; // force a mismatch, check o3 to see if this makes sense anymore!
        }
        state_ = preReset_;           // restore previous state
        resetPhase_ = ResetPhase::Idle;
        return;
    }

    if (justEntered_) {
        // 1) Arm: ask XPB to show countdown UI
        char line[40];
        snprintf(line, sizeof(line), "CMD;RESET=ARM;SECS=%u", (unsigned)resetArmSecs_);
        ttlComms_.sendMessage(line, MessageType::CRITICAL);
        resetPhase_ = ResetPhase::Armed;
        resetTmr_ = 0;
        xpbBootSeen_ = false;
    }

    switch (resetPhase_) {
        case ResetPhase::Armed:
            // Let XPB own the countdown visuals; we just wait out the time.
            if (resetTmr_ >= (uint32_t)resetArmSecs_ * 1000UL) {
                // 2) Tell XPB to actually reset now
                ttlComms_.sendMessage("CMD;RESET=EXEC", MessageType::CRITICAL);
                resetPhase_ = ResetPhase::ExecSent;
                xpbBootWaitTmr_ = 0;
            }
            break;

        case ResetPhase::ExecSent:
            // 3) Wait a bit for XPB to reboot and announce itself
            if (xpbBootSeen_ || xpbBootWaitTmr_ >= 1500) {
                // 4) Now reset ClearCore itself
                // (Optional: briefly tell XPB we're about to reset, but EXEC already happened.)
                SysMgr.ResetBoard(); // we won’t return
            }
            break;

        default:
            break;
    }

    if (ledTmr_ > 100) { // rapidly flash LED
        ledTmr_ = 0;
        LED_PIN.State(!LED_PIN.State());
    }
    return;
}

void ClearCoreRTM::handleEStop(bool resetActive, bool justEntered_) {
    
    if (justEntered_) {
        /* lcdClearScreen();
        lcdLineCenter(0, "!!! E-STOP !!!");
        lcdLineCenter(1, "Press Reset to Clear");
        lcdLineBlank (2);
        lcdLineCenter(3, "Test is now void!");
        lcdFlush(); */
        motor.MoveStopAbrupt();
        motor.EnableRequest(false);
    }
    
    // flash LED rapidly
    if (ledTmr_ > 50) {
        ledTmr_ = 0;
        LED_PIN.State(!LED_PIN.State());
    }

    if (resetActive) {
        /* lcdClearScreen();
        lcdLineCenter(1, "Resetting board...");
        lcdFlush(); */
        SysMgr.ResetBoard();
    }
    return;
}

void ClearCoreRTM::handleResume(bool runActive, bool justEntered_) {
    if (justEntered_) {
        motor.EnableRequest(true);
        motor.AccelMax(targetAccel_);
        motor.MoveVelocity(currentSpeed_); // restore speed when paused
        //renderScreen();
    }

    if (motor.StepsComplete()) { // resume running once back up to speed
        dwellTmr_ = pause_time_;
        state_ = State::Running;
        motor.MoveVelocity(targetSpeed_);
        resumeFromPause_ = true;
        //renderScreen();
    }
}

void ClearCoreRTM::handleCompleted(bool resetActive, bool justEntered_) {
    if (justEntered_) {
        //renderScreen();
        motor.MoveStopDecel(targetAccel_); // decel to 0 RPM
    }

    if (motor.StepsComplete()) {
        motor.EnableRequest(false);
    }

    return;
}