#include "ClearCoreRTM.h"

static constexpr uint16_t STAT_PERIOD_MS = 1000;
static constexpr uint8_t  STALE_MULT     = 5;

const char* const ClearCoreRTM::kStateNames[11] = {
    "DEBUG",            // State::Debug
    "BOOT",             // State::BOOT
    "PROTO_LOADING",    // State::PROTO_LOADING
    "IDLE",             // State::Idle
    "PREHEAT",          // State::Preheat
    "RUNNING",          // State::Running
    "PAUSED",           // State::Paused
    "RESETTING",        // State::Resetting
    "RESUME",           // State::Resume
    "COMPLETED",        // State::Completed
    "E-STOP"            // State::EStop
};
static_assert(
    static_cast<size_t>(ClearCoreRTM::State::EStop) + 1 ==
        sizeof(ClearCoreRTM::kStateNames) / sizeof(ClearCoreRTM::kStateNames[0]),
    "kStateNames must match ClearCoreRTM::State");

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
    HEATER_OUTPUT_PIN.Mode(Connector::OUTPUT_PWM);
    HEATER_SAFETY_PIN.Mode(Connector::OUTPUT_DIGITAL);
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

    Delay_ms(250);

    /* TTL Comms */
    ttlComms_.begin();
    ttlComms_.setRxUsbLogging(true, "XPB");
    dbgln("TTL Ready");

    // listen for XPB ready
    {
        uint32_t tReady = Milliseconds();
        while (Milliseconds() - tReady < 250) {
            ttlComms_.checkForMessages();
            ttlComms_.checkRetries();
        }
    }
    
    ttlComms_.sendMessage("READY;ID=CC", MessageType::NORMAL);
    delay(2);
    ttlComms_.sendMessage("NOTICE;PROTO=2;CC_FW=2025.08", MessageType::NORMAL);
    delay(2);
    ttlComms_.sendCommand("REQ:SW",      MessageType::IMPORTANT);

    // Initialize with empty protocol - will receive from XPB
    state_ = State::BOOT;

    // MOVED TO handleBoot()
    //dbgln("Awaiting protocol from XPB...");
    //ttlComms_.sendCommand("REQ:PROTO", MessageType::IMPORTANT);
    //delay(2);

    // MOVED to handleProtoLoad()
    //heartbeatTmr_ = 0;
    
    
    dwellTmr_ = 0;
    return true;
}

void ClearCoreRTM::tick() {
    ttlComms_.checkForMessages();
    ttlComms_.checkRetries();

    // --- Comms health & stale guard ---
    const bool maskActiveNow = (xpbMaskActive_ && Milliseconds() < xpbMaskUntilMs_);
    if (commsHealthy_ && 
        xpbStaleTmr_ > (STALE_MULT * STAT_PERIOD_MS) && state_ != State::EStop) {
        if (!maskActiveNow) {
            estopReason_ |= ESTOP_STALE_STAT;
            const char *why = xpbMaskActive_ ? "XPB stale (mask expired)" : "XPB STAT stale > 5s";
            eStopAll_(why);
        }
    }

    bool eStopActive  = !SAFETY_PIN.State();
    const bool runLineLow = runActiveRemote_;   // XPB publishes RUN=1 when the active-low line is asserted
    bool resetActive  = resetActiveRemote_;

    // first check for E-Stop
    if (eStopActive && state_ != State::EStop) {
        estopReason_ |= ESTOP_SAFETY;      // <— tag hardware cause
        eStopAll_("HW E-STOP input");
        return;
    }

    // transition logic for pause / resume / start
    if (state_ != State::BOOT && state_ != State::PROTO_LOADING) {

        // --- RESET rising edge (as you had) ---
        if (resetActive && !prevResetActive_ && state_ != State::EStop) {
            if (state_ != State::Running) {
                preReset_ = state_;
                state_ = State::ResetRequested;
                resetTmr_ = 0;
            }
        }
        prevResetActive_ = resetActive;

        const bool runRoseLow = runLineLow && !prevRunActive_;
        const bool runWentHigh = !runLineLow && prevRunActive_;
        if (!runGateReleased_ && runWentHigh) {
            runGateReleased_ = true;   // XPB line returned high, treat future low transitions as intentional
            dbgln("[RUN] Gate released: XPB RUN returned high");
        }

        const bool runRiseAllowed = runRoseLow && runGateReleased_;
        if (runRoseLow && !runGateReleased_) {
            dbgln("[RUN] Ignoring RUN line held low before XPB resume");
        }

        // --- RUN logic: pause on level, start/resume on RISING EDGE only ---
        if (!runLineLow && !resetActive && state_ == State::Running) {
            // switch moved out of RUN while running -> pause
            state_ = State::Paused;
        }
        else if (runRiseAllowed && state_ == State::Paused) {
            // If cold start and targetC > 0 we should preheat before resuming motion
            const uint16_t targetC = steps_[currentStep_].tempC;
            if (coldStart_ && targetC > 0) {
                state_                 = State::Preheat;
                preheatTargetC_        = targetC;
                waitingForTemp_        = true;
                autoStartAfterPreheat_ = true;   // user-initiated start
                dbgln("PAUSED→PREHEAT (system resume)");
                // ask XPB to set heater
                char cmd[48];
                snprintf(cmd, sizeof(cmd), "CMD;SP=%u", preheatTargetC_);
                ttlComms_.sendMessage(cmd, MessageType::IMPORTANT);
            } else {
                state_ = State::Resume;          // fast path, no preheat needed
            }
        }
        else if (runRiseAllowed && state_ == State::Idle) {
            const uint16_t targetC = steps_[currentStep_].tempC;
            if (coldStart_ && targetC > 0) {
                state_                 = State::Preheat;
                preheatTargetC_        = targetC;
                waitingForTemp_        = true;
                autoStartAfterPreheat_ = true;   // user-initiated start
                dbgln("IDLE→PREHEAT (user start)");
                char cmd[48];
                snprintf(cmd, sizeof(cmd), "CMD;SP=%u", preheatTargetC_);
                ttlComms_.sendMessage(cmd, MessageType::IMPORTANT);
            } else {
                state_ = State::Running;         // fast path, no preheat needed
            }
        }

        // latch for next tick
        prevRunActive_ = runLineLow;
    }

    // justEntered_ allows us to do things once upon first entering a state handler
    bool justEntered_ = (state_ != prevState_); // did we just state change?
    if (justEntered_) {
        dbg("STATE -> ");
        dbgln(stateToString(state_));
    }
    prevState_ = state_; // capture previous state
    
    // dispatch to state handlers
    switch (state_) {
        case State::BOOT:
            handleBoot(resetActive, justEntered_);
            break;
        case State::PROTO_LOADING:
            handleProtoLoad(justEntered_);
            break;
        case State::Idle:
            handleIdle(runLineLow, justEntered_);
            break;
        case State::Preheat:
            handlePreheat(runLineLow, justEntered_);
            break;
        case State::Running:
            handleRunning(runLineLow, justEntered_);
            break;
        case State::Paused:
            handlePaused(runLineLow, justEntered_);
            break;
        case State::ResetRequested:
            handleReset(resetActive, justEntered_);
            break;
        case State::EStop:
            handleEStop(resetActive, justEntered_);
            break;
        case State::Resume:
            handleResume(runLineLow, justEntered_);
            break;
        case State::Completed:
            handleCompleted(resetActive, justEntered_);
            break;
        default:
            break;
    }

    if (heartbeatTmr_ >= 1000 && heartbeatSystemEnabled_) {
        heartbeatTmr_ = 0;
        const bool maskActiveNowHb = (xpbMaskActive_ && Milliseconds() < xpbMaskUntilMs_);
        const char *stateStr = maskActiveNowHb ? "WAITING_XPB" : stateToString(state_);

        char msg[96];
        snprintf(msg, sizeof(msg),
                "HB;SEQ=%u;STATE=%s;STEP=%u;LOOP=%lu/%lu;SW_AGE=%lu;E=%d;E_CODE=%02X",
                hbSeq_++,
                stateStr,
                (unsigned)(currentStep_ + 1),
                (unsigned long)(totalLoops_ - loopCount_ + 1),
                (unsigned long)totalLoops_,
                (unsigned long)(Milliseconds() - swLastUpdateMs_),
                (estopReason_ != 0) ? 1 : 0, // probably not needed
                (unsigned)estopReason_);
        ttlComms_.sendMessage(msg, MessageType::INFO);
        ttlComms_.checkForMessages();

        // Check periodically for mem corruption
        if (guardBefore_ != 0xDEAD || guardAfter_ != 0xBEEF) {
            eStopAll_("Memory corruption detected");
        }
    }

}

/* ——— State Handlers ——— */
void ClearCoreRTM::handleBoot(bool resetActive, bool justEntered_) {
    if (justEntered_) {
        runGateReleased_ = false;   // active-low RUN stays masked until XPB grants it again
        heartbeatSystemEnabled_ = false;
        protoRequestTmr_ = 0;

        if (!isProtoLoaded_) {
            dbgln("BOOT: Requesting protocol from XPB...");
            ttlComms_.sendCommand("REQ:PROTO", MessageType::IMPORTANT);

            protocolName_ = "Awaiting Upload";
            stepCount_ = 0;
            loopCount_ = 1;
            totalLoops_ = 1;
            progHash_ = 0;

        }
    }

    if (protoRequestTmr_ > 5000 && !isProtoLoaded_) {
        dbgln("Awaiting protocol from XPB...");
        ttlComms_.sendCommand("REQ:PROTO", MessageType::IMPORTANT);
        delay(2);
        protoRequestTmr_ = 0;
    }

    return;
}

void ClearCoreRTM::handleProtoLoad(bool justEntered_) {
    if (justEntered_) { 
        heartbeatSystemEnabled_ = false;
        dbgln("PROTO_LOADING: Protocol chunks being recieved...");
    }
    // do nothing while we wait for proto to load?
    // do we even need handleProtoLoad if we aren't doing anything?
    return;
}

void ClearCoreRTM::handleIdle(bool runActive, bool justEntered_) {
    if (justEntered_) {
        // start beating
        heartbeatSystemEnabled_ = true;
        xpbStaleTmr_ = 0;
        dbgln("IDLE: Heatbeet system enabled");
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
        testRunTmr_ = 0; // prevent run timer from ticking
    }
    return;
}

void ClearCoreRTM::handleRunning(bool runActive, bool justEntered_) {
    if (justEntered_) {
        // Check if we have a valid protocol
        if (stepCount_ == 0) {
            dbgln("ERROR: No protocol loaded - cannot run");
            state_ = State::Idle;
            return;
        }

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
        ttlComms_.sendCommand("CMD;RESET=CANCEL", MessageType::IMPORTANT);
        motor.EnableRequest(true);
        if (state_ != preReset_) {
            prevState_ = State::Debug; // force a mismatch, check o3 to see if this makes sense anymore!
        }
        state_ = preReset_;           // restore previous state
        resetPhase_ = ResetPhase::Idle;
        return;
    }

    if (justEntered_) {
        xpbBootSeen_ = false;
        resetTmr_    = 0;

        if (resetImmediate_) {
            ttlComms_.sendCommand("CMD;RESET=EXEC", MessageType::CRITICAL);
            resetPhase_      = ResetPhase::ExecSent;
            xpbBootWaitTmr_  = 0;
            resetImmediate_  = false;  // one-shot
        } else {
            char line[40];
            snprintf(line, sizeof(line), "CMD;RESET=ARM;SECS=%u", (unsigned)resetArmSecs_);
            ttlComms_.sendCommand(line, MessageType::IMPORTANT);
            resetPhase_ = ResetPhase::Armed;
        }
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
        motor.MoveStopAbrupt();
        motor.EnableRequest(false);

        // Kill heater output, latch safety low
        HEATER_OUTPUT_PIN.PwmDuty(0);
        HEATER_SAFETY_PIN.State(false);
    }

    // flash LED rapidly
    if (ledTmr_ > 50) { ledTmr_ = 0; LED_PIN.State(!LED_PIN.State()); }

    if (resetActive) {
        // Instead of resetting CC locally, perform a coordinated dual reset:
        //  - tell XPB to reboot now
        //  - wait for XPB boot (HELLO/READY or small timeout)
        //  - then reset CC
        resetImmediate_ = true;          // skip ARM UI; jump straight to EXEC
        preReset_       = State::Idle;   // state to return to after reset (unused)
        prevState_      = State::Debug;  // force justEntered_ on next state
        state_          = State::ResetRequested;
        return;
    }

    // If you want to allow leaving E-STOP without reset once hardware is safe,
    // you could add a branch here, but current design requires a reset.
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

void ClearCoreRTM::handlePreheat(bool runActive, bool justEntered_) {
    if (justEntered_) {
        motor.EnableRequest(false);
        LED_PIN.State(true);
    }

    if (!waitingForTemp_) {
        coldStart_ = false;

        if (autoStartAfterPreheat_ && runActive) {
            state_ = State::Running;
            dbgln("Preheat complete - starting motion");
        } else {
            state_ = State::Idle;
            dbgln("Preheat complete - idle");
        }
    }

    if (ledTmr_ > 500) {
        ledTmr_ = 0;
        LED_PIN.State(!LED_PIN.State());
    }
}

// --------- output control handlers ------------
void ClearCoreRTM::setHeaterOutput(int out) {
    if (state_ == State::EStop || heaterInhibit_) {
        HEATER_OUTPUT_PIN.PwmDuty(0);
        HEATER_SAFETY_PIN.State(false);
        return;
    }

    // ClearCore PWM max out is 255
    if (out < 0)    out = 0;
    if (out > 255)  out = 255;

    HEATER_SAFETY_PIN.State(true);
    HEATER_OUTPUT_PIN.PwmDuty(out);
}

void ClearCoreRTM::eStopAll_(const char *reason) {
    // Motor: stop immediately and disable
    motor.MoveStopAbrupt();
    motor.EnableRequest(false);

    // Heater: drop to zero and disable safety if used
    HEATER_OUTPUT_PIN.PwmDuty(0);
    HEATER_SAFETY_PIN.State(false);
    heaterInhibit_ = true;

    // Flag state + log
    state_ = State::EStop;
    sendAlarm_("ESTOP", reason);
    dbg("E-STOP: "); dbgln(reason ? reason : "unspecified");
}

void ClearCoreRTM::sendAlarm_(const char *type, const char *reason) {
    // Keep the frame short (< ~75 chars total): "ALARM;TYPE=...;MSG=..."
    char msg[80];
    const char *t = type   ? type   : "GEN";
    const char *r = reason ? reason : "";

    // Truncate reason to ~40 safe chars (ASCII, no semicolons)
    char buf[41];
    size_t i = 0;
    for (; r[i] && i < sizeof(buf)-1; ++i) {
        char c = r[i];
        if (c == ';' || c == '\r' || c == '\n') c = ' ';
        buf[i] = c;
    }
    buf[i] = '\0';

    snprintf(msg, sizeof(msg), "ALARM;TYPE=%s;MSG=%s", t, buf);
    ttlComms_.sendMessage(msg, MessageType::INFO);
}
