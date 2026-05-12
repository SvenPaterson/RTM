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
    "RESETTING",        // State::ResetRequested
    "RESUME",           // State::Resume
    "COMPLETED",        // State::Completed
    "E-STOP"            // State::EStop
};

bool ClearCoreRTM::begin() {
    /* USB Serial Comms for Debugging */
    SerialPort.Mode(Connector::USB_CDC);
    SerialPort.Speed(9600);
    SerialPort.PortOpen();

    uint32_t t0 = Milliseconds();
    while (!SerialPort && Milliseconds() - t0 < 5000) {}

    dbgln("SerialReady");

    /* GPIO */
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
    motor.HlfbFilterLength(1);   // 200 µs filter (min) — needed for 16 PPR at high RPM
    motor.VelMax(kMotorMaxRpm * kStepsPerRev / 60);
    motor.AccelMax(kMotorMaxRpm * kStepsPerRev / 60);
    motor.EStopDecelMax(kMotorMaxRpm * kStepsPerRev / 60);
    dbgln("Motor ready");

    Delay_ms(250);

    /* TTL Comms */
    ttlComms_.begin();
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

// Poll HLFB rising edge flag and update span counters.
// Called from multiple points in tick() so that serial I/O
// blocking cannot cause missed edges (HlfbHasRisen is clear-on-read).
inline void ClearCoreRTM::pollHlfbEdge_() {
    const uint32_t nowUs = Microseconds();
    if (motor.HlfbHasRisen()) {
        if (hlfbFirstEdge_) {
            hlfbWindowStartUs_ = nowUs;
            hlfbFirstEdge_ = false;
        }
        hlfbEdgeCount_++;
        hlfbWindowLastUs_ = nowUs;
        hlfbLastEdgeUs_   = nowUs;
    } else if ((nowUs - hlfbLastEdgeUs_) > kHlfbStaleUs) {
        measuredRpm_    = 0;
        hlfbFirstEdge_  = true;
        hlfbEdgeCount_  = 0;
    }
}

void ClearCoreRTM::tick() {
    pollHlfbEdge_();                        // HLFB poll 1 — before serial I/O
    ttlComms_.checkForMessages();
    ttlComms_.checkRetries();
    pollHlfbEdge_();                        // HLFB poll 2 — after serial I/O

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
    const bool runRoseLow  = runLineLow && !prevRunActive_;
    const bool runWentHigh = !runLineLow && prevRunActive_;

    // first check for E-Stop
    if (eStopActive && state_ != State::EStop) {
        estopReason_ |= ESTOP_SAFETY;      // <— tag hardware cause
        eStopAll_("HW E-STOP input");
        return;
    }

    if (!runGateReleased_ && runWentHigh) {
        runGateReleased_   = true;   // XPB line returned high, treat future low transitions as intentional
        latchedRunPending_ = false;
        dbgln("[RUN] Gate released: XPB RUN returned high");
    }

    bool runRiseManual = false;
    bool runRiseLatched = false;
    if (runGateReleased_) {
        if (runRoseLow) {
            runRiseManual = true;
        } else if (latchedRunPending_ && runLineLow) {
            runRiseLatched = true;
        }
    } else if (runRoseLow) {
        dbgln("[RUN] Ignoring RUN line held low before XPB resume");
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

        if (runRoseLow && !runGateReleased_) {
            dbgln("[RUN] Ignoring RUN line held low before XPB resume");
        }

        // --- RUN logic: pause on level, start/resume on RISING EDGE only ---
        if (!runLineLow && !resetActive && state_ == State::Running) {
            // switch moved out of RUN while running -> pause
            state_ = State::Paused;
        }
        else if (runRiseManual) {
            (void)promoteRun_(RunTrigger::ManualEdge);
        }
        else if (runRiseLatched) {
            if (promoteRun_(RunTrigger::LatchedAuto)) {
                latchedRunPending_ = false;
            }
        }
    }

    // latch for next tick (even during BOOT/PROTO_LOADING so we catch high transitions)
    prevRunActive_ = runLineLow;

    // justEntered_ allows us to do things once upon first entering a state handler
    bool justEntered_ = (state_ != prevState_); // did we just state change?
    if (justEntered_) {
        dbg("STATE -> ");
        dbgln(stateToString(state_));
    }
    prevState_ = state_; // capture previous state
    
    pollHlfbEdge_();                        // HLFB poll 3 — before state handler

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

    pollHlfbEdge_();                        // HLFB poll 4 — before HB send

    if (heartbeatTmr_ >= 250 && heartbeatSystemEnabled_) {
        heartbeatTmr_ = 0;
        const bool maskActiveNowHb = (xpbMaskActive_ && Milliseconds() < xpbMaskUntilMs_);
        const char *stateStr = maskActiveNowHb ? "BOOTING" : stateToString(state_);


        const uint32_t loopsTotal = totalLoops_;
        const uint32_t loopsRemaining = loopCount_;
        uint32_t completed = 0;
        if (loopsRemaining <= loopsTotal) {
            completed = loopsTotal - loopsRemaining;
        }

        uint32_t loopDisplay = 0;
        if (loopsTotal == 0) {
            loopDisplay = 0;
        } else if (loopsRemaining == 0) {
            loopDisplay = loopsTotal;
        } else {
            loopDisplay = completed + 1;
            if (loopDisplay > loopsTotal) {
                loopDisplay = loopsTotal;
            }
        }

        // Compute RPM from edges accumulated since last HB
        if (hlfbEdgeCount_ >= 2) {
            const uint32_t spanUs = hlfbWindowLastUs_ - hlfbWindowStartUs_;
            if (spanUs > 0) {
                int16_t rpm = (int16_t)(
                    (uint32_t)(hlfbEdgeCount_ - 1) * (60000000UL / kHlfbPPR) / spanUs);
                measuredRpm_ = (motor.VelocityRefCommanded() < 0) ? -rpm : rpm;
            }
        }

        char msg[96];
        snprintf(msg, sizeof(msg),
                "HB;SEQ=%u;STATE=%s;STEP=%u;LOOP=%lu/%lu;SW_AGE=%lu;E=%d;E_CODE=%02X;RPM=%d",
                hbSeq_++,
                stateStr,
                (unsigned)(currentStep_ + 1),
                (unsigned long)loopDisplay,
                (unsigned long)totalLoops_,
                (unsigned long)(Milliseconds() - swLastUpdateMs_),
                (estopReason_ != 0) ? 1 : 0, // probably not needed
                (unsigned)estopReason_,
                (int)measuredRpm_);
        ttlComms_.sendMessage(msg, MessageType::INFO);
        pollHlfbEdge_();                    // HLFB poll 5 — after HB serial send
        ttlComms_.checkForMessages();

        // Reset edge counters for next HB window
        hlfbEdgeCount_ = 0;
        hlfbFirstEdge_ = true;

        // Check periodically for mem corruption
        if (guardBefore_ != 0xDEAD || guardAfter_ != 0xBEEF) {
            eStopAll_("Memory corruption detected");
        }
    }

}

bool ClearCoreRTM::promoteRun_(RunTrigger trigger) {
    if (!runActiveRemote_) {
        return false;
    }

    const bool latched = (trigger == RunTrigger::LatchedAuto);
    const uint16_t targetC = steps_[currentStep_].tempC;

    auto queuePreheat = [&](const char *logManual, const char *logLatched) {
        state_                 = State::Preheat;
        preheatTargetC_        = targetC;
        waitingForTemp_        = true;
        autoStartAfterPreheat_ = true;
        activeSetpointC_       = targetC;
        if (latched) {
            if (logLatched) dbgln(logLatched);
        } else {
            if (logManual) dbgln(logManual);
        }

        char cmd[48];
        snprintf(cmd, sizeof(cmd), "CMD;SP=%u", preheatTargetC_);
        ttlComms_.sendMessage(cmd, MessageType::IMPORTANT);
    };

    if (state_ == State::Paused) {
        if (protocolUsesHeat_ && targetC > 0) {
            queuePreheat("PAUSED→PREHEAT (system resume)", "[RUN] Latched resume -> PREHEAT");
        } else {
            state_ = State::Resume;
            if (latched) {
                dbgln("[RUN] Latched resume -> RESUME");
            }
        }
        return true;
    }

    if (state_ == State::Idle) {
        if (coldStart_ && targetC > 0) {
            queuePreheat("IDLE→PREHEAT (user start)", "[RUN] Latched start -> PREHEAT");
        } else {
            state_ = State::Running;
            if (latched) {
                dbgln("[RUN] Latched start -> RUNNING");
            }
        }
        return true;
    }

    if (latched) {
        dbgln("[RUN] Latched RUN held waiting for Idle/Pause state");
    }
    return false;
}

/* ——— State Handlers ——— */
void ClearCoreRTM::handleBoot(bool resetActive, bool justEntered_) {
    if (justEntered_) {
        runGateReleased_   = false;   // active-low RUN stays masked until XPB grants it again
        latchedRunPending_ = false;
        heartbeatSystemEnabled_ = true;  // keep HBs alive so XPB doesn't show LostComms

        if (fromLogicalReset_) {
            // Logical reset: XPB protocol is already in RAM.
            // Pre-load the timer so the first REQ:PROTO fires immediately.
            protoRequestTmr_ = 5000;
            fromLogicalReset_ = false;
        } else {
            protoRequestTmr_ = 0;
        }

        if (!isProtoLoaded_) {
            protocolName_ = "Awaiting Upload";
            stepCount_ = 0;
            loopCount_ = 1;
            totalLoops_ = 1;
            progHash_ = 0;
        }
    }

    // Cold boot: 5s delay lets XPB boot + init SD.
    // Logical reset: timer pre-loaded above, fires immediately.
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
        heartbeatSystemEnabled_ = true;   // keep HBs alive so XPB doesn't show LostComms
        protoRequestTmr_ = 0;             // start safety timeout
        dbgln("PROTO_LOADING: Protocol chunks being received...");
    }

    // Safety: if no PR_DAT arrives within 10s, revert to BOOT
    // so CC can re-request the protocol.
    if (protoRequestTmr_ > 10000) {
        dbgln("[PROTO] Timeout waiting for PR_DAT — reverting to BOOT");
        protoRx_ = {};   // cancel partial reception
        state_ = State::BOOT;
    }
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
        motor.MoveStopDecel((1000 * kStepsPerRev) / 60); // decel to 0 RPM
        if (protocolUsesHeat_) sendHeaterOff_();
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
            if (protocolUsesHeat_ && steps_[currentStep_].tempC > 0) {
                // Route through preheat to reheat before resuming motion
                preheatTargetC_ = steps_[currentStep_].tempC;
                waitingForTemp_ = true;
                autoStartAfterPreheat_ = true;
                activeSetpointC_ = preheatTargetC_;

                char cmd[48];
                snprintf(cmd, sizeof(cmd), "CMD;SP=%u", preheatTargetC_);
                ttlComms_.sendMessage(cmd, MessageType::IMPORTANT);

                state_ = State::Preheat;
                dbgln("PAUSED->PREHEAT (re-heat before resume)");
            } else {
                state_ = State::Resume;
            }
        } else {
            motor.EnableRequest(false);
        }
    }
    return;
}

void ClearCoreRTM::handleReset(bool resetActive, bool justEntered_) {
    if (!resetActive && resetPhase_ != ResetPhase::ExecSent) {
        // Cancel only if we haven't already told XPB to reboot.
        // Once ExecSent, XPB is committed to rebooting and CC must follow.
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
            // 3) Wait a bit for XPB to complete its logical reset
            if (xpbBootSeen_ || xpbBootWaitTmr_ >= 1500) {
                // 4) Logical reset: reinitialize all CC state in-place
                logicalReset();
                return;
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
        motor.MoveStopDecel(targetAccel_); // decel to 0 RPM
        if (protocolUsesHeat_) sendHeaterOff_();
    }

    if (motor.StepsComplete()) {
        motor.EnableRequest(false);
    }

    return;
}

// ---------------------------------------------------------------------------
// logicalReset — in-place state reset (replaces SysMgr.ResetBoard)
// ---------------------------------------------------------------------------
void ClearCoreRTM::logicalReset() {
    dbgln("[RESET] CC logical reset starting");

    // --- Motor: safe stop ---
    motor.MoveStopAbrupt();
    motor.EnableRequest(false);

    // --- Heater: safe stop ---
    sendHeaterOff_();
    heaterInhibit_ = false;

    // --- Protocol: force re-upload from XPB ---
    isProtoLoaded_  = false;
    protocolName_   = "Awaiting Upload";
    stepCount_      = 0;
    loopCount_      = 1;
    totalLoops_     = 1;
    progHash_       = 0;
    protoRx_        = {};
    protoRequestTmr_ = 0;

    // --- Reset state machine ---
    resetPhase_     = ResetPhase::Idle;
    resetImmediate_ = false;
    xpbBootSeen_    = false;

    // --- Motion execution ---
    currentStep_     = 0;
    stepInit_        = false;
    targetMet_       = false;
    targetSpeed_     = 0;
    currentSpeed_    = 0;
    targetAccel_     = 0;
    currentAccel_    = 0;
    pause_time_      = 0;
    resumeFromPause_ = false;

    // --- Comms health ---
    commsHealthy_            = false;
    heartbeatSystemEnabled_  = false;
    heartbeatTmr_            = 0;
    xpbStaleTmr_             = 0;
    statAgeTmr_              = 0;
    hbSeq_                   = 0;

    // --- E-stop ---
    estopReason_ = 0;

    // --- User input ---
    runGateReleased_   = false;
    latchedRunPending_ = false;
    prevRunActive_     = false;
    prevResetActive_   = false;

    // --- HLFB ---
    measuredRpm_   = 0;
    hlfbFirstEdge_ = true;
    hlfbEdgeCount_ = 0;

    // --- Preheat / heating ---
    coldStart_             = true;
    waitingForTemp_        = false;
    autoStartAfterPreheat_ = false;
    protocolUsesHeat_      = false;
    sealTempC_             = 0;
    activeSetpointC_       = 0;
    sumpOverTempCount_     = 0;

    // --- Runtime timers ---
    testRunTmr_ = 0;
    runMins_    = 0;
    ledTmr_     = 0;
    lcdTmr_     = 0;
    dwellTmr_   = 0;

    // --- LCD ---
    lcdToggle_        = false;
    lcdRuntimeToggle_ = false;
    modeTorqueToggle_ = false;
    for (uint8_t i = 0; i < kNumRows; ++i) dirty_[i] = true;

    // --- TTL comms ---
    ttlComms_.resetState();

    // --- XPB mask: keep active to cover XPB transition ---
    // Logical reset is fast (~2s total); 5s mask is plenty.
    xpbMaskActive_  = true;
    xpbMaskUntilMs_ = Milliseconds() + 5000UL;
    fromLogicalReset_ = true;   // tell handleBoot to skip 5s SD-init delay

    // --- Transition to BOOT ---
    prevState_ = State::Debug;   // force justEntered_ on next tick
    state_     = State::BOOT;

    // --- Announce readiness (mirrors begin()) ---
    ttlComms_.sendMessage("READY;ID=CC", MessageType::NORMAL);
    delay(2);
    ttlComms_.sendCommand("REQ:SW", MessageType::IMPORTANT);

    LED_PIN.State(true);
    dbgln("[RESET] CC logical reset complete — entering BOOT");
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
void ClearCoreRTM::sendHeaterOff_() {
    // Tell XPB to stop PID
    ttlComms_.sendMessage("CMD;SP=0", MessageType::IMPORTANT);

    // Cut both SSR legs locally
    HEATER_OUTPUT_PIN.PwmDuty(0);
    HEATER_SAFETY_PIN.State(false);

    activeSetpointC_ = 0;
    dbgln("[HEAT] Heater OFF (CMD;SP=0 sent)");
}

void ClearCoreRTM::setHeaterOutput(int out) {
    if (state_ == State::EStop || heaterInhibit_ || !protocolUsesHeat_) {
        HEATER_OUTPUT_PIN.PwmDuty(0);
        HEATER_SAFETY_PIN.State(false);
        return;
    }

    // ClearCore PWM max out is 255
    if (out < 0)    out = 0;
    if (out > 255)  out = 255;

    if (out > 0) {
        HEATER_SAFETY_PIN.State(true);
        HEATER_OUTPUT_PIN.PwmDuty(out);
    } else {
        HEATER_OUTPUT_PIN.PwmDuty(0);
        // IO2 stays energised while a setpoint is active;
        // only sendHeaterOff_ / eStopAll_ cut it.
        if (activeSetpointC_ == 0) {
            HEATER_SAFETY_PIN.State(false);
        }
    }
}

void ClearCoreRTM::eStopAll_(const char *reason) {
    // Motor: stop immediately and disable
    motor.MoveStopAbrupt();
    motor.EnableRequest(false);

    // Heater: best-effort tell XPB to stop PID, then cut locally
    ttlComms_.sendMessage("CMD;SP=0", MessageType::IMPORTANT);
    activeSetpointC_ = 0;
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
