#pragma once
/*
 * MotorController – state‑machine wrapper around the torque‑stand firmware.
 *
 *  ▸ Call `begin()` once from `main()` after the hardware is up.
 *  ▸ Call `tick()` from your loop – it is non‑blocking.
 */

/* ================================================================
 * Example usage inside your main.cpp
 *
 *  MotorController ctrl;
 *
 *  int main() {
 *      ... // hardware init
 *      if (!ctrl.begin()) {
 *          // SD or CSV load failed – halt
 *          while (true) {}
 *      }
 *      while (true) {
 *          ctrl.tick();
 *      }
 *  }
 * ================================================================ */

#include "ClearCore.h"
#include "ElapsedMillis.h"   // <‑‑ added as requested
#include "SPI.h"
#include "SD.h"

// ClearCore (and other Arduino cores) define min/max macros that clash with
// <algorithm> templates in <array> on GCC. Undef them before any STL headers
// need the templates.
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

#include <array>
#include <cstring>
#include <stdint.h>

/* === MOTOR & DISPLAY CONFIGURATION ===================================== */
#define motor               ConnectorM0
#define PRGM_RUN_BUS_PIN    ConnectorDI6
#define LED_PIN             ConnectorIO0
#define PRGM_RESET_BUS_PIN  ConnectorDI7
#define SerialPort          ConnectorUsb
#define SAFETY_PIN          ConnectorDI8

class MotorController {
public:
    /* ——— Public API ——— */
    bool begin();       // call once from main() after hardware init
    void tick();        // call from your loop() – non‑blocking
    inline void torqueMode() {modeTorqueToggle_ = true; lcdToggle_ms_ = 1000;} // setup device for torque stand, defaults to RTM controller

    /* ——— compile‑time LCD dimensions (exposed for other modules) ——— */
    static constexpr uint8_t  kNumCols = 20;
    static constexpr uint8_t  kNumRows = 4;

private:
    /* ——— LCD SPI settings ——— */
    static const SPISettings spiCfg_;

    /* ——— runtime states ——— */
    enum class State : uint8_t {
        Debug,
        Idle,
        Running,
        Paused,
        ResetRequested,
        Resume,
        Completed,
        EStop
    };
    char debugBuf_[150]; // for debugging to Serial

    // string mapping for displaying active state on LCD
    static inline constexpr const char *kStateNames[8] = {
        "DEBUG", "IDLE", "RUNNING", "PAUSED", "RESETTING",
        "RESUME", "COMPLETED", "E-STOP"
    };
    static inline const char * stateToString(State s) {
        return kStateNames[static_cast<uint8_t>(s)];
    }

    /* ——— protocol steps ——— */
    struct Step {
        int32_t  speedSteps_s   {0};   //!< target speed in steps/s
        uint32_t accelSteps_s2  {0};   //!< accel in steps/s²
        uint32_t dwellMs        {0};   //!< dwell after speed reached (ms)
    };

    static constexpr uint8_t  kMaxProtocolSteps = 50;
    static constexpr uint16_t kStepsPerRev      = 3200; // set this using ClearPath software on Stepper Motor, don't go lower than 3200
    static constexpr uint16_t kMotorMaxRpm      = 2760;
    static const uint8_t kRowAddr[kNumRows];

    /* ——— protocol state ——— */
    std::array<Step, kMaxProtocolSteps> steps_{};
    uint8_t  stepCount_   {0};
    uint8_t  loopCount_   {1};
    uint8_t  totalLoops_  {1};
    String   protocolName_;

    /* ——— runtime state ——— */
    State    state_{State::Idle}, prevState_{State::Idle}, preReset_{State::Idle};
    uint16_t currentStep_ {0};
    uint8_t  lastResetSec_{0}, prevStepIndex_{0};
    bool     stepInit_{false}, targetMet_{false};
    bool     resumeFromPause_{false}, prevResetActive_{false};
    int32_t  targetSpeed_ {0}, currentSpeed_{0};
    uint32_t targetAccel_ {0}, currentAccel_{0};
    uint32_t pause_time_{0}, test_run_time_{0};

    /* ——— timers ——— */
    elapsedMillis ledTmr_, dwellTmr_, resetTmr_, lcdTmr_, testRunTmr_;
    uint16_t lcdToggle_ms_{3000}; // default to every 3s
    uint32_t runMins_{0};

    /* ——— LCD front/shadow buffers ——— */
    char buf_[kNumCols + 1] = {};
    char front_[kNumRows][kNumCols + 1] = {};
    char sent_ [kNumRows][kNumCols + 1] = {};
    bool dirty_[kNumRows]               = {true, true, true, true};

    /* ——— LCD behaviour toggles ——— */
    bool lcdToggle_{false}, lcdRuntimeToggle_{false}, modeTorqueToggle_{false};

    /* ——— protocol helpers ——— */
    bool loadProtocol(File &csv);

    /* ——— LCD helpers ——— */
    static inline uint8_t fastLen_(const char *s) { uint8_t n = 0; while (n < kNumCols && s[n]) ++n; return n; }
    void lcdBlank_     (char *dst);
    void lcdLineBlank  (uint8_t row);                                   // blank a line in the front buffer
    void lcdLineLeft   (uint8_t row, const char *txt);                  // fill a line w/ a left justified string
    void lcdLineRight  (uint8_t row, const char *txt);                  // fill a line w/ a right justified string
    void lcdLineCenter (uint8_t row, const char *txt);                  // fill a line w/ a center justified string
    void lcdLineLR     (uint8_t row, const char *l, const char *r);     // fill a line w/ two strings, right and left justified
    void lcdFlush      ();                                              // print all lines to screen
    void lcdClearScreen();                                              // does what it says on the tin...
    void renderScreen  ();                                              // call this to update screen with test details

    /* ——— state handlers ——— */
    void handleIdle     (bool runActive, bool justEntered_);
    // void handleRunningAlt  (bool runActive, bool justEntered_);
    void handleRunning  (bool runActive, bool justEntered_);
    void handlePaused   (bool runActive, bool justEntered_);
    void handleReset    (bool resetActive, bool justEntered_);
    void handleEStop    (bool resetActive, bool justEntered_);
    void handleResume   (bool runActive, bool justEntered_);
    void handleCompleted(bool resetActive, bool justEntered_);
};

/* ——— static data definitions (link-time) ——— */
// for debugging via USB and Terminal
const SPISettings MotorController::spiCfg_{ 80000, MSBFIRST, SPI_MODE3 };

/* ——— row base addresses for the 4-line Nehaven LCD Module ——— */
const uint8_t MotorController::kRowAddr[MotorController::kNumRows] = {0x00, 0x40, 0x14, 0x54};

/* ——— LCD helpers ——— */
inline void MotorController::lcdBlank_(char *dst) {
    memset(dst, ' ', kNumCols);
    dst[kNumCols] = '\0';
}

inline void MotorController::lcdLineLeft(uint8_t row, const char *txt) {
    lcdBlank_(front_[row]);
    memcpy(front_[row], txt, fastLen_(txt));
    if (memcmp(front_[row], sent_[row], kNumCols)) dirty_[row] = true;
}

inline void MotorController::lcdLineRight(uint8_t row, const char *txt) {
    lcdBlank_(front_[row]);
    uint8_t len = fastLen_(txt);
    memcpy(front_[row] + kNumCols - len, txt, len);
    if (memcmp(front_[row], sent_[row], kNumCols)) dirty_[row] = true;
}

inline void MotorController::lcdLineCenter(uint8_t row, const char *txt) {
    lcdBlank_(front_[row]);
    uint8_t len = fastLen_(txt);
    uint8_t start = (kNumCols - len) / 2;
    memcpy(front_[row] + start, txt, len);
    if (memcmp(front_[row], sent_[row], kNumCols)) dirty_[row] = true;
}

inline void MotorController::lcdLineLR(uint8_t row, const char *left, const char *right) {
    lcdBlank_(front_[row]);
    uint8_t rLen = fastLen_(right);
    uint8_t lMax = (rLen < kNumCols) ? kNumCols - rLen - 1 : 0;
    uint8_t lLen = (lMax ? (fastLen_(left) > lMax ? lMax : fastLen_(left)) : 0);
    memcpy(front_[row],               left,  lLen);
    memcpy(front_[row] + kNumCols - rLen, right, rLen);
    if (memcmp(front_[row], sent_[row], kNumCols)) dirty_[row] = true;
}

inline void MotorController::lcdFlush() {
    SPI.beginTransaction(spiCfg_);
    for (uint8_t row = 0; row < kNumRows; ++row) {
        if (!dirty_[row]) continue;
        SPI.transfer(0xFE); SPI.transfer(0x45); SPI.transfer(kRowAddr[row]);
        SPI.transfer(front_[row], nullptr, kNumCols);
        memcpy(sent_[row], front_[row], kNumCols + 1);
        dirty_[row] = false;
    }
    SPI.endTransaction();
}

inline void MotorController::lcdLineBlank(uint8_t row) {
    lcdBlank_(front_[row]);
    if (memcmp(front_[row], sent_[row], kNumCols)) dirty_[row] = true;
}

inline void MotorController::lcdClearScreen() {
    for (uint8_t i = 0; i < kNumRows; ++i) {
        lcdLineBlank(i); // clear front buffer
    }
    lcdFlush();
}

inline void MotorController::renderScreen() {
    char buf_1[kNumCols+1];
    /* char buf_2[kNumCols+1];
    snprintf(buf_2, sizeof(buf_2), "%s", protocolName_.c_str()) */

    // line 1: toggle between protocol name and runtime
    char left0[kNumCols+1];
    if (lcdToggle_) {
        // show protocol name
        strncpy(left0, protocolName_.c_str(), kNumCols);
        left0[kNumCols] = '\0';
    } else {
        // show total minutes runtime
        if (runMins_ < 60) {
            snprintf(left0, sizeof(left0), "%2lu mins", runMins_);
        } else {
            snprintf(left0, sizeof(left0), "%4.1f hrs", float(runMins_) / 60.0f);
        }
    }
    lcdLineLR(0, left0, stateToString(state_));

    // line 2: current step & loop count
    uint8_t step = currentStep_;
    uint8_t loop = totalLoops_ - loopCount_;
    snprintf(buf_1, sizeof(buf_1), "STEP:%2u  Loop:%u/%u", step, loop, totalLoops_);
    lcdLineLeft(1, buf_1);
    
    float rpm = static_cast<float>(targetSpeed_) * 60.0f / kStepsPerRev;
    int16_t v = (rpm >= 0.0f) ? static_cast<int16_t>(rpm + 0.5f) : static_cast<int16_t>(rpm - 0.5f);
    uint16_t a = static_cast<uint16_t>((static_cast<float>(targetAccel_) * 60.0f / kStepsPerRev) + 0.5f);

    if (modeTorqueToggle_) {
        lcdLineLR(2, "RPM/s    RPM", "Dwell");
        char dwell_buf[6];
        
        if (!targetMet_) {
            snprintf(dwell_buf, sizeof(dwell_buf), "ramp");
        } else if (state_ == State::Paused) {
            uint16_t t = (steps_[currentStep_].dwellMs - pause_time_) / 1000;
            snprintf(dwell_buf, sizeof(dwell_buf), "%4us", t);
        } else {
            uint16_t t = (steps_[currentStep_].dwellMs - dwellTmr_) / 1000;
            snprintf(dwell_buf, sizeof(dwell_buf), "%4us", t);
        }
        snprintf(buf_1, sizeof(buf_1), "%5u  %5d", a, v);
        
        lcdLineLR(3, buf_1, dwell_buf);
    }

    else { // display typical RTM stats
        // line 3: toggle torque vs setpoint
        if (lcdToggle_) {
            float torqueA = 0.00f, torqueB = 0.00f;
            snprintf(buf_1, sizeof(buf_1), "Torque:%6.2f/%6.2f", torqueA, torqueB);
        } else {
            int setpoint = 300;     // PLACEHOLDER  
            float pressure = 14.1f; // PLACEHOLDER
            if (pressure < 100) {
                snprintf(buf_1, sizeof(buf_1), "Heat:%3u\xDF""F P:%3.1fpsi", setpoint, pressure);
            } else {
            snprintf(buf_1, sizeof(buf_1), "Heat:%3u\xDF""F Pr:%3.0fpsi", setpoint, pressure);
            }
        }
        lcdLineLeft(2, buf_1);
        // line 4: temps, drop ° if three-digit
        int seal = 120, sump = 140; // PLACEHOLDERS
        if (sump < 100) {
            snprintf(buf_1, sizeof(buf_1), "Seal:%3u\xDF""F Sump:%2u\xDF""F", seal, sump);
        } else {
            snprintf(buf_1, sizeof(buf_1), "Seal:%3u\xDF""F Sump:%3uF", seal, sump);
        }
        lcdLineLeft(3, buf_1);
    }
    
    // commit
    lcdFlush();

    snprintf(debugBuf_, sizeof(debugBuf_),
        "State: %9s | Loop: %3u | Step: %3u | Speed: %5d RPM | Accel: %4u RPM/s² | Total Runtime: %5u | dwellTmr: %lu",// | VelRef: %li",
        stateToString(state_),
        (unsigned)loopCount_,
        (unsigned)currentStep_,
        v,
        (unsigned)a,
        (unsigned)runMins_,
        (unsigned long int)(uint32_t)dwellTmr_);
        //(signed long)(motor.VelocityRefCommanded()));
    SerialPort.SendLine(debugBuf_);
}


/* ——— API Definitions ——— */
inline bool MotorController::begin() {
    /* Serial */
    SerialPort.Mode(Connector::USB_CDC); SerialPort.Speed(9600); SerialPort.PortOpen();
    uint32_t t0 = Milliseconds(); while (!SerialPort && Milliseconds() - t0 < 5000) {}
    SerialPort.SendLine("Serial ready");

    /* GPIO */
    PRGM_RUN_BUS_PIN.Mode(Connector::INPUT_DIGITAL);
    PRGM_RESET_BUS_PIN.Mode(Connector::INPUT_DIGITAL);
    SAFETY_PIN.Mode(Connector::INPUT_DIGITAL);
    LED_PIN.Mode(Connector::OUTPUT_DIGITAL); LED_PIN.State(true);
    SerialPort.SendLine("GPIO ready");

    /* MOTOR */
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
    MotorMgr.MotorModeSet(MotorManager::MOTOR_M0M1, Connector::CPM_MODE_STEP_AND_DIR);
    motor.HlfbMode(MotorDriver::HLFB_MODE_STATIC);
    motor.VelMax(kMotorMaxRpm * kStepsPerRev / 60);
    motor.AccelMax(kMotorMaxRpm * kStepsPerRev / 60);
    motor.EStopDecelMax(kMotorMaxRpm * kStepsPerRev / 60);
    SerialPort.SendLine("Motor ready");

    // ----------- DISPLAY ---------
    SPI.begin();
    Delay_ms(120); // power up delay

    SPI.beginTransaction(spiCfg_);
    SPI.transfer(0xFE);
    SPI.transfer(0x53);
    SPI.transfer(4); // brightness = 4
    SPI.endTransaction();
    SerialPort.SendLine("LCD ready");

    lcdClearScreen();
    Delay_ms(250);

    lcdLineLeft(0, "Reading SD...");
    lcdFlush();
    Delay_ms(250);

    // ----------- SD CARD ---------
    if (!SD.begin()) {
        SerialPort.SendLine("SD begin failed");
        lcdLineLR(0, "Reading SD...", "FAIL");
        lcdLineCenter(3, "check SD card");
        lcdFlush();
        return false;
    }
    SerialPort.SendLine("SD ready");
    lcdLineLR(0, "Reading SD...", "DONE");
    lcdFlush();
    Delay_ms(250);

    // ----------- LOAD PROTOCOL ---------
    lcdLineLeft(1, "Load config...");
    lcdFlush();
    Delay_ms(250);
    File csv = SD.open("protocol.csv", FILE_READ);
    if (!loadProtocol(csv)) {
        SerialPort.SendLine("Load config failed");
        lcdLineLR(1, "Load config...", "FAIL");
        lcdLineCenter(3, "verify csv file!");
        lcdFlush();
        return false;
    };
    csv.close();
    SerialPort.SendLine("Load config done");
    lcdLineLR(1, "Load config...", "DONE");
    lcdFlush();
    Delay_ms(250);

    lcdLineCenter(3, protocolName_.c_str());
    lcdFlush();
    Delay_ms(1000);

    dwellTmr_ = 0;
    return true;
}

inline void MotorController::tick() {
    bool estopActive  = !SAFETY_PIN.State();
    bool runActive    = PRGM_RUN_BUS_PIN.State();
    bool resetActive  = PRGM_RESET_BUS_PIN.State();

    // first check for E-Stop
    if (estopActive && state_ != State::EStop) {
        state_ = State::EStop;
    }

    // check for reset request
    if (resetActive && !prevResetActive_ && state_ != State::EStop) {
        if (state_ != State::Running) {
            // capture re-reset state so it can be restored later
            preReset_ = state_;
            state_ = State::ResetRequested;
            renderScreen();
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
}

inline bool MotorController::loadProtocol(File &csv) {
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
        s.speedSteps_s = (rpmTarget >= 0) // round‑nearest
                       ? (rpmTarget * kStepsPerRev + 30) / 60
                       : (rpmTarget * kStepsPerRev - 30) / 60;
        // s.accelSteps_s2 = (1.5 * (rpmAccel  * kStepsPerRev + 30) / 60) ; // 1.5 compensates for motor settings
        s.accelSteps_s2 = ((rpmAccel  * kStepsPerRev + 30) / 60);
        s.dwellMs       = dwellS     * 1000UL;
    }

    // 5) Print out protocol to Serial
    SerialPort.SendLine("==== Loaded Protocol ====");
    SerialPort.Send("Protocol Name: "); SerialPort.SendLine(protocolName_.c_str());
    SerialPort.Send("Loop Count: "); SerialPort.SendLine(loopCount_);
    SerialPort.SendLine("Step Count: "); SerialPort.SendLine(stepCount_);

    totalLoops_ = loopCount_; // to help display current test state

    // 6) Prints entire protocol to Terminal for debugging purposes
    for (uint8_t i = 0; i < stepCount_; ++i) {
        int32_t rpm = (steps_[i].speedSteps_s * 60 + (steps_[i].speedSteps_s >= 0 ? kStepsPerRev / 2 : -kStepsPerRev / 2)) / kStepsPerRev;
        uint32_t accel = (steps_[i].accelSteps_s2 * 60 + kStepsPerRev / 2) / kStepsPerRev;
        uint32_t dwell = steps_[i].dwellMs / 1000;

        snprintf(debugBuf_, sizeof(debugBuf_), "Step %2u: %6ld RPM  %4lu RPM/s²  %3lu s",
                 i + 1, rpm, accel, dwell);
        SerialPort.SendLine(debugBuf_);
    }

    SerialPort.SendLine("=========================");
    
    return (stepCount_ > 0);
}

inline void MotorController::handleEStop(bool resetActive, bool justEntered_) {
    
    if (justEntered_) {
        lcdClearScreen();
        lcdLineCenter(0, "!!! E-STOP !!!");
        lcdLineCenter(1, "Press Reset to Clear");
        lcdLineBlank (2);
        lcdLineCenter(3, "Test is now void!");
        lcdFlush();
        motor.MoveStopAbrupt();
        motor.EnableRequest(false);
    }
    
    // flash LED rapidly
    if (ledTmr_ > 50) {
        ledTmr_ = 0;
        LED_PIN.State(!LED_PIN.State());
    }

    if (resetActive) {
        lcdClearScreen();
        lcdLineCenter(1, "Resetting board...");
        lcdFlush();
        SysMgr.ResetBoard();
    }
    return;
}

inline void MotorController::handleIdle(bool, bool justEntered_) {
    if (justEntered_) {
        renderScreen(); // no need to update the screen before a test starts
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
        renderScreen();
        testRunTmr_ = 0; // prevent run timer from ticking
    }
    return;
}

/* inline void MotorController::handleRunningAlt(bool runActive, bool justEntered_) {
    if (justEntered_) {
        // solid LED
        LED_PIN.State(true);
        renderScreen();
    }
    
    if (!runActive) {
        state_ = State::Paused;
        // currentSpeed_ = motor.VelocityRefCommanded();
        // currentAccel_ = targetAccel_;
        pause_time_ = dwellTmr_;
        renderScreen();
        return;
    }

    // toggle display every 3secs
    if (lcdTmr_ > 3000 && state_ != State::EStop) {

        lcdTmr_ = 0;
        lcdToggle_ = !lcdToggle_;
        renderScreen();
    }

    // Only run at start of step
    if (!stepInit_) {
        motor.EnableRequest(true);

        targetAccel_ = steps_[currentStep_].accelSteps_s2;
        motor.AccelMax(targetAccel_);
        targetSpeed_ = steps_[currentStep_].speedSteps_s;
        motor.VelMax(targetSpeed_);
        stepInit_ = true;
        targetMet_ = false;

        motor.MoveVelocity(targetSpeed_);
        if (targetSpeed_ == 0) {
            motor.MoveStopDecel(targetAccel_);
        }
    }

    // run step until target speed reached
    if (!targetMet_) {
        dwellTmr_ = 0;
        // for non-zero targets, check speed reached
        if (targetSpeed_ != 0 && 
            fabs(motor.VelocityRefCommanded()) >= fabs(0.99 * targetSpeed_)) {
            targetMet_ = true;
        }
        // For zero targets, check full stop reached
        else if (targetSpeed_ == 0 && motor.StepsComplete()) {
            targetMet_ = true;
        }

    }

    // once target speed reached, check dwell time reached
    if (targetMet_ && dwellTmr_ >= steps_[currentStep_].dwellMs) {
        stepInit_ = false;
        prevStepIndex_ = currentStep_;
        currentStep_ = (currentStep_ + 1) % stepCount_;
        targetMet_ = false;
        // check if we are at the end of the protocol
        if (prevStepIndex_ == stepCount_ - 1 && currentStep_ == 0) {
            loopCount_--;
            if (loopCount_ == 0) {
                state_ = State::Completed;
                renderScreen();
            }
        }
    }
    return;
}
 */

 inline void MotorController::handleRunning(bool runActive, bool justEntered_) {
    if (justEntered_) {
        // solid LED
        LED_PIN.State(true);
        targetMet_ = false; // false during a ramp to target speed
        renderScreen();
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
        renderScreen();
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
            renderScreen(); // ensure we render the start of the dwell timer
            return;
        }
    }

    if (lcdTmr_ > lcdToggle_ms_ && state_ != State::EStop) {
        // while running the LCD will toggle info every lcdToggle_ms_ milliseconds
        lcdTmr_ = 0;
        lcdToggle_ = !lcdToggle_;
        renderScreen();
    }
}

inline void MotorController::handlePaused(bool runActive, bool justEntered_) {
    if (justEntered_) {
        renderScreen();
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
        renderScreen();
    }

    if (motor.StepsComplete()) {
        // spin motor down to full stop
        if (runActive) {
            state_ = State::Resume;
            renderScreen();
        } else {
            motor.EnableRequest(false);
        }
    }
    return;
}

inline void MotorController::handleReset(bool resetActive, bool justEntered_) {
    if (!resetActive) {
        motor.EnableRequest(true);
        if (state_ != preReset_) {
            prevState_ = State::Debug; // force a mismatch, check o3 to see if this makes sense anymore!
        }
        state_ = preReset_;           // restore previous state
        return;
    }

    if (resetTmr_ >= 5000) {
        lcdClearScreen();
        lcdLineCenter(1, "Resetting board...");
        lcdFlush();
        Delay_ms(1000);
        SysMgr.ResetBoard();
        return; // we'll never get here
    }

    // Switch in Reset position for 5secs to reset system
    uint8_t remaining = 5 - (resetTmr_ / 1000);
    if (remaining != lastResetSec_) {
        lastResetSec_ = remaining;
        snprintf(buf_, sizeof(buf_), "...in %1u sec", remaining);
        lcdLineLR(0, "RESET", buf_);
        lcdFlush();
    }

    if (ledTmr_ > 100) { // rapidly flash LED
        ledTmr_ = 0;
        LED_PIN.State(!LED_PIN.State());
    }
    return;
}

inline void MotorController::handleResume(bool runActive, bool justEntered_) {
    if (justEntered_) {
        motor.EnableRequest(true);
        motor.AccelMax(targetAccel_);
        motor.MoveVelocity(currentSpeed_); // restore speed when paused
        renderScreen();
    }

    if (motor.StepsComplete()) { // resume running once back up to speed
        dwellTmr_ = pause_time_;
        state_ = State::Running;
        motor.MoveVelocity(targetSpeed_);
        resumeFromPause_ = true;
        renderScreen();
    }
    
}

inline void MotorController::handleCompleted(bool resetActive, bool justEntered_) {
    if (justEntered_) {
        renderScreen();
        motor.MoveStopDecel(targetAccel_); // decel to 0 RPM
    }

    if (motor.StepsComplete()) {
        motor.EnableRequest(false);
    }

    return;
}