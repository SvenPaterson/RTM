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
    bool begin(); // call once from main() after hardware init
    void tick();  // call from your loop() – non‑blocking

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
    static constexpr uint16_t kStepsPerRev      = 3200;
    static constexpr uint16_t kMotorMaxRpm      = 2760;
    static const uint8_t kRowAddr[kNumRows];

    /* ——— protocol state ——— */
    std::array<Step, kMaxProtocolSteps> steps_{};
    uint8_t  stepCount_   {0};
    uint8_t  loopCount_   {1};
    String   protocolName_;

    /* ——— runtime state ——— */
    State    state_{State::Idle}, preReset_{State::Idle};
    uint16_t currentStep_ {0};
    bool     stepInit_{false}, targetMet_{false};;
    int32_t  targetSpeed_ {0}, currentSpeed_{0};
    uint32_t targetAccel_ {0}, currentAccel_{0};

    /* ——— timers ——— */
    elapsedMillis ledTmr_, dwellTmr_, resetTmr_, streenTmr_;

    /* ——— LCD front/shadow buffers ——— */
    char front_[kNumRows][kNumCols + 1] = {};
    char sent_ [kNumRows][kNumCols + 1] = {};
    bool dirty_[kNumRows]               = {true, true, true, true};
    bool screenToggle_                  = false; // toggle between torque and setpoint

    /* ——— protocol helpers ——— */
    bool loadProtocol(File &csv);

    /* ——— LCD helpers ——— */
    static inline uint8_t fastLen_(const char *s) { uint8_t n = 0; while (n < kNumCols && s[n]) ++n; return n; }
    void lcdBlank_(char *dst);
    void lcdLineBlank(uint8_t row); // blank a line in the front buffer
    void lcdLineLeft  (uint8_t row, const char *txt);
    void lcdLineRight (uint8_t row, const char *txt);
    void lcdLineCenter(uint8_t row, const char *txt);
    void lcdLineLR    (uint8_t row, const char *l, const char *r);
    void lcdFlush();
    void lcdClearScreen();
    void renderScreen();

    /* ——— state handlers ——— */
    void handleIdle(bool runBtn);
    void handleRunning(bool runBtn);
    void handlePaused(bool runBtn);
    void handleReset(bool resetBtn);
    void handleEStop(bool safetyBtn, bool resetBtn);
};

/* ——— static data definitions (link-time) ——— */
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
    char buf[kNumCols+1];
        // line 1: name and state
        lcdLineLR(0, protocolName_.c_str(), stateToString(state_));
        // line 2: pressure & loop count
        float pressure = 0.00f;
        uint16_t loop = loopCount_;
        snprintf(buf, sizeof(buf), "P:%5.2fpsi LOOP:%4u", pressure, loop);
        lcdLineLeft(1, buf);
        // line 3: toggle torque vs setpoint
        if (screenToggle_) {
            float torqueA = 0.00f, torqueB = 0.00f;
            snprintf(buf, sizeof(buf), "Torque:%6.2f/%6.2f", torqueA, torqueB);
        } else {
            int setpoint = 0;
            snprintf(buf, sizeof(buf), "Setpoint:%12u°F", setpoint);
        }
        lcdLineLeft(2, buf);
        // line 4: temps, drop ° if three-digit
        int seal = 74, sump = 73;
        if (sump < 100) {
            snprintf(buf, sizeof(buf), "Seal:%3u°F Sump:%3u°F", seal, sump);
        } else {
            snprintf(buf, sizeof(buf), "Seal:%3u°F Sump:%4u", seal, sump);
        }
        lcdLineLeft(3, buf);
        // commit
        lcdFlush();
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
    stepCount_ = loadProtocol(csv);
    if (!stepCount_) {
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

    return true;
}


inline void MotorController::tick() {
    /* // read the state of the buttons
    bool runBtn   = PRGM_RUN_BUS_PIN.State();
    bool resetBtn = PRGM_RESET_BUS_PIN.State();
    bool safetyBtn = SAFETY_PIN.State();

    // handle the current state
    switch (state_) {
        case State::Idle:
            handleIdle(runBtn);
            break;
        case State::Running:
            handleRunning(runBtn);
            break;
        case State::Paused:
            handlePaused(runBtn);
            break;
        case State::ResetRequested:
            handleReset(resetBtn);
            break;
        case State::EStop:
            handleEStop(safetyBtn, resetBtn);
            break;
        default:
            break;
    } */
    const char *stateStr = stateToString(state_);
    
    if (state_ == State::Idle) {
        lcdLineLR(0, "Reading SD...", stateStr);
    } else if (state_ == State::Running) {
        lcdLineLR(0, "Running...", stateStr);
    } else if (state_ == State::Paused) {
        lcdLineLR(0, "Paused...", stateStr);
    } else if (state_ == State::ResetRequested) {
        lcdLineLR(0, "Resetting...", stateStr);
    } else if (state_ == State::EStop) {
        lcdLineLR(0, "E-Stop...", stateStr);
    } else {
        lcdLineLR(0, "Unknown...", stateStr);
    }

    // toggle display every 2000ms
    if (streenTmr_ > 2000) {
        streenTmr_ = 0;
        screenToggle_ = !screenToggle_;
        renderScreen();
    }

    if (ledTmr_ > 500) {
        ledTmr_ = 0;
        LED_PIN.State(!LED_PIN.State()); // toggle LED
    }

    bool isSafetyActive = SAFETY_PIN.State();
    bool runActive = PRGM_RUN_BUS_PIN.State();
    bool resetActive = PRGM_RESET_BUS_PIN.State();
    bool prevResetActive = false;

    

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
    
    // 1) Protocol name line – PROTOCOL_NAME=XXX
    String line = csv.readStringUntil('\n');
    line.trim();
    const char *pfxName = "PROTOCOL_NAME=";
    if (!line.startsWith(pfxName)) return false;
    protocolName_ = line.substring(strlen(pfxName));

    // 2) Loop count – LOOP_COUNT=N
    line = csv.readStringUntil('\n');
    line.trim();
    const char *pfxLoop = "LOOP_COUNT=";
    if (!line.startsWith(pfxLoop)) return false;
    String lc = line.substring(strlen(pfxLoop));
    lc.trim();
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
        s.speedSteps_s  = (rpmTarget * kStepsPerRev + 30) / 60; // round‑nearest
        s.accelSteps_s2 = (rpmAccel  * kStepsPerRev + 30) / 60 ;
        s.dwellMs       = dwellS     * 1000UL;
    }

    return (stepCount_ > 0);
}