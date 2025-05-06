#pragma once
/*
 * MotorController – state‑machine wrapper around the torque‑stand firmware.
 *
 *  ▸ Call `begin()` once from `main()` after the hardware is up.
 *  ▸ Call `tick()` from your loop – it is non‑blocking.
 */

#include "ClearCore.h"
#include "ElapsedMillis.h"   // <‑‑ added as requested
#include "SPI.h"
#include "SD.h"
#include <array>

#define PRGM_RUN_BUS_PIN ConnectorDI6
#define LED_PIN ConnectorIO0
#define MOTOR_ENABLE_PIN ConnectorIO2
#define PRGM_RESET_BUS_PIN ConnectorDI7
#define SerialPort ConnectorUsb
#define SAFETY_PIN ConnectorDI8

class MotorController {
public:
    /* === Public API ==================================================== */

    /** Initialise hardware and read the CSV protocol. */
    bool begin();

    /** Run one iteration of the state machine (call as often as possible). */
    void tick();

private:
    /* === Types & constants ============================================ */

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

    struct Step {
        int32_t  speedSteps_s   {0};   //!< target speed in steps/s
        uint32_t accelSteps_s2  {0};   //!< accel in steps/s²
        uint32_t dwellMs        {0};   //!< dwell after speed reached (ms)
    };

    static constexpr uint8_t  kMaxProtocolSteps = 50;
    static constexpr uint8_t  kNumCols          = 20;
    static constexpr uint8_t  kNumRows          = 4;
    static constexpr uint16_t kStepsPerRev      = 3200;

    /* === Private state ================================================= */

    // protocol
    std::array<Step, kMaxProtocolSteps> steps_{};
    uint8_t  stepCount_   {0};
    uint8_t  loopCount_   {1};
    String   protocolName_;

    // run‑time state
    State    state_       {State::Idle};
    State    preReset_    {State::Idle};

    uint16_t currentStep_ {0};
    bool     stepInit_    {false};
    bool     targetMet_   {false};

    int32_t  targetSpeed_ {0};
    uint32_t targetAccel_ {0};
    int32_t  currentSpeed_{0};
    uint32_t currentAccel_{0};

    // timers
    elapsedMillis ledTmr_, dwellTmr_, resetTmr_;

    // display line buffers (front buffer)
    char line1_[kNumCols + 1] = "                    ";
    char line2_[kNumCols + 1] = "                    ";
    char line3_[kNumCols + 1] = "                    ";
    char line4_[kNumCols + 1] = "                    ";

    /* === Helper methods =============================================== */

    /** Read protocol.csv and fill internal tables. */
    bool loadProtocol(File &csv) {
        if (!csv) {
            return false;            // file handle invalid
        }

        // 1) Protocol name line – PROTOCOL_NAME=XXX
        String line = csv.readStringUntil('\n');
        line.trim();
        if (!line.startsWith("PROTOCOL_NAME=")) {
            return false;
        }
        protocolName_ = line.substring(strlen("PROTOCOL_NAME="));

        // 2) Loop count – LOOP_COUNT=N
        line = csv.readStringUntil('\n');
        line.trim();
        if (!line.startsWith("LOOP_COUNT=")) {
            return false;
        }
        loopCount_ = line.substring(strlen("LOOP_COUNT=")).toInt();
        if (loopCount_ == 0) loopCount_ = 1; // safeguard

        // 3) Skip header row
        csv.readStringUntil('\n');

        // 4) Steps
        stepCount_ = 0;
        while (csv.available() && stepCount_ < kMaxProtocolSteps) {
            String row = csv.readStringUntil('\n');
            row.trim();
            if (row.length() == 0) continue; // skip blanks

            int c1 = row.indexOf(',');
            int c2 = row.indexOf(',', c1 + 1);
            if (c1 < 0 || c2 < 0) {
                continue; // malformed line – ignore
            }

            // split & convert
            int32_t rpmTarget = row.substring(0, c1).toInt();
            uint32_t rpmAccel = row.substring(c1 + 1, c2).toInt();
            uint32_t dwellS   = row.substring(c2 + 1).toInt();

            Step &s = steps_[stepCount_++];
            s.speedSteps_s  = static_cast<int32_t>( (rpmTarget * kStepsPerRev + 30) / 60 ); // round‑nearest
            s.accelSteps_s2 = static_cast<uint32_t>( (rpmAccel * kStepsPerRev + 30) / 60 );
            s.dwellMs       = dwellS * 1000UL;
        }

        return (stepCount_ > 0);
    }

    void setBrightness(uint8_t level);
    void pad(char *buf);
    void render();
    void printCurrent(const char *msg = "");
    void printAlerts();
    void setCursor(uint8_t row, uint8_t col);
    void clearScreen();

    /* --- state handlers --- */
    void handleIdle(bool runBtn);
    void handleRunning(bool runBtn);
    void handlePaused(bool runBtn);
    void handleReset(bool resetBtn);
    void handleEStop(bool safetyBtn, bool resetBtn);
};

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
