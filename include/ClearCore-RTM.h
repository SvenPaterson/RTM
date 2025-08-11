#pragma once
/*
 * ClearCoreRTM – state‑machine wrapper around the torque‑stand firmware.
 *
 *  ▸ Call `begin()` once from `main()` after the hardware is up.
 *  ▸ Call `tick()` from your loop – it is non‑blocking.
 */

/* ================================================================
 * Example usage inside your main.cpp
 *
 *  ClearCoreRTM ctrl;
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
#include "ClearCoreElapsedMillis.h"
#include "SPI.h"
#include "SD.h"
#include "TTLComms.h"

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

class ClearCoreRTM {
public:
    /* ——— Public API ——— */
    bool begin();       // call once from main() after hardware init
    void tick();        // call from your loop() – non‑blocking
    inline void torqueMode() {modeTorqueToggle_ = true; lcdToggle_ms_ = 1000;} // setup device for torque stand, defaults to RTM controller

    /* ——— compile‑time LCD dimensions (exposed for other modules) ——— */
    static constexpr uint8_t  kNumCols = 20;
    static constexpr uint8_t  kNumRows = 4;

private:
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

    /* ——— state handlers ——— */
    void handleIdle     (bool runActive, bool justEntered_);
    void handleRunning  (bool runActive, bool justEntered_);
    void handlePaused   (bool runActive, bool justEntered_);
    void handleReset    (bool resetActive, bool justEntered_);
    void handleEStop    (bool resetActive, bool justEntered_);
    void handleResume   (bool runActive, bool justEntered_);
    void handleCompleted(bool resetActive, bool justEntered_);

    class ClearCoreTTL : public TTLComms {
    public:
        void begin() {
            ConnectorCOM1.Mode(Connector::TTL);
            ConnectorCOM1.Speed(9600);
            ConnectorCOM1.PortOpen();
        }
        
        // Implement serial interface for ClearCore COM1
        void serialSend(const char* data) override {
            SerialPort.Send("CC -> XPB: ");
            SerialPort.Send(data);  // Debug output
            ConnectorCOM1.Send(data);
        }
        
        bool serialAvailable() override {
            return (ConnectorCOM1.CharPeek() != -1);
        }
        
        char serialRead() override {
            return ConnectorCOM1.CharGet();
        }
        
        int serialPeek() override {
            return ConnectorCOM1.CharPeek();
        }
        
        // Handle received messages
        void onMessageReceived(const String& data) override {
            // Send ACK first
            sendMessage("ACK:OK");
            
            SerialPort.Send("Received valid message from ExpansionBoard: ");
            SerialPort.SendLine(data.c_str());
        }
        
        void onBadChecksum(const String& rawMsg) override {
            sendMessage("ACK:BAD_CHECKSUM");
            SerialPort.Send("Bad checksum from ExpansionBoard: ");
            SerialPort.SendLine(rawMsg.c_str());
        }
    };

    // In ClearCore-RTM.h private members:
    ClearCoreTTL ttlComms_;
    elapsedMillis heartbeatTmr_;
};



