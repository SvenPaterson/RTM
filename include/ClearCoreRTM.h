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
#include <type_traits>

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
#define HEATER_OUTPUT_PIN   ConnectorIO1
#define HEATER_SAFETY_PIN   ConnectorIO2

class ClearCoreRTM {
public:
    /* ——— Public API ——— */
    bool begin();       // call once from main() after hardware init
    void tick();        // call from your loop() – non‑blocking
    inline void torqueMode() {modeTorqueToggle_ = true; lcdToggle_ms_ = 1000;} // setup device for torque stand, defaults to RTM controller
    //inline void setHeaterOutput() {HEATER_OUTPUT_PIN.OUTPUT_ANALOG} ... TODO
    /* ——— compile‑time LCD dimensions (exposed for other modules) ——— */
    static constexpr uint8_t  kNumCols = 20;
    static constexpr uint8_t  kNumRows = 4;

    ClearCoreRTM() : ttlComms_(this) {} // ctor

private:
    /* ——— XPB QUIESCE mask ——— */
    /// @brief True when CC is intentionally ignoring XPB-stale during an XPB reboot window.
    bool     xpbMaskActive_{false};
    /// @brief Milliseconds timestamp when the XPB mask expires.
    uint32_t xpbMaskUntilMs_{0};

    /* ——— comms health ——— */
    uint16_t hbSeq_ = 0;
    elapsedMillis statAgeTmr_;   // time since last fresh STAT from XPB
    bool commsHealthy_{false};

    /* ——— safety helpers ——— */
    void eStopAll_(const char *reason);
    void sendAlarm_(const char *type, const char *reason);
    bool heaterInhibit_{false};

    /* ——— debug helpers ——— */
    char debugBuf_[150];
    inline void dbg(const char *s)                    { if (SerialPort) SerialPort.Send(s); }
    inline void dbgln(const char *s)                  { if (SerialPort) SerialPort.SendLine(s); }
    inline void dbgkv(const char *k, const char *v)   { if (SerialPort) { SerialPort.Send(k); SerialPort.SendLine(v); } }
    inline void dbgkv(const char *k, const String &v) { dbgkv(k, v.c_str()); }
    inline void dbgkv(const char *k, int32_t v)       { if (SerialPort) { SerialPort.Send(k); SerialPort.SendLine(v); } }
    inline void dbgkv(const char *k, uint32_t v)      { if (SerialPort) { SerialPort.Send(k); SerialPort.SendLine(v); } }
    inline void dbgkv(const char *k, uint16_t v)      { dbgkv(k, (uint32_t)v); }
    inline void dbgkv(const char *k, uint8_t v)       { dbgkv(k, (uint32_t)v); }

    /* ——— Reset / Reboot ——— */
    enum class ResetPhase : uint8_t { Idle, Armed, ExecSent, AwaitXpbBoot };
    ResetPhase resetPhase_{ResetPhase::Idle};
    uint8_t    resetArmSecs_{5};      // UI seconds to show on XPB
    elapsedMillis resetTmr_;          // tick between phase transitions
    bool       xpbBootSeen_{false};   // saw BOOT;ID=XPB
    elapsedMillis xpbBootWaitTmr_;    // how long we’ve waited after EXEC
    bool resetImmediate_{false};  // skip ARM countdown; still coordinate XPB reset

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

    /* ——— user input ——— */
    bool        runActiveRemote_   = false;
    bool        resetActiveRemote_ = false;
    uint32_t    swLastUpdateMs_    = 0;

    // string mapping for displaying active state on LCD
    static const char* const kStateNames[8];
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
    elapsedMillis ledTmr_, dwellTmr_, testRunTmr_, lcdTmr_;
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

    /* ——— heater helpers ——— */
    void setHeaterOutput(int out);

    /* ——— state handlers ——— */
    void handleIdle      (bool runActive,   bool justEntered_);
    void handleRunning   (bool runActive,   bool justEntered_);
    void handlePaused    (bool runActive,   bool justEntered_);
    void handleReset     (bool resetActive, bool justEntered_);
    void handleEStop     (bool resetActive, bool justEntered_);
    void handleResume    (bool runActive,   bool justEntered_);
    void handleCompleted (bool resetActive, bool justEntered_);

    class ClearCoreTTL : public TTLComms {
    public:
        explicit ClearCoreTTL(ClearCoreRTM *owner) : owner_(owner) {}
        void begin() {
            ConnectorCOM1.Mode(Connector::TTL);
            ConnectorCOM1.Speed(9600);
            ConnectorCOM1.PortOpen();
            beginBase();
        }
        
        // Implement serial interface for ClearCore COM1
        void serialSend(const char* data) override {
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
            sendMessage("ACK:OK");

            // 1) Discovery / readiness
            if (data.startsWith("HELLO;ID=XPB")) {
                if (owner_) {
                    owner_->xpbBootSeen_ = true;
                    owner_->xpbMaskActive_ = false;                    
                }
                char line[64];
                snprintf(line, sizeof(line), "READY;ID=CC;VER=1.0;UPT=%lu",
                        (unsigned long)Milliseconds());
                sendMessage(line, MessageType::CRITICAL);
                sendMessage("REQ:SW", MessageType::CRITICAL);
                return;
            }

            if (data.startsWith("READY;ID=XPB")) {
                if (owner_) {
                    owner_->xpbBootSeen_ = true;
                    owner_->xpbMaskActive_ = false;                    
                }
                return;
            }

            if (data.startsWith("QUIESCE;")) {                         
                int reqSecs = kvGetIntClamped(data, "SECS=", 10, 1, 60);
                // clamp 3..15s, 10s typical
                int maskSecs = reqSecs;
                if (maskSecs < 3)  maskSecs = 3;
                if (maskSecs > 15) maskSecs = 15;

                if (owner_) {
                    owner_->xpbMaskActive_  = true;
                    owner_->xpbMaskUntilMs_ = Milliseconds() + (uint32_t)maskSecs * 1000UL;
                    owner_->dbgln("[QUIESCE] XPB mask started");
                }
                char ack[40];
                snprintf(ack, sizeof(ack), "ACK;QUIESCE=OK;MASK=%d", maskSecs);
                sendMessage(ack, MessageType::CRITICAL);
                return;
            }

            // Switch state pushed from XPB (critical edges)
            if (data.startsWith("SW;")) {
                int run = kvGetIntClamped(data, "RUN=", 0, 0, 1);
                int rst = kvGetIntClamped(data, "RST=", 0, 0, 1);
                if (owner_) {
                    owner_->runActiveRemote_   = (run != 0);
                    owner_->resetActiveRemote_ = (rst != 0);
                    owner_->swLastUpdateMs_    = Milliseconds();
                    owner_->dbg("[SW←XPB] run="); owner_->dbgkv("", (unsigned long)run);
                    owner_->dbg(" rst=");         owner_->dbgkv("", (unsigned long)rst);
                }
                return;
            }

            // Telemetry from XPB (heartbeat)
            if (data.startsWith("STAT;")) {
                static int lastSeq = -1;
                const int seq = kvGet(data, "SEQ=").toInt();
                const int out = kvGetIntClamped(data, "OUT=", 0, 0, 150);

                const bool dup = (seq >= 0 && seq == lastSeq);
                if (seq >= 0) lastSeq = seq;

                if (dup) return;  // drop retried STAT frames (idempotent side effects)

                if (owner_) {
                    owner_->commsHealthy_ = true;
                    owner_->statAgeTmr_   = 0;           // fresh data just arrived
                    owner_->setHeaterOutput(out);        // drive AO/PWM once per fresh STAT
                }
                return;
            }


            // else ignore silently
        }
        
        void onBadChecksum(const String& rawMsg) override {
            sendMessage("ACK:BAD_CHECKSUM");
            SerialPort.Send("BAD CHKSUM: ");
            SerialPort.SendLine(rawMsg.c_str());
        }

    protected:
        void usbLog(const char *s) override {
            if (SerialPort) SerialPort.SendLine(s);
        }

    private:
        ClearCoreRTM *owner_{nullptr};
    };

    // In ClearCore-RTM.h private members:
    ClearCoreTTL ttlComms_;
    elapsedMillis heartbeatTmr_;
};



