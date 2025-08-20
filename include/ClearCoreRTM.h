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

    // --- E-STOP reason tracking (bitmask) ---
    enum : uint8_t {
        ESTOP_STALE_STAT = 0x01,    // comms stale: no STAT from XPB
        ESTOP_SAFETY     = 0x02,    // hardware safety chain
        // future: ESTOP_OVERTEMP = 0x04, ...
    };
    uint8_t estopReason_ { 0 };

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

    struct ProtoRx {
        bool active{false};
        uint16_t seq{0};
        uint32_t crc{0};
        uint32_t phash{0};
        uint8_t stepsRcvd{0};
    } protoRx_;
    uint32_t progHash_;

    static constexpr uint8_t  kMaxProtocolSteps = 50;
    static constexpr uint16_t kStepsPerRev      = 3200; // set this using ClearPath software on Stepper Motor, don't go lower than 3200
    static constexpr uint16_t kMotorMaxRpm      = 2760;

    /* ——— protocol state ——— */
    std::array<Step, kMaxProtocolSteps> steps_{};
    uint8_t  stepCount_   {0};
    uint32_t loopCount_   {1};
    uint32_t totalLoops_  {1};
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
            Delay_ms(30);
            while (ConnectorCOM1.AvailableForRead() > 0) {
                ConnectorCOM1.CharGet();
            }
            beginBase();
        }
        
        // Implement serial interface for ClearCore COM1
        void serialSend(const char* data) override {
            ConnectorCOM1.Send(data);
            //ConnectorCOM1.Flush(); // ensure CRLF is sent
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
        /* void onMessageReceived(const String& data) override {
            if (!data.startsWith("STAT;")) {
                 sendMessage("ACK:OK");
             }

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
                // clamp 3 to 15s, 10s typical
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

                if (dup) return;  // drop retried STAT frames

                if (owner_) {
                    owner_->commsHealthy_ = true;
                    owner_->statAgeTmr_   = 0;                 // fresh data just arrived
                    owner_->setHeaterOutput(out);

                    // --- Auto-clear stale-STAT E-STOP on first good STAT ---
                    if (owner_->state_ == State::EStop && (owner_->estopReason_ & ESTOP_STALE_STAT)) {
                        owner_->estopReason_ &= ~ESTOP_STALE_STAT;
                        owner_->state_ = State::Idle;          // or a "safe idle" if you prefer
                        owner_->dbgln("[CC] Auto-cleared E-STOP (stale-STAT recovered)");
                    }
                }
                return;
            }

            // else ignore silently
        } */
        
        // Handle received messages
        void onMessageReceived(const String& data) override {
            // === CHANGED: parse optional ;REF= once and reuse ===
            const int refPos = data.indexOf(F(";REF="));                // CHANGED
            const bool hasRef = (refPos > 0);                           // CHANGED
            const uint16_t refVal = hasRef ?                           // CHANGED
                (uint16_t)data.substring(refPos + 5).toInt() : 0;       // CHANGED

            // === CHANGED: remove unconditional ACK entirely ===
            // (No "ACK:OK" for everything-not-STAT anymore.)

            // 1) Legacy discovery path (XPB says HELLO). Keep for back-compat.
            if (data.startsWith("HELLO;ID=XPB")) {
                if (owner_) {
                    owner_->xpbBootSeen_  = true;
                    owner_->xpbMaskActive_ = false;
                }

                // READY is a notice; no ACK expected
                char line[64];
                snprintf(line, sizeof(line), "READY;ID=CC;VER=1.0;UPT=%lu",
                        (unsigned long)Milliseconds());
                sendMessage(line, MessageType::NORMAL);                 // CHANGED: was CRITICAL

                // Ask for switches as a request/response with REF (this reply is the ACK)
                sendCommand("REQ:SW", MessageType::IMPORTANT);          // CHANGED: was sendMessage(...)

                return;
            }

            // 2) XPB READY (notice) — no ACK
            if (data.startsWith("READY;ID=XPB")) {
                if (owner_) {
                    owner_->xpbBootSeen_   = true;
                    owner_->xpbMaskActive_ = false;
                }
                return;
            }

            // 3) QUIESCE (command from XPB) — ACK ONCE, mirror REF if present
            if (data.startsWith("QUIESCE;")) {
                int reqSecs = kvGetIntClamped(data, "SECS=", 10, 1, 60);
                int maskSecs = reqSecs;
                if (maskSecs < 3)  maskSecs = 3;
                if (maskSecs > 15) maskSecs = 15;

                if (owner_) {
                    owner_->xpbMaskActive_  = true;
                    owner_->xpbMaskUntilMs_ = Milliseconds() + (uint32_t)maskSecs * 1000UL;
                    owner_->dbgln("[QUIESCE] XPB mask started");
                }

                char ack[48];
                if (hasRef) {
                    snprintf(ack, sizeof(ack), "ACK;QUIESCE=OK;MASK=%d;REF=%u", maskSecs, refVal);  // CHANGED: mirror REF
                } else {
                    snprintf(ack, sizeof(ack), "ACK;QUIESCE=OK;MASK=%d", maskSecs);
                }
                sendMessage(ack, MessageType::INFO);                    // CHANGED: ACKs are INFO (no ACK-of-ACK)

                return;
            }

            // ===== Protocol Upload: PR_BEG =====
            if (data.startsWith("PR_BEG;")) {
                // Only accept in IDLE or PAUSED states
                if (owner_ && (owner_->state_ == State::Idle || owner_->state_ == State::Paused)) {
                    // Parse protocol metadata
                    String name = kvGet(data, "NAME=");
                    String loopsStr = kvGet(data, "LOOPS=");
                    uint32_t loops = loopsStr.length() ? strtoul(loopsStr.c_str(), nullptr, 10) : 1;
                    if (loops == 0) loops = 1;
                    int steps = kvGetIntClamped(data, "STEPS=", 0, 1, owner_->kMaxProtocolSteps);
                    String phashStr = kvGet(data, "PHASH=");
                    
                    // Initialize reception
                    owner_->protoRx_.active = true;
                    owner_->protoRx_.seq = 0;
                    owner_->protoRx_.crc = 0;
                    owner_->protoRx_.phash = phashStr.toInt();  // TODO: parse as unsigned long
                    owner_->protoRx_.stepsRcvd = 0;
                    
                    // Pre-fill metadata (will activate on successful END)
                    owner_->protocolName_ = name;
                    owner_->loopCount_ = loops;
                    owner_->totalLoops_ = loops;
                    owner_->stepCount_ = steps;  // Expected count
                    
                    // ACK with REF if present
                    char ack[40];
                    if (hasRef) {
                        snprintf(ack, sizeof(ack), "ACK;PR_BEG=OK;REF=%u", refVal);
                    } else {
                        snprintf(ack, sizeof(ack), "ACK;PR_BEG=OK");
                    }
                    sendMessage(ack, MessageType::INFO);
                } else {
                    // Reject - wrong state
                    char nak[40];
                    if (hasRef) {
                        snprintf(nak, sizeof(nak), "ACK;PR_BEG=BUSY;REF=%u", refVal);
                    } else {
                        snprintf(nak, sizeof(nak), "ACK;PR_BEG=BUSY");
                    }
                    sendMessage(nak, MessageType::INFO);
                }
                return;
            }

            // ===== Protocol Upload: PR_DAT =====
            if (data.startsWith("PR_DAT;")) {
                if (owner_ && owner_->protoRx_.active) {
                    // Check sequence number
                    int seq = kvGetIntClamped(data, "SEQ=", -1, 0, 255);
                    if (seq != owner_->protoRx_.seq) {
                        // Sequence mismatch - send NAK
                        char nak[40];
                        snprintf(nak, sizeof(nak), "ACK;PR_DAT=BAD_SEQ;EXP=%u;GOT=%d", 
                                owner_->protoRx_.seq, seq);
                        sendMessage(nak, MessageType::INFO);
                        return;
                    }
                    
                    // Parse DATA field: "rpm,accel,dwell[,temp]"
                    String dataStr = kvGet(data, "DATA=");
                    if (dataStr.length() > 0 && owner_->protoRx_.seq < owner_->kMaxProtocolSteps) {
                        int c1 = dataStr.indexOf(',');
                        int c2 = dataStr.indexOf(',', c1+1);
                        int c3 = dataStr.indexOf(',', c2+1);
                        
                        if (c1 > 0 && c2 > 0) {
                            long rpm = dataStr.substring(0, c1).toInt();
                            long accel = dataStr.substring(c1+1, c2).toInt();
                            long dwellS = (c3 > 0) ? 
                                dataStr.substring(c2+1, c3).toInt() : 
                                dataStr.substring(c2+1).toInt();
                            // Ignore temp for now (c3 to end) - CC doesn't use it
                            
                            // Convert and store in steps_ array  
                            Step &s = owner_->steps_[owner_->protoRx_.seq];
                            s.speedSteps_s = (rpm * owner_->kStepsPerRev + (rpm >= 0 ? 30 : -30)) / 60;
                            s.accelSteps_s2 = (accel * owner_->kStepsPerRev + 30) / 60;
                            s.dwellMs = dwellS * 1000UL;
                        }
                    }
                    
                    owner_->protoRx_.seq++;
                    owner_->protoRx_.stepsRcvd++;
                    
                    // ACK this chunk
                    char ack[40];
                    if (hasRef) {
                        snprintf(ack, sizeof(ack), "ACK;OK;REF=%u", refVal);
                    } else {
                        snprintf(ack, sizeof(ack), "ACK;OK");
                    }
                    sendMessage(ack, MessageType::INFO);
                }
                return;
            }

            // ===== Protocol Upload: PR_END =====  
            if (data.startsWith("PR_END;")) {
                if (owner_ && owner_->protoRx_.active) {
                    String crcStr = kvGet(data, "CRC=");
                    uint32_t expectedCrc = strtoul(crcStr.c_str(), nullptr, 10);
                    
                    // For now, just accept it (TODO: implement CRC check)
                    owner_->protoRx_.active = false;
                    
                    // Send success notice
                    char notice[64];
                    snprintf(notice, sizeof(notice), "NOTICE;PROTO_RX=OK;PHASH=%lu",
                            (unsigned long)owner_->protoRx_.phash);
                    sendMessage(notice, MessageType::INFO);
                    
                    owner_->progHash_ = owner_->protoRx_.phash;
                    owner_->dbgln("[PROTO] Upload complete");
                }
                return;
            }

            // 5) Switch state from XPB (either unsolicited or reply to REQ:SW)
            //    Never ACK; if this was a reply to our REQ:SW, transport will correlate on REF.
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

            // 5) Telemetry from XPB (heartbeat/stat) — never ACK
            if (data.startsWith("STAT;")) {
                static int lastSeq = -1;
                const int seq = kvGet(data, "SEQ=").toInt();
                const int out = kvGetIntClamped(data, "OUT=", 0, 0, 150);

                const bool dup = (seq >= 0 && seq == lastSeq);
                if (seq >= 0) lastSeq = seq;
                if (dup) return;  // drop retried STAT frames

                if (owner_) {
                    owner_->commsHealthy_ = true;
                    owner_->statAgeTmr_   = 0;                 // fresh data just arrived
                    owner_->setHeaterOutput(out);

                    // Auto-clear stale-STAT E-STOP on first good STAT
                    if (owner_->state_ == State::EStop && (owner_->estopReason_ & ESTOP_STALE_STAT)) {
                        owner_->estopReason_ &= ~ESTOP_STALE_STAT;
                        owner_->state_ = State::Idle;
                        owner_->dbgln("[CC] Auto-cleared E-STOP (stale-STAT recovered)");
                    }
                }
                return;
            }

            // 6) RESUME? / NOTICE / HB from XPB:
            //    Treat as notices/telemetry. Do not ACK. If you need UI updates, handle here.
            // (Currently ignoring silently unless you already have handlers elsewhere.)

            // else ignore silently
        }

        void onBadChecksum(const String& raw) override {
            static uint32_t badCrcCount = 0;
            ++badCrcCount;
            if (badCrcCount % 10 == 1) {
                usbLog("WARN: TTL bad checksum (rate-limited)");
            }
            // Do NOT send any frame here.
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



