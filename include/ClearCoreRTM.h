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
    bool heartbeatSystemEnabled_{false};
    elapsedMillis heartbeatTmr_, xpbStaleTmr_;

    /* ——— debug helpers ——— */
    char debugBuf_[150];
    inline void dbg(const char *s)                    { if (SerialPort) SerialPort.Send(s); }
    inline void dbgln(const char *s)                  { if (SerialPort) SerialPort.SendLine(s); }
    inline void dbgkv(const char *k, const char *v)   { if (SerialPort) { SerialPort.Send(k); SerialPort.Send(v); } }
    inline void dbgkv(const char *k, const String &v) { dbgkv(k, v.c_str()); }
    inline void dbgkv(const char *k, int32_t v)       { if (SerialPort) { SerialPort.Send(k); SerialPort.Send(v); } }
    inline void dbgkv(const char *k, uint32_t v)      { if (SerialPort) { SerialPort.Send(k); SerialPort.Send(v); } }
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
        BOOT,
        PROTO_LOADING,
        Idle,
        Preheat,
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
    bool        runGateReleased_   = false; //!< RUN line (active-low) ignored until XPB grants start or we observe a post-boot high
    bool        latchedRunPending_ = false; //!< Latched RUN request awaiting protocol verification
    uint32_t    swLastUpdateMs_    = 0;

    // string mapping for displaying active state on LCD
    static const char* const kStateNames[11];
    static_assert(
        static_cast<uint8_t>(State::EStop) + 1 ==
        sizeof(kStateNames) / sizeof(kStateNames[0]),
        "kStateNames must match ClearCoreRTM::State");
    static inline const char * stateToString(State s) {
        return kStateNames[static_cast<uint8_t>(s)];
    }

    /* ——— protocol steps ——— */
    struct Step {
        int32_t  speedSteps_s   {0};   //!< target speed in steps/s
        uint32_t accelSteps_s2  {0};   //!< accel in steps/s²
        uint32_t dwellMs        {0};   //!< dwell after speed reached (ms)
        uint16_t tempC          {0};
    };

    struct ProtoRx {
        bool active{false};
        uint16_t seq{0};
        uint32_t crc{0};
        uint32_t phash{0};
        uint8_t stepsRcvd{0};
    } protoRx_;
    uint32_t progHash_;
    elapsedMillis protoRequestTmr_;

    static constexpr uint8_t  kMaxProtocolSteps = 50;
    static constexpr uint16_t kStepsPerRev      = 3200; // set this using ClearPath software on Stepper Motor, don't go lower than 3200
    static constexpr uint16_t kMotorMaxRpm      = 2760;

    /* ——— protocol state ——— */
    std::array<Step, kMaxProtocolSteps> steps_{};
    uint8_t  stepCount_   {0};
    uint32_t loopCount_   {1};
    uint32_t totalLoops_  {1};
    String   protocolName_;
    bool     isProtoLoaded_{false};

    /* ——— runtime state ——— */
    State    state_{State::BOOT}, prevState_{State::BOOT}, preReset_{State::BOOT};
    uint16_t guardBefore_ = 0xDEAD;
    uint16_t currentStep_ {0};
    uint16_t guardAfter_ = 0xBEEF;
    uint8_t  lastResetSec_{0}, prevStepIndex_{0};
    bool     stepInit_{false}, targetMet_{false};
    bool     resumeFromPause_{false};
    bool     prevRunActive_{false}, prevResetActive_{false};
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
    // no loadProtocol needed, SD card moved to XPB

    /* ——— heater behaviour ——— */
    bool coldStart_{true};           // True on boot, false once running
    bool waitingForTemp_{false};     // True when preheating
    uint16_t preheatTargetC_{0};     // Target temp for preheat
    bool autoStartAfterPreheat_{false}; // Whether to auto-start after preheat
    void setHeaterOutput(int out);

    /* ——— state handlers ——— */
    void handleBoot      (bool resetActive, bool justEntered_);
    void handleProtoLoad (                  bool justEntered_);
    void handleIdle      (bool runActive,   bool justEntered_);
    void handleRunning   (bool runActive,   bool justEntered_);
    void handlePaused    (bool runActive,   bool justEntered_);
    void handleReset     (bool resetActive, bool justEntered_);
    void handleEStop     (bool resetActive, bool justEntered_);
    void handleResume    (bool runActive,   bool justEntered_);
    void handleCompleted (bool resetActive, bool justEntered_);
    void handlePreheat   (bool runActive,   bool justEntered_);

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
            const int refPos = data.indexOf(F(";REF="));               
            const bool hasRef = (refPos > 0);                           
            const uint16_t refVal = hasRef ?                          
                (uint16_t)data.substring(refPos + 5).toInt() : 0;      

            // === CHANGED: remove unconditional ACK entirely ===
            // (No "ACK:OK" for everything-not-STAT anymore.)

            // 1) Legacy discovery path (XPB says HELLO). Keep for back-compat.
            if (data.startsWith("CMD;")) {
                const String resume = kvGet(data, "RESUME=");
                if (resume.length()) {
                    if (resume == "AUTO" || resume == "YES") {
                        // 1) must have protocol
                        if (owner_->state_ != State::Idle) {
                            owner_->dbgln("ERROR: Cannot resume - not in IDLE");
                            sendMessage("ACK;RESUME=ERR_WRONG_STAT", MessageType::IMPORTANT);
                            return;
                        }

                        if (!owner_->isProtoLoaded_) {
                            owner_->dbgln("ERROR: Cannot resume - no protocol loaded");
                            sendMessage("ACK;RESUME=ERR_NO_PROTO", MessageType::IMPORTANT);
                            return;
                        }

                        // Parse resume parameters
                        int resumeStep = kvGetIntClamped(data, "STEP=", 1, 1, owner_->stepCount_);
                        int resumeLoop = kvGetIntClamped(data, "LOOP=", 1, 1, owner_->totalLoops_);
                        int autoStart  = kvGetIntClamped(data, "AUTOSTART=", 0, 0, 1);
                        
                        // Is run switch still engaged?
                        const bool runIsEngaged = owner_->runActiveRemote_;

                        // Validate step bounds
                        if (resumeStep > owner_->stepCount_ || resumeStep < 1) {
                            owner_->dbgln("ERROR: Resume step out of bounds");
                            sendMessage("ACK;RESUME=ERR_BAD_STEP", MessageType::IMPORTANT);
                            return;
                        }

                        // Apply the resume position
                        owner_->currentStep_ = (uint16_t)(resumeStep - 1);  // Convert to 0-based index
                        owner_->loopCount_ = owner_->totalLoops_ - (uint32_t)resumeLoop + 1;

                        // Allow future RUN edges now that XPB has explicitly coordinated resume
                        owner_->runGateReleased_ = true;   // XPB explicitly allowed coordinated start

                        // --- Gating Rules ---
                        // if AUTOSTART==0 or RUN is not LOW, don't preheat nor start.
                        if (autoStart != 1 || !runIsEngaged) {
                            owner_->autoStartAfterPreheat_ = false;
                            owner_->waitingForTemp_        = false;
                            owner_->coldStart_             = false;
                            owner_->dbgln("RESUMED: position loaded, AUTOSTART=0 or RUN=OFF -> IDLE");
                            sendMessage("ACK;RESUME=OK", MessageType::IMPORTANT);
                            return;
                        }

                        // AUTOSTART = 1 and RUN is engaged
                        const uint16_t targetC = owner_->steps_[owner_->currentStep_].tempC;
                        if (owner_->coldStart_ && targetC > 0) {
                            owner_->state_                 = State::Preheat;
                            owner_->preheatTargetC_        = targetC;
                            owner_->waitingForTemp_        = true;
                            owner_->autoStartAfterPreheat_ = true; // implied by AUTOSTART=1

                            char cmd[48];
                            snprintf(cmd, sizeof(cmd), "CMD;SP=%u", owner_->preheatTargetC_);
                            sendMessage(cmd, MessageType::IMPORTANT);

                            owner_->dbgln("PREHEAT queued (AUTOSTART=1)");
                        } else {
                            // No preheat path
                            owner_->coldStart_ = false;  // important so a later resume doesn't re-preheat
                            owner_->state_ = State::Resume;        // funnel through unified resume
                            owner_->dbgln("RESUMED: AUTOSTART=1, RUN=ON (no preheat)");
                            
                        }
                        
                        sendMessage("ACK;RESUME=OK", MessageType::IMPORTANT);
                        return;
                    }
                    return;
                }       
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
                sendMessage(ack, MessageType::INFO);                    

                return;
            }

            // ===== Protocol Upload: PR_BEG =====
            if (data.startsWith("PR_BEG;")) {
                //owner_->currentlyLoadingProto_ = true;
                // Only accept in BOOT state
                if (owner_ && (owner_->state_ == State::BOOT)) {
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
                    owner_->stepCount_ = steps;
                    
                    // ACK with REF if present
                    char ack[40];
                    if (hasRef) {
                        snprintf(ack, sizeof(ack), "ACK;PR_BEG=OK;REF=%u", refVal);
                    } else {
                        snprintf(ack, sizeof(ack), "ACK;PR_BEG=OK");
                    }
                    sendMessage(ack, MessageType::INFO);
                    owner_->state_ = State::PROTO_LOADING;
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
                    // bounds checking to bug hunt currentSteps_ corruption
                    if (owner_->protoRx_.seq >= owner_->kMaxProtocolSteps) {
                        // Send NAK and abort
                        char nak[40];
                        snprintf(nak, sizeof(nak), "ACK;PR_DAT=SEQ_OOB;MAX=%u", 
                                owner_->kMaxProtocolSteps);
                        sendMessage(nak, MessageType::INFO);
                        owner_->protoRx_.active = false;
                        return;
                    }
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
                    (void)expectedCrc;  // TODO: implement CRC check
                    
                    // Protocol successfully received - activate it
                    owner_->protoRx_.active = false;
                    owner_->progHash_ = owner_->protoRx_.phash;
                    
                    // Reset to step 0, restore full loop count
                    owner_->currentStep_ = 0;
                    owner_->loopCount_ = owner_->totalLoops_;
                    
                    // Log the received protocol
                    owner_->dbgln("==== Protocol Received ====");
                    owner_->dbgkv("Name: ", owner_->protocolName_);
                    owner_->dbgln("");
                    owner_->dbgkv("Loops: ", owner_->totalLoops_);
                    owner_->dbgln("");
                    owner_->dbgkv("Steps: ", owner_->stepCount_);
                    owner_->dbgln("");
                    owner_->dbgkv("PHASH: ", owner_->progHash_);
                    owner_->dbgln("");
                    
                    // Print steps for verification
                    for (uint8_t i = 0; i < owner_->stepCount_; ++i) {
                        int32_t rpm = (owner_->steps_[i].speedSteps_s * 60 + 
                                    (owner_->steps_[i].speedSteps_s >= 0 ? 
                                    owner_->kStepsPerRev / 2 : -owner_->kStepsPerRev / 2)) 
                                    / owner_->kStepsPerRev;
                        uint32_t accel = (owner_->steps_[i].accelSteps_s2 * 60 + 
                                        owner_->kStepsPerRev / 2) / owner_->kStepsPerRev;
                        uint32_t dwell = owner_->steps_[i].dwellMs / 1000;
                        
                        char buf[80];
                        snprintf(buf, sizeof(buf), "Step %2u: %6ld RPM  %4lu RPM/s²  %3lu s",
                                i + 1, rpm, accel, dwell);
                        owner_->dbgln(buf);
                    }
                    owner_->dbgln("=========================");
                    
                    char ack[40];
                    if (hasRef) {
                        snprintf(ack, sizeof(ack), "ACK;OK;REF=%u", refVal);
                    } else {
                        snprintf(ack, sizeof(ack), "ACK;OK");
                    }
                    sendMessage(ack, MessageType::INFO);
                    delay(3);

                    // Send success notice
                    char notice[64];
                    snprintf(notice, sizeof(notice), "NOTICE;PROTO_RX=OK;PHASH=%lu",
                            (unsigned long)owner_->progHash_);
                    sendMessage(notice, MessageType::INFO);

                    owner_->dbgln("[PROTO] Upload complete - ready to run");
                    owner_->state_ = State::Idle;
                    owner_->isProtoLoaded_ = true;
                    owner_->runGateReleased_ = true;
                    owner_->latchedRunPending_ = owner_->runActiveRemote_;
                    if (owner_->latchedRunPending_) {
                        owner_->dbgln("[RUN] Latched RUN will auto-start after proto verification");
                    } else {
                        owner_->dbgln("[RUN] Gate reopened after protocol verification");
                    }

                } else {
                    // Harmless duplicate PR_END (likely XPB retry): ACK & ignore
                    char ack[40];
                    if (hasRef) snprintf(ack, sizeof(ack), "ACK;OK;REF=%u", refVal);
                    else        snprintf(ack, sizeof(ack), "ACK;OK");
                    sendMessage(ack, MessageType::INFO);

                    if (owner_ && owner_->isProtoLoaded_) {
                        owner_->dbgln("[PROTO] Duplicate PR_END ignored");
                        // keep state as-is
                    } else if (owner_) {
                        owner_->dbgln("[PROTO] Upload failed (no active RX)");
                        owner_->state_ = State::BOOT;
                    }
                }

                // what do we do here if there wasn't a successful proto upload?
                // do we compare the XPB provided phash against a calculated phash
                // then ack we have successfully recieved?
                //owner_->currentlyLoadingProto_ = false;
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
                    owner_->dbgln("");
                }
                return;
            }

            // 5) Telemetry from XPB (heartbeat/stat) — never ACK
            if (data.startsWith("STAT;")) {
                static int lastSeq = -1;
                const int seq = kvGet(data, "SEQ=").toInt();
                const int out = kvGetIntClamped(data, "OUT=", 0, 0, 150);
                const int temp = kvGetIntClamped(data, "TEMP=", 0, 0, 200);

                const bool dup = (seq >= 0 && seq == lastSeq);
                if (seq >= 0) lastSeq = seq;
                if (dup) return;  // drop retried STAT frames

                if (owner_) {
                    owner_->commsHealthy_ = true;
                    owner_->xpbStaleTmr_   = 0;                 // fresh data just arrived
                    owner_->setHeaterOutput(out);

                    // Auto-clear stale-STAT E-STOP on first good STAT
                    if (owner_->state_ == State::EStop && (owner_->estopReason_ & ESTOP_STALE_STAT)) {
                        owner_->estopReason_ &= ~ESTOP_STALE_STAT;
                        owner_->state_ = State::Idle;
                        owner_->dbgln("[CC] Auto-cleared E-STOP (stale-STAT recovered)");
                    }
                    
                    // preheat completion check
                    if (owner_->waitingForTemp_ && owner_->state_ == State::Preheat) {
                        static uint8_t inRangeCount = 0;
                        const int err = temp - (int)owner_->preheatTargetC_;
                        if (err >= -2 && err <= 2) {
                            if (++inRangeCount >= 2) {  // tiny debounce
                                owner_->waitingForTemp_ = false;
                                owner_->coldStart_ = false;
                                owner_->dbgln("Pre-heat target reached");
                                owner_->state_ = State::Resume;  // always resume from preheat
                                owner_->dbgln("Auto-continue -> RESUME");
                                inRangeCount = 0;
                            }
                        } else {
                            inRangeCount = 0;
                        }
                    }

                }
                return;
            }

            // 6) / NOTICE / HB from XPB:
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
    ClearCoreTTL ttlComms_;
    
};



