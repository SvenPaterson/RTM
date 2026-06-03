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
#include "RtmComms.h"
#include "RtmNet.h"
#include <PID_v1.h>
#include <type_traits>


#include <Ethernet.h>

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
#define LED_PIN             ConnectorIO0
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

    ClearCoreRTM() : comms_(this) {} // ctor

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
    elapsedMillis linkRecoveryTmr_;     // debounces UDP socket reinit attempts
    bool          linkDownActive_{false};
    uint32_t      linkDownSinceMs_{0};
    uint16_t      linkReinitCount_{0};
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
    bool fromLogicalReset_{false};    // skip 5s BOOT delay after logical reset
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
        ESTOP_THERMAL    = 0x04,    // sump over-temperature
    };
    uint8_t estopReason_ { 0 };

    /* ——— user input ——— */
    bool        runActiveRemote_   = false;
    bool        resetActiveRemote_ = false;
    bool        runGateReleased_   = false; //!< RUN line (active-low) ignored until XPB grants start or we observe a post-boot high
    bool        resetGateReleased_ = false; //!< RESET line ignored until switch returns high after boot/reset
    bool        latchedRunPending_ = false; //!< Latched RUN request awaiting protocol verification
    enum class RunTrigger : uint8_t { ManualEdge, LatchedAuto };
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

    /* ——— HLFB speed feedback (span-based frequency measurement) ——— */
    static constexpr uint8_t  kHlfbPPR     = 16;        // pulses per rev (match motor MSP config)
    static constexpr uint32_t kHlfbStaleUs = 400000;    // 400 ms no-edge timeout → RPM = 0
    int16_t  measuredRpm_{0};        // latest computed RPM (signed via VelocityRefCommanded)
    uint32_t hlfbLastEdgeUs_{0};     // timestamp of last HLFB rising edge (μs)
    bool     hlfbFirstEdge_{true};   // true until first edge of current window
    uint16_t hlfbEdgeCount_{0};      // edges counted in current HB window
    uint32_t hlfbWindowStartUs_{0};  // timestamp of first edge in window
    uint32_t hlfbWindowLastUs_{0};   // timestamp of most recent edge in window
    inline void pollHlfbEdge_();     // poll HlfbHasRisen and update edge counters

    /* ——— heater behaviour ——— */
    bool coldStart_{true};           // True on boot, false once running
    bool waitingForTemp_{false};     // True when preheating
    uint16_t preheatTargetC_{0};     // Target temp for preheat
    bool autoStartAfterPreheat_{false}; // Whether to auto-start after preheat
    bool protocolUsesHeat_{false};       // True if any step has tempC > 0
    uint16_t sealTempC_{0};              // Latest seal temp from XPB STAT
    void setHeaterOutput(int out);
    void sendHeaterOff_();

    /* ——— heater PID (moved from XPB) ——— */
    double pidSp_{0}, pidPv_{0}, pidOut_{0};
    double pidKp_{60}, pidKi_{40}, pidKd_{25};
    PID    pid_{&pidPv_, &pidOut_, &pidSp_, pidKp_, pidKi_, pidKd_, DIRECT};
    bool   pidActive_{false};
    void runHeaterPid_(int sumpC);

    /* ——— thermal safety ——— */
    static constexpr uint16_t kSumpAbsoluteCeilingC = 150;
    static constexpr uint16_t kSumpDeltaAlarmC      = 35;
    static constexpr uint8_t  kSumpAlarmDebounce    = 3;
    uint16_t activeSetpointC_{0};        // last CMD;SP sent to XPB
    uint8_t  sumpOverTempCount_{0};      // consecutive over-temp STAT frames

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
    bool promoteRun_(RunTrigger trigger);
    void logicalReset();  ///< In-place state reset (replaces SysMgr.ResetBoard)

    class ClearCoreComms : public RtmComms {
    public:
        explicit ClearCoreComms(ClearCoreRTM *owner) : owner_(owner) {}
        // ----- UDP byte-transport bridging RtmComms.serialSend/Available/Read/Peek -----
        // TX: buffer characters until '\n' (or near overflow), then ship as
        //     one datagram to peer IP:port — preserves RtmComms's frame
        //     boundaries.
        // RX: pumpRx_() drains udp.parsePacket()/read() into a ring buffer
        //     consumed by serialAvailable/Read/Peek. Caller invokes
        //     pumpRx_() once per tick alongside checkForMessages().
        void begin() {
            // Static IP setup. Teknic Ethernet stack is built-in; no CS pin
            // dance needed (unlike the W5500 on the XPB).
            uint8_t mac[6];
            for (uint8_t i = 0; i < 6; ++i) mac[i] = RtmNet::kCcMac[i];
            IPAddress ip(RtmNet::kCcIp[0], RtmNet::kCcIp[1],
                         RtmNet::kCcIp[2], RtmNet::kCcIp[3]);
            Ethernet.begin(mac, ip);
            udp_.begin(RtmNet::kUdpPort);
            txLen_ = 0;
            rxHead_ = rxTail_ = 0;
            beginBase();
        }

        /**
         * @brief Tear down + re-open the UDP socket.
         * @details Recovery path for the case where the underlying Ethernet
         *          stack has gone silent (router reboot, link bounce, peer
         *          reboot, socket soft-lock). Caller is responsible for
         *          cadence; ClearCoreRTM::tick() debounces to 5s while
         *          xpbStaleTmr_ > 5s.
         */
        void reinitUdp() {
            udp_.stop();
            udp_.begin(RtmNet::kUdpPort);
            txLen_ = 0;
            rxHead_ = rxTail_ = 0;
        }

        void serialSend(const char* data) override {
            while (*data) {
                if (txLen_ >= sizeof(txBuf_)) {
                    flushTx_();
                }
                txBuf_[txLen_++] = *data;
                if (*data == '\n') {
                    flushTx_();
                }
                ++data;
            }
        }

        bool serialAvailable() override {
            pumpRx_();
            return rxHead_ != rxTail_;
        }

        char serialRead() override {
            pumpRx_();
            if (rxHead_ == rxTail_) return -1;
            char c = rxBuf_[rxTail_];
            rxTail_ = (rxTail_ + 1) % kRxRingSize;
            return c;
        }

        int serialPeek() override {
            pumpRx_();
            if (rxHead_ == rxTail_) return -1;
            return (uint8_t)rxBuf_[rxTail_];
        }

        bool observerActive() const {
    #if RTM_TEE_TO_PC
            return pcObserverActive_();
    #else
            return false;
    #endif
        }

    private:
        EthernetUDP udp_;
        IPAddress   peerIp_{RtmNet::kXpbIp[0], RtmNet::kXpbIp[1],
                            RtmNet::kXpbIp[2], RtmNet::kXpbIp[3]};
#if RTM_TEE_TO_PC
        IPAddress   pcIp_{RtmNet::kPcIp[0], RtmNet::kPcIp[1],
                          RtmNet::kPcIp[2], RtmNet::kPcIp[3]};
    IPAddress   pcTeeIp_{RtmNet::kObserverBroadcastIp[0], RtmNet::kObserverBroadcastIp[1],
                 RtmNet::kObserverBroadcastIp[2], RtmNet::kObserverBroadcastIp[3]};
    uint32_t pcObserverUntilMs_{0};
#endif

        // TX line-buffer: one datagram per framed line. 192 bytes covers
        // MAX_MSG_LEN (160) plus checksum/REF overhead.
        static constexpr size_t kTxBufSize = 192;
        char     txBuf_[kTxBufSize]{};
        uint16_t txLen_{0};
        uint32_t unicastFailCount_{0};
        uint32_t unicastRecycleCount_{0};
        uint16_t unicastFailStreak_{0};
        bool     unicastLastOk_{true};
        // After this many consecutive endPacket()==0 results we assume the
        // ARP entry for peerIp_ has gone stale (or the W5500 ARP cache on the
        // far side is wedged) and recycle the local UDP socket to force a
        // fresh ARP probe on the next transmit. Tuned conservatively: at
        // 100 Hz heartbeat that is ~50 ms of dropped unicast before we kick.
        static constexpr uint16_t kUnicastFailRecycleThresh = 5;

        // RX ring buffer fed by pumpRx_(). Power-of-two size for cheap mod.
        static constexpr size_t kRxRingSize = 256;
        char     rxBuf_[kRxRingSize]{};
        uint16_t rxHead_{0};  // write index
        uint16_t rxTail_{0};  // read index
#if RTM_TEE_TO_PC
        static constexpr uint32_t kUcastDiagPeriodMs = 1000;
        uint32_t nextUcastDiagMs_{0};
#endif

        void flushTx_() {
            if (txLen_ == 0) return;
            udp_.beginPacket(peerIp_, RtmNet::kUdpPort);
            udp_.write((const uint8_t*)txBuf_, txLen_);
            const bool unicastOk = (udp_.endPacket() == 1);
            unicastLastOk_ = unicastOk;
            if (!unicastOk) {
                ++unicastFailCount_;
                if (unicastFailStreak_ < 0xFFFF) ++unicastFailStreak_;
                if (unicastFailStreak_ >= kUnicastFailRecycleThresh) {
                    // Stale ARP / wedged socket recovery: tear down and rebind
                    // the local UDP endpoint so the next beginPacket() issues
                    // a fresh ARP request for peerIp_.
                    udp_.stop();
                    udp_.begin(RtmNet::kUdpPort);
                    ++unicastRecycleCount_;
                    unicastFailStreak_ = 0;
                }
            } else {
                unicastFailStreak_ = 0;
            }
#if RTM_TEE_TO_PC
            if (pcObserverActive_()) {
                udp_.beginPacket(pcTeeIp_, RtmNet::kObserverPort);
                udp_.write((const uint8_t*)txBuf_, txLen_);
                if (!udp_.endPacket()) pcObserverUntilMs_ = 0;

                const uint32_t now = Milliseconds();
                if ((int32_t)(now - nextUcastDiagMs_) >= 0) {
                    char diag[64];
                    int n = snprintf(diag, sizeof(diag),
                                     "CCDBG;UCAST_FAIL=%lu;LAST=%u;RECYC=%lu\n",
                                     (unsigned long)unicastFailCount_,
                                     (unsigned)(unicastLastOk_ ? 1U : 0U),
                                     (unsigned long)unicastRecycleCount_);
                    if (n > 0) {
                        size_t diagLen = (size_t)n;
                        if (diagLen >= sizeof(diag)) diagLen = sizeof(diag) - 1;
                        udp_.beginPacket(pcTeeIp_, RtmNet::kObserverPort);
                        udp_.write((const uint8_t*)diag, diagLen);
                        if (!udp_.endPacket()) pcObserverUntilMs_ = 0;
                    }
                    nextUcastDiagMs_ = now + kUcastDiagPeriodMs;
                }
            }
#endif
            txLen_ = 0;
        }

        void pumpRx_() {
            int sz = udp_.parsePacket();
            while (sz > 0) {
#if RTM_TEE_TO_PC
                if (consumePcObserverPacket_(sz)) {
                    sz = udp_.parsePacket();
                    continue;
                }
#endif
                if (!isPeerIp_(udp_.remoteIP())) {
                    while (sz-- > 0) (void)udp_.read();
                    sz = udp_.parsePacket();
                    continue;
                }
                // Read up to sz bytes into the ring. Drop on overflow rather
                // than block — framing layer handles dropped frames via
                // ACK/REF retries.
                while (sz > 0) {
                    uint16_t next = (rxHead_ + 1) % kRxRingSize;
                    if (next == rxTail_) {
                        // overflow — discard remaining datagram
                        while (sz-- > 0) (void)udp_.read();
                        break;
                    }
                    int b = udp_.read();
                    if (b < 0) break;
                    rxBuf_[rxHead_] = (char)b;
                    rxHead_ = next;
                    --sz;
                }
                sz = udp_.parsePacket();
            }
        }

#if RTM_TEE_TO_PC
        bool pcObserverActive_() const {
            return (int32_t)(Milliseconds() - pcObserverUntilMs_) < 0;
        }
#endif

        bool isPeerIp_(const IPAddress &ip) const {
            return ip[0] == RtmNet::kXpbIp[0] && ip[1] == RtmNet::kXpbIp[1] &&
                   ip[2] == RtmNet::kXpbIp[2] && ip[3] == RtmNet::kXpbIp[3];
        }

#if RTM_TEE_TO_PC
        bool isPcIp_(const IPAddress &ip) const {
            return ip[0] == RtmNet::kPcIp[0] && ip[1] == RtmNet::kPcIp[1] &&
                   ip[2] == RtmNet::kPcIp[2] && ip[3] == RtmNet::kPcIp[3];
        }

        bool consumePcObserverPacket_(int sz) {
            if (!isPcIp_(udp_.remoteIP())) return false;

            char prefix[RtmNet::kObserverBeaconLen];
            uint8_t n = 0;
            while (sz > 0) {
                int b = udp_.read();
                if (b < 0) break;
                if (n < RtmNet::kObserverBeaconLen) prefix[n++] = (char)b;
                --sz;
            }

            bool match = (n == RtmNet::kObserverBeaconLen);
            for (uint8_t i = 0; match && i < RtmNet::kObserverBeaconLen; ++i) {
                match = (prefix[i] == RtmNet::kObserverBeacon[i]);
            }
            if (match) pcObserverUntilMs_ = Milliseconds() + RtmNet::kObserverTtlMs;
            return true;
        }
#endif

    public:
        
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
                        // Helper: echo the sender's REF in every ACK so the
                        // transport layer on the other side can correlate it.
                        char ackBuf[48];

                        // 1) must have protocol
                        if (owner_->state_ != State::Idle) {
                            owner_->dbgln("ERROR: Cannot resume - not in IDLE");
                            snprintf(ackBuf, sizeof(ackBuf), "ACK;RESUME=ERR_WRONG_STAT;REF=%u", (unsigned)refVal);
                            sendMessage(ackBuf, MessageType::NORMAL);
                            return;
                        }

                        if (!owner_->isProtoLoaded_) {
                            owner_->dbgln("ERROR: Cannot resume - no protocol loaded");
                            snprintf(ackBuf, sizeof(ackBuf), "ACK;RESUME=ERR_NO_PROTO;REF=%u", (unsigned)refVal);
                            sendMessage(ackBuf, MessageType::NORMAL);
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
                            snprintf(ackBuf, sizeof(ackBuf), "ACK;RESUME=ERR_BAD_STEP;REF=%u", (unsigned)refVal);
                            sendMessage(ackBuf, MessageType::NORMAL);
                            return;
                        }

                        // Apply the resume position
                        owner_->currentStep_ = (uint16_t)(resumeStep - 1);  // Convert to 0-based index
                        owner_->loopCount_ = owner_->totalLoops_ - (uint32_t)resumeLoop + 1;

                        // --- Gating Rules ---
                        // AUTOSTART=1 means XPB found valid resume data AND
                        // the RUN switch was engaged at boot — this is a
                        // power-loss recovery, not a fresh start.  Release
                        // the boot gate so the system resumes immediately.
                        if (!owner_->runGateReleased_ && autoStart == 1 && runIsEngaged) {
                            owner_->runGateReleased_ = true;
                            owner_->latchedRunPending_ = false;
                            owner_->dbgln("[RESUME] AUTOSTART=1 + RUN=ON — gate released for power-loss recovery");
                            // fall through to normal AUTOSTART=1 handling below
                        }

                        // if AUTOSTART==0 or RUN is not LOW, don't preheat nor start.
                        // Keep coldStart_ intact so the first manual RUN triggers preheat.
                        if (autoStart != 1 || !runIsEngaged) {
                            owner_->autoStartAfterPreheat_ = false;
                            owner_->waitingForTemp_        = false;
                            owner_->dbgln("RESUMED: position loaded, AUTOSTART=0 or RUN=OFF -> IDLE");
                            snprintf(ackBuf, sizeof(ackBuf), "ACK;RESUME=OK;REF=%u", (unsigned)refVal);
                            sendMessage(ackBuf, MessageType::NORMAL);
                            return;
                        }

                        // AUTOSTART = 1 and RUN is engaged
                        const uint16_t targetC = owner_->steps_[owner_->currentStep_].tempC;
                        if (owner_->coldStart_ && targetC > 0) {
                            owner_->state_                 = State::Preheat;
                            owner_->preheatTargetC_        = targetC;
                            owner_->waitingForTemp_        = true;
                            owner_->autoStartAfterPreheat_ = true; // implied by AUTOSTART=1
                            owner_->activeSetpointC_       = targetC;

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
                        
                        snprintf(ackBuf, sizeof(ackBuf), "ACK;RESUME=OK;REF=%u", (unsigned)refVal);
                        sendMessage(ackBuf, MessageType::NORMAL);
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
                    owner_->dbgln("[PROTO] PR_BEG accepted (BOOT state)");
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
                    owner_->protoRx_.phash = strtoul(phashStr.c_str(), nullptr, 10);
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
                            uint16_t tempC = 0;
                            if (c3 > 0) {
                                tempC = (uint16_t)constrain(dataStr.substring(c3+1).toInt(), 0, 200);
                            }
                            
                            // Convert and store in steps_ array  
                            Step &s = owner_->steps_[owner_->protoRx_.seq];
                            s.speedSteps_s = (rpm * owner_->kStepsPerRev + (rpm >= 0 ? 30 : -30)) / 60;
                            s.accelSteps_s2 = (accel * owner_->kStepsPerRev + 30) / 60;
                            s.dwellMs = dwellS * 1000UL;
                            s.tempC = tempC;
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

                    // Scan steps for any tempC > 0
                    owner_->protocolUsesHeat_ = false;
                    for (uint8_t i = 0; i < owner_->stepCount_; ++i) {
                        if (owner_->steps_[i].tempC > 0) {
                            owner_->protocolUsesHeat_ = true;
                            break;
                        }
                    }

                    owner_->dbgln("[PROTO] Upload complete - ready to run");
                    if (owner_->protocolUsesHeat_) {
                        owner_->dbgln("[PROTO] Protocol uses heating");
                    }
                    owner_->state_ = State::Idle;
                    owner_->isProtoLoaded_ = true;
                    owner_->heartbeatSystemEnabled_ = true;

                    // Fresh protocol upload: gate stays closed when RUN
                    // is held at boot.  Auto-start only happens via the
                    // RESUME;AUTOSTART=1 path (XPB found Ra.bin/Rb.bin).
                    // If no resume data exists the operator must release
                    // and re-assert RUN to start.
                    owner_->latchedRunPending_ = owner_->runActiveRemote_;
                    if (owner_->runGateReleased_ && owner_->latchedRunPending_) {
                        if (owner_->promoteRun_(ClearCoreRTM::RunTrigger::LatchedAuto)) {
                            owner_->latchedRunPending_ = false;
                            owner_->dbgln("[RUN] Gate was open, auto-started");
                        }
                    } else if (owner_->latchedRunPending_) {
                        owner_->dbgln("[RUN] RUN held at boot — gate closed, waiting for RESUME or RUN toggle");
                    } else {
                        owner_->runGateReleased_ = true;
                        owner_->dbgln("[RUN] Gate open, no RUN pending");
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
                const int sump = kvGetIntClamped(data, "SUMP=", 0, 0, 200);
                const int seal = kvGetIntClamped(data, "SEAL=", 0, 0, 200);

                const bool dup = (seq >= 0 && seq == lastSeq);
                if (seq >= 0) lastSeq = seq;
                if (dup) return;  // drop retried STAT frames

                if (owner_) {
                    owner_->commsHealthy_ = true;
                    owner_->xpbStaleTmr_   = 0;                 // fresh data just arrived
                    owner_->sealTempC_ = (uint16_t)seal;
                    owner_->runHeaterPid_(sump);

                    // --- Over-temperature safety ---
                    if (owner_->protocolUsesHeat_) {
                        const uint16_t sumpC = (uint16_t)sump;
                        // Absolute ceiling — immediate E-STOP
                        if (sumpC >= kSumpAbsoluteCeilingC) {
                            owner_->estopReason_ |= ESTOP_THERMAL;
                            owner_->eStopAll_("SUMP >= 150C ceiling");
                            return;
                        }
                        // Delta runaway — debounced (3 consecutive frames)
                        if (owner_->activeSetpointC_ > 0 &&
                            sumpC > owner_->activeSetpointC_ + kSumpDeltaAlarmC) {
                            if (++owner_->sumpOverTempCount_ >= kSumpAlarmDebounce) {
                                owner_->estopReason_ |= ESTOP_THERMAL;
                                owner_->eStopAll_("SUMP runaway > SP+35C");
                                owner_->sumpOverTempCount_ = 0;
                                return;
                            }
                        } else {
                            owner_->sumpOverTempCount_ = 0;
                        }
                    }

                    // Auto-clear stale-STAT E-STOP on first good STAT
                    if (owner_->state_ == State::EStop && (owner_->estopReason_ & ESTOP_STALE_STAT)) {
                        owner_->estopReason_ &= ~ESTOP_STALE_STAT;
                        owner_->state_ = State::Idle;
                        owner_->dbgln("[CC] Auto-cleared E-STOP (stale-STAT recovered)");
                    }
                    
                    // preheat completion check
                    if (owner_->waitingForTemp_ && owner_->state_ == State::Preheat) {
                        static uint8_t inRangeCount = 0;
                        const int err = sump - (int)owner_->preheatTargetC_;
                        if (err >= -2) {  // at or above (target - 2°C)
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

            // === XPB protocol ready announcement (PHASH drift detection) ===
            if (data.startsWith("NOTICE;PROTO_READY")) {
                if (owner_) {
                    String phStr = kvGet(data, "PHASH=");
                    uint32_t advertised = strtoul(phStr.c_str(), nullptr, 10);
                    // Only act when CC is in a benign state. Never disturb a running test.
                    bool benign = (owner_->state_ == State::BOOT) ||
                                  (owner_->state_ == State::Idle) ||
                                  (owner_->state_ == State::Completed);
                    bool drifted = (!owner_->isProtoLoaded_) || (advertised != owner_->progHash_);
                    if (benign && drifted) {
                        owner_->dbgln("[PROTO] PHASH drift detected; clearing local proto and re-requesting");
                        owner_->isProtoLoaded_  = false;
                        owner_->protocolName_   = "Awaiting Upload";
                        owner_->stepCount_      = 0;
                        owner_->loopCount_      = 1;
                        owner_->totalLoops_     = 1;
                        owner_->progHash_       = 0;
                        owner_->protoRx_        = {};
                        // Force handleBoot to fire REQ:PROTO immediately
                        owner_->protoRequestTmr_ = 5001;
                        if (owner_->state_ != State::BOOT) owner_->state_ = State::BOOT;
                    }
                }
                return;
            }

            // else ignore silently
        }

        void onBadChecksum(const String& raw) override {
            static uint32_t badCrcCount = 0;
            ++badCrcCount;
            // Always log protocol-related bad checksums (PR_BEG/PR_DAT/PR_END)
            // to help diagnose silent proto upload failures
            if (raw.indexOf("PR_") >= 0) {
                char line[80];
                snprintf(line, sizeof(line),
                         "WARN: bad CRC on proto frame (len=%u, #%lu)",
                         (unsigned)raw.length(), (unsigned long)badCrcCount);
                usbLog(line);
            } else if (badCrcCount % 10 == 1) {
                usbLog("WARN: bad checksum (rate-limited)");
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
    ClearCoreComms comms_;
    
};

