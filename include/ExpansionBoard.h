// ExpansionBoard.h
#pragma once

// === Includes: Arduino & std ===
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

// === Includes: third-party libs ===
#include <elapsedMillis.h>
#include <PID_v1.h>
#include <Bounce2.h>
#include "Adafruit_MAX31855.h"

// === Includes: project headers ===
#include "LCDDriver.h"
#include "TTLComms.h"

// === MCU-specific (guarded) ===
#if defined(ARDUINO_ARCH_AVR)
  #include <avr/io.h>
#endif

/**
 * @brief Expansion board main controller (UI, sensors, comms, heater).
 * @details Call begin() once from setup(), then tick() every loop().
 */
class ExpansionBoard {
public:
    /**
     * @brief Initialize hardware, peripherals, SD, comms, and UI.
     * @return true on success; false if a fatal init fails (e.g., LCD init).
     * @post If ClearCore link is detected, the LCD shows “ClearCore READY”.
     */
    bool begin();

    /**
     * @brief Main periodic task. Drive comms, inputs, sensors, PID, heartbeat, and UI.
     * @details Non-blocking; render exactly one UI page per call.
     *          Also handles “Resume?” prompt and reset countdown UI flows.
     */
    void tick();

    /**
     * @brief Set the maximum interval between successive sensor updates.
     * @param milli_secs Interval in milliseconds.
     */
    void setDataInterval(uint16_t milli_secs);

    /**
     * @brief Set heater setpoint in °C.
     * @param temp Target temperature (°C). 0 disables PID output.
     */
    void setHeaterTarget(double temp) { heater_.setTargetTemp(temp); }

    ExpansionBoard() : ttlComms_(this) {}

    /// @brief Reset all runtime state to "just booted, protocol in RAM, waiting for CC."
    void logicalReset();

private:

    // ---------- Boot & Reset ----------
    bool          resetUiActive_ = false;
    uint8_t       resetUiSecs_   = 0;       // total seconds armed
    elapsedMillis resetUiTmr_;              // for 1 Hz decrement
    uint8_t       resetUiRemaining_ = 0;    // current ETA to show
    bool          ccReady_ = false;
    bool          ccAnySeen_ = false;
    elapsedMillis sinceBoot;
    uint32_t estopUiMaskUntilMs_ = 0;       // While now < this, show "Resetting" instead of E-STOP.
    /// @brief If true, suppress the Resume? prompt after boot (XPB-only reboot).


    // ---------- Debug Helpers ---------
#if XPB_DEBUG
    inline void dbg(const char *s)   const { if (Serial) Serial.print(s); }
    inline void dbgln(const char *s) const { if (Serial) Serial.println(s); }
    inline void dbgln()              const { if (Serial) Serial.println(); }
    inline void dbgkv(const char *k, const char *v)   const { if (Serial) { Serial.print(k); Serial.print(v); } }
    inline void dbgkv(const char *k, const String &v) const { if (Serial) { Serial.print(k); Serial.print(v); } }
    inline void dbgkv(const char *k, unsigned long v) const { if (Serial) { Serial.print(k); Serial.print(v); } }
    inline void dbgkv(const char *k, long v)          const { if (Serial) { Serial.print(k); Serial.print(v); } }
#else
    inline void dbg(const char *)                     const {}
    inline void dbgln(const char *)                   const {}
    inline void dbgln()                               const {}
    inline void dbgkv(const char *, const char *)     const {}
    inline void dbgkv(const char *, const String &)   const {}
    inline void dbgkv(const char *, unsigned long)    const {}
    inline void dbgkv(const char *, long)             const {}
#endif

    // ---------- CC heartbeat / state mirror ----------
    enum class LinkState : uint8_t {NoLink, Alive};
    LinkState  linkState_ = LinkState::NoLink;
    bool warnedNoLink_ = false;

    elapsedMillis ccHbAgeTmr_;
    bool          ccHbSeen_{false};
    uint16_t      hbSeq_ = 0;

    bool  ccAlarmActive_ = false;
    char  ccAlarmMsg_[LCDDriver::kNumCols + 1] = {0};

    char     ccState_[12] = "IDLE";
    uint8_t  ccStep_ = 0;
    uint32_t ccLoopCur_ = 0, ccLoopTot_ = 0;
    uint32_t ccSwAgeMs_ = 0;
    int16_t  ccRpm_ = 0;
    bool     ccEstop_ = false; // probably not needed
    uint8_t  ccEstopCode_ = 0;   // raw E_CODE bitmask from CC


    /**
     * @brief Prints eStop cause to text
     * @param e eCode
     * @param out char array to print text
     */
    static void ecodeToText(uint8_t e, char *out, size_t n = LCDDriver::kNumCols) {
        // keep it brief; list first matching cause
        if (e & 0x02) { snprintf(out, n, "Safety input"); return; }
        if (e & 0x01) { snprintf(out, n, "XPB comms stale"); return; }
        snprintf(out, n, "Unknown (0x%02X)", e);
    }


    // ---------- Pins ----------
    static constexpr uint8_t LCD_CS_      = 8;
    static constexpr uint8_t TC1_CS_      = 9;
    static constexpr uint8_t TC2_CS_      = 10;
    static constexpr uint8_t RUN_SW_PIN_  = 2;
    static constexpr uint8_t RESET_SW_PIN_= 3;
    static constexpr uint8_t SD_CS_       = 4;

    // ---------- User Input ----------
    Bounce runSw_;
    Bounce resetSw_;
    /**
     * @brief Publish debounced RUN/RESET switch state to ClearCore.
     * @param force When true, publish regardless of last sent state.
     */
    void publishSwitchState_(bool force = false, int ref = -1);
    uint32_t lastSwPublishMs_ = 0;

    // ---------- SD / Protocol ----------
    /**
     * @brief Drive all SPI chip-selects HIGH and start SPI.
     * @details Prevents other devices (LCD/TC) from holding MISO and breaking SD init.
     *          Safe to call repeatedly.
     */
    void spiQuiesceAll_();

    /**
     * @brief Initialize SD with retries/backoff.
     * @param tries     Number of attempts (default 5).
     * @param backoffMs Delay between attempts in ms (default 40).
     * @return true if SD.begin() eventually succeeds.
     * @details Re-asserts CS-high and SPI.begin() on each try to recover from soft-resets.
     *          Uses dbg* for status; never blocks for long.
     */
    bool sdInitWithRetry_(uint8_t tries = 10, uint16_t backoffMs = 100);

    /**
     * @brief Load protocol CSV from SD and populate steps_.
     * @param path Absolute path to CSV (e.g., "/protocol.csv").
     * @return true if parsed successfully and at least one step was loaded.
     * @details Expected format:
     *          - Line 1: PROTOCOL_NAME=Name
     *          - Line 2: LOOP_COUNT=N
     *          - Line 3: header (ignored)
     *          - Subsequent: RPM,ACCEL_RPM_S,DWELL_S[,TEMP_C]
     *          TEMP_C is optional and clamped to [0,200].
     * @post Updates protocolName_, loopCount_, stepCount_, steps_[], progHash_.
     */
    bool loadProtocolFromSD_(const char *path);
    void clearResumeSlots_();
    
    /**
     * @brief Incremental CRC-32 (poly 0xEDB88320) updater.
     * @param crc Running CRC (use 0 to start a new CRC).
     * @param data Pointer to bytes.
     * @param len  Number of bytes.
     * @return Updated CRC value.
     */
    static uint32_t crc32_update_(uint32_t crc, const uint8_t *data, size_t len);
    
    /**
     * @brief Dump the loaded protocol to Serial in human-friendly form.
     */
    void logProtocol_() const;

    bool     haveStoredResume_{false};
    bool     successfulProtoLoadFromSD_{false};
    uint32_t storedPhash_{0};
    uint16_t storedStep_{0}, storedLoopCur_{0}, storedLoopTot_{0};

    // ---------- Sensors ----------
    Adafruit_MAX31855 tc1_{TC1_CS_};
    Adafruit_MAX31855 tc2_{TC2_CS_};
    uint16_t      kDataIntervalMs_ = 100;
    elapsedMillis dataTmr_;
    double        latestSealC_ = NAN;
    double        latestSumpC_ = NAN;

    /**
     * @brief Read one MAX31855 in °C and report faults to Serial.
     * @param TC MAX31855 instance.
     * @param label Label used in fault prints (e.g., "TC1").
     * @return Temperature in °C, or NAN on fault.
     */
    double readTC(Adafruit_MAX31855 &TC, const char *label);

    /**
     * @brief Update onboard sensor readings on a timed cadence.
     * @details Uses kDataIntervalMs_ and dataTmr_ to throttle reads.
     *          Updates latestSealC_ and latestSumpC_.
     */
    void updateData();

    // ---------- Display / UI ----------
    LCDDriver     lcd_{LCD_CS_};
    bool          lcdToggle_{false};
    bool          modeTorqueToggle_{false}; // torque stand only
    elapsedMillis lcdTmr_;
    uint16_t      lcdToggle_ms_{2000};

    /// @brief UI pages.
    enum class UiPage : uint8_t {
        Boot, ProtoMissingSD, ProtoTxFail,
        Resetting, LostComms, EStop, ResetCountdown, 
        Preheat, Normal
    };
    UiPage lastUi_{UiPage::Boot};

    enum class BootPhase : uint8_t {
        Start,
        SDLoaded,
        TxInProgress,
        TxSuccess,
        Done
    };
    BootPhase bootPhase_ = BootPhase::Start;
    elapsedMillis bootMsgSince_{0};
    static constexpr uint16_t kBootSuccessShowMs = 2000;

    /**
     * @brief Render the top-level UI page (one page per tick).
     * @param page Target page to render.
     * @details Clears the LCD only when the page changes to prevent flicker,
     *          then draws that page and flushes once.
     */
    void renderUi_(UiPage page);
    
    /**
     * @brief Draw the “Normal” runtime page (no clear/flush here).
     * @details Caller is responsible for lcd_.flush() after drawing.
     *          Shows protocol/state on lines 0–1 and RTM/torque views on 2–3.
     */
    void renderNormal_();
    void refreshStepCountdown_(bool stepOrLoopChanged);
    void formatStepCountdown_(char *dst, size_t len) const;
    
    // ---------- Protocol model ----------
    struct Step {
        int32_t  rpmTarget_ {0};
        uint32_t rpmAccel_  {0};
        uint32_t dwellS_    {0};
        uint16_t tempC_     {0};
    };
    static constexpr uint8_t kMaxProtocolSteps_ = 50;
    Step     steps_[kMaxProtocolSteps_];
    uint8_t  stepCount_   {0};
    uint32_t loopCount_   {1};
    char     protocolName_[21] = "Test Code";
    uint32_t progHash_    {0};
    uint32_t totalLoops_  {1};
    bool     targetMet_   {false};
    bool     everRan_     {false};
    bool uploadProtocolToCC_();
    bool ccProtoReq_      {false};
    bool needResumeAfterProto_ = false;
    
    // ------- Protocol Tx State --------
    enum class ProtoTxState : uint8_t {
        Idle,          // CC hasn't asked yet
        WaitingReq,    // Proto loaded from SD, waiting for REQ:PROTO from CC
        SDFail,        // Loading from SD failed
        BegSent,       // PR_BEG sent, waiting for ACK
        Sending,       // PR_DAT streaming
        EndSent,       // PR_END sent
        AwaitResult,   // waiting for NOTICE;PROTO_RX=*
        Complete,      // NOTICE;PROTO_RX=OK received (and PHASH matches)
        Failed,        // PROTO_RX=FAIL
        Timeout        // our own timeout
    };
    ProtoTxState protoState_ = ProtoTxState::Idle;
    elapsedMillis sdRecoveryTmr_{0};   // non-blocking SD retry cadence
    elapsedMillis protoSince_{0};      
    uint16_t     lastProtoRef_ = 0;
    uint8_t      protoStepSent_ = 0;
    static constexpr uint16_t kProtoAckTimeoutMs_ = 1500;
    static constexpr uint16_t kProtoSilenceTimeoutMs = 3000;

    // ---------- Preheat control ----------
    bool           preheatActive_{false};
    uint16_t       preheatSpC_{0};
    elapsedMillis  preheatTmr_;
    static constexpr uint8_t  kPreheatBandC_  = 2;  // °C hysteresis
    static constexpr uint16_t kPreheatSoakMs_ = 0;  // optional soak
    inline double  preheatPv_() const { return isnan(latestSumpC_) ? 0 : latestSumpC_; }

    // --- USB simulation hold for injection ---
    bool     usbSimHold_ = false;
    uint32_t usbSimHoldUntilMs_ = 0;
    bool usbInjecting_ = false;

    uint8_t   countdownStepSnapshot_{0};
    uint32_t  countdownLoopSnapshot_{0};
    uint32_t  stepStartAgeMs_{0};
    uint32_t  stepTotalMs_{0};
    uint32_t  stepRemainingMs_{0};

    // ---------- Comms adapter (Serial1 TTL) ----------
    /**
     * @brief TTL serial adapter bound to ExpansionBoard (routes callbacks to owner).
     */
    class ExpansionBoardTTL : public TTLComms {
    public:
        /**
         * @brief Construct with back-reference to owning ExpansionBoard.
         */
        explicit ExpansionBoardTTL(ExpansionBoard *owner) : owner_(owner) {}

        /**
         * @brief Initialize the UART and common base plumbing.
         * @note Uses Serial1 @ 9600 baud for ClearCore.
         */
        void begin() {
            Serial1.begin(9600);
            delay(100);
            // flush garbage
            while (Serial1.available()) { Serial1.read(); }
            beginBase();
        }
        
        /** @brief Send raw bytes to the TTL link (Serial1). */
        void serialSend(const char* data) override {
            Serial1.print(data);
        }
        /** @brief @return true if bytes are available on Serial1 RX. */
        bool serialAvailable()  override { return Serial1.available(); }
        /** @brief Read one byte from Serial1 RX. */
        char serialRead()       override { return Serial1.read(); }
        /** @brief Peek next byte from Serial1 RX without consuming. */
        int  serialPeek()       override { return Serial1.peek(); }
        
        /**
         * @brief Frame handler: process decoded messages from ClearCore.
         * @details Handles HELLO/READY handshakes, CMD;* control frames,
         *          HB;* heartbeats, and ALARM;* events.
         * @note Persists step/loop changes to SD for resume.
         */
        void onMessageReceived(const String& data) override;

        /**
         * @brief Callback for frames that fail checksum validation.
         * @details Rate-limited warning printed to USB Serial for diagnostics.
         */
        void onBadChecksum(const String& rawMsg) override;
    
    protected:
        /**
         * @brief Optional USB log hook used by TTLComms for RX tracing.
         */
        void usbLog(const char *s) override {
#if XPB_DEBUG
            if (Serial) Serial.println(s);
#else
            (void)s;
#endif
        }

    private:
        ExpansionBoard *owner_{nullptr};
        uint32_t badCrcCount_{0};

    };
    ExpansionBoardTTL ttlComms_;
    elapsedMillis heartbeatTmr_;

    // ---------- Heater control ----------
    /**
     * @brief Simple PID-based heater controller.
     * @details Wraps PID_v1 with °C setpoint and 0–150 output range.
     */
    class HeatingController {
    public:
        /**
         * @brief Construct with reference to comms (reserved forq future use).
         */
        HeatingController(TTLComms &comms) 
        : comms_(comms), pid_(&pv_, &out_, &sp_, Kp_, Ki_, Kd_, DIRECT) {}
        
        /**
         * @brief Initialize PID (sample time 500ms, automatic mode, output 0–150).
         */
        void begin() {
            pid_.SetSampleTime(500); // 0.5s
            pid_.SetMode(AUTOMATIC);
            pid_.SetOutputLimits(0, 150);
        }

        /**
         * @brief Set the heater setpoint (°C). Zero disables output.
         * @param celsius Target temperature in °C.
         */
        void setTargetTemp(double celsius);

        /**
         * @brief Run one PID compute step.
         * @param processValue Current PV in °C.
         * @param outInt Output (0–150) written on successful compute.
         * @return true if PID computed a new output this call.
         */
        bool compute (double processValue, int &outInt);

        int    lastOut()    const { return (int)lround(out_); }
        double setpoint()   const { return sp_; }
        double pv()         const { return pv_; }
    
    private:
        TTLComms &comms_;
        double sp_ = 0, pv_ = 0, out_ = 0;
        double Kp_ = 60, Ki_ = 40, Kd_ = 25;
        PID pid_;
        bool active_ = false;
    };
    HeatingController heater_{ttlComms_};
    elapsedMillis pidTmr_;
};

