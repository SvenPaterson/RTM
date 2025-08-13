// ExpansionBoard.h
#pragma once

#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include "LCDDriver.h"
#include "TTLComms.h"
#include <elapsedMillis.h>
#include <PID_v1.h>
#include <Bounce2.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/cpufunc.h>  // for _PROTECTED_WRITE (megaAVR-0)

// Unified soft reset for AVR targets
static void xpbSoftResetNow() {
#if defined(__AVR_ATmega4809__) || defined(ARDUINO_AVR_NANO_EVERY)
  // megaAVR-0 (Nano Every): use software reset register
  // Some cores name it SWRST, some SWRR – guard both.
  #if defined(RSTCTRL_SWRST)
    _PROTECTED_WRITE(RSTCTRL.SWRST, 1);
  #elif defined(RSTCTRL_SWRR)
    _PROTECTED_WRITE(RSTCTRL.SWRR, 1);
  #else
    // Fallback to WDT if symbol names differ
    wdt_enable(WDTO_15MS);
    for (;;) {}
  #endif
#else
  // Classic AVRs (e.g., ATmega328P): WDT nuke
  wdt_enable(WDTO_15MS);
  for (;;) {}
#endif
}

class ExpansionBoard {
public:
  /// Call once from main (or setup())
  bool begin();

  /// Call from loop()
  void tick();

  void setDataInterval(uint16_t milli_secs);
  void setHeaterTarget(double temp) { heater_.setTargetTemp(temp); }

  ExpansionBoard() : ttlComms_(this) {}

private:

    /* ——— On Boot & Resetting ——— */
    bool          resetUiActive_ = false;
    uint8_t       resetUiSecs_   = 0;         // total seconds armed
    elapsedMillis resetUiTmr_;                // for 1 Hz decrement
    uint8_t       resetUiRemaining_ = 0;      // current ETA to show
    bool ccReady_ = false;

    /* ——— debug helpers ——— */
    inline void dbg(const char *s)   { if (Serial) Serial.print(s); }
    inline void dbgln(const char *s) { if (Serial) Serial.println(s); }
    inline void dbgln()              { if (Serial) Serial.println(); }

    inline void dbgkv(const char *k, const char *v)   { if (Serial) { Serial.print(k); Serial.println(v); } }
    inline void dbgkv(const char *k, const String &v) { if (Serial) { Serial.print(k); Serial.println(v); } }

    // single numeric overload
    inline void dbgkv(const char *k, unsigned long v) { if (Serial) { Serial.print(k); Serial.println(v); } }

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
    uint16_t hbSeq_ = 0;
    
    /* ——— CC heartbeat mirror for LCD ——— */
    char     ccState_[12] = "IDLE";
    uint8_t  ccStep_ = 0;
    uint32_t ccLoopCur_ = 0, ccLoopTot_ = 0;
    uint32_t ccSwAgeMs_ = 0;
    bool     ccEstop_ = false;
    uint16_t ccHbSeqPrev_ = 0, ccHbSeq_ = 0;

    /* ——— pinouts ——— */
    static constexpr uint8_t LCD_CS_ = 8;
    static constexpr uint8_t TC1_CS_ = 9;
    static constexpr uint8_t TC2_CS_ = 10;
    static constexpr uint8_t RUN_SW_PIN = 2;
    static constexpr uint8_t RESET_SW_PIN = 3;

    /* ——— User Input ——— */
    Bounce runSw_;
    Bounce resetSw_;
    void publishSwitchState_(bool force = false);
    uint32_t lastSwPublishMs_ = 0;

    /* ——— Sensor Settings ——— */
    Adafruit_MAX31855 tc1_{TC1_CS_}, tc2_{TC2_CS_};
    uint16_t kDataIntervalMs_ = 100;
    elapsedMillis   dataTmr_;  
    double          latestSealC_     = NAN;
    double          latestSumpC_     = NAN;

    /* ——— Display Settings  ——— */
    LCDDriver lcd_{LCD_CS_};
    bool lcdToggle_{false}, lcdRuntimeToggle_{false};
    bool modeTorqueToggle_{false}; // torque mode is for torque stand only
    elapsedMillis lcdTmr_;
    uint16_t lcdToggle_ms_{2000};
    uint32_t runMins_{42};
    void renderScreen  ();         // call this to update screen with test details

    /* ——— protocol steps ——— */
    struct Step {
        int32_t  speedSteps_s_   {0};   //!< target speed in steps/s
        uint32_t accelSteps_s2_  {0};   //!< accel in steps/s²
        uint32_t dwellMs_        {0};   //!< dwell after speed reached (ms)
    };

    /* ——— protocol state ——— */
    static constexpr uint8_t  kMaxProtocolSteps_ = 50;
    Step steps_[kMaxProtocolSteps_] = {};

    uint8_t  stepCount_    {0};
    uint32_t loopCount_    {1};
    uint32_t totalLoops_   {1};
    String   protocolName_ {"Test Code"};
    bool     targetMet_    {false};

    /* ——— Sensor helpers ——— */
    double readTC(Adafruit_MAX31855 &TC, const char *label);
    void updateData();

    class ExpansionBoardTTL : public TTLComms {
    public:
        explicit ExpansionBoardTTL(ExpansionBoard *owner) : owner_(owner) {}

        void begin() {
            Serial1.begin(9600);
            delay(100);
            beginBase();
        }
        
        // Implement serial interface for Arduino Serial1
        void serialSend(const char* data) override {
            Serial1.print(data);
        }
        
        // RX plumb
        bool serialAvailable()  override { return Serial1.available(); }
        char serialRead()       override { return Serial1.read(); }
        int  serialPeek()       override { return Serial1.peek(); }
        
        // RX: parse commands from CC, no prints here
        void onMessageReceived(const String& data) override {
            sendMessage("ACK:OK");

            // 1) Discovery / readiness
            if (data.startsWith("HELLO;ID=CC")) {
                // Peer is probing; reply READY (don't set ccReady_ here)
                char line[64];
                snprintf(line, sizeof(line), "READY;ID=XPB;VER=1.0;UPT=%lu",
                        (unsigned long)millis());
                sendMessage(line, MessageType::CRITICAL);
                return;
            }
            if (data.startsWith("READY;ID=CC")) {
                if (owner_) owner_->ccReady_ = true;
                return;
            }

            // Assert switches if requested
            if (data == "REQ:SW") {
                if (owner_) owner_->publishSwitchState_(true);
                return;
            }

            // ----- Unified command block -----
            if (data.startsWith("CMD;") && owner_) {
                // 1) RESET flow (may come with SECS)
                String reset = kvGet(data, "RESET=");
                if (reset.length()) {
                    if (reset == "ARM") {
                        int secs = kvGetIntClamped(data, "SECS=", 5, 1, 30);
                        owner_->resetUiActive_    = true;
                        owner_->resetUiSecs_      = (uint8_t)secs;
                        owner_->resetUiRemaining_ = (uint8_t)secs;
                        owner_->resetUiTmr_       = 0;     // start 1 Hz UI countdown
                    }
                    else if (reset == "CANCEL") {
                        owner_->resetUiActive_ = false;
                    }
                    else if (reset == "EXEC") {
                        // Reboot XPB now (watchdog)
                        delay(5);
                        xpbSoftResetNow();
                    }
                    // No return — allow other keys in the same frame to apply too.
                }

                // 2) Heater setpoint (°C)
                String sSP = kvGet(data, "SP=");
                if (sSP.length()) {
                    owner_->setHeaterTarget(sSP.toFloat());
                }

                // 3) Display mode
                String mode = kvGet(data, "MODE=");
                if (mode.length()) {
                    owner_->modeTorqueToggle_ = (mode == "TORQUE");
                }

                return;
            }

            // ----- Heartbeat from ClearCore (for LCD/status) -----
            if (data.startsWith("HB;") && owner_) {
                String sSTATE = kvGet(data, "STATE=");
                String sE     = kvGet(data, "E=");
                String sSTEP  = kvGet(data, "STEP=");
                String sLOOP  = kvGet(data, "LOOP=");
                String sAGE   = kvGet(data, "SW_AGE=");

                if (sSTATE.length()) sSTATE.toCharArray(owner_->ccState_, sizeof(owner_->ccState_));
                if (sE.length())     owner_->ccEstop_ = (sE.toInt() != 0);

                if (sSTEP.length()) {
                    long v = sSTEP.toInt();
                    if (v < 0) v = 0; if (v > 255) v = 255;
                    owner_->ccStep_ = (uint8_t)v;
                }

                if (sLOOP.length()) {
                    int slash = sLOOP.indexOf('/');
                    if (slash > 0) {
                        const char *cstr = sLOOP.c_str();
                        char *endp = nullptr;
                        unsigned long cur = strtoul(cstr, &endp, 10);
                        unsigned long tot = 0;
                        if (endp && *endp == '/') tot = strtoul(endp + 1, nullptr, 10);
                        owner_->ccLoopCur_ = (uint32_t)cur;
                        owner_->ccLoopTot_ = (uint32_t)tot;
                    }
                }

                if (sAGE.length()) owner_->ccSwAgeMs_ = (uint32_t)sAGE.toInt();
                return;
            }

            // else ignore quietly
        }


        
        void onBadChecksum(const String& rawMsg) override {
                ++badCrcCount_;
                if ((badCrcCount_ % 10 == 1 && Serial)) {
                    Serial.println("WARN: TTL bad checksum (rate-limited)");
                }
            }
    
    protected:
        void usbLog(const char *s) override {
            if (Serial) Serial.println(s);
        }

    private:
        ExpansionBoard *owner_{nullptr};
        uint32_t badCrcCount_{0};

    };
    ExpansionBoardTTL ttlComms_;
    elapsedMillis heartbeatTmr_;

    class HeatingController {
    public:
        HeatingController(TTLComms &comms) 
        : comms_(comms), pid_(&pv_, &out_, &sp_, Kp_, Ki_, Kd_, DIRECT) {}
        
        void begin() {
            pid_.SetSampleTime(500); // 0.5s
            pid_.SetMode(AUTOMATIC);
            pid_.SetOutputLimits(0, 150);
        }

        void setTargetTemp(double celsius) {
            sp_ = celsius;
            active_ = (celsius > 0);
        }

        bool compute (double processValue, int &outInt) {
            pv_ = processValue;
            if (!active_) { out_ = 0; outInt = 0; return true; }
            bool did = pid_.Compute();
            if (did) outInt = (int)lround(out_);
            return did;
        }

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

