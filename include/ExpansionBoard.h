// ExpansionBoard.h
#pragma once

#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include "LCDDriver.h"
#include "TTLComms.h"
#include <elapsedMillis.h>
#include <PID_v1.h>

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

    /* ——— debug helpers ——— */
    // ---- Debug helpers (USB guarded) ----
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
    //char debugBuf_[150]; // for debugging to Serial
    bool usbAvail_ = false;
    //elapsedMillis usbPollTmr_;
    uint16_t hbSeq_ = 0;

    /* ——— pinouts ——— */
    static constexpr uint8_t LCD_CS_ = 8;
    static constexpr uint8_t TC1_CS_ = 9;
    static constexpr uint8_t TC2_CS_ = 10;

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

    uint8_t stepCount_      {0};
    uint8_t loopCount_      {1};
    uint8_t totalLoops_     {1};
    String  protocolName_   {"Test Code"};
    bool    targetMet_      {false};

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
            // Example command schema: "CMD;SP=120.0" (set heater setpoint °C)
            if (data.startsWith("CMD;")) {
                int k = data.indexOf("SP=");
                if (k >= 0 && owner_) {
                    k += 3;
                    int e = data.indexOf(';', k);
                    if (e < 0) e = data.length();
                    double sp = data.substring(k, e).toFloat();
                    owner_->setHeaterTarget(sp);
                }
                return;
            }

            // Add other commands as needed, e.g. "CMD;MODE=TORQUE"
            // else: ignore silently
        }
        
        void onBadChecksum(const String& rawMsg) override {
                ++badCrcCount_;
                if ((badCrcCount_ % 10 == 1 && Serial)) {
                    Serial.println("WARN: TTL bad checksum (rate-limited)");
                }
            }

        private:
            ExpansionBoard *owner_{nullptr};
            //bool usbAvail_{false};
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

