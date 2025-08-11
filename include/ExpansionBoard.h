// ExpansionBoard.h
#pragma once

#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include "LCDDriver.h"
#include "TTLComms.h"
#include <elapsedMillis.h>

class ExpansionBoard {
public:
  /// Call once from main (or setup())
  bool begin();

  /// Call from loop()
  void tick();

  void setDataInterval(uint16_t milli_secs);

private: 
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

  uint8_t  stepCount_   {0};
  uint8_t  loopCount_   {1};
  uint8_t  totalLoops_  {1};
  String   protocolName_ {"Test Code"};
  bool targetMet_ {false};

   /* ——— Sensor helpers ——— */
  double readTC(Adafruit_MAX31855 &TC, const char *label);
  void updateData();

    class ExpansionBoardTTL : public TTLComms {
    public:
        void begin() {
            Serial1.begin(9600);
            delay(100);
        }
        
        // Implement serial interface for Arduino Serial1
        void serialSend(const char* data) override {
            Serial1.print(data);
            Serial.print("XPB -> CC: ");
            Serial.print(data);  // Debug output
        }
        
        bool serialAvailable() override {
            return Serial1.available();
        }
        
        char serialRead() override {
            return Serial1.read();
        }
        
        int serialPeek() override {
            return Serial1.peek();
        }
        
        // Handle received messages
        void onMessageReceived(const String& data) override {
            if (data.startsWith("ACK:")) {
                String ackType = data.substring(4);
                if (ackType == "OK") {
                    Serial.println("ClearCore acknowledged message OK");
                } else if (ackType == "BAD_CHECKSUM") {
                    Serial.println("ClearCore reported bad checksum!");
                }
            } else if (data == "HEARTBEAT") {
                Serial.println("Received valid heartbeat with checksum!");
            } else {
                Serial.print("Received valid message: ");
                Serial.println(data);
            }
        }
        
        void onBadChecksum(const String& rawMsg) override {
            Serial.print("Bad checksum: ");
            Serial.println(rawMsg);
        }
    };
    ExpansionBoardTTL ttlComms_;
    elapsedMillis heartbeatTmr_;
};