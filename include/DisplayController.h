// DaughterController.h
#pragma once

#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <elapsedMillis.h>

class DisplayController {
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

  /* ——— LCD Display Settings ——— */
  const SPISettings LCDspiCfg_{ 100000, MSBFIRST, SPI_MODE3 };
  static constexpr uint8_t  kNumCols_ = 20;
  static constexpr uint8_t  kNumRows_ = 4;
  const uint8_t kRowAddr_[kNumRows_] = {0x00, 0x40, 0x14, 0x54};

  /* ——— LCD front/shadow buffers ——— */
  char buf_[kNumCols_ + 1]             = {};
  char front_[kNumRows_][kNumCols_ + 1] = {};
  char sent_ [kNumRows_][kNumCols_ + 1] = {};
  bool dirty_[kNumRows_]               = {true, true, true, true};

  /* ——— LCD behaviour toggles ——— */
  bool lcdToggle_{false}, lcdRuntimeToggle_{false}, modeTorqueToggle_{false}; // torque mode is for torque stand only
  elapsedMillis lcdTmr_;
  uint16_t lcdToggle_ms_{2000}; // default to every 3s
  uint32_t runMins_{42};

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

    
  /* ——— LCD helpers ——— */
  static inline uint8_t fastLen_(const char *s) { uint8_t n = 0; while (n < kNumCols_ && s[n]) ++n; return n; }
  void lcdBlank      (char *dst);
  void lcdLineBlank  (uint8_t row);                                   // blank a line in the front buffer
  void lcdLineLeft   (uint8_t row, const char *txt);                  // fill a line w/ a left justified string
  void lcdLineRight  (uint8_t row, const char *txt);                  // fill a line w/ a right justified string
  void lcdLineCenter (uint8_t row, const char *txt);                  // fill a line w/ a center justified string
  void lcdLineLR     (uint8_t row, const char *l, const char *r);     // fill a line w/ two strings, right and left justified
  void lcdFlush      ();                                              // print all lines to screen
  void lcdClearScreen();                                              // does what it says on the tin...
  void renderScreen  ();                                              // call this to update screen with test details

  /* ——— Sensor helpers ——— */
  double readTC(Adafruit_MAX31855 &TC, const char *label);
  void updateData();

};