#include "DisplayController.h"
#include <Arduino.h>
#include <SPI.h>

void DisplayController::setDataInterval(uint16_t milli_secs) {
    kDataIntervalMs_ = milli_secs;
}

bool DisplayController::begin() {
    Serial.begin(9600);
    while (!Serial) delay(1);
    Serial.println("\nUSB Serial Connected!");

    Serial.print("Initializing LCD Screen...");
    pinMode(LCD_CS_, OUTPUT);
    digitalWrite(LCD_CS_, HIGH);
    SPI.begin();
    delay(200);

    // init display
    clearScreen();
    displayOn();
    setBrightness();

    lcdLineLR(0, "Nano Every Demo", "");
    lcdLineLR(1, "Initializing...", "");
    lcdFlush();
    delay(1000);
    Serial.println("DONE");

    Serial.print("Initializing MAX31855 sensor...");
    // wait for MAX chip to stabilize
    delay(500);

    Serial.print("Initializing TC1...");
    if (!tc1_.begin()) {
        Serial.println("ERROR.");
        while (1) delay(10);
    } else Serial.println("DONE");

    /*Serial.print("Initializing TC2...");
    if (!tc2_.begin()) {
        Serial.println("ERROR.");
        while (1) delay(10);
    } else Serial.println("DONE"); */

    tc1_.setFaultChecks(MAX31855_FAULT_ALL);
    //tc2_.setFaultChecks(MAX31855_FAULT_ALL);
}

void DisplayController::tick() {
    updateData();
    
    if (lcdTmr_ >= lcdToggle_ms_) {
        lcdTmr_ = 0;
        lcdToggle_ = !lcdToggle_;
    } renderScreen();
    
}



/*********************** HELPERS ***********************/

/* ——— Sensor helpers ——— */
double DisplayController::readTC(Adafruit_MAX31855 &TC, const char *label) {
  double c = TC.readCelsius();
  if (isnan(c)) {
    uint8_t e = TC.readError();
    Serial.print(label);
    Serial.println(" fault(s):");
    if (e & MAX31855_FAULT_OPEN)      Serial.println("  • open circuit");
    if (e & MAX31855_FAULT_SHORT_GND) Serial.println("  • short to GND");
    if (e & MAX31855_FAULT_SHORT_VCC) Serial.println("  • short to VCC");
    return NAN;
  }
  return c;
}

void DisplayController::updateData() {
    if (dataTmr_ < kDataIntervalMs_) return;
    dataTmr_ = 0;

    latestSealC_ = readTC(tc1_, "TC1");
    latestSumpC_ = 120; //readTC(tc2_, "TC2"); // PLACEHOLDER
}

/* ——— LCD helpers ——— */
// See Table of Commands, p7 of NHD-0420D3Z-NSW-BBW-V3 manual 
// lookup the built-in execution times (in µs or ms) for each command:
static uint16_t lcdExecTime(uint8_t cmd) {
  switch (cmd) {
    case 0x70:      // Display Firmware
      return 4000;  /// 4ms

    case 0x46:      // Home
    case 0x47:      // Underline on
    case 0x48:      // Underline off
    case 0x51:
      return 1500;  /// 1.5ms

    case 0x52:      // Contrast
      return 500;   /// 0.5ms

                    // …add others as needed…
    default:
      return 100;   /// 0.1ms for all other writes
  }
}

void DisplayController::sendLCDCommand(uint8_t cmd, const uint8_t *params,
                                       uint8_t pLen) {
    digitalWrite(LCD_CS_, LOW);
    SPI.beginTransaction(LCDspiCfg_);
        SPI.transfer(0xFE);
        SPI.transfer(cmd);
        for (uint8_t i = 0; i < pLen; ++i) 
        SPI.transfer(params[i]);

    // wait the required execution time:
    uint16_t t = lcdExecTime(cmd);
    if (t >= 1000) {
        delay(t/1000);
    } else {
        delayMicroseconds(t);
    }

    SPI.endTransaction();
    digitalWrite(LCD_CS_, HIGH);
}

void DisplayController::sendLCDData(const char *data, size_t len) {
    digitalWrite(LCD_CS_, LOW);
    SPI.beginTransaction(LCDspiCfg_);
    for (size_t i = 0; i < len; ++i) {
        SPI.transfer(data[i]);
        delayMicroseconds(100); // req per-byte execution gap
    }
    SPI.endTransaction();
    digitalWrite(LCD_CS_, HIGH);
}

void DisplayController::lcdFlush() {
  for (uint8_t row = 0; row < kNumRows_; ++row) {
    if (!dirty_[row]) continue;

    // 1) move the cursor to the start of this row
    sendLCDCommand(0x45, &kRowAddr_[row], 1);

    // 2) blast out the 20 characters, with the 100 µs/byte delay
    sendLCDData(front_[row], kNumCols_);

    // 3) mark it clean
    memcpy(sent_[row], front_[row], kNumCols_);
    dirty_[row] = false;
  }
}


/* void DisplayController::lcdFlush() {
  for (uint8_t row = 0; row < kNumRows_; ++row) {
    if (!dirty_[row]) continue;

    // 1) Lower CS & begin SPI
    digitalWrite(LCD_CS_, LOW);
    SPI.beginTransaction(LCDspiCfg_);

    // 2) Send the 'Set Cursor' command and address in one shot
    SPI.transfer(0xFE);
    SPI.transfer(0x45);
    SPI.transfer(kRowAddr_[row]);

    // 3) Send the entire line of text
    for (uint8_t i = 0; i < kNumCols_; ++i) {
      SPI.transfer(front_[row][i]);
      delayMicroseconds(100);
    }

    // 4) End transaction & raise CS
    SPI.endTransaction();
    digitalWrite(LCD_CS_, HIGH);

    // 5) Wait out the exec time: cursor set 100 µs + 100 µs × num chars
    // (Set Cursor is 100 µs; each char write ~100 µs)
    uint16_t wait = 100 + (100 * kNumCols_);
    delayMicroseconds(wait);

    // Mark row clean
    memcpy(sent_[row], front_[row], kNumCols_);
    dirty_[row] = false;
  }
} */

void DisplayController::lcdBlank(char *dst) {
    memset(dst, ' ', kNumCols_);
    dst[kNumCols_] = '\0';
}

void DisplayController::lcdLineLeft(uint8_t row, const char *txt) {
    lcdBlank(front_[row]);
    memcpy(front_[row], txt, fastLen_(txt));
    if (memcmp(front_[row], sent_[row], kNumCols_)) dirty_[row] = true;
}

void DisplayController::lcdLineRight(uint8_t row, const char *txt) {
    lcdBlank(front_[row]);
    uint8_t len = fastLen_(txt);
    memcpy(front_[row] + kNumCols_ - len, txt, len);
    if (memcmp(front_[row], sent_[row], kNumCols_)) dirty_[row] = true;
}

void DisplayController::lcdLineCenter(uint8_t row, const char *txt) {
    lcdBlank(front_[row]);
    uint8_t len = fastLen_(txt);
    uint8_t start = (kNumCols_ - len) / 2;
    memcpy(front_[row] + start, txt, len);
    if (memcmp(front_[row], sent_[row], kNumCols_)) dirty_[row] = true;
}

void DisplayController::lcdLineLR(uint8_t row, const char *left, const char *right) {
    lcdBlank(front_[row]);
    uint8_t rLen = fastLen_(right);
    uint8_t lMax = (rLen < kNumCols_) ? kNumCols_ - rLen - 1 : 0;
    uint8_t lLen = (lMax ? (fastLen_(left) > lMax ? lMax : fastLen_(left)) : 0);
    memcpy(front_[row],               left,  lLen);
    memcpy(front_[row] + kNumCols_ - rLen, right, rLen);
    if (memcmp(front_[row], sent_[row], kNumCols_)) dirty_[row] = true;
}

void DisplayController::lcdLineBlank(uint8_t row) {
    lcdBlank(front_[row]);
    if (memcmp(front_[row], sent_[row], kNumCols_)) dirty_[row] = true;
}

void DisplayController::renderScreen() {
    char buf_1[kNumCols_+1];
    /* char buf_2[DisplayController::kNumCols_+1];
    snprintf(buf_2, sizeof(buf_2), "%s", protocolName_.c_str()) */

    // line 1: toggle between protocol name and runtime
    char left0[kNumCols_+1];
    if (lcdToggle_) {
        // show protocol name
        strncpy(left0, protocolName_.c_str(), kNumCols_);
        left0[kNumCols_] = '\0';
    } else {
        // show total minutes runtime
        if (runMins_ < 60) {
            snprintf(left0, sizeof(left0), "%2lu mins", runMins_);
        } else {
            snprintf(left0, sizeof(left0), "%4.1f hrs", float(runMins_) / 60.0f);
        }
    }
    lcdLineLR(0, left0, "RUNNING"); //stateToString(state_));

    // line 2: current step & loop count
    uint8_t step = 1; //currentStep_;
    uint8_t loop = 3; //totalLoops_ - loopCount_;
    snprintf(buf_1, sizeof(buf_1), "STEP:%2u  Loop:%u/%u", step, loop, totalLoops_);
    lcdLineLeft(1, buf_1);
    
    /// dummy data ///
    float rpm = 2560;
    int16_t v = 2123;
    uint16_t a = 500;
    targetMet_ = true;

    /* float rpm = static_cast<float>(targetSpeed_) * 60.0f / kStepsPerRev;
    int16_t v = (rpm >= 0.0f) ? static_cast<int16_t>(rpm + 0.5f) : static_cast<int16_t>(rpm - 0.5f);
    uint16_t a = static_cast<uint16_t>((static_cast<float>(targetAccel_) * 60.0f / kStepsPerRev) + 0.5f); */

    if (modeTorqueToggle_) {
        lcdLineLR(2, "RPM/s    RPM", "Dwell");
        char dwell_buf[6];
        
        // deal with this LATER!
        /* if (!targetMet_) {
            snprintf(dwell_buf, sizeof(dwell_buf), "ramp");
        } else if (state_ == State::Paused) {
            uint16_t t = (steps_[currentStep_].dwellMs - pause_time_) / 1000;
            snprintf(dwell_buf, sizeof(dwell_buf), "%4us", t);
        } else {
            uint16_t t = (steps_[currentStep_].dwellMs - dwellTmr_) / 1000;
            snprintf(dwell_buf, sizeof(dwell_buf), "%4us", t);
        } */
        snprintf(buf_1, sizeof(buf_1), "%5u  %5d", a, v);
        
        lcdLineLR(3, buf_1, dwell_buf);
    }

    else { // display typical RTM stats
        // line 3: toggle torque vs setpoint
        if (lcdToggle_) {
            float torqueA = 0.00f, torqueB = 0.00f;
            snprintf(buf_1, sizeof(buf_1), "Torque:%6.2f/%6.2f", torqueA, torqueB);
        } else {
            int setpoint = 300;     // PLACEHOLDER  
            float pressure = 14.1f; // PLACEHOLDER
            if (pressure < 100) {
                snprintf(buf_1, sizeof(buf_1), "Heat:%3u\xDF""C P:%3.1fpsi", setpoint, pressure);
            } else {
            snprintf(buf_1, sizeof(buf_1), "Heat:%3u\xDF""C Pr:%3.0fpsi", setpoint, pressure);
            }
        }
        lcdLineLeft(2, buf_1);
        // line 4: temps, drop ° if three-digit
        int latestSumpC_ = 140.4; // PLACEHOLDER

        uint16_t sealInt = isnan(latestSealC_)
                           ? 0
                           : uint16_t(latestSealC_ + 0.5);
        uint16_t sumpInt = isnan(latestSumpC_)
                           ? 0
                           : uint16_t(latestSumpC_ + 0.5);
        
        if (sumpInt < 100) {
            snprintf(buf_1, sizeof(buf_1), "Seal:%3u\xDF""C Sump:%2u\xDF""C", sealInt, sumpInt);
        } else {
            snprintf(buf_1, sizeof(buf_1), "Seal:%3u\xDF""C Sump:%3uC", sealInt, sumpInt);
        }
        lcdLineLeft(3, buf_1);
    }
    
    // commit
    lcdFlush();

    /* snprintf(debugBuf_, sizeof(debugBuf_),
        "State: %9s | Loop: %3u | Step: %3u | Speed: %5d RPM | Accel: %4u RPM/s² | Total Runtime: %5u | dwellTmr: %lu",// | VelRef: %li",
        stateToString(state_),
        (unsigned)loopCount_,
        (unsigned)currentStep_,
        v,
        (unsigned)a,
        (unsigned)runMins_,
        (unsigned long int)(uint32_t)dwellTmr_);
        //(signed long)(motor.VelocityRefCommanded()));
    SerialPort.SendLine(debugBuf_); */
}