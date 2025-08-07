#include "DisplayController.h"
#include <Arduino.h>
#include <SPI.h>

void DisplayController::setDataInterval(uint16_t milli_secs) {
    kDataIntervalMs_ = milli_secs;
}

bool DisplayController::begin() {
    Serial.begin(9600);
    while (!Serial) {
        // wait for serial bus to start
        delay(1);
    }
    Serial.println("\nUSB Serial Connected!");

    Serial.print("Initializing LCD Screen...");
    pinMode(LCD_CS_, OUTPUT);
    digitalWrite(LCD_CS_, HIGH);
    delay(200);
    SPI.begin();
    delay(200);

    lcdClearScreen();

    // Turn display on
    digitalWrite(LCD_CS_, LOW);
    SPI.beginTransaction(LCDspiCfg_);
        SPI.transfer(0xFE); 
        SPI.transfer(0x41);
    SPI.endTransaction();
    digitalWrite(LCD_CS_, HIGH);

    // Set backlight brightness
    digitalWrite(LCD_CS_, LOW);
    SPI.beginTransaction(LCDspiCfg_);
        SPI.transfer(0xFE); 
        SPI.transfer(0x53); 
        SPI.transfer(4);
    SPI.endTransaction();
    digitalWrite(LCD_CS_, HIGH);

    // Initial screen
    lcdClearScreen();
    lcdLineLR(0, "Nano Every Demo", "");
    lcdLineLR(1, "Initializing...", "");
    lcdFlush();
    delay(1000);
    Serial.println("DONE");

    Serial.print("Initializing MAX31855 sensor...");
    // wait for MAX chip to stabilize
    delay(500);


    if (!tc1_.begin()) {
        Serial.println("ERROR.");
        while (1) delay(10);
    } else Serial.println("DONE");

    /* if (!tc2_.begin()) {
        Serial.println("ERROR.");
        while (1) delay(10);
    } else Serial.println("DONE"); */

    tc1_.setFaultChecks(MAX31855_FAULT_ALL);  // short to GND fault is ignored
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
    latestSumpC_ = readTC(tc2_, "TC2");
}

/* ——— LCD helpers ——— */
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

void DisplayController::lcdFlush() {
  for (uint8_t row = 0; row < kNumRows_; ++row) {
    if (!dirty_[row]) continue;
    digitalWrite(LCD_CS_, LOW);
    SPI.beginTransaction(LCDspiCfg_);
    SPI.transfer(0xFE);
    SPI.transfer(0x45);
    SPI.transfer(kRowAddr_[row]);     // set cursor
    // SPI.transfer(front_[row], DisplayController::kNumCols_); // write characters
    for (uint8_t i = 0; i < kNumCols_; i++) {
        SPI.transfer(front_[row][i]);
        delayMicroseconds(100);  // give the PIC time to clock it through
    }
    SPI.endTransaction();
    digitalWrite(LCD_CS_, HIGH);
    memcpy(sent_[row], front_[row], kNumCols_);
    dirty_[row] = false;
    delayMicroseconds(100 * kNumCols_);  // optional pacing
  }
}

void DisplayController::lcdLineBlank(uint8_t row) {
    lcdBlank(front_[row]);
    if (memcmp(front_[row], sent_[row], kNumCols_)) dirty_[row] = true;
}

void DisplayController::lcdClearScreen() {
    for (uint8_t i = 0; i < kNumRows_; ++i) {
        lcdLineBlank(i); // clear front buffer
    }
    digitalWrite(LCD_CS_, LOW);
    SPI.beginTransaction(LCDspiCfg_);
    SPI.transfer(0xFE);
    SPI.transfer(0x51);    // clear screen
    SPI.endTransaction();
    digitalWrite(LCD_CS_, HIGH);
    delay(5);                // ≥1.5 ms per datasheet
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