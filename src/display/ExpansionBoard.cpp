#include "ExpansionBoard.h"
#include <Arduino.h>
#include <SPI.h>

void ExpansionBoard::setDataInterval(uint16_t milli_secs) {
    kDataIntervalMs_ = milli_secs;
}

bool ExpansionBoard::begin() {
    while (!Serial) delay(1);
    Serial.println("\nUSB Serial Connected!");

    if (!lcd_.begin()) {
        Serial.println("FATAL: LCD initialization failed!");
        return false;    
    }

    /* Serial.print("Initializing LCD Screen...");
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
    Serial.println("DONE"); */

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

    return true;
}

void ExpansionBoard::tick() {
    updateData();
    
    if (lcdTmr_ >= lcdToggle_ms_) {
        lcdTmr_ = 0;
        lcdToggle_ = !lcdToggle_;
    } renderScreen();
    
}



/*********************** HELPERS ***********************/

/* ——— Sensor helpers ——— */
double ExpansionBoard::readTC(Adafruit_MAX31855 &TC, const char *label) {
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

void ExpansionBoard::updateData() {
    if (dataTmr_ < kDataIntervalMs_) return;
    dataTmr_ = 0;

    latestSealC_ = readTC(tc1_, "TC1");
    latestSumpC_ = 120; //readTC(tc2_, "TC2"); // PLACEHOLDER
}

void ExpansionBoard::renderScreen() {
    // line 1: toggle between protocol name and runtime
    char buff[LCDDriver::kNumCols+1];
    if (lcdToggle_) {
        // show protocol name
        strncpy(buff, protocolName_.c_str(), LCDDriver::kNumCols);
        buff[LCDDriver::kNumCols] = '\0';
    } else {
        // show total minutes runtime
        if (runMins_ < 60) {
            snprintf(buff, sizeof(buff), "%2lu mins", runMins_);
        } else {
            snprintf(buff, sizeof(buff), "%4.1f hrs", float(runMins_) / 60.0f);
        }
    }
    lcd_.setLineLR(0, buff, "RUNNING"); // PLACEHOLDER - stateToString(state_));

    // line 2: current step & loop count
    uint8_t step = 1; //currentStep_;
    uint8_t loop = 3; //totalLoops_ - loopCount_;
    snprintf(buff, sizeof(buff), "STEP:%2u  Loop:%u/%u", step, loop, totalLoops_);
    lcd_.setLineLeft(1, buff);
    
    /// dummy data ///
    float rpm = 2560;
    int16_t v = 2123;
    uint16_t a = 500;
    targetMet_ = true;

    /* float rpm = static_cast<float>(targetSpeed_) * 60.0f / kStepsPerRev;
    int16_t v = (rpm >= 0.0f) ? static_cast<int16_t>(rpm + 0.5f) : static_cast<int16_t>(rpm - 0.5f);
    uint16_t a = static_cast<uint16_t>((static_cast<float>(targetAccel_) * 60.0f / kStepsPerRev) + 0.5f); */

    if (modeTorqueToggle_) {
        lcd_.setLineLR(2, "RPM/s    RPM", "Dwell");
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
        snprintf(buff, sizeof(buff), "%5u  %5d", a, v);
        
        lcd_.setLineLR(3, buff, dwell_buf);
    }

    else { // display typical RTM stats
        // line 3: toggle torque vs setpoint
        if (lcdToggle_) {
            float torqueA = 0.00f, torqueB = 0.00f;
            snprintf(buff, sizeof(buff), "Torque:%6.2f/%6.2f", torqueA, torqueB);
        } else {
            int setpoint = 300;     // PLACEHOLDER  
            float pressure = 14.1f; // PLACEHOLDER
            if (pressure < 100) {
                snprintf(buff, sizeof(buff), "Heat:%3u\xDF""C P:%3.1fpsi", setpoint, pressure);
            } else {
            snprintf(buff, sizeof(buff), "Heat:%3u\xDF""C Pr:%3.0fpsi", setpoint, pressure);
            }
        }
        lcd_.setLineLeft(2, buff);
        // line 4: temps, drop ° if three-digit
        int latestSumpC_ = 140.4; // PLACEHOLDER

        uint16_t sealInt = isnan(latestSealC_)
                           ? 0
                           : uint16_t(latestSealC_ + 0.5);
        uint16_t sumpInt = isnan(latestSumpC_)
                           ? 0
                           : uint16_t(latestSumpC_ + 0.5);
        
        if (sumpInt < 100) {
            snprintf(buff, sizeof(buff), "Seal:%3u\xDF""C Sump:%2u\xDF""C", sealInt, sumpInt);
        } else {
            snprintf(buff, sizeof(buff), "Seal:%3u\xDF""C Sump:%3uC", sealInt, sumpInt);
        }
        lcd_.setLineLeft(3, buff);
    }
    
    // commit
    lcd_.flush();

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