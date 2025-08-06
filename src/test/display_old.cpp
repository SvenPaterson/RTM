#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <elapsedMillis.h>

/* ——— pinouts ——— */
#define LCD_CS   8
#define TC1_CS   9
#define TC2_CS  10

Adafruit_MAX31855 TC1(TC1_CS);
Adafruit_MAX31855 TC2(TC2_CS);

#define readTC(x) readTCImpl(x, #x)
double readTCImpl(Adafruit_MAX31855 &TC, const char *n);

/* ——— protocol steps ——— */
struct Step {
    int32_t  speedSteps_s   {0};   //!< target speed in steps/s
    uint32_t accelSteps_s2  {0};   //!< accel in steps/s²
    uint32_t dwellMs        {0};   //!< dwell after speed reached (ms)
};

/* ——— protocol state ——— */
static constexpr uint8_t  kMaxProtocolSteps = 50;
Step steps_[kMaxProtocolSteps] = {};

uint8_t  stepCount_   {0};
uint8_t  loopCount_   {1};
uint8_t  totalLoops_  {1};
String   protocolName_ {"Test Code"};
bool targetMet_ {false};

/* ——— LCD Display Settings ——— */
static constexpr uint8_t  kNumCols = 20;
static constexpr uint8_t  kNumRows = 4;
const uint8_t kRowAddr[kNumRows] = {0x00, 0x40, 0x14, 0x54};

/* ——— LCD front/shadow buffers ——— */
char buf_[kNumCols + 1]             = {};
char front_[kNumRows][kNumCols + 1] = {};
char sent_ [kNumRows][kNumCols + 1] = {};
bool dirty_[kNumRows]               = {true, true, true, true};

const SPISettings LCDspiCfg_{ 100000, MSBFIRST, SPI_MODE3 };
// const SPISettings TCspiCfg_{1000000, MSBFIRST, SPI_MODE0};

/* ——— LCD helpers ——— */
static inline uint8_t fastLen_(const char *s) { uint8_t n = 0; while (n < kNumCols && s[n]) ++n; return n; }
void lcdBlank_     (char *dst);
void lcdLineBlank  (uint8_t row);                                   // blank a line in the front buffer
void lcdLineLeft   (uint8_t row, const char *txt);                  // fill a line w/ a left justified string
void lcdLineRight  (uint8_t row, const char *txt);                  // fill a line w/ a right justified string
void lcdLineCenter (uint8_t row, const char *txt);                  // fill a line w/ a center justified string
void lcdLineLR     (uint8_t row, const char *l, const char *r);     // fill a line w/ two strings, right and left justified
void lcdFlush      ();                                              // print all lines to screen
void lcdClearScreen();                                              // does what it says on the tin...
void renderScreen  ();                                              // call this to update screen with test details

/* ——— LCD behaviour toggles ——— */
bool lcdToggle_{false}, lcdRuntimeToggle_{false}, modeTorqueToggle_{false}; // torque mode is for torque stand only
elapsedMillis lcdTmr_;
uint16_t lcdToggle_ms_{3000}; // default to every 3s
uint32_t runMins_{42};

void setup() {
    Serial.begin(9600);
    while (!Serial) {
        // wait for serial bus to start
        delay(1);
    }
    Serial.println("\nUSB Serial Connected!");

    Serial.print("Initializing LCD Screen...");
    pinMode(LCD_CS, OUTPUT);
    digitalWrite(LCD_CS, HIGH);
    delay(200);
    SPI.begin();
    delay(200);

    lcdClearScreen();

    // Turn display on
    digitalWrite(LCD_CS, LOW);
    SPI.beginTransaction(LCDspiCfg_);
        SPI.transfer(0xFE); 
        SPI.transfer(0x41);
    SPI.endTransaction();
    digitalWrite(LCD_CS, HIGH);

    // Set backlight brightness
    digitalWrite(LCD_CS, LOW);
    SPI.beginTransaction(LCDspiCfg_);
        SPI.transfer(0xFE); 
        SPI.transfer(0x53); 
        SPI.transfer(4);
    SPI.endTransaction();
    digitalWrite(LCD_CS, HIGH);

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


    if (!TC1.begin()) {
        Serial.println("ERROR.");
        while (1) delay(10);
    } else Serial.println("DONE");

    TC1.setFaultChecks(MAX31855_FAULT_ALL);  // short to GND fault is ignored
 
}

void loop() {
    // 1) Update line 1 with a flipping demo caption
    lcdLineLeft(0, lcdToggle_ ? "SPI + TC Demo A" : "SPI + TC Demo B");

    // 2) Read and format TC1
    double t = readTC(TC1);
    char tbuf[21];
    if (isnan(t)) {
        strcpy(tbuf, "TC1: FAULT      ");
    } else {
        // convert the float into a string yourself
        char tmp[10];
        dtostrf(t, 6, 2, tmp);             // width=6, precision=2 → e.g. " 23.45"
        snprintf(tbuf, sizeof(tbuf),
                "TC1: %s C   ", tmp);     // now inject that into your buffer
    }
    lcdLineLeft(1, tbuf);

    // 3) (optional) show a static footer
    lcdLineLR(2, "Line 3 static", "");
    lcdLineLR(3, "Line 4 static", "");

    // 4) Flush to LCD
    lcdFlush();

    // 5) Toggle for next pass
    lcdToggle_ = !lcdToggle_;
    Serial.println(tbuf);
    delay(lcdToggle_ms_);
}

double readTCImpl(Adafruit_MAX31855 &TC, const char *n) {
  double c = TC.readCelsius();
  if (isnan(c)) {
    uint8_t e = TC.readError();
    Serial.print(n); Serial.println(" fault(s):"); // here is where we would print the fault to a log file on SD card i.e. if !Serial then SD
    if (e & MAX31855_FAULT_OPEN)      Serial.println("  • open circuit");
    if (e & MAX31855_FAULT_SHORT_GND) Serial.println("  • short to GND");
    if (e & MAX31855_FAULT_SHORT_VCC) Serial.println("  • short to VCC");
    return NAN;   // <— make sure you return something!
  } 
  // no fault → return the real temperature
  return c;
}

/* ——— LCD helpers ——— */
void lcdBlank_(char *dst) {
    memset(dst, ' ', kNumCols);
    dst[kNumCols] = '\0';
}

void lcdLineLeft(uint8_t row, const char *txt) {
    lcdBlank_(front_[row]);
    memcpy(front_[row], txt, fastLen_(txt));
    if (memcmp(front_[row], sent_[row], kNumCols)) dirty_[row] = true;
}

void lcdLineRight(uint8_t row, const char *txt) {
    lcdBlank_(front_[row]);
    uint8_t len = fastLen_(txt);
    memcpy(front_[row] + kNumCols - len, txt, len);
    if (memcmp(front_[row], sent_[row], kNumCols)) dirty_[row] = true;
}

void lcdLineCenter(uint8_t row, const char *txt) {
    lcdBlank_(front_[row]);
    uint8_t len = fastLen_(txt);
    uint8_t start = (kNumCols - len) / 2;
    memcpy(front_[row] + start, txt, len);
    if (memcmp(front_[row], sent_[row], kNumCols)) dirty_[row] = true;
}

void lcdLineLR(uint8_t row, const char *left, const char *right) {
    lcdBlank_(front_[row]);
    uint8_t rLen = fastLen_(right);
    uint8_t lMax = (rLen < kNumCols) ? kNumCols - rLen - 1 : 0;
    uint8_t lLen = (lMax ? (fastLen_(left) > lMax ? lMax : fastLen_(left)) : 0);
    memcpy(front_[row],               left,  lLen);
    memcpy(front_[row] + kNumCols - rLen, right, rLen);
    if (memcmp(front_[row], sent_[row], kNumCols)) dirty_[row] = true;
}

void lcdFlush() {
  for (uint8_t row = 0; row < kNumRows; ++row) {
    if (!dirty_[row]) continue;
    digitalWrite(LCD_CS, LOW);
    SPI.beginTransaction(LCDspiCfg_);
    SPI.transfer(0xFE);
    SPI.transfer(0x45);
    SPI.transfer(kRowAddr[row]);     // set cursor
    // SPI.transfer(front_[row], kNumCols); // write characters
    for (uint8_t i = 0; i < kNumCols; i++) {
        SPI.transfer(front_[row][i]);
        delayMicroseconds(100);  // give the PIC time to clock it through
    }
    SPI.endTransaction();
    digitalWrite(LCD_CS, HIGH);
    memcpy(sent_[row], front_[row], kNumCols);
    dirty_[row] = false;
    delayMicroseconds(100 * kNumCols);  // optional pacing
  }
}

void lcdLineBlank(uint8_t row) {
    lcdBlank_(front_[row]);
    if (memcmp(front_[row], sent_[row], kNumCols)) dirty_[row] = true;
}

void lcdClearScreen() {
    for (uint8_t i = 0; i < kNumRows; ++i) {
        lcdLineBlank(i); // clear front buffer
    }
    digitalWrite(LCD_CS, LOW);
    SPI.beginTransaction(LCDspiCfg_);
    SPI.transfer(0xFE);
    SPI.transfer(0x51);    // clear screen
    SPI.endTransaction();
    digitalWrite(LCD_CS, HIGH);
    delay(5);                // ≥1.5 ms per datasheet
}

void renderScreen() {
    char buf_1[kNumCols+1];
    /* char buf_2[kNumCols+1];
    snprintf(buf_2, sizeof(buf_2), "%s", protocolName_.c_str()) */

    // line 1: toggle between protocol name and runtime
    char left0[kNumCols+1];
    if (lcdToggle_) {
        // show protocol name
        strncpy(left0, protocolName_.c_str(), kNumCols);
        left0[kNumCols] = '\0';
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
                snprintf(buf_1, sizeof(buf_1), "Heat:%3u\xDF""F P:%3.1fpsi", setpoint, pressure);
            } else {
            snprintf(buf_1, sizeof(buf_1), "Heat:%3u\xDF""F Pr:%3.0fpsi", setpoint, pressure);
            }
        }
        lcdLineLeft(2, buf_1);
        // line 4: temps, drop ° if three-digit
        int seal = 120, sump = 140; // PLACEHOLDERS
        if (sump < 100) {
            snprintf(buf_1, sizeof(buf_1), "Seal:%3u\xDF""F Sump:%2u\xDF""F", seal, sump);
        } else {
            snprintf(buf_1, sizeof(buf_1), "Seal:%3u\xDF""F Sump:%3uF", seal, sump);
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