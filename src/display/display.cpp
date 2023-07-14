#include <Arduino.h>
#include <Wire.h>
#include <SerLCD.h>
#include <SparkFun_MicroPressure.h>

// define the board I/O pin numbers
#define SD_DETECT_PIN 0
#define LOOP_BUS_PIN 1
#define MOTOR_HLFB_PIN 2
#define HEAT_OUTPUT_PIN 6
#define HEAT_SAFETY_PIN 7
#define AIR_SUPPLY_PIN 8
#define AIR_DUMP_PIN 9
#define PRGM_RUN_BUS_PIN 10
#define TORQ_FLAG_BUS_PIN 11
#define AIR_SUPPLY_BUS_PIN 14
#define AIR_DUMP_BUS_PIN 15
#define RUN_SW_PIN 16
#define RESET_SW_PIN 17
#define SDA0_PIN 18
#define SDL0_PIN 19
#define RESET_BUS_PIN 20

// INITIALIZE TIMERS
elapsedMillis SysTimer;
elapsedMillis LCDTimer;
elapsedMillis errorTimer;
elapsedMillis avgTimer; // for initial pressure sensor calibration
elapsedMillis sampleTimer;

// LCD SCREEN
#define MAX_CHARS_PER_LINE 20
#define WHITE_RGB 255, 255, 255
#define RED_RGB 255, 255, 0
SerLCD lcd;
bool isScreenUpdate = true;
bool isBacklightOn = true;
String status_msg;

// PRESSURE SENSORS
SparkFun_MicroPressure mpr; // use default values with reset and EOC pins unused
double press_cal = 0.0;
int sample_count = 0;

// FUNCTION DECLARATIONS
void updateLCD(double press_cal);
void showError(const String& errorMessage);
void initScreen(const String& init_msg, const String status_msg = "");

/* ----------------------------- INTIAL SETUP ----------------------------- */
void setup() {

  // Initialize Serial bus
  Serial.begin(9600);
  delay(1000);
  Serial.println("");
  Serial.println("Initializing display controller...");

  // Initialize the I2C bus
  Wire.begin();
  Serial.println(" - i2c bus initialized");

  // Initialize the LCD screen
  lcd.begin(Wire);
  lcd.setBacklight(WHITE_RGB);
  lcd.setContrast(5);
  lcd.clear();
  Serial.println(" - lcd screen initialized");

  // Initialize the pressure sensor and set zero offset
  initScreen("Connecting to MicroPressor Sensor");
  delay(2000);
  if (!mpr.begin()) {
    showError("Cannot connect to MicroPressure Sensor!");
  }
  delay(5);
  avgTimer = 0;
  sampleTimer = 0;
  String zero_offset_msg = "Averaging ambient pressure:";
  while (avgTimer <= 11000) {
    if (sampleTimer > 1000) {
      initScreen(zero_offset_msg);
      press_cal += mpr.readPressure();
      sample_count++;  
      sampleTimer = 0;
      lcd.setCursor(3, 3);
      lcd.print(sample_count);
      lcd.setCursor(String(sample_count).length()+4, 3);
      lcd.print("secs");
    }
  }
  press_cal /= sample_count;
  press_cal = 14.7;
  initScreen(zero_offset_msg);
  delay(1000);
  initScreen(zero_offset_msg, "done!");
  delay(2000);
  Serial.println(" - pressure sensor initialized");
  initScreen("Sensor offset by", String(press_cal)+" psi");
  delay(3000);

  // Initialize the thermocouple sensor
  
  lcd.clear();
}

// MAIN LOOP
void loop() {
  updateLCD(press_cal);
}

// FUNCTION DEFINITIONS //
void updateLCD(double press_cal) { // This script Updates the LCD withinformation.
// include these args: double& cw_torq, double& ccw_torq
  if (LCDTimer > 2000) {
    LCDTimer = 0;
    double psi = (mpr.readPressure() - press_cal);
    delay(5);
    double Sealtemp = 74;//SealtempSensor.getThermocoupleTemp(false);
    delay(5);
    double Sumptemp = 72;//SumptempSensor.getThermocoupleTemp(false);// false returns temp in farenheight.

    double CW_torque = 1.26;
    double CCW_torque = -0.98;
    int LoopCount = 123;
    String Status = "RUNNING";
    int Setpoint = 160;

  if (isScreenUpdate == true) {
      isScreenUpdate = !isScreenUpdate;
      lcd.setCursor(0, 0);
      lcd.print("Status");
      lcd.setCursor(11, 0);
      lcd.print("SetPoint");
      lcd.setCursor(0, 1);
      lcd.print(Status);
      lcd.setCursor(8, 1);
      lcd.print( "    " + String(Setpoint, 0) + String((char)223) + "F   ");
      lcd.setCursor(19, 1);
      lcd.print("  ");
      lcd.setCursor(0, 2);
      lcd.print("PSI: " + String(psi, 1) + " ");
      lcd.setCursor(11, 2);
      lcd.print("Loop:    " );
      lcd.setCursor(16, 2);
      lcd.print(String(LoopCount));
      lcd.setCursor(0, 3);
      lcd.print("Temp: " + String(Sealtemp, 0) + String((char)223) + "F  ");
      lcd.setCursor(11, 3);
      if (Sumptemp >= 100) {
        lcd.print("Sump: " + String(Sumptemp, 0) + String((char)223));
      } else     lcd.print("Sump: " + String(Sumptemp, 0) + String((char)223));
    }
    else {
      isScreenUpdate = !isScreenUpdate;
      lcd.setCursor(0, 0);
      lcd.print("Status");
      lcd.setCursor(11, 0);
      //lcd.print("SetPoint");
      lcd.print("Torque  ");
      lcd.setCursor(0, 1);
      lcd.print(Status);
      lcd.setCursor(8, 1);
      //lcd.print( String(Setpoint, 0) + String((char)223) + "F   ");
      lcd.print(String(CW_torque) + " / " + String(CCW_torque));
      lcd.setCursor(0, 2);
      lcd.print("PSI: " + String(psi, 1) + " ");
      lcd.setCursor(11, 2);
      lcd.print("Loop:    " );
      lcd.setCursor(16, 2);
      lcd.print(String(LoopCount));
      lcd.setCursor(0, 3);
      lcd.print("Temp: " + String(Sealtemp, 0) + String((char)223) + "F ");
      lcd.setCursor(11, 3);
      if (Sumptemp >= 100) {
        lcd.print("Sump: " + String(Sumptemp, 0) + String((char)223));
      } else     lcd.print("Sump: " + String(Sumptemp, 0) + String((char)223));

    }
  }
}

void showError(const String& errorMessage) {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Error! Error! Error!");
  lcd.setCursor(0,1);
  lcd.print(errorMessage);

  while (true) {
    if (errorTimer >= 1000) {
      if (isBacklightOn) {
        lcd.setBacklight(RED_RGB);
        isBacklightOn = false;
      }
      else {
        lcd.setBacklight(0, 0, 0);
        isBacklightOn = true;
      }

    errorTimer = 0;
    }
  }
}

void initScreen(const String& init_msg, const String status_msg) {
  lcd.clear();
  // Print the message with line wrapping
  int startPos = 0;
  int endPos = 0;
  int numLines = 0;
  int messageLength = init_msg.length();

  while (startPos < messageLength) {
    // Calculate the end position for the line
    endPos = startPos + MAX_CHARS_PER_LINE - 1;

    // Check if the end position goes beyond the message length
    if (endPos >= messageLength) {
      endPos = messageLength - 1;
    }
    else {
      // Find the last space character within the line's range
      while (endPos > startPos && init_msg[endPos] != ' ') {
        endPos--;
      }

      // If no space is found, adjust the end position to the maximum limit
      if (endPos == startPos) {
        endPos = startPos + MAX_CHARS_PER_LINE - 1;
      }
    }

    // Print the line
    lcd.setCursor(0, numLines);
    lcd.print(init_msg.substring(startPos, endPos + 1));

    // Increment the line counter
    numLines++;

    // Update the start position for the next line
    startPos = endPos + 1;
  }
  

  lcd.setCursor(3, 7);

  if (status_msg != "") {
    lcd.print(status_msg);
  }
}