#include <Arduino.h>
#include <string.h>
#include <Wire.h>
#include <SerLCD.h>
#include <SparkFun_MicroPressure.h>
#include <SparkFun_MCP9600.h>

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
elapsedMillis avgPressTimer;
elapsedMillis sampleTimer;
elapsedMillis debugTimer; // DELETE WHEN COMPLETE

// LCD SCREEN
#define MAX_CHARS_PER_LINE 20
#define WHITE_RGB 255, 255, 255
#define RED_RGB 255, 255, 0
SerLCD lcd;
bool isScreenUpdate = true;
bool isBacklightOn = true;
String status_msg;
String test_status;

// PRESSURE SENSORS
#define PRESS_AVG_TIME 1500
SparkFun_MicroPressure mpr; // use default values with reset and EOC pins unused
double press_cal = 0.0;
int sample_count = 0;

// TEMPERATURE SENSORS
MCP9600 sealTempSensor;
MCP9600 sumpTempSensor;

// PID LOOP
double setpoint = 160.0;

// TEST CONTROL
int loop_count = 1234;

// FUNCTION DECLARATIONS
void updateLCD(double& press_cal, int& loop_count, String& test_status, double& setpoint);
void printLCDRow(int row, const String& text1, const String& text2);
void printRowPair(const int& col, const int& row, const int& width,
                  const String& str1, const String& str2);
void printFourColumnRow(const int& row,
                        const String& str1, const String& str2,
                        const String& str3, const String& str4);
String padBetweenChars(const int& num_chars, const String& str1, const String& str2);              
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
  initScreen("Connecting to MicroPressure Sensor...");
  delay(2000);
  if (!mpr.begin()) {
    showError("Cannot connect to MicroPressure Sensor!");
  }
  delay(5);
  avgPressTimer = 0;
  sampleTimer = 0;
  String zero_offset_msg = "Averaging ambient pressure in lab:";
  while (avgPressTimer <= PRESS_AVG_TIME) {
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
  sealTempSensor.begin(0x60);
  sealTempSensor.setAmbientResolution(RES_ZERO_POINT_25);
  sealTempSensor.setThermocoupleResolution(RES_14_BIT);
  sumpTempSensor.begin(0x67);
  sumpTempSensor.setAmbientResolution(RES_ZERO_POINT_25);
  sumpTempSensor.setThermocoupleResolution(RES_14_BIT);

  // Clear screen to begin test protocol
  lcd.clear();

  // DEBUG VARS
  test_status = "TESTING";
}

// MAIN LOOP
void loop() {
  // DEBUGGING LOOP
  // add simulated actions here
  if (debugTimer > 4000) {
    debugTimer = 0;
    loop_count++;
  }

  updateLCD(press_cal, loop_count, test_status, setpoint);
}

// FUNCTION DEFINITIONS //

void updateLCD(double& press_cal, int& loop_count, String& test_status, double& setpoint) {
  // I think this function could be passed a struct instead of each and 
  // every argument

  // Read all sensors
  String degF = String((char)223) + "F";
  String loops = String(loop_count);
  String seal_temp = String(sealTempSensor.getThermocoupleTemp(false), 0) + degF;
  delay(5);
  String sump_temp = String(sumpTempSensor.getThermocoupleTemp(false), 0) + degF;
  delay(5);
  String pressure = String((mpr.readPressure() - press_cal));
  
  pressure = "12.3";                                // DEBUGGING ONLY, DELETE ON RELEASE !!!!
  double CW_torque = 1.26;
  double CCW_torque = -0.98;
  String torques = String(CW_torque) + " / " + String(CCW_torque);
  
  String set_point = String(setpoint, 0) + degF;

  if (LCDTimer > 2000) {
    LCDTimer = 0;
    printRowPair(0, 0, MAX_CHARS_PER_LINE, "Status:", test_status);
  

    if (isScreenUpdate) {
      isScreenUpdate = !isScreenUpdate;
      printRowPair(0, 1, MAX_CHARS_PER_LINE, "Setpoint:", set_point);
      printFourColumnRow(2, "P:", pressure +"psi", " Loop:", loops);
      printFourColumnRow(3, "Seal:", seal_temp, " Sump:", sump_temp);
    }
    else {
      isScreenUpdate = !isScreenUpdate;
      printRowPair(0, 1, MAX_CHARS_PER_LINE, "Torque:", torques);
    }
  }
}

void printRowPair(const int& col, const int& row, const int& width,
                  const String& str1, const String& str2) {
  /* Takes two strings and prints 'str1' left justified 
   * and 'str2' right justified on the specified 'row' 
   * between a distance, specified by width and a start
   * location specified by 'col' on the lcd screen.
   */
  lcd.setCursor(col, row);
  lcd.print(str1);
  lcd.setCursor(str1.length()+col, row);
  lcd.print(padBetweenChars(width, str1, str2));
}

void printFourColumnRow(const int& row,
                        const String& str1, const String& str2,
                        const String& str3, const String& str4) {
  /* Prints four strings on a row of the LCD screen,
   * equally spaced apart.
   */
  int width = MAX_CHARS_PER_LINE / 2;
  printRowPair(0, row, width, str1, str2);
  printRowPair(width, row, width, str3, str4);
}

String padBetweenChars(const int& num_chars, const String& str1, const String& str2) {
  /* Function calculates the correct amount of padding 
   * and pre-pends str2 with " " characters and returns
   * paddedStr.
   */
  unsigned int space = num_chars - (str1).length();
  String paddedStr = str2;
  while (paddedStr.length() < space) {
    paddedStr = " " + paddedStr;
  }
  return paddedStr;
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