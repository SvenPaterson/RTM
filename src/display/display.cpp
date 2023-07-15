#include <Arduino.h>
#include <string.h>
#include <Wire.h>
#include <SerLCD.h>
#include <SparkFun_MicroPressure.h>
#include <SparkFun_MCP9600.h>
#include <PID_v1.h>
#include <TimeLib.h>

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
#define REFRESH_INTERVAL 2000 // interval, in millis, between screen refreshes
SerLCD lcd;
bool isScreenUpdate = true;
bool isBacklightOn = true;
String status_msg;
String msg;
String test_status;

// PRESSURE SENSORS
#define PRESS_AVG_TIME 10000 // duration over which sensor will be averaged in millis
SparkFun_MicroPressure mpr; // use default values with reset and EOC pins unused
double press_cal = 0.0;
int sample_count = 0;

// TEMPERATURE SENSORS
MCP9600 sealTempSensor;
MCP9600 sumpTempSensor;

// PID LOOP FOR HEATER CONTROL
double setpoint, input, output;
double Kp = 60, Ki = 40, Kd = 25;
PID heaterPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// TEST CONTROL
int loop_count = 1234;

// FUNCTION DECLARATIONS
time_t getTeensyTime();
void updateLCD(double& press_cal, int& loop_count, String& test_status, double& setpoint);
String tempToStr(const double& temp);
String dateTimeStr();
void printLCDRow(int row, const String& text1, const String& text2);
void printRowPair(const int& col, const int& row, const int& width,
                  const String& str1, const String& str2);
void printFourColumnRow(const int& row,
                        const String& str1, const String& str2,
                        const String& str3, const String& str4);
String padBetweenChars(const int& num_chars, const String& str1, const String& str2);
String rightJustifiedString(const String& str);            
void showError(const String& errorMessage);
void initScreen(const String& init_msg, const bool& cls = true, const String& status_msg = "");


/* ----------------------------- INTIAL SETUP ----------------------------- */
void setup() {

  // Initialize Serial bus
  Serial.begin(115200);
  while (!Serial && (millis() < 4000)) {
    // wait for Serial bus to connect
  }
  Serial.println("");
  Serial.println("Initializing display controller...");

  // Initialize the I2C bus
  Wire.begin();
  Serial.println(" - i2c bus initialized");

  setSyncProvider(getTeensyTime);

  // Initialize the LCD screen
  lcd.begin(Wire);
  lcd.setBacklight(WHITE_RGB);
  lcd.setContrast(5);
  lcd.clear();
  Serial.println(" - lcd screen initialized");

  // Initialize the pressure sensor and set zero offset
  if (!mpr.begin()) {
    showError("Cannot connect to MicroPressure Sensor!");
  }
  delay(5);
  avgPressTimer = 0;
  sampleTimer = 0;
  String zero_offset_msg = "Determining zero offset for pressure sensor:";
  initScreen(zero_offset_msg);
  while (avgPressTimer <= PRESS_AVG_TIME) {
    if (sampleTimer > 1000) {
      press_cal += mpr.readPressure();
      sample_count++;  
      sampleTimer = 0;
      msg = "please wait... " + String(sample_count) + "s";
      initScreen(zero_offset_msg, false, msg);
    }
  }
  press_cal /= sample_count;
  press_cal = 14.7;   // DEBUGGING ONLY, DELETE ON ISSUE
  Serial.println(" - pressure sensor initialized");
  msg = "offset: " + String(press_cal) + " psi"; 
  initScreen(zero_offset_msg, false, msg);
  delay(3000);

  // Initialize the thermocouple sensor
  sealTempSensor.begin(0x60);
  sealTempSensor.setAmbientResolution(RES_ZERO_POINT_25);
  sealTempSensor.setThermocoupleResolution(RES_14_BIT);
  sumpTempSensor.begin(0x67);
  sumpTempSensor.setAmbientResolution(RES_ZERO_POINT_25);
  sumpTempSensor.setThermocoupleResolution(RES_14_BIT);

  // Initialize the heater band PID loop
  heaterPID.SetMode(AUTOMATIC);
  heaterPID.SetOutputLimits(0, 150);
  input = sumpTempSensor.getThermocoupleTemp(false);
  setpoint = 160.0;
  String PID_init_msg = "PID Loop initialized";
  initScreen(PID_init_msg);
  printFourColumnRow(3, "Sump:", tempToStr(input), " Sp:", tempToStr(setpoint));
  delay(3000);
  Serial.println(" - PID loop initialized");

  // Clear screen to begin test protocol
  lcd.clear();

  // DEBUG VARS
  test_status = "TESTING";
}

/* -------------------------------- MAIN LOOP -------------------------------- */
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

time_t getTeensyTime() {
  return Teensy3Clock.get();
}

void updateLCD(double& press_cal, int& loop_count, String& test_status, double& setpoint) {
  // I think this function could be passed a struct instead of each and 
  // every argument
  
  // Read all sensors and construct strings
  String loops = String(loop_count);
  String seal_temp = tempToStr(sealTempSensor.getThermocoupleTemp(false));
  delay(5);
  String sump_temp = tempToStr(sumpTempSensor.getThermocoupleTemp(false));
  delay(5);
  String pressure = String((mpr.readPressure() - press_cal));
  pressure = "12.3";         // DEBUGGING ONLY, DELETE ON RELEASE !!!!
  double CW_torque = 1.26;   // DEBUGGING ONLY, DELETE ON RELEASE !!!!
  double CCW_torque = -0.98; // DEBUGGING ONLY, DELETE ON RELEASE !!!!
  String torques = String(CW_torque) + " / " + String(CCW_torque);
  String set_point = tempToStr(setpoint);
  String date = String(month()) + "/" + String(day());
  String time = String(hour()) + ":" + String(minute());
  

  if (LCDTimer > REFRESH_INTERVAL) {
    LCDTimer = 0;
    if (isScreenUpdate) {
      isScreenUpdate = !isScreenUpdate;
      printRowPair(0, 0, MAX_CHARS_PER_LINE, "Status:", test_status);
      printRowPair(0, 1, MAX_CHARS_PER_LINE, "Setpoint:", set_point);
      printFourColumnRow(2, "P:", pressure +"psi", " Loop:", loops);
      printFourColumnRow(3, "Seal:", seal_temp, " Sump:", sump_temp);
    }
    else {
      isScreenUpdate = !isScreenUpdate;
      printRowPair(0, 0, MAX_CHARS_PER_LINE, "Date:", dateTimeStr());
      printRowPair(0, 1, MAX_CHARS_PER_LINE, "Torque:", torques);
    }
  }
}

String tempToStr(const double& temp) {
  return String(temp, 0) + String((char)223) + "F";
}

String dateTimeStr() {
  String date = String(month()) + "/" + String(day()) + "/" + String(year());
  String time = String(hour()) + ":" + String(minute());
  return date + " " + time;
}
String rightJustifiedString(const String& str) {
  String paddedStr = str;
  while (paddedStr.length() < MAX_CHARS_PER_LINE) {
    paddedStr = " " + paddedStr;
  }
  Serial.println(paddedStr);
  return paddedStr;
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

void initScreen(const String& init_msg, const bool& cls, const String& status_msg) {
  
  if (cls) {
    lcd.clear();
  }

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
      // adjust end pos to max limit if no space found
      if (endPos == startPos) {
        endPos = startPos + MAX_CHARS_PER_LINE - 1;
      }
    }

    lcd.setCursor(0, numLines);
    lcd.print(init_msg.substring(startPos, endPos + 1));
    numLines++;
    startPos = endPos + 1;
  }

  lcd.setCursor(0, 3);

  if (status_msg != "") {
    lcd.print(rightJustifiedString(status_msg));
  }
}

