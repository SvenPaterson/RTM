#include <Arduino.h>
#include <string.h>
#include <Wire.h>
#include <SerLCD.h>
#include <SparkFun_MicroPressure.h>
#include <SparkFun_MCP9600.h>
#include <PID_v1.h>
#include <TimeLib.h>
#include <Bounce2.h>

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
elapsedMillis LoopTimer;
elapsedMillis LCDTimer;
elapsedMillis errorTimer;
elapsedMillis debugTimer; // DELETE WHEN COMPLETE

// LCD SCREEN
#define MAX_CHARS_PER_LINE 20
#define WHITE_RGB 255, 255, 255
#define RED_RGB 255, 255, 0
#define REFRESH_INTERVAL 2000 // interval, in millis, between screen refreshes
SerLCD lcd;
bool isScreenUpdate = true;
bool isBacklightOn = true;
String test_status_str = "READY";
String msg;

// PRESSURE SENSORS
#define PRESS_AVG_TIME 3000 // duration over which sensor will be averaged in millis
SparkFun_MicroPressure mpr; // use default values with reset and EOC pins unused
double press_offset;

// TEMPERATURE SENSORS
MCP9600 sealTempSensor;
MCP9600 sumpTempSensor;

// PID LOOP FOR HEATER CONTROL
double setpoint, input, output;
double Kp = 60, Ki = 40, Kd = 25;
PID heaterPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
elapsedMillis PIDTimer;
double sump_temp, seal_temp;

// RUN & RESET SWITCH BOUNCERS
Bounce RunSwitch = Bounce();
Bounce ResetSwitch = Bounce();

// TEST CONTROL
#define RESET_TIME 10000
elapsedMillis resetTimer = 0;
bool temp_units = false; // 'false' for farenheit, 'true' for celcius
bool isRunSwitchOn = false;
bool isResetSwitchOn = false;
bool hasTestStarted = false;
bool isPreHeated = false;


// DEBUGGING ONLY
bool debug = true;
int loop_count = 1234;
double CW_torque;
double CCW_torque;


// FUNCTION DECLARATIONS
time_t getTeensyTime();
int pgm_lastIndexOf(uint8_t c, const char *p);
String srcfile_details();
double setPressureOffset(const bool& debug=false);
void updateLCD(double& offset, int& loop_count, String& test_status_str, double& setpoint);
void updateHeaterPID();
String tempToStr(const double& temp, const bool& unit);
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
  }

  // Get source file information
  String source_file = srcfile_details();
  Serial.println();
  Serial.println(source_file);
  Serial.println("\nInitializing display controller...");

  // Initialize the I2C bus
  Wire.begin();
  Serial.println(" - i2c bus initialized");

  // Initialize Teensy Board Time
  setSyncProvider(getTeensyTime);

  // Initialize the LCD screen
  lcd.begin(Wire);
  lcd.setBacklight(WHITE_RGB);
  lcd.setContrast(5);
  lcd.clear();
  Serial.println(" - lcd screen initialized");

  // Display source file version on LCD
  initScreen(source_file);
  delay(5000);

  // Initialize the pressure sensor and set zero offset
  if (!mpr.begin()) {
    showError("Cannot connect to MicroPressure Sensor!");
  }
  delay(5);
  press_offset = setPressureOffset(debug);

  // Initialize the thermocouple sensors
  sealTempSensor.begin(0x60);
  sealTempSensor.setAmbientResolution(RES_ZERO_POINT_25);
  sealTempSensor.setThermocoupleResolution(RES_14_BIT);
  sumpTempSensor.begin(0x67);
  sumpTempSensor.setAmbientResolution(RES_ZERO_POINT_25);
  sumpTempSensor.setThermocoupleResolution(RES_14_BIT);

  // Initialize the heater band PID loop
  pinMode(HEAT_OUTPUT_PIN, OUTPUT);
  heaterPID.SetMode(AUTOMATIC);
  heaterPID.SetOutputLimits(0, 150);
  input = sumpTempSensor.getThermocoupleTemp(temp_units);
  setpoint = 90.0; // This needs to be read from SD
  msg = "Heater PID control ready";
  initScreen(msg);
  printFourColumnRow(3, "Sump:", tempToStr(input, temp_units),
                        " Sp:", tempToStr(setpoint, temp_units));
  delay(3000);
  Serial.println(" - PID loop initialized");

  // Initialize the Run and Reset switch and bus pins
  pinMode(RUN_SW_PIN, INPUT_PULLDOWN);
  pinMode(RESET_SW_PIN, INPUT_PULLDOWN);
  pinMode(PRGM_RUN_BUS_PIN, OUTPUT);
  pinMode(RESET_BUS_PIN, OUTPUT);
  RunSwitch.attach(RUN_SW_PIN);
  ResetSwitch.attach(RESET_SW_PIN);
  RunSwitch.interval(100);
  ResetSwitch.interval(100);

  // Clear screen to begin test protocol
  lcd.clear();
}

/* -------------------------------- MAIN LOOP -------------------------------- */
void loop() {
  LoopTimer = 0;
  RunSwitch.update();
  isRunSwitchOn = RunSwitch.read();

  if (!isRunSwitchOn && !hasTestStarted) {
    test_status_str = "READY";
  }

  if (isRunSwitchOn) {
    if (!isPreHeated && sump_temp < setpoint) {
      test_status_str = "PRE-HEATING";
    }
    else {
      isPreHeated = true;
      test_status_str = "RUNNING";
      if (debugTimer > 1000) {
        debugTimer = 0;
      loop_count++;
      }
    }
    updateHeaterPID();
  }
  else {
    ResetSwitch.update();
    isResetSwitchOn = ResetSwitch.read();
  }

  if (!isRunSwitchOn && hasTestStarted) {
    test_status_str = "PAUSED";
    // don't update PID loop here
    // disable heaters
  }

  if (isResetSwitchOn) {
    ResetSwitch.update();
    isResetSwitchOn = ResetSwitch.read();
    msg = "Test will be RESET in:";
    initScreen(msg);
    for (int i = 10; i > 0; i--) {
      initScreen(msg, false, String(i)+" seconds");
      delay(1000);
      ResetSwitch.update();
      isResetSwitchOn = ResetSwitch.read();
      if (!isResetSwitchOn) {
        i = 0; // kick out of reset loop
        lcd.clear();
      }
    }
    if (isResetSwitchOn) {
      loop_count = 0;
      lcd.clear();
      digitalWrite(RESET_BUS_PIN, HIGH);
      delay(10);
      digitalWrite(RESET_BUS_PIN, LOW);
      msg = "Test has been successfully RESET!";
      delay(4000);
      initScreen(srcfile_details());
      delay(2000);
      press_offset = setPressureOffset(debug);
      ResetSwitch.update();
      isResetSwitchOn = ResetSwitch.read();
    }
  }

  updateLCD(press_offset, loop_count, test_status_str, setpoint);
}

// FUNCTION DEFINITIONS //

time_t getTeensyTime() {
  return Teensy3Clock.get();
}

double setPressureOffset(const bool& debug) {
  String zero_offset_msg = "Determining zero offset for pressure sensor:";
  initScreen(zero_offset_msg);
  double press_cal = 0.0;
  int sample_count = 0;
  elapsedMillis avgPressTimer = 0;
  elapsedMillis sampleTimer = 0;
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

  if (debug) {
    press_cal = 14.7;
  }

  Serial.println(" - pressure sensor initialized");
  msg = "offset by " + String(press_cal) + " psi"; 
  initScreen(zero_offset_msg, false, msg);
  delay(3000);
  return press_cal;
}

void updateLCD(double& offset, int& loop_count, String& test_status_str, double& setpoint) {
  String loops_str = loop_count;
  String seal_temp_str = tempToStr(sealTempSensor.getThermocoupleTemp(temp_units), temp_units);
  delay(5);
  String sump_temp_str = tempToStr(sump_temp, temp_units);
  delay(5);
  String pressure = String((mpr.readPressure() - offset));
  if (debug) {
    pressure = "12.3";
    CW_torque = 1.26;
    CCW_torque = -0.98;
  }
  String torques_str = String(CCW_torque) + " / " + String(CW_torque);
  String set_point_str = tempToStr(setpoint, temp_units);
  String date = String(month()) + "/" + String(day());
  String time = String(hour()) + ":" + String(minute());
  

  if (LCDTimer > REFRESH_INTERVAL) {
    LCDTimer = 0;
    if (isScreenUpdate) {
      isScreenUpdate = !isScreenUpdate;
      printRowPair(0, 0, MAX_CHARS_PER_LINE, "Status:", test_status_str);
      printRowPair(0, 1, MAX_CHARS_PER_LINE, "Setpoint:", set_point_str);
      printFourColumnRow(2, "P:", pressure +"psi", " Loop:", loops_str);
      printFourColumnRow(3, "Seal:", seal_temp_str, " Sump:", sump_temp_str);
    }
    else {
      isScreenUpdate = !isScreenUpdate;
      // printRowPair(0, 0, MAX_CHARS_PER_LINE, "Date:", dateTimeStr());
      // print the remaining test time here
      printRowPair(0, 1, MAX_CHARS_PER_LINE, "Torque:", torques_str);
    }
  }
}

String tempToStr(const double& temp, const bool& unit) {
  /* Returns a String with correct suffix for farenheit
   * by default. Pass 'true' after temp to set units to
   * celcius.
   */
  if (unit) {
    return String(temp, 0) + String((char)223) + "C";
  }
  else return String(temp, 0) + String((char)223) + "F";
}

String dateTimeStr() {
  /* Function returns the datetime as a String formatted as m/d hh/mm */
  String date = String(month()) + "/" + String(day()) + "/" + String(year());
  String time = String(hour()) + ":" + String(minute());
  return date + " " + time;
}
String rightJustifiedString(const String& str) {
  /* Function returns a String with enough padding
   * prepended for the LCD screen to right justify the text
   * on an empty line
   */
  String paddedStr = str;
  while (paddedStr.length() < MAX_CHARS_PER_LINE) {
    paddedStr = " " + paddedStr;
  }
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
  /* Function displays an error message on the LCD
   * and flashes the backlight red. It contains an 
   * infinite while loop so the only way to clear
   * the error is the restart the controller 
   */
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
  /* Function takes both a main message and a status update and displays the main
   * message on the LCD screen, left justified with text wrapping. The status
   * message is displayed in the lower right corner and is right justified.
   * 
   * The function can be called to clear the screen (default) prior to displaying 
   * message. Setting this to false helps prevent flicker during multiple calls
   * of function while updating only the status_msg.
  */

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

String srcfile_details() { 
  /* Function fetches the source file filename
   * as well as the date and time at compile and
   * returns that as a String.
  */ 
  const char *the_path = PSTR(__FILE__);
  String msg = "Firmware Version: ";

  int slash_loc = pgm_lastIndexOf('/', the_path);
  if (slash_loc < 0)
    slash_loc = pgm_lastIndexOf('\\', the_path);

  int dot_loc = pgm_lastIndexOf('.', the_path);
  if (dot_loc < 0)
    dot_loc = pgm_lastIndexOf(0, the_path);

  for (int i = slash_loc + 1; i < dot_loc; i++)
  {
    uint8_t b = pgm_read_byte(&the_path[i]);
    if (b != 0) {
      msg += (char)b;
    }
    else {
      break;
    }
  }
  String d = day();
  String mo = month();
  String y = year();

  msg += " - compiled on: ";
  msg += __DATE__;
  msg += " at ";
  msg += __TIME__;
  return msg;
}

int pgm_lastIndexOf(uint8_t c, const char *p) {
  /* finds last index of char 'c', withing string 'p' */
  int last_index = -1; // -1 indicates no match
  uint8_t b;
  for (int i = 0; true; i++) {
    b = pgm_read_byte(p++);
    if (b == c) {
      last_index = i;
    }
    else {
      if (b == 0) {
        break;
      }
    }
  }
  return last_index;
}

void updateHeaterPID() {
  if (PIDTimer > 100) {
    sump_temp = sumpTempSensor.getThermocoupleTemp(temp_units);
    PIDTimer = 0;
    input = sump_temp;
    heaterPID.Compute();
    analogWrite(HEAT_OUTPUT_PIN, output);
  }
}

void resetController() {
  
}