#include <Arduino.h>
#include <string.h>
#include <SerLCD.h>
#include <TimeLib.h>
#include <Bounce2.h>

#include "DisplayController.h"

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
elapsedMillis heaterPIDTimer;
elapsedMillis errorTimer;
elapsedMillis debugTimer; // DELETE WHEN COMPLETE

// LCD SCREEN
#define MAX_CHARS_PER_LINE 20
#define WHITE_RGB 255, 255, 255
#define RED_RGB 255, 255, 0
#define REFRESH_INTERVAL 2000 // interval, in millis, between screen refreshes
bool isScreenUpdate = true;
bool isBacklightOn = true;
String test_status_str = "READY";
String msg;

// Initialize DisplayController
DisplayController displayController;
// temp vars
#define PRESS_AVG_TIME 3000
double press_offset;

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
bool torqueRequested = false;

// INTERRUPT VOLITILE VARIABLES
volatile unsigned long start_micros = 0;
volatile unsigned long end_micros = 0;

// DEBUGGING ONLY
bool debug = true;
int loop_count = 1234;
double CW_torque;
double CCW_torque;
double temp_setpoint = 60.0; // will be set by SD card file

// FUNCTION DECLARATIONS
time_t getTeensyTime();
int pgm_lastIndexOf(uint8_t c, const char *p);
String srcfile_details();
//double setPressureOffset(const bool& debug=false);
void updateLCD(String& test_status_str);
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
//void initScreen(const String& init_msg, const bool& cls = true, const String& status_msg = "");
void loopPinRisingEdge();
void loopPinFallingEdge();
void torqueRequestRisingEdge();

/* ----------------------------- INTIAL SETUP ----------------------------- */
void setup() {
  
  // Get source file information
  String source_file = srcfile_details();
  Serial.println();
  Serial.println(source_file);

  displayController.setHeaterPins(HEAT_SAFETY_PIN, HEAT_OUTPUT_PIN);
  displayController.begin();

  // Initialize Teensy Board Time
  setSyncProvider(getTeensyTime);

  // Display source file version on LCD
  displayController.messageScreen(source_file);
  delay(5000);

  // measure ambient lab pressure and set offset
  displayController.setPressureOffset();
  delay(3000);

  // Initialize the heater band PID loop
  displayController.updateSensors();
  displayController.setTempSetpoint(temp_setpoint);
  msg = "Heater PID control ready";
  displayController.messageScreen(msg);
  String input_str = tempToStr(displayController.getSumpTemp(), temp_units);
  String setpoint_str = tempToStr(temp_setpoint, temp_units);
  printFourColumnRow(3, "Sump:", input_str, " Sp:", setpoint_str);
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

  // Initialize bus pins and attach interrupts
  pinMode(LOOP_BUS_PIN, INPUT_PULLDOWN);
  pinMode(TORQ_FLAG_BUS_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(LOOP_BUS_PIN), loopPinRisingEdge, RISING);
  attachInterrupt(digitalPinToInterrupt(TORQ_FLAG_BUS_PIN), torqueRequestRisingEdge, RISING);

  // Initialize Motor PWM pin for reading torque
  pinMode(MOTOR_HLFB_PIN, INPUT_PULLUP);
  
  // Clear screen to begin test protocol
  displayController.lcd.clear();
}

/* -------------------------------- MAIN LOOP -------------------------------- */
void loop() {
  LoopTimer = 0;
  displayController.updateSensors();
  RunSwitch.update();
  isRunSwitchOn = RunSwitch.read();

  // PRE-TEST LOGIC //
  if (!isRunSwitchOn && !hasTestStarted) {
    test_status_str = "READY";
    digitalWrite(HEAT_SAFETY_PIN, LOW);
  } // END PRE-TEST LOGIC

  // RUN LOGIC //
  if (isRunSwitchOn) {
    if (!isPreHeated && displayController.getSumpTemp() < temp_setpoint) {
      test_status_str = "HEATING";
      digitalWrite(HEAT_SAFETY_PIN, HIGH);
    }
    else { // TEST HAS PRE-HEATED //
      isPreHeated = true;
      test_status_str = "RUNNING";
      hasTestStarted = true;

      if (torqueRequested) {
        // calculate torque here
        delay(100);
      }
       
      // PROFILE LOGIC GOES HERE //
      if (debugTimer > 1000) {
        debugTimer = 0;
      loop_count++;
      } // END OF PROFILE LOGIC
    }
    displayController.computeHeaterOutput();
  }
  else {
    ResetSwitch.update();
    isResetSwitchOn = ResetSwitch.read();
  } // END OF RUN LOGIC

  // PAUSED LOGIC //
  if (!isRunSwitchOn && hasTestStarted) {
    test_status_str = "PAUSED";
    isPreHeated = false; // test may have cooled off
    digitalWrite(HEAT_OUTPUT_PIN, LOW);
    digitalWrite(HEAT_SAFETY_PIN, LOW);
  } // END OF PAUSED LOGIC

  // RESET LOGIC //
  if (isResetSwitchOn) {
    digitalWrite(HEAT_OUTPUT_PIN, LOW);
    digitalWrite(HEAT_SAFETY_PIN, LOW);
    ResetSwitch.update();
    isResetSwitchOn = ResetSwitch.read();
    msg = "Test will be RESET in:";
    displayController.messageScreen(msg);
    for (int i = 10; i > 0; i--) {
      displayController.messageScreen(msg, false, String(i)+" seconds");
      delay(1000);
      ResetSwitch.update();
      isResetSwitchOn = ResetSwitch.read();
      if (!isResetSwitchOn) {
        i = 0; // kick out of reset loop
        displayController.lcd.clear();
      }
    }
    if (isResetSwitchOn) {
      loop_count = 0;
      displayController.lcd.clear();
      digitalWrite(RESET_BUS_PIN, HIGH);
      delay(10);
      digitalWrite(RESET_BUS_PIN, LOW);
      msg = "Test has been successfully RESET!";
      delay(4000);
      displayController.messageScreen(srcfile_details());
      delay(2000);
      displayController.setPressureOffset();
      delay(3000);
      ResetSwitch.update();
      isResetSwitchOn = ResetSwitch.read();
    }
  } // END RESET LOGIC
  if (heaterPIDTimer > 100) {
    heaterPIDTimer = 0;
    updateLCD(test_status_str);
  }
}

// FUNCTION DEFINITIONS //

time_t getTeensyTime() {
  return Teensy3Clock.get();
}

double setPressureOffset(const bool& debug) {
  String zero_offset_msg = "Determining zero offset for pressure sensor:";
  displayController.messageScreen(zero_offset_msg);
  double press_cal = 0.0;
  int sample_count = 0;
  elapsedMillis avgPressTimer = 0;
  elapsedMillis sampleTimer = 0;
  while (avgPressTimer <= PRESS_AVG_TIME) {
    if (sampleTimer > 1000) {
      displayController.updateSensors();
      press_cal += displayController.getPressure();
      sample_count++;  
      sampleTimer = 0;
      msg = "please wait... " + String(sample_count) + "s";
      displayController.messageScreen(zero_offset_msg, false, msg);
    }
  }
  press_cal /= sample_count;

  if (debug) {
    press_cal = 14.7;
  }

  Serial.println(" - pressure sensor initialized");
  msg = "offset by " + String(press_cal) + " psi"; 
  displayController.messageScreen(zero_offset_msg, false, msg);
  delay(3000);
  return press_cal;
}

void updateLCD(String& test_status_str) {
  String loops_str = loop_count;
  String seal_temp_str = tempToStr(displayController.getSealTemp(), temp_units);
  delay(5);
  String sump_temp_str = tempToStr(displayController.getSumpTemp(), temp_units);
  delay(5);
  String pressure = String((displayController.getPressure() - press_offset));
  if (debug) {
    pressure = "12.3";
    CW_torque = 1.26;
    CCW_torque = -0.98;
  }
  String torques_str = String(CCW_torque) + " / " + String(CW_torque);
  String set_point_str = tempToStr(temp_setpoint, temp_units);
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
  displayController.lcd.setCursor(col, row);
  displayController.lcd.print(str1);
  displayController.lcd.setCursor(str1.length()+col, row);
  displayController.lcd.print(padBetweenChars(width, str1, str2));
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
  displayController.lcd.clear();
  displayController.lcd.setCursor(0,0);
  displayController.lcd.print("Error! Error! Error!");
  displayController.lcd.setCursor(0,1);
  displayController.lcd.print(errorMessage);

  while (true) {
    if (errorTimer >= 1000) {
      if (isBacklightOn) {
        displayController.lcd.setBacklight(RED_RGB);
        isBacklightOn = false;
      }
      else {
        displayController.lcd.setBacklight(0, 0, 0);
        isBacklightOn = true;
      }
    errorTimer = 0;
    }
  }
}
/* 
void initScreen(const String& init_msg, const bool& cls, const String& status_msg) {
  /* Function takes both a main message and a status update and displays the main
   * message on the LCD screen, left justified with text wrapping. The status
   * message is displayed in the lower right corner and is right justified.
   * 
   * The function can be called to clear the screen (default) prior to displaying 
   * message. Setting this to false helps prevent flicker during multiple calls
   * of function while updating only the status_msg.
  

  if (cls) {
    displayController.lcd.clear();
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

    displayController.lcd.setCursor(0, numLines);
    displayController.lcd.print(init_msg.substring(startPos, endPos + 1));
    numLines++;
    startPos = endPos + 1;
  }

  displayController.lcd.setCursor(0, 3);

  if (status_msg != "") {
    displayController.lcd.print(rightJustifiedString(status_msg));
  }
}
 */
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

void loopPinRisingEdge() {
  attachInterrupt(digitalPinToInterrupt(LOOP_BUS_PIN), loopPinFallingEdge, FALLING);
  start_micros = micros();
}

void loopPinFallingEdge() {
  end_micros = micros();
  volatile unsigned long duration = (end_micros - start_micros);
  if ((duration >= 990 && duration < 1010)) {
    loop_count++;
  }
  attachInterrupt(digitalPinToInterrupt(LOOP_BUS_PIN), loopPinRisingEdge, RISING); // Enable the interupt.
}

void torqueRequestRisingEdge() {
  torqueRequested = true;
}