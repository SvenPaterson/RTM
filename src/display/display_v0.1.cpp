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
#define PRGM_RESET_BUS_PIN 20

// Initialize DisplayController
DisplayController displayController;

// TEST CONTROL
String msg, test_status_str = "READY";
uint8_t loop_count = 0;
uint8_t total_loops = 10;
double temp_setpoint = 90.0; // will be set by SD card file
bool temp_units = false; // 'false' for farenheit, 'true' for celcius

// TEST LOGIC
bool hasTestStarted = false;
bool isPreHeated = false;
bool torqueRequested = false;
bool isSDCardInserted = false;

// INTERRUPT VOLITILE VARIABLES
volatile unsigned long start_micros = 0;
volatile unsigned long end_micros = 0;

// FUNCTION DECLARATIONS
int pgm_lastIndexOf(uint8_t c, const char *p);
String srcfile_details();
void loopPinRisingEdge();
void loopPinFallingEdge();
void torquePinRisingEdge();
void beginSDCard();

/* ----------------------------- INTIAL SETUP ----------------------------- */
void setup() {
  /****** DEBUGGING *******/

  // pin mappings
  std::map<String, uint8_t> pinMappings = {
    {"SD_DETECT_PIN", SD_DETECT_PIN},
    {"LOOP_BUS_PIN", LOOP_BUS_PIN},
    {"MOTOR_HLFB_PIN", MOTOR_HLFB_PIN},
    {"HEAT_OUTPUT_PIN", HEAT_OUTPUT_PIN},
    {"HEAT_SAFETY_PIN", HEAT_SAFETY_PIN}, 
    {"AIR_SUPPLY_PIN", AIR_SUPPLY_PIN}, 
    {"AIR_DUMP_PIN", AIR_DUMP_PIN}, 
    {"PRGM_RUN_BUS_PIN", PRGM_RUN_BUS_PIN}, 
    {"TORQ_FLAG_BUS_PIN", TORQ_FLAG_BUS_PIN}, 
    {"AIR_SUPPLY_BUS_PIN", AIR_SUPPLY_BUS_PIN}, 
    {"AIR_DUMP_BUS_PIN", AIR_DUMP_BUS_PIN}, 
    {"RUN_SW_PIN", RUN_SW_PIN},
    {"RESET_SW_PIN", RESET_SW_PIN},
    {"SDA0_PIN", SDA0_PIN}, 
    {"SDL0_PIN", SDL0_PIN},
    {"PRGM_RESET_BUS_PIN", PRGM_RESET_BUS_PIN}
  };

  // Initialize bus pins and attach interrupts
  pinMode(LOOP_BUS_PIN, INPUT_PULLDOWN);
  pinMode(TORQ_FLAG_BUS_PIN, INPUT_PULLDOWN);
  //pinMode(SD_DETECT_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(LOOP_BUS_PIN), loopPinRisingEdge, RISING);
  attachInterrupt(digitalPinToInterrupt(TORQ_FLAG_BUS_PIN), torquePinRisingEdge, RISING);
  attachInterrupt(digitalPinToInterrupt(SD_DETECT_PIN), beginSDCard, RISING);
  // does beginSDcard even need to be an interrupt if I am updating the 
  // displayController._isSDCardInserted bool every loop????

  // Initialize the displayController
  displayController.begin(pinMappings);

  // Display source file version on LCD
    // Get source file information
  String source_file = srcfile_details();
  displayController.messageScreen(source_file);
  displayController.writeToLog(source_file, "INFO");
  delay(5000);

  // measure ambient lab pressure and automatically set offset
  displayController.setPressureOffset();

  // Initialize the heater band PID loop
  displayController.setTempSetpoint(temp_setpoint, temp_units);

  // Clear screen to begin test protocol
  Serial.println("Test Status: " + test_status_str);
  displayController.lcd.clear();
}

/* -------------------------------- MAIN LOOP -------------------------------- */
void loop() {
  // TEST COMPLETED LOGIC //
  if (loop_count >= total_loops) {
    // display test completed screen
    displayController.turnOffHeaters();
    displayController.stopProgram();
    displayController.testDoneScreen(loop_count, total_loops);
  }

  else {
    displayController.update();
    displayController.updateLCD(test_status_str, loop_count);

    // PRE-TEST LOGIC //
    if (!displayController.getRunSwitch() && !hasTestStarted) {
      test_status_str = "READY";
      displayController.turnOffHeaters();
    } // END PRE-TEST LOGIC

    // RUN LOGIC //
    if (displayController.getRunSwitch()) {
      if (!isPreHeated && displayController.getSumpTemp() < temp_setpoint) {
        test_status_str = "HEATING";
        displayController.armHeaters();
        displayController.stopProgram();
      }
      else { // TEST HAS PRE-HEATED AND NOW RUNNING //
        isPreHeated = true;
        test_status_str = "RUNNING";
        hasTestStarted = true;
        displayController.runProgram();

        if (torqueRequested) {
          torqueRequested = !torqueRequested;
          displayController.readTorque();
          delay(100);
        }

        displayController.computeHeaterOutput();

        // PROFILE LOGIC GOES HERE //
        // loop_pin from motor is looping every 10s for debugging.
        // END OF PROFILE LOGIC

      }
    }
    else {
    } // END OF RUN LOGIC

    // PAUSED LOGIC //
    if (!displayController.getRunSwitch() && hasTestStarted) {
      test_status_str = "PAUSED";
      isPreHeated = false; // test may have cooled off
      displayController.turnOffHeaters();
      displayController.stopProgram();
    } // END OF PAUSED LOGIC

    // RESET LOGIC //
    if (displayController.getResetSwitch()) {
      displayController.turnOffHeaters();
      displayController.stopProgram();
      msg = "Test will be RESET in:";
      displayController.messageScreen(msg);
      for (int i = 10; i > 0; i--) {
        displayController.messageScreen(msg, false, String(i)+" seconds");
        delay(1000);
        displayController.update();
        if (!displayController.getResetSwitch()) {
          i = 0; // kick out of reset loop
          displayController.lcd.clear();
        }
      }
      if (displayController.getResetSwitch()) {
        loop_count = 0;
        hasTestStarted = false;
        displayController.resetTest();
        displayController.messageScreen(srcfile_details());
        delay(4000);
      }
    } // END RESET LOGIC
  }
}


///////////////////////////////////////////////////////////////////////////////////
// FUNCTION DEFINITIONS //

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

/****** INTERRUPT FUNCTIONS ******/

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
  attachInterrupt(digitalPinToInterrupt(LOOP_BUS_PIN), loopPinRisingEdge, RISING);
}

void torquePinRisingEdge() {
  torqueRequested = true;
}

void beginSDCard() {
  isSDCardInserted = true;
}