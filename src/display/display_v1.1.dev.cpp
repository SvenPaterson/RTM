#include <Arduino.h>
#include <string.h>
#include <TimeLib.h>
#include <Bounce2.h>

#include <avr/wdt.h>
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
DisplayController rtm;

// TEST CONTROL
String msg, prev_status_str, test_status_str = "STANDBY";
uint32_t current_loop_count = 0;
uint32_t requested_loops = 0;
bool torqueRequested = false;

enum ProgramState {
  STATE_STANDBY,
  STATE_HEATING,
  STATE_RUNNING,
  STATE_PAUSED,
  STATE_TEST_COMPLETED,
  STATE_RESET_REQUESTED,
};
ProgramState currentState = STATE_STANDBY;

// INTERRUPT VOLATILE VARIABLES
volatile unsigned long start_micros = 0;
volatile unsigned long end_micros = 0;

// FUNCTION DECLARATIONS
int pgm_lastIndexOf(uint8_t c, const char *p);
String srcfile_details();
void loopPinRisingEdge();
void loopPinFallingEdge();
void torquePinRisingEdge();

String source_file = srcfile_details();

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
  attachInterrupt(digitalPinToInterrupt(LOOP_BUS_PIN), loopPinRisingEdge, RISING);
  attachInterrupt(digitalPinToInterrupt(TORQ_FLAG_BUS_PIN), torquePinRisingEdge, RISING);

  // Initialize the rtm
  rtm.begin(pinMappings);

  // Display source file version on LCD
  rtm.messageScreen(source_file);
  rtm.writeToLog(source_file, "INFO");
  delay(5000);

  // measure ambient lab pressure and automatically set offset
  rtm.setPressureOffset();


  // Clear screen to begin test protocol
  Serial.println("Test Status: " + test_status_str);
  rtm.writeToLog("Display Controller initialized", "STATUS");
  rtm.lcd.clear();

  requested_loops = rtm.getRequestedNumberOfLoops();
  current_loop_count = rtm.getCurrentLoopCount();
}

/* -------------------------------- MAIN LOOP -------------------------------- */
void loop() {
  rtm.update(current_loop_count);

  if (test_status_str != prev_status_str) {
    // force the screen to update whenever a switch is 
    // pressed. Otherwise the screen will update at the 
    // default update interval of every 2 seconds.
    rtm.statusUpdate(test_status_str);
  }
  prev_status_str = test_status_str;

  switch (currentState) {
    case STATE_STANDBY:
      if (rtm.getRunSwitch()) {
        currentState = STATE_HEATING;
        test_status_str = "HEATING";
        rtm.writeToLog(test_status_str, "STATUS");
      }
      else if (rtm.getResetSwitch()) {
        currentState = STATE_RESET_REQUESTED;
      }
      else {
        rtm.updateLCD(test_status_str);
        rtm.turnOffHeaters();
      }
      return;

    case STATE_HEATING:
      rtm.updateLCD(test_status_str);
      rtm.computeHeaterOutput();
      if (!rtm.getRunSwitch()) {
        test_status_str = "PAUSED";
        currentState = STATE_PAUSED;
        rtm.writeToLog(test_status_str, "STATUS");
      }
      else if (rtm.getSumpTemp() < rtm.getSetpointTemp()) {
        rtm.stopProgram();
        rtm.updateLCD(test_status_str);
        rtm.armHeaters();
        rtm.computeHeaterOutput();
      }
      else if (rtm.getResetSwitch()) {
        test_status_str = "RESETTING";
        currentState = STATE_RESET_REQUESTED;
      }
      else {
        currentState = STATE_RUNNING;
        test_status_str = "RUNNING";
        rtm.writeToLog(test_status_str, "STATUS");
      }
      return;

    case STATE_RUNNING:
      rtm.updateLCD(test_status_str);
      rtm.runProgram();
      rtm.computeHeaterOutput();
      rtm.writeToDataFile();
      if (current_loop_count >= requested_loops) {
        currentState = STATE_TEST_COMPLETED;
        test_status_str = "TEST DONE";
        rtm.writeToLog(test_status_str, "STATUS");

      }
      else if(torqueRequested) {
        torqueRequested = false;
        rtm.readTorque();
      }
      else if (rtm.getResetSwitch()) {
        test_status_str = "RESETTING";
        currentState = STATE_RESET_REQUESTED;
      }
      if (!rtm.getRunSwitch()) {
        currentState = STATE_PAUSED;
        test_status_str = "PAUSED";
        rtm.writeToLog(test_status_str, "STATUS");
      }
      return;  

    case STATE_PAUSED:
      rtm.updateLCD(test_status_str);
      rtm.stopProgram();
      rtm.turnOffHeaters();
      if (rtm.getRunSwitch()) {
        currentState = STATE_HEATING;
        test_status_str = "HEATING";
        rtm.writeToLog(test_status_str, "STATUS");
      }
      else if (rtm.getResetSwitch()) {
        currentState = STATE_RESET_REQUESTED;
        test_status_str = "RESETTING";
      }
      return;

    case STATE_TEST_COMPLETED:
      rtm.updateLCD(test_status_str);
      rtm.lcd.setBacklight(0,255,0);
      rtm.turnOffHeaters();
      rtm.stopProgram();
      if (rtm.getResetSwitch()) {
        test_status_str = "RESETTING";
        currentState = STATE_RESET_REQUESTED;
        rtm.lcd.setBacklight(255,255,255);
      }
      return;

    case STATE_RESET_REQUESTED:
      if (!rtm.getResetSwitch() && current_loop_count < requested_loops) {
        test_status_str = "PAUSED";
        currentState = STATE_PAUSED;
        rtm.writeToLog(test_status_str, "STATUS");
      }
      else if (!rtm.getResetSwitch() && current_loop_count >= requested_loops) {
        test_status_str = "TEST DONE";
        currentState = STATE_TEST_COMPLETED;
        rtm.writeToLog(test_status_str, "STATUS");
      }
      else {
        rtm.updateLCD(test_status_str);
        rtm.turnOffHeaters();
        rtm.stopProgram();
        msg = "Test will be RESET in:";
        rtm.messageScreen(msg);
        for (int i = 10; i > 0; i--) {
          rtm.messageScreen(msg, false, String(i) + " seconds");
          delay(1000);
          rtm.update(current_loop_count);
          if (!rtm.getResetSwitch()) {
            i = 0; // kick out of reset loop
            rtm.lcd.clear();
          }
        }
        if (rtm.getResetSwitch()) {
          current_loop_count = 0;
          rtm.resetTest();
          requested_loops = rtm.getRequestedNumberOfLoops();
          rtm.messageScreen(source_file);
          delay(4000);
          rtm.lcd.clear();
          test_status_str = "STANDBY";
          currentState = STATE_STANDBY;
        }
      }
      return;
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
    current_loop_count++;
  }
  attachInterrupt(digitalPinToInterrupt(LOOP_BUS_PIN), loopPinRisingEdge, RISING);
}

void torquePinRisingEdge() {
  torqueRequested = true;
}

void resetFunc() {
  wdt_enable(WDTO_15MS);
  while (true) {}
}