#include <Arduino.h>
#include <string.h>
#include <TimeLib.h>
#include <Bounce2.h>

#include "DisplayController.h"

// RTM Display Controller Version
// Last Update: 4/10/24
#define SRC_FILE_VERSION "Display v1.3"

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
#define HEAT_BUS_PIN 12
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
#define RESET_TIMER 5

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
    {"HEAT_BUS_PIN", HEAT_BUS_PIN},
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

        break;

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

        break;

    case STATE_RUNNING:
        rtm.updateLCD(test_status_str);
        rtm.runProgram();
        rtm.computeHeaterOutput();
        rtm.writeToDataFile();
        if (current_loop_count >= requested_loops) {
            test_status_str = "TEST DONE";
            rtm.writeToLog(test_status_str, "STATUS");
            currentState = STATE_TEST_COMPLETED;
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

        break;  

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

        break;

    case STATE_TEST_COMPLETED:     
        rtm.testCompleted(test_status_str);
        if (rtm.getResetSwitch()) {
            currentState = STATE_RESET_REQUESTED;
            test_status_str = "RESETTING";
            rtm.resetScreen();
        }

        break;

    case STATE_RESET_REQUESTED:
        // cancel reset request before count is up and test has not completed
        if (!rtm.getResetSwitch() && current_loop_count < requested_loops) {
            test_status_str = "PAUSED";
            currentState = STATE_PAUSED;
            rtm.writeToLog(test_status_str, "STATUS");
        }
        
        // cancel reset request before count is up, but test has completed
        else if (!rtm.getResetSwitch() && current_loop_count >= requested_loops) {
            test_status_str = "TEST DONE";
            rtm.writeToLog(test_status_str, "STATUS");
            currentState = STATE_TEST_COMPLETED;
        }

        // countdown before completing system reset
        else {
            rtm.updateLCD(test_status_str);
            rtm.turnOffHeaters();
            rtm.stopProgram();
            msg = "Test will be RESET in:";
            rtm.messageScreen(msg);

            for (int i = RESET_TIMER; i > 0; i--) {
                rtm.messageScreen(msg, false, String(i) + " seconds");
                delay(1000);
                rtm.update(current_loop_count);
                if (!rtm.getResetSwitch()) {
                    i = 0; // kick out of reset loop
                    rtm.resetScreen();
                }
            }

            if (rtm.getResetSwitch()) {
                current_loop_count = 0;
                rtm.resetTest();
                requested_loops = rtm.getRequestedNumberOfLoops();
                rtm.messageScreen(source_file);
                delay(4000);
                rtm.resetScreen();
                test_status_str = "STANDBY";
                currentState = STATE_STANDBY;
            }
        }

        break;
    }
}



///////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////Functions//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

String srcfile_details() { 
    String msg = SRC_FILE_VERSION;
    msg += "  compiled on: ";
    msg += __DATE__;
    msg += " at ";
    msg += __TIME__;
    return msg;
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