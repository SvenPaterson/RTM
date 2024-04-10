#include "motor_config.h"
#include <FlexyStepper.h>
#include <avr/pgmspace.h>

// RTM Motor Controller Version
// Last Update: 4/10/24
#define SRC_FILE_VERSION "Motor v1.3"

/******* INTERRUPT VARIABLES *******/
volatile unsigned long start_micros = 0;
volatile unsigned long end_micros = 0;
volatile unsigned long duration = 0;
volatile bool askingToRun = false;

/******* I/O PINS *******/
const uint8_t LOOP_BUS_PIN = 1;  
const uint8_t MOTOR_STEP_PIN = 3;
const uint8_t MOTOR_DIRECTION_PIN = 4;
const uint8_t MOTOR_ENABLE_PIN = 5;
const uint8_t PRGM_RUN_BUS_PIN = 10;
const uint8_t TORQUE_FLAG_BUS_PIN = 11;
const uint8_t HEAT_BUS_PIN = 12;
const uint8_t LED_PIN = 13;
const uint8_t AIR_SUPPLY_BUS_PIN = 14;
const uint8_t AIR_DUMP_BUS_PIN = 15 ;
const uint8_t PRGM_RESET_BUS_PIN = 20;


/******* SYSTEM STATE CONTROL *******/
enum SystemState {
DEBUG,  // debug mode: currentState = DEBUG
IDLE,   // normal mode: currentState = IDLE
RUNNING,
PAUSED,
RESET_REQUESTED,
RESUME
};
SystemState currentState = IDLE;
bool isFullyStopped = false;
bool isStepInitialized = false;
bool isPauseInitiated = false;
uint16_t currentStepIndex = 0;
elapsedMillis LED_timer, test_step_timer;
unsigned long pause_start_time = 0;

/******* STEPPER MOTOR INIT *******/
FlexyStepper stepper;
uint16_t cnts_per_rev = SPR;
int32_t MAX_REVS = (__LONG_MAX__ / cnts_per_rev) - 1;
uint8_t size_steps = sizeof(steps) / sizeof(steps[0]);
float time_spent_accelerating_s = 0.0;
float prev_speed_rpm = 0.0;

/******* FUNC DECLARATIONS *******/
void display_srcfile_details();
void reset_rising();
void reset_falling();
void run_rising();
void run_falling();
void debugStepInfo();
void printCurrentState();


void setup() {
    pinMode(PRGM_RUN_BUS_PIN, INPUT_PULLDOWN);
    pinMode(PRGM_RESET_BUS_PIN, INPUT_PULLDOWN); 
    pinMode(LED_PIN, OUTPUT);
    pinMode(AIR_SUPPLY_BUS_PIN, OUTPUT);  
    pinMode(AIR_DUMP_BUS_PIN, OUTPUT);
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    pinMode(MOTOR_STEP_PIN, OUTPUT);
    pinMode(MOTOR_DIRECTION_PIN, OUTPUT);
    pinMode(LOOP_BUS_PIN, OUTPUT);
    pinMode(TORQUE_FLAG_BUS_PIN, OUTPUT);
    pinMode(HEAT_BUS_PIN, OUTPUT);

    digitalWrite (MOTOR_ENABLE_PIN, LOW);
    digitalWrite (AIR_SUPPLY_BUS_PIN, LOW);
    digitalWrite (AIR_DUMP_BUS_PIN, LOW);
    digitalWrite (LED_PIN, HIGH);
    digitalWrite (LOOP_BUS_PIN, LOW);
    digitalWrite (TORQUE_FLAG_BUS_PIN, LOW);

    Serial.begin(115200);
    stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
    stepper.setStepsPerRevolution(cnts_per_rev);

    delay(5000);
    display_srcfile_details();
    if (currentState != DEBUG) {
        attachInterrupt(digitalPinToInterrupt(PRGM_RESET_BUS_PIN), reset_rising, RISING);
        attachInterrupt(digitalPinToInterrupt(PRGM_RUN_BUS_PIN), run_rising, RISING);
    }
    LED_timer = 0;
    printCurrentState();
}


void loop() {
    switch (currentState) {
        case DEBUG:
            /* // move motor back and forth at 180rpm
            digitalWrite(MOTOR_ENABLE_PIN, HIGH);
            stepper.setAccelerationInRevolutionsPerSecondPerSecond(90 / 60.0);
            stepper.setSpeedInRevolutionsPerSecond(180 / 60.0);

            // Manually set to move CW
            stepper.setTargetPositionRelativeInRevolutions(10);
            while(!stepper.motionComplete()) stepper.processMovement();
            delay(1000);

            // Manually set to move CCW
            stepper.setTargetPositionRelativeInRevolutions(-10);
            while(!stepper.motionComplete()) stepper.processMovement(); */

            if (digitalRead(PRGM_RUN_BUS_PIN)) {
                digitalWrite(LOOP_BUS_PIN, HIGH);
                delay(1);
                digitalWrite(LOOP_BUS_PIN, LOW);
                delay(3000);
            }

            break;
        
        case IDLE:
            // flash LED slowly to signal IDLE state
            if (LED_timer > 1000) { 
                digitalWrite(LED_PIN, !digitalRead(LED_PIN));
                LED_timer = 0;
            }
        
            // power down motor and heaters
            digitalWrite(MOTOR_ENABLE_PIN, LOW);
            digitalWrite(HEAT_BUS_PIN, LOW);

            // check for run request
            if (askingToRun) {
                Serial.println("Transitioning to RUNNING state");
                currentState = RUNNING;
                printCurrentState();
            }
            
            break;

        case RESET_REQUESTED:
            display_srcfile_details();
            // reset test steps
            currentStepIndex = 0;
            currentState = IDLE;
            printCurrentState();

            break;

        case PAUSED:
            // store the test step timer value upon pausing test
            pause_start_time = test_step_timer;

            // Flash LED quickly to signal PAUSED state
            if (LED_timer > 100) {
                digitalWrite(LED_PIN, !digitalRead(LED_PIN));
                LED_timer = 0;
            }

            // upon entering a pause, call for a stop
            if (!isPauseInitiated) {
                Serial.println("Motor called to stop");
                stepper.setTargetPositionToStop();
                isPauseInitiated = true;
            }

            // Run until stopped, then disable motor
            if (!stepper.processMovement()) {
                digitalWrite(MOTOR_ENABLE_PIN, LOW);
                isFullyStopped = true;
            }

            // Check if it's time to resume
            if (askingToRun && isFullyStopped) {
                Serial.println("Test called to resume");
                currentState = RESUME;
                isPauseInitiated = false;
                printCurrentState();
            }

            break;

        case RESUME:
            Serial.println("Resuming the following step:");
            debugStepInfo();

            // re-initialize common test settings
            digitalWrite(LED_PIN, HIGH);
            digitalWrite(HEAT_BUS_PIN, steps[currentStepIndex].turnOnHeat);
            digitalWrite(MOTOR_ENABLE_PIN, HIGH);
            
            currentState = RUNNING;
            printCurrentState();

            // resume the test step timer
            test_step_timer = pause_start_time;
            
            break;

        case RUNNING:
            // only perform these actions at start of test step
            if (!isStepInitialized) {
                digitalWrite(LED_PIN, HIGH);
                digitalWrite(HEAT_BUS_PIN, steps[currentStepIndex].turnOnHeat);
                digitalWrite(MOTOR_ENABLE_PIN, HIGH);
                stepper.setAccelerationInRevolutionsPerSecondPerSecond(steps[currentStepIndex].accel / 60.0);
                stepper.setSpeedInRevolutionsPerSecond(steps[currentStepIndex].target_speed / 60.0);
                stepper.setTargetPositionRelativeInRevolutions(steps[currentStepIndex].is_CCW ? MAX_REVS : -MAX_REVS);
                
                // calculate the time spent accelerating for dwell period logic
                prev_speed_rpm = abs(stepper.getCurrentVelocityInStepsPerSecond() * 60.0 / cnts_per_rev);
                time_spent_accelerating_s = abs(steps[currentStepIndex].target_speed - prev_speed_rpm) / 
                                                steps[currentStepIndex].accel;
                
                // prevent re-initialization of test step
                isStepInitialized = true;
                debugStepInfo();

                // begin timing the test step
                test_step_timer = 0;
            }

            // If currently in a dwell period, disable the motor
            if (steps[currentStepIndex].target_speed == 0 && 
                test_step_timer > time_spent_accelerating_s * 1000) {
                if (digitalRead(MOTOR_ENABLE_PIN)) {
                    digitalWrite(MOTOR_ENABLE_PIN, LOW);
                }
            } 
            else { // otherwise advance the motor
                stepper.processMovement();
            }

            // Check if the test step has completed
            if (test_step_timer >= steps[currentStepIndex].time * 1000) {
                isStepInitialized = false;
                currentStepIndex = (currentStepIndex + 1) % size_steps;
                
            }

            // Inform display controller of loop completion
            if (currentStepIndex == 0 && !isStepInitialized) {
                digitalWrite(LOOP_BUS_PIN, HIGH);
                delay(1);
                digitalWrite(LOOP_BUS_PIN, LOW);
            }

            break;
    }
}



///////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////Functions//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

/******* RUN, RESET, PAUSE INTERRUPT FUNCTIONS *******/
void reset_rising() {
    attachInterrupt(digitalPinToInterrupt(PRGM_RESET_BUS_PIN), reset_falling, FALLING);
    start_micros = micros();
}

void reset_falling() {
    end_micros = micros();
    duration = end_micros - start_micros;
    attachInterrupt(digitalPinToInterrupt(PRGM_RESET_BUS_PIN), reset_rising, RISING);
    // if reset button toggled for 10s, then switch to reset requested state.
    if (duration >= 9900 && duration < 10100) {
        currentState = RESET_REQUESTED;
        printCurrentState();
    }
}

void run_rising() {
    attachInterrupt(digitalPinToInterrupt(PRGM_RUN_BUS_PIN), run_falling, FALLING);
    askingToRun = true;
    Serial.println(isFullyStopped);
}

void run_falling() {
    attachInterrupt(digitalPinToInterrupt(PRGM_RUN_BUS_PIN), run_rising, RISING);
    askingToRun = false;
    currentState = PAUSED;
    printCurrentState();
}

void display_srcfile_details(void) {
    Serial.print(SRC_FILE_VERSION);
    Serial.print("  Compiled on: ");
    Serial.print(__DATE__);
    Serial.print(" at ");
    Serial.print(__TIME__);
    Serial.print("\n");
}

void debugStepInfo() {
    Serial.print("Step ");
    Serial.print(currentStepIndex + 1);
    Serial.print("---->\tDir: ");
    Serial.print(steps[currentStepIndex].is_CCW ? "CCW" : "CW");
    Serial.print("\t\tTarget, revs: ");
    Serial.print(steps[currentStepIndex].is_CCW ? MAX_REVS : -MAX_REVS);
    Serial.print("\t\tTime, s: ");
    Serial.print(steps[currentStepIndex].time);
    Serial.print("\tSpeed, rpm: ");
    Serial.print(steps[currentStepIndex].target_speed);
    Serial.print("\tAccel, rpm/s: ");
    Serial.println(steps[currentStepIndex].accel);
}

void printCurrentState() {
    switch (currentState) {
        case DEBUG:
        Serial.println("Current State: DEBUG");
        break;
        case IDLE:
        Serial.println("Current State: IDLE");
        break;
        case RUNNING:
        Serial.println("Current State: RUNNING");
        break;
        case PAUSED:
        Serial.println("Current State: PAUSED");
        break;
        case RESET_REQUESTED:
        Serial.println("Current State: RESET REQUESTED");
        break;
        case RESUME:
        Serial.println("Current State: RESUME");
        break;
        default:
        Serial.println("Unknown State");
        break;
    }
}