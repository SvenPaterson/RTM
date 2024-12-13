#include "motor_config.h"
#include <AccelStepper.h>
#include <avr/pgmspace.h>

// RTM Motor Controller Version
// Last Update: 12/13/24
// change log:
// 12/13/24: Fixed pause, resume and restart logic

#define SRC_FILE_VERSION "Motor v1.4"

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
bool isTargetSpeedMet = false;
bool isDwellOver = false;
uint16_t currentStepIndex = 0;
elapsedMillis LED_timer, dwell_timer, pause_timer, debugTimer;


/******* STEPPER MOTOR INIT *******/
AccelStepper stepper(AccelStepper::DRIVER, MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
uint16_t steps_per_rev = SPR;
int32_t MAX_REVS = (__LONG_MAX__ / steps_per_rev) - 1;
uint8_t size_steps = sizeof(steps) / sizeof(steps[0]);
double target_speed_steps_s = 0;
long target_position = 0;
double accel_steps_s2 = 0;
double current_speed = 0; 
double current_accel = 0;

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
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    pinMode(MOTOR_STEP_PIN, OUTPUT);
    pinMode(MOTOR_DIRECTION_PIN, OUTPUT);
    pinMode(LOOP_BUS_PIN, OUTPUT);

    digitalWrite (MOTOR_ENABLE_PIN, LOW);
    digitalWrite (LED_PIN, HIGH);
    digitalWrite (LOOP_BUS_PIN, LOW);

    Serial.begin(115200);

    delay(2000);
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
            // anything here you need
            break;
        
        case IDLE:
            // flash LED slowly to signal IDLE state
            if (LED_timer > 1000) { 
                digitalWrite(LED_PIN, !digitalRead(LED_PIN));
                LED_timer = 0;
            }
        
            // power down motor and heaters
            digitalWrite(MOTOR_ENABLE_PIN, LOW);

            // check for run request
            if (askingToRun) {
                currentState = RUNNING;
                printCurrentState();
            }
            
            break;

        case RESET_REQUESTED:
            display_srcfile_details();
            // reset test steps
            currentStepIndex = 0;
            isStepInitialized = false;
            isPauseInitiated = false;
            currentState = IDLE;
            printCurrentState();

            break;

        case PAUSED:
            // Flash LED quickly to signal PAUSED state
            if (LED_timer > 100) {
                digitalWrite(LED_PIN, !digitalRead(LED_PIN));
                LED_timer = 0;
            }

            // upon entering a pause, call for a stop
            if (!isPauseInitiated) {
                stepper.stop();
                isPauseInitiated = true;
            }
            
            // Run until stopped, then disable motor
            stepper.run();
            if (stepper.distanceToGo() == 0) {
                digitalWrite(MOTOR_ENABLE_PIN, LOW);
                isFullyStopped = true;             
            }

            // Check if it's time to resume
            if (askingToRun && isFullyStopped) {
                currentState = RESUME;
                isPauseInitiated = false;
                printCurrentState();
            }

            break;

        case RESUME:
            Serial.println("\nResuming the following step:");
            debugStepInfo();

            // re-initialize common test settings
            digitalWrite(LED_PIN, HIGH);
            digitalWrite(MOTOR_ENABLE_PIN, HIGH);
            
            currentState = RUNNING;
            printCurrentState();
            
            // re- initialize test step
            stepper.setAcceleration(current_accel);
            stepper.setSpeed(current_speed);
            stepper.moveTo(target_position);
            dwell_timer = pause_timer;

            break;

        case RUNNING:
            // only perform these actions at start of test step
            if (!isStepInitialized) {
                Serial.print("Running:");
                debugStepInfo();
                digitalWrite(LED_PIN, HIGH);
                digitalWrite(MOTOR_ENABLE_PIN, HIGH);

                // Set accel and target speed for given step
                accel_steps_s2 = (steps[currentStepIndex].accel / 60.0) * steps_per_rev;
                stepper.setAcceleration(accel_steps_s2);
                target_speed_steps_s = (steps[currentStepIndex].target_speed / 60.0) * steps_per_rev;
                stepper.setMaxSpeed(target_speed_steps_s);

                // set target position based on direction of spin required from step
                target_position = steps[currentStepIndex].is_CCW ? -MAX_REVS : MAX_REVS;
                stepper.moveTo(target_position);
                
                // Reset flags for new step
                isStepInitialized = true;
                isTargetSpeedMet = false;
                isDwellOver = false;             
            }

            // Continuously run the stepper motor to ensure smooth movement
            stepper.run();

            // If target speed is zero, check if motor has stopped and handle dwell logic
            if (target_speed_steps_s == 0 && fabs(stepper.speed()) < 0.1 && !isTargetSpeedMet) {
                // If target speed is zero, the motor should be stopped
                digitalWrite(MOTOR_ENABLE_PIN, LOW);
                isTargetSpeedMet = true;
                dwell_timer = 0;
            }

            // Check if non-zero target speed has been reached and handle dwell logic
            if (!isTargetSpeedMet && fabs(stepper.speed()) >= 0.99 * target_speed_steps_s) {
                isTargetSpeedMet = true;
                dwell_timer = 0;
            }

            // If the dwell period is complete, mark the dwell as over
            if (isTargetSpeedMet && dwell_timer >= steps[currentStepIndex].dwell_time * 1000) {
                if (target_speed_steps_s != 0 && !isDwellOver) {
                    // If dwell is over and motor is still running then stop it
                    stepper.stop();
                }
                isDwellOver = true; 
            }

            // If dwell is over and the motor has reached a stop, move to the next step
            if (isDwellOver && (target_speed_steps_s == 0 || stepper.distanceToGo() == 0)) {
                isStepInitialized = false;
                currentStepIndex = (currentStepIndex + 1) % size_steps;
            }

            // Inform display controller of loop completion (after the last step)
            if (currentStepIndex == 0 && !isStepInitialized) {
                // Briefly pulse the loop bus pin high
                digitalWrite(LOOP_BUS_PIN, HIGH);
                delay(1);  
                digitalWrite(LOOP_BUS_PIN, LOW);
            }

            // Transition to PAUSED state if necessary
            if (!askingToRun) {
                currentState = PAUSED;
                pause_timer = dwell_timer;
                current_speed = stepper.speed();
                current_accel = accel_steps_s2;
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
    // if reset button toggled for 10 millis, then switch to reset requested state.
    if (duration >= 9900 && duration < 10100) {
        currentState = RESET_REQUESTED;
        printCurrentState();
    }
}

void run_rising() {
    attachInterrupt(digitalPinToInterrupt(PRGM_RUN_BUS_PIN), run_falling, FALLING);
    askingToRun = true;
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
    Serial.print(steps[currentStepIndex].dwell_time);
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