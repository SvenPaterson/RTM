#include "motor_config.h"
#include "ClearCore.h"
#include "ElapsedMillis.h"

// RTM Motor Controller Version - ClearCore
// Last Update: 01/23/25
// change log:
// 12/13/24: Fixed pause, resume and restart logic
// 01/23/25: Added ClearCore support

#define SRC_FILE_VERSION "Torque Stand ClearCore v0.1"

/******* I/O PINS *******/
#define LOOP_BUS_PIN ConnectorIO1
#define PRGM_RUN_BUS_PIN ConnectorDI6
#define LED_PIN ConnectorIO0
#define MOTOR_ENABLE_PIN ConnectorIO2
#define PRGM_RESET_BUS_PIN ConnectorDI7
#define SerialPort ConnectorUsb

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
SystemState preResetState = IDLE;
bool askingToRun = false;
bool isFullyStopped = false;
bool isStepInitialized = false;
bool isPauseInitiated = false;
bool isTargetSpeedMet = false;
uint16_t currentStepIndex = 0;
uint16_t lastDisplayedSecond = 5;
elapsedMillis LED_timer, dwell_timer, pause_timer, reset_timer;

/******* STEPPER MOTOR INIT *******/
#define motor ConnectorM0
uint16_t steps_per_rev = SPR;
uint8_t size_steps = sizeof(steps) / sizeof(steps[0]);
int32_t target_speed_steps_s = 0;
uint64_t target_position = 0;
uint32_t accel_steps_s2 = 0;
int32_t current_speed = 0; 
uint32_t current_accel = 0;

/******* FUNC DECLARATIONS *******/
void display_srcfile_details();
void debugStepInfo();
void printCurrentState();
void PrintAlerts();

int main() {
    PRGM_RUN_BUS_PIN.Mode(Connector::INPUT_DIGITAL); // docs suggest that pullup is default
    PRGM_RESET_BUS_PIN.Mode(Connector::INPUT_DIGITAL); // docs suggest that pullup is default
    MOTOR_ENABLE_PIN.Mode(Connector::OUTPUT_DIGITAL);
    LOOP_BUS_PIN.Mode(Connector::OUTPUT_DIGITAL);
    LOOP_BUS_PIN.State(false);
    LED_PIN.Mode(Connector::OUTPUT_DIGITAL);
    LED_PIN.State(true);

    SerialPort.Mode(Connector::USB_CDC);
    SerialPort.Speed(9600);
    uint32_t timeout = 5000;
    uint32_t startTime = Milliseconds();
    SerialPort.PortOpen();
    while (!SerialPort && Milliseconds() - startTime < timeout) {
        // wait for serial port to connect. Needed for native USB port only
        continue;
    }
    SerialPort.Send("Serial port connected\r\n");

    // Initialize stepper motor
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
    MotorMgr.MotorModeSet(MotorManager::MOTOR_M0M1,
                          Connector::CPM_MODE_STEP_AND_DIR);
    motor.HlfbMode(MotorDriver::HLFB_MODE_STATIC);
    motor.VelMax(MOTOR_MAX_VEL_RPM * steps_per_rev / 60);
    motor.AccelMax(MOTOR_MAX_VEL_RPM * steps_per_rev / 6);

    Delay_ms(2000);
    display_srcfile_details();

    LED_timer = 0;
    printCurrentState();

    SerialPort.Send("Size of steps array: ");
    SerialPort.Send(size_steps);
    SerialPort.Send("\r\n");

    while (true) {
        bool runActive = PRGM_RUN_BUS_PIN.State();
        bool resetActive = PRGM_RESET_BUS_PIN.State();
        static bool prevResetActive = false;

        if (resetActive && !prevResetActive) {  // Rising edge
            if (currentState != RUNNING) {      // Only allow reset from non-running states
                preResetState = currentState;   // Remember previous state
                currentState = RESET_REQUESTED;
                reset_timer = 0;
                lastDisplayedSecond = 5;
                LED_PIN.State(true);            // Solid LED during countdown
                SerialPort.Send("RESETTING in 5 seconds...\r\n");
            }
        }
        prevResetActive = resetActive;
        askingToRun = runActive && (currentState != RESET_REQUESTED);

        switch (currentState) {
            case DEBUG:
                // anything here you need
                break;
            
            case IDLE:
                // flash LED slowly to signal IDLE state
                if (LED_timer > 1000) {
                    LED_PIN.State(!LED_PIN.State());
                    LED_timer = 0;
                }
            
                // power down motor and heaters
                motor.EnableRequest(false);
                MOTOR_ENABLE_PIN.State(false);

                // check for run request
                if (askingToRun) {
                    currentState = RUNNING;
                    printCurrentState();
                }
                
                break;

            case RESET_REQUESTED:
                if (!resetActive) {
                    SerialPort.Send("Reset cancelled\r\n");
                    currentState = preResetState;
                    printCurrentState();
                } else if (reset_timer >= 5000) {
                    SerialPort.Send("Performing a full reset...\r\n");
                    Delay_ms(100);
                    SysMgr.ResetBoard();
                } else {
                    uint8_t remaining = 5 - (reset_timer / 1000);
                    if (remaining != lastDisplayedSecond) {
                        SerialPort.Send("Resetting in ");
                        SerialPort.Send(remaining);
                        SerialPort.Send(" seconds...\r\n");
                        lastDisplayedSecond = remaining;
                    }
                    // Blink LED rapidly during countdown
                    if (LED_timer > 100) {
                        LED_PIN.State(!LED_PIN.State());
                    }
                }
                break;

            case PAUSED:
                // Flash LED quickly to signal PAUSED state
                if (LED_timer > 250) {
                    LED_PIN.State(!LED_PIN.State());
                    LED_timer = 0;
                }

                // upon entering a pause, call for a stop
                if (!isPauseInitiated) {
                    motor.MoveStopDecel((1000 / 60) * steps_per_rev);
                    isPauseInitiated = true;
                }
                
                if (motor.StepsComplete()) {
                    MOTOR_ENABLE_PIN.State(false);
                    motor.EnableRequest(false);
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
                SerialPort.Send("\nResuming the following step:\r\n");
                debugStepInfo();

                // re-initialize common test settings
                LED_PIN.State(true);
                MOTOR_ENABLE_PIN.State(true);
                motor.EnableRequest(true);
                
                currentState = RUNNING;
                printCurrentState();
                
                // re- initialize test step
                motor.AccelMax(current_accel);
                motor.MoveVelocity(current_speed);
                dwell_timer = pause_timer;

                break;

            case RUNNING:
                // only perform these actions at start of test step
                if (!isStepInitialized) {
                    debugStepInfo();
                    LED_PIN.State(true);
                    MOTOR_ENABLE_PIN.State(true);
                    motor.EnableRequest(true);

                    // Calculate speed and accel in steps for given step
                    accel_steps_s2 = std::ceil((steps[currentStepIndex].accel * steps_per_rev) / 60.0);
                    motor.AccelMax(accel_steps_s2);
                    target_speed_steps_s = std::ceil((steps[currentStepIndex].target_speed * steps_per_rev) / 60.0);
                    
                    SerialPort.Send("Accel steps/s^2: ");
                    SerialPort.Send(accel_steps_s2);
                    SerialPort.Send(" Target speed steps/s: ");
                    SerialPort.SendLine(target_speed_steps_s);

                    // In RUNNING state, after move command:
                    if (motor.StatusReg().bit.AlertsPresent) {
                        SerialPort.Send("Motor alert: ");
                        PrintAlerts();
                    }
                    // Reset flags for new step
                    isStepInitialized = true;
                    isTargetSpeedMet = false;

                    motor.MoveVelocity(target_speed_steps_s);
                    if (target_speed_steps_s == 0) {
                        motor.MoveStopDecel(accel_steps_s2);
                    }

                }

                if (!isTargetSpeedMet) {
                    dwell_timer = 0;
                    // For non-zero targets: check speed reached
                    if (target_speed_steps_s != 0 && 
                        fabs(motor.VelocityRefCommanded()) >= fabs(0.99 * target_speed_steps_s)) {
                        isTargetSpeedMet = true;
                    }
                    // For zero targets: check full stop
                    else if (target_speed_steps_s == 0 && motor.StepsComplete()) {
                        isTargetSpeedMet = true;
                    }
                }

                // Handle dwell timing and step advancement
                if (isTargetSpeedMet && dwell_timer >= steps[currentStepIndex].dwell_time * 1000) {
                        isStepInitialized = false;
                        currentStepIndex = (currentStepIndex + 1) % size_steps;
                        isTargetSpeedMet = false;
                }

                // Transition to PAUSED state if necessary
                if (!askingToRun) {
                    currentState = PAUSED;
                    printCurrentState();
                    pause_timer = dwell_timer;
                    current_speed = motor.VelocityRefCommanded();
                    current_accel = accel_steps_s2;
                }

                /*  // Inform display controller of loop completion (after the last step)
                if (currentStepIndex == 0 && !isStepInitialized) {
                    // Briefly pulse the loop bus pin high
                    LOOP_BUS_PIN.State(true);
                    Delay_ms(1); 
                    LOOP_BUS_PIN.State(false);
                } */

                break;
        }
    }

return 0;
}

///////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////Functions//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

void display_srcfile_details(void) {
    char buffer[128]; // Adjust size as needed
    
    snprintf(buffer, sizeof(buffer),
        "%s  Compiled on: %s at %s\r\n",
        SRC_FILE_VERSION, __DATE__, __TIME__
    );
    
    if (SerialPort) {  // Only send if connected
        SerialPort.Send(buffer);
    }
}

void PrintAlerts(){
    // report status of alerts
    SerialPort.SendLine("Alerts present: ");
    if(motor.AlertReg().bit.MotionCanceledInAlert){
        SerialPort.SendLine("    MotionCanceledInAlert "); }
    if(motor.AlertReg().bit.MotionCanceledPositiveLimit){
        SerialPort.SendLine("    MotionCanceledPositiveLimit "); }
    if(motor.AlertReg().bit.MotionCanceledNegativeLimit){
        SerialPort.SendLine("    MotionCanceledNegativeLimit "); }
    if(motor.AlertReg().bit.MotionCanceledSensorEStop){
        SerialPort.SendLine("    MotionCanceledSensorEStop "); }
    if(motor.AlertReg().bit.MotionCanceledMotorDisabled){
        SerialPort.SendLine("    MotionCanceledMotorDisabled "); }
    if(motor.AlertReg().bit.MotorFaulted){
        SerialPort.SendLine("    MotorFaulted ");
    }
 }

void debugStepInfo() {
    SerialPort.Send("\nStep ");
    SerialPort.Send(currentStepIndex + 1);  // Step index (1-based)
    SerialPort.Send("\tTime, s: ");
    SerialPort.Send(steps[currentStepIndex].dwell_time);  // Dwell time
    SerialPort.Send("\tSpeed, rpm: ");
    SerialPort.Send(steps[currentStepIndex].target_speed);  // Target speed
    SerialPort.Send("\tAccel, rpm/s: ");
    SerialPort.Send(steps[currentStepIndex].accel);  // Acceleration
    SerialPort.Send("\r\n");  // Newline at the end
}

void printCurrentState() {
    if (!SerialPort) return;  // Only send if USB is connected

    const char *stateStr;
    switch (currentState) {
        case DEBUG:
            stateStr = "Current State: DEBUG\r\n";
            break;
        case IDLE:
            stateStr = "Current State: IDLE\r\n";
            break;
        case RUNNING:
            stateStr = "Current State: RUNNING\r\n";
            break;
        case PAUSED:
            stateStr = "Current State: PAUSED\r\n";
            break;
        case RESET_REQUESTED:
            stateStr = "Current State: RESET REQUESTED\r\n";
            break;
        case RESUME:
            stateStr = "Current State: RESUME\r\n";
            break;
        default:
            stateStr = "Unknown State\r\n";
            break;
    }
    
    SerialPort.Send(stateStr);
}