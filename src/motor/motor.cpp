#include "motor_config.h"
#include "ClearCore.h"
#include "ElapsedMillis.h"
#include "SPI.h"
#include "SD.h"

// RTM Motor Controller Version - ClearCore
// Last Update: 01/23/25
// change log:
// 12/13/24: Fixed pause, resume and restart logic
// 01/23/25: Added ClearCore support

#define SRC_FILE_VERSION "Torque Stand ClearCore v0.1"

SPISettings spiConfig(80000, MSBFIRST, SPI_MODE3);
#define NUM_ROWS 4
#define NUM_COLS 20
const uint8_t line1[21] = "Torque Stand Test   ";
const uint8_t line2[21] = "                    ";
char          line3[21] = "                    ";
char          line4[21] = "                    ";

File myFile;

/******* I/O PINS *******/
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
uint64_t pause_time = 0;
elapsedMillis LED_timer, dwell_timer, reset_timer;

/******* STEPPER MOTOR INIT *******/
#define motor ConnectorM0
uint16_t steps_per_rev = SPR;
uint8_t size_torque_steps = sizeof(torque_steps) / sizeof(torque_steps[0]);
int32_t target_speed_steps_s = 0;
uint64_t target_position = 0;
uint32_t accel_steps_s2 = 0;
int32_t current_speed = 0; 
uint32_t current_accel = 0;

/******* FUNC DECLARATIONS *******/
void display_srcfile_details();
void debugTorqueStepInfo();
void PrintCurrentState();
void PrintAlerts();
void SetBrightness(uint8_t level);
void SetCursor(uint8_t row, uint8_t col);
void ClearScreen();
void PadString(char *str, size_t length);

int main() {
    PRGM_RUN_BUS_PIN.Mode(Connector::INPUT_DIGITAL); // docs suggest that pullup is default
    PRGM_RESET_BUS_PIN.Mode(Connector::INPUT_DIGITAL); // docs suggest that pullup is default
    MOTOR_ENABLE_PIN.Mode(Connector::OUTPUT_DIGITAL);
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
    Delay_ms(1000);
    SerialPort.Send("Serial port connected\r\n");

    // Initialize stepper motor
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
    MotorMgr.MotorModeSet(MotorManager::MOTOR_M0M1,
                          Connector::CPM_MODE_STEP_AND_DIR);
    motor.HlfbMode(MotorDriver::HLFB_MODE_STATIC);
    motor.VelMax(MOTOR_MAX_VEL_RPM * steps_per_rev / 60);
    motor.AccelMax(MOTOR_MAX_VEL_RPM * steps_per_rev / 6);

    // Initialize display
    SPI.begin();
    ClearScreen();
    SetBrightness(4);
    SetCursor(0, 0);
    SPI.beginTransaction(spiConfig);
    // Send lines "out of order" to display them in the correct order
    // without resetting the cursor position for each line, this is the
    // order in which lines must be sent to be displayed correctly
    SPI.transfer(line1, NULL, 20);
    SPI.transfer(line3, NULL, 20);
    SPI.transfer(line2, NULL, 20);
    SPI.transfer(line4, NULL, 20);
    SPI.endTransaction();

    display_srcfile_details();

    LED_timer = 0;
    PrintCurrentState();

    SerialPort.SendLine("Initializing SD Card...");
    if (!SD.begin()) {
        SerialPort.SendLine("SD Card initialization failed!");
        while (true) {
            continue;
        }
    }
    SerialPort.SendLine("SD Card initialized successfully!");
    /* myFile = SD.open("test.txt", FILE_WRITE);
    if (myFile) {
        myFile.println("Testing 1, 2, 3...");
        myFile.close();
        SerialPort.SendLine("File written successfully!");
    } else {
        SerialPort.SendLine("Error opening file!");
    } */

    unsigned long lastDebugTime = 0; // Tracks the last time the debug message was sent
    const unsigned long debugInterval = 500; // Interval in milliseconds for debug messages

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
                SerialPort.SendLine("Resetting in 5 seconds...");
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
                    PrintCurrentState();
                }
                
                break;

            case RESET_REQUESTED:
                if (!resetActive) {
                    SerialPort.Send("Reset cancelled\r\n");
                    currentState = preResetState;
                    PrintCurrentState();
                } else if (reset_timer >= 5000) {
                    sprintf(line3, "Resetting system...");
                    SerialPort.SendLine(line3);

                    // Send message to display here
                    PadString(line3, 20);
                    SetCursor(0, 0);
                    SPI.beginTransaction(spiConfig);
                    SPI.transfer(line3, NULL, 20); // First line
                    SPI.transfer("                    ", NULL, 20); // Third line
                    SPI.transfer("                    ", NULL, 20); // Second line
                    SPI.transfer("                    ", NULL, 20); // Fourth line
                    SPI.endTransaction();
                    Delay_ms(2000);
                    SysMgr.ResetBoard();
                } else {
                    uint8_t remaining = 5 - (reset_timer / 1000);
                    if (remaining != lastDisplayedSecond) {
                        snprintf(line3, sizeof(line3), "Resetting in %d sec", remaining);
                        PadString(line3, 20);
                        if (SerialPort) {
                            SerialPort.SendLine(line3);
                        }
                        // Update the display
                        SetCursor(0, 0);
                        SPI.beginTransaction(spiConfig);
                        SPI.transfer(line1, NULL, 20);
                        SPI.transfer(line2, NULL, 20);
                        SPI.transfer(line3, NULL, 20); // Display the updated line3
                        SPI.transfer(line4, NULL, 20);
                        SPI.endTransaction();

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
                    PrintCurrentState();
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
                    PrintCurrentState();
                }

                break;

            case RESUME:
                SerialPort.Send("\nResuming the following step:\r\n");
                debugTorqueStepInfo();

                // re-initialize common test settings
                LED_PIN.State(true);
                MOTOR_ENABLE_PIN.State(true);
                motor.EnableRequest(true);
                
                currentState = RUNNING;
                PrintCurrentState();
                
                // re- initialize test step
                motor.AccelMax(current_accel);
                motor.MoveVelocity(current_speed);
                dwell_timer = pause_time;

                break;

            case RUNNING:
                // only perform these actions at start of test step
                if (!isStepInitialized) {
                    debugTorqueStepInfo();
                    PrintCurrentState();
                    LED_PIN.State(true);
                    MOTOR_ENABLE_PIN.State(true);
                    motor.EnableRequest(true);

                    // Calculate speed and accel in steps for given step
                    accel_steps_s2 = std::ceil((torque_steps[currentStepIndex].accel * steps_per_rev) / 60.0);
                    motor.AccelMax(accel_steps_s2);
                    target_speed_steps_s = std::ceil((torque_steps[currentStepIndex].target_speed * steps_per_rev) / 60.0);
                    
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

                if (Milliseconds() - lastDebugTime >= debugInterval) {
                    SerialPort.Send("Dwell timer: ");
                    SerialPort.SendLine(dwell_timer);
                    // update last debug time
                    lastDebugTime = Milliseconds();
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
                if (isTargetSpeedMet && dwell_timer >= torque_steps[currentStepIndex].dwell_time * 1000) {
                        isStepInitialized = false;
                        currentStepIndex = (currentStepIndex + 1) % size_torque_steps;
                        isTargetSpeedMet = false;
                }

                // Transition to PAUSED state if necessary
                if (!askingToRun) {
                    currentState = PAUSED;
                    PrintCurrentState();
                    pause_time = dwell_timer;
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

void debugTorqueStepInfo() {
    SerialPort.Send("\nStep ");
    SerialPort.Send(currentStepIndex + 1);  // Step index (1-based)
    SerialPort.Send("\tTime, s: ");
    SerialPort.Send(torque_steps[currentStepIndex].dwell_time);  // Dwell time
    SerialPort.Send("\tSpeed, rpm: ");
    SerialPort.Send(torque_steps[currentStepIndex].target_speed);  // Target speed
    SerialPort.Send("\tAccel, rpm/s: ");
    SerialPort.Send(torque_steps[currentStepIndex].accel);  // Acceleration
    SerialPort.Send("\r\n");  // Newline at the end
}

void SetBrightness(uint8_t level) {
    SPI.beginTransaction(spiConfig);
    SPI.transfer(0xfe);
    SPI.transfer(0x53);
    SPI.transfer(level);
    SPI.endTransaction();
}

void PrintCurrentState() {
    const char* stateStr;
    switch (currentState) {
        case DEBUG:
            stateStr = "DEBUG";
            break;
        case IDLE:
            stateStr = "IDLE";
            break;
        case RUNNING:
            stateStr = "RUNNING";
            break;
        case PAUSED:
            stateStr = "PAUSED";
            break;
        case RESET_REQUESTED:
            stateStr = "RESETTING";
            break;
        case RESUME:
            stateStr = "RESUME";
            break;
        default:
            stateStr = "UNKNOWN!";
            break;
    }
    
    snprintf(line3, sizeof(line3), "Status: %-12s", stateStr);
    snprintf(line4, sizeof(line4), "Current step:%7d", currentStepIndex + 1);
    if (SerialPort) {
        SerialPort.SendLine(line3);
    }

    SetCursor(0, 0);
    SPI.beginTransaction(spiConfig);
    SPI.transfer(line1, NULL, 20);
    SPI.transfer(line2, NULL, 20);
    SPI.transfer(line3, NULL, 20);
    SPI.transfer(line4, NULL, 20);
    SPI.endTransaction();
}

void SetCursor (uint8_t row, uint8_t col) {
    if (row >= NUM_ROWS) {
        row = 0;
    }
    if (col >= NUM_COLS) {
        col = 0;
    }
    uint8_t position = row * NUM_COLS + col;
    SPI.beginTransaction(spiConfig);
    SPI.transfer(0xfe);
    SPI.transfer(0x45);
    SPI.transfer(position);
    SPI.endTransaction();
}

void ClearScreen() {
    SPI.beginTransaction(spiConfig);
    SPI.transfer(0xfe);
    SPI.transfer(0x51);
    SPI.endTransaction();
}

void PadString(char *str, size_t length) {
    size_t strLen = strlen(str);

    if (strLen < length) {
        for (size_t i = strLen; i < length; i++) {
            str[i] = ' '; // Add spaces
        }
    }

    str[length] = '\0'; // Ensure the string is null-terminated
}
