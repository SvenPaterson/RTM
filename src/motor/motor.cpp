// RTM Motor Controller Version - ClearCore
// Last Update: 05/06/25
// change log:
// 12/13/24: Fixed pause, resume and restart logic
// 01/23/25: Added ClearCore support
// 05/05/25: Added SD card support

#include "MotorController.h"
#include "ClearCore.h"
#include "ElapsedMillis.h"
#include "SPI.h"
#include "SD.h"

MotorController gCtrl;
// ─── SPI settings for the Newhaven LCD — 80 kHz, MODE3 ───
// const SPISettings MotorController::spiCfg_{ 80000, MSBFIRST, SPI_MODE3 };

static constexpr char SRC_FILE_VERSION[] = "Torque Stand v2025.5.6";


/* static constexpr uint8_t NUM_ROWS = 4;
static constexpr uint8_t NUM_COLS = 20;
char line1[NUM_COLS + 1] = "                    ";
char line2[NUM_COLS + 1] = "       BOOTING      ";
char line3[NUM_COLS + 1] = "       SYSTEM       ";
char line4[NUM_COLS + 1] = "                    ";
char msg[NUM_COLS + 1] =   "                    ";
static constexpr char BLANK_LINE[NUM_COLS + 1] = "                    ";

/******* SYSTEM STATE CONTROL *******/
/* enum SystemState {
DEBUG,  // debug mode: currentState = DEBUG
IDLE,   // normal mode: currentState = IDLE
RUNNING,
PAUSED,
RESET_REQUESTED,
RESUME,
COMPLETED,
E_STOP
};
SystemState currentState = IDLE;
SystemState prevState = currentState;
SystemState preResetState = IDLE;
bool askingToRun = false;
bool isFullyStopped = false;
bool isStepInitialized = false;
bool isPauseInitiated = false;
bool isTargetSpeedMet = false;
bool isEStop = false;
bool isCompleted = false;
uint16_t currentStepIndex = 0;
uint16_t prevStepIndex = 0;
uint16_t lastDisplayedSecond = 5;
uint64_t pause_time = 0;
elapsedMillis LED_timer, dwell_timer, reset_timer, debug_timer; */

/******* STEPPER MOTOR INIT *******/
/* #define motor ConnectorM0
static constexpr uint32_t MOTOR_MAX_VEL_RPM = 2760; // 2760rpm for CPM-SDHP-N0563A-ELN
static constexpr uint16_t STEPS_PER_REV = 3200;
// uint8_t torque_step_count = sizeof(torque_steps) / sizeof(torque_steps[0]);
int32_t target_speed_steps_s = 0;
uint64_t target_position = 0;
uint32_t accel_steps_s2 = 0;
int32_t current_speed = 0; 
uint32_t current_accel = 0;

// Struct to define each test step
String protocolName;
struct Step {
    int32_t target_speed; 
    uint32_t accel;        // Acceleration in RPM/sec
    uint32_t dwell_time;   // amount of time to dwell after target speed is reached in sec
};
static constexpr uint8_t MAX_PROTOCOL_STEPS = 50;
Step torque_steps[MAX_PROTOCOL_STEPS];
uint16_t torque_step_count = 0;
uint8_t loopCount = 1;  */

/******* FUNC DECLARATIONS *******/
void display_srcfile_details();

/* -----------------------------------------------------------------
 *  Transitional wrappers – call through to gCtrl
 * -----------------------------------------------------------------*/

/* void SetCursor(uint8_t r, uint8_t c) { gCtrl.setCursor(r, c); }
void ClearScreen() { gCtrl.clearScreen(); }
void RenderDisplay() { gCtrl.render(); }
void PadString(char *buf, size_t len = 20) { gCtrl.pad(buf); }   // default 20
void PrintCurrentState(const char *msg = "") { gCtrl.printCurrent(msg); }
void PrintAlerts() { gCtrl.printAlerts(); }
void debugTorqueStepInfo() { gCtrl.debugTorqueStepInfo(); } */

int main() {
    if (!gCtrl.begin()) {
        LED_PIN.State(true); // turn on LED
        while (true) { /* hang */}
        // need to allow user to reset the board
    };

    while (true) {
        gCtrl.tick(); 
    }
}
    /* 

    LED_timer = 0; 
} 
*/

/*     while (true) {


        // first check for a reset request
        // done
        askingToRun = runActive && (currentState != RESET_REQUESTED);

        // then check to see if test is complete
        if (loopCount == 0) {
            if (currentStepIndex == 0 && !isStepInitialized) {
                motor.MoveStopDecel(0);
                motor.EnableRequest(false);
                currentState = COMPLETED;
                PrintCurrentState();
            }
        }

        // proceed to run test
        switch (currentState) {
            case DEBUG:
                // anything here you need
                break;

            case COMPLETED:
                // show a message on LCD
                if (!isCompleted) {
                    PrintCurrentState();
                    RenderDisplay();
                    isCompleted = true;
                    //motor.EnableConnector
                }
                break;

            case E_STOP:
                // show a message on LCD
                if (!isEStop) {
                    sniprintf(msg, sizeof(msg), "Reset clears E-STOP");
                    PrintCurrentState(msg);
                    isEStop = true;
                }
        
                // If test is safe and user requests reset, then reset the system
                if (!isSafetyActive && resetActive) { 
                    Delay_ms(100); // debounce
                    ClearScreen();
                    Delay_ms(100);
                    SetCursor(0,0);
                    SPI.beginTransaction(spiConfig);
                    SPI.transfer("Resetting system... ", NULL, 20);
                    // now blank the other three rows:
                    SPI.transfer("                    ", NULL, 20);
                    SPI.transfer("                    ", NULL, 20);
                    SPI.transfer("                    ", NULL, 20);
                    SPI.endTransaction();
                    Delay_ms(2000);
                    SysMgr.ResetBoard();
                }
                break;
            
            case IDLE:
                // flash LED slowly to signal IDLE state
                if (LED_timer > 1000) {
                    LED_PIN.State(!LED_PIN.State());
                    LED_timer = 0;
                }
            
                // power down motor and heaters
                motor.EnableRequest(false);
                // MOTOR_ENABLE_PIN.State(false);

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
                    ClearScreen();
                    Delay_ms(100);
                    sprintf(line3, "Resetting system...");
                    SerialPort.SendLine(line3);
                    PadString(line3, 20);
                    RenderDisplay();
                    Delay_ms(2000);
                    SysMgr.ResetBoard();
                } else {
                    uint8_t remaining = 5 - (reset_timer / 1000);
                    if (remaining != lastDisplayedSecond) {
                        snprintf(msg, sizeof(msg), "Resetting in %d sec", remaining);
                        if (SerialPort) {
                            SerialPort.SendLine(msg);
                        }
                        PrintCurrentState(msg);

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
                    motor.MoveStopDecel((1000 / 60) * STEPS_PER_REV);
                    isPauseInitiated = true;
                }
                
                if (motor.StepsComplete()) {
                    // MOTOR_ENABLE_PIN.State(false);
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
                SerialPort.Send("\nResuming the following step:\r");
                debugTorqueStepInfo();

                // re-initialize common test settings
                LED_PIN.State(true);
                // MOTOR_ENABLE_PIN.State(true);
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
                    // MOTOR_ENABLE_PIN.State(true);
                    motor.EnableRequest(true);

                    // Calculate speed and accel in steps for given step
                    accel_steps_s2 = std::ceil((torque_steps[currentStepIndex].accel * STEPS_PER_REV) / 60.0);
                    motor.AccelMax(accel_steps_s2);
                    target_speed_steps_s = std::ceil((torque_steps[currentStepIndex].target_speed * STEPS_PER_REV) / 60.0);

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
                if (isTargetSpeedMet && dwell_timer >= torque_steps[currentStepIndex].dwell_time * 1000) {
                        isStepInitialized = false;
                        prevStepIndex = currentStepIndex;
                        currentStepIndex = (currentStepIndex + 1) % torque_step_count;
                        isTargetSpeedMet = false;
                        PrintCurrentState();

                        // Check if we are at the end of the test sequence
                        if (prevStepIndex == torque_step_count - 1 && currentStepIndex == 0) {
                            loopCount--;
                            if (loopCount == 0) {
                                currentState = COMPLETED;
                            }
                        }
                }

                // Transition to PAUSED state if necessary
                if (!askingToRun) {
                    currentState = PAUSED;
                    PrintCurrentState();
                    pause_time = dwell_timer;
                    current_speed = motor.VelocityRefCommanded();
                    current_accel = accel_steps_s2;
                }

                break;
        }
    }

return 0;
} */

///////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////Functions//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

#if 0
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
    SerialPort.Send("\t\tSpeed, rpm: ");
    SerialPort.Send(torque_steps[currentStepIndex].target_speed);  // Target speed
    SerialPort.Send("\t\tAccel, rpm/s: ");
    SerialPort.Send(torque_steps[currentStepIndex].accel);  // Acceleration
    SerialPort.Send("\t\tAdd'l Dwell Time, s: ");
    SerialPort.SendLine(torque_steps[currentStepIndex].dwell_time);  // Dwell time
}

void SetBrightness(uint8_t level) {
    SPI.beginTransaction(spiConfig);
    SPI.transfer(0xfe);
    SPI.transfer(0x53);
    SPI.transfer(level);
    SPI.endTransaction();
}

void PrintCurrentState(const char* msg_line) {
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
        case COMPLETED:
            stateStr = "COMPLETED";
            break;
        case E_STOP:
            stateStr = "E-STOP";
            break;
        default:
            stateStr = "UNKNOWN!";
            break;
    }
    
    // build the first line
    const char* rightStr = stateStr;
    size_t rightLen = strlen(rightStr);

    // truncate left side if too long
    size_t maxLeft = NUM_COLS - rightLen - 1;
    char leftBuf[NUM_COLS + 1];
    size_t protoLen = protocolName.length();
    if (protoLen > maxLeft) {
        protocolName.substring(0, maxLeft)
                    .toCharArray(leftBuf, maxLeft + 1);
    } else {
        protocolName.toCharArray(leftBuf, maxLeft + 1);
    }

    // compute length of left side
    size_t leftLen = strlen(leftBuf);

    // pad space between left and right sides
    int pad = NUM_COLS - leftLen - rightLen;
    if (pad < 1) pad = 1;

    // create the line
    snprintf(line1, sizeof(line1), "%s%*s%s", leftBuf, pad, "", rightStr);
    PadString(line1,NUM_COLS);

    // build the second line
    snprintf(line2, sizeof(line2), "%s", msg_line);
    PadString(line2,NUM_COLS);

    // build the third line
    snprintf(leftBuf, sizeof(leftBuf), "Remaining Loops:");
    leftLen = strlen(leftBuf);
    rightStr = String(loopCount).c_str();
    rightLen = strlen(rightStr);
    pad = NUM_COLS - leftLen - rightLen;
    if (pad < 1) pad = 1;
    snprintf(line3, sizeof(line3), "%s%*s%s", leftBuf, pad, "", rightStr);
    PadString(line3,NUM_COLS);

    // build the fourth line
    snprintf(leftBuf, sizeof(leftBuf), "Current step:");
    leftLen = strlen(leftBuf);
    rightStr = String(currentStepIndex + 1).c_str(); // Step index (1-based)
    rightLen = strlen(rightStr);
    pad = NUM_COLS - leftLen - rightLen;
    if (pad < 1) pad = 1;
    snprintf(line4, sizeof(line4), "%s%*s%s", leftBuf, pad, "", rightStr);
    PadString(line4,NUM_COLS);

    // print all lines to the display
    RenderDisplay();
}

void RenderDisplay() {
    SetCursor(0,0);
    SPI.beginTransaction(spiConfig);
      SPI.transfer(line1, NULL, 20);
      SPI.transfer(line3, NULL, 20);
      SPI.transfer(line2, NULL, 20);
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

void ClearLines() {
    snprintf(line1, sizeof(line1), "                    ");
    PadString(line1,20);
    snprintf(line2, sizeof(line2), "                    ");
    PadString(line2,20);
    snprintf(line3, sizeof(line3), "                    ");
    PadString(line3,20);
    snprintf(line4, sizeof(line4), "                    ");
    PadString(line4,20);
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

#endif