// RTM Motor Controller Version - ClearCore
// Last Update: 05/06/25
// change log:
// 12/13/24: Fixed pause, resume and restart logic
// 01/23/25: Added ClearCore support
// 05/05/25: Added SD card support

#include "MotorController.h"

MotorController gCtrl;

static constexpr char SRC_FILE_VERSION[] = "Torque Stand v2025.5.6";

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
    
/*     while (true) {

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

        }
    }

return 0;
} */
