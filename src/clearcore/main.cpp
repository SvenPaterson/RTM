// RTM Motor Controller Version - ClearCore
// Last Update: 05/06/25
// change log:
// 12/13/24: Fixed pause, resume and restart logic
// 01/23/25: Added ClearCore support
// 05/05/25: Added SD card support
// 05/08/25: Converted logic to

#include "ClearCore-RTM.h"

ClearCoreRTM gCtrl;

static constexpr char SRC_FILE_VERSION[] = "Torque Stand v2025.5.6";

int main() {
    if (!gCtrl.begin()) {
        LED_PIN.State(true); // turn on LED
        while (true) { /* hang */}
        // need to allow user to reset the board
    };
    //gCtrl.torqueMode();

    while (true) {
        gCtrl.tick(); 
    }
}
