#include <Arduino.h>
#include "DisplayController.h"

DisplayController displayController;

void setup() {
    displayController.begin();
}

void loop() {
    displayController.tick();
}