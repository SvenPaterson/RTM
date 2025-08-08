#include <Arduino.h>
#include "ExpansionBoard.h"

ExpansionBoard board;

void setup() {
    Serial.begin(9600);
    if (!board.begin()) {
        Serial.println("FATAL: Display Controller initialization failed!");
    } 
}

void loop() {
    board.tick();
}