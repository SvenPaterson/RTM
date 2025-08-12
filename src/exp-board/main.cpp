#include <Arduino.h>
#include "ExpansionBoard.h"

ExpansionBoard board;

void setup() {
    board.begin();
}

void loop() {
    board.tick();
}