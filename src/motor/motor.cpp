#include <Arduino.h>

// define the board I/O pin numbers
#define LOOP_BUS_PIN 1
#define MOTOR_HLFB_PIN 2
#define MOTOR_STEP_PIN 3
#define MOTOR_DIR_PIN 4
#define MOTOR_ENABLE_PIN 5
#define PRGM_RUN_BUS_PIN 10
#define TORQ_FLAG_BUS_PIN 11
#define AIR_SUPPLY_BUS_PIN 14
#define AIR_DUMP_BUS_PIN 15
#define SDA0_PIN 18
#define SDL0_PIN 19
#define RESET_BUS_PIN 20

elapsedMillis test;

void setup() {
  test = 0;
}

void loop() {
  if (test % 100) {
    Serial.println('test');
  }
}