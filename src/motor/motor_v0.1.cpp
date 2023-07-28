#include <Arduino.h>
#include <Wire.h>

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

elapsedMillis test, air;
String str = "test";
bool air_flip = true; 

void setup() {
  Serial.begin(112500);
  Wire.begin();
  pinMode(LOOP_BUS_PIN, OUTPUT);
  pinMode(PRGM_RUN_BUS_PIN, INPUT_PULLDOWN);
  pinMode(TORQ_FLAG_BUS_PIN, OUTPUT);
  pinMode(AIR_DUMP_BUS_PIN, OUTPUT);
  pinMode(AIR_SUPPLY_BUS_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (digitalRead(PRGM_RUN_BUS_PIN)) {
    if (test > 2000) {
    // simulate a 10 second loop
    digitalWrite(LOOP_BUS_PIN, HIGH);
    delay(1);
    digitalWrite(LOOP_BUS_PIN, LOW);
    test = 0;
    }
    if (test > 4000) {
      digitalWrite(TORQ_FLAG_BUS_PIN, HIGH);
      delay(10);
      digitalWrite(TORQ_FLAG_BUS_PIN, LOW);
    }
  }
  
  if (air > 1000) {
    air = 0;
    digitalWrite(AIR_SUPPLY_BUS_PIN, air_flip);
    air_flip = !air_flip;
    digitalWrite(AIR_DUMP_BUS_PIN, air_flip);
    digitalWrite(LED_BUILTIN, air_flip);
  }
  
}