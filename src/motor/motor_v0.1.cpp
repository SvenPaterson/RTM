#include "MotorController.h"

// define the board I/O pin numbers
#define LOOP_BUS_PIN 1
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

MotorController rtm;

elapsedMillis debugTimer, profileTimer;

void setup() {
  // pin mappings
  std::map<String, uint8_t> pinMappings = {
    {"LOOP_BUS_PIN", LOOP_BUS_PIN},
    {"MOTOR_STEP_PIN", MOTOR_STEP_PIN},
    {"MOTOR_DIR_PIN", MOTOR_DIR_PIN},
    {"MOTOR_ENABLE_PIN", MOTOR_ENABLE_PIN},
    {"PRGM_RUN_BUS_PIN", PRGM_RUN_BUS_PIN},
    {"PRGM_RESET_BUS_PIN", PRGM_RUN_BUS_PIN}, 
    {"TORQ_FLAG_BUS_PIN", TORQ_FLAG_BUS_PIN}, 
    {"AIR_SUPPLY_BUS_PIN", AIR_SUPPLY_BUS_PIN}, 
    {"AIR_DUMP_BUS_PIN", AIR_DUMP_BUS_PIN}, 
    {"SDA0_PIN", SDA0_PIN}, 
    {"SDL0_PIN", SDL0_PIN},
  };

  pinMode(LOOP_BUS_PIN, OUTPUT);
  
  rtm.begin(pinMappings);
  rtm.setMaxSpeed(60.0);
  rtm.setSpeed(30.0);
  rtm.enableMotor();
}

void loop() {

  if (digitalRead(PRGM_RUN_BUS_PIN)) {
    if (profileTimer < 4000) {
      rtm.moveTo(-0.5);
      rtm.setSpeed(30.0);
    }
    else if (profileTimer > 4000 && profileTimer < 8000) {
      rtm.moveTo(0.5);
      rtm.setSpeed(30.0);
    }
    else {
      // simulate a 10 second loop
      digitalWrite(LOOP_BUS_PIN, HIGH);
      delay(1);
      digitalWrite(LOOP_BUS_PIN, LOW);
      profileTimer = 0;
    }
  }  
}