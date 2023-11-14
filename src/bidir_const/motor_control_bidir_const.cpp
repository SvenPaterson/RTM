#include <Wire.h>
#include <FlexyStepper.h>
#include <AccelStepper.h>
#include <avr/pgmspace.h>
#include <Bounce2.h>

/*
* Motor controller source code will run test spec for 
* Harley Davidson constant speed test
*/
//Interupt variables
volatile unsigned long start_micros = 0.0;
volatile unsigned long end_micros = 0;
volatile unsigned long duration = 0;
volatile bool Resetbool = false;

const uint8_t LOOP_BUS_PIN = 1;  // This Pin is used to keeps track of the loops compleated by the motor controller.
const uint8_t MOTOR_STEP_PIN = 3;
const uint8_t MOTOR_DIRECTION_PIN = 4;
const uint8_t MOTOR_EN_PIN = 5;        // This is the pin that enables the motor.
const uint8_t PRESS_SUPPLY_FLAG_PIN = 14;        // the number of the PRESS_SUPPLY_FLAG_PIN pin
const uint8_t PRESS_DUMP_FLAG_PIN = 15 ;        // the number of the PRESS_SUPPLY_FLAG_PIN pin
const uint8_t PRGM_FLAG_BUS_PIN = 10; //  Pin That tell the to run the motor program.
const uint8_t TORQUE_FLAG_PIN = 11; //Pin That tell the to run the motor program.
const uint8_t LEDPIN = 13;
const uint8_t PGM_RESET_BUS_PIN = 20;  // This Pin is used
const uint8_t HEAT_FLAG_BUS_PIN = 12;

// Define a stepper and the pins it will use
FlexyStepper stepper;
// AccelStepper stepper = AccelStepper(AccelStepper::DRIVER, 3, 4);
#define HIGH_SPEED_MOTOR 3
#define STD_SPEED_MOTOR 1
#define MOTOR_PULSES 800 // as set in the ClearPath MSP software, per motor basis
uint16_t PPR = (MOTOR_PULSES / STD_SPEED_MOTOR);  // pulses per rev for motor

/****** TEST STEP PARAMS ******/
/* struct Step {
  bool run;           // only set false if the motor is already stopped
  bool heat;          // true for requesting heat, false for none
  uint16_t speed;     // rpm
  double accel;       // rpm per second
  float revs;         // target # of revolutions (-2,147,483,648 revs for "infinite" / constant speed)
  uint32_t time;      // milliseconds
};

Step steps[] = { // RTM TEST PROTOCOL (active high EN pin)
  {true,  true,   2000,   1000,      -960000,     (8*3600-3)*1000},
  {true,  true,   0,      1000/2,    -3*2000/60,           3*1000},
  {true,  true,   4500,   4500/3,    540000,      (2*3600-3)*1000},
  {true,  true,   0,      4500/3,    3*4500/60,            3*1000},
  {false, true,   0,      1000,      0,               2*3600*1000}
}; */

struct Step { // HARLEY OSCILLATORY TEST
  bool run;           // only set false if the motor is already stopped
  bool heat;          // true for requesting heat, false for none
  uint16_t speed;     // rpm
  double accel;       // rpm per second
  float revs;         // target # of revolutions (-2,147,483,648 revs for "infinite" / constant speed)
};

Step steps[] = {
  {true, true, 100, 955, 15.0/360.0},
  {true, true, 100, 955, -15.0/360.0},
  {true, true, 100, 955, 0}
};

/* Step steps[] = { // RTM TEST DEMO TIMES (active high EN pin)
  {true,  true,   2000,   1000,    960000,         10*1000},
  {true,  true,   0,      1000/2,    3*2000/60,     3*1000},
  {true,  true,   4500,   4500/3,    -540000,        10*1000},
  {true,  true,   0,      4500/3,    3*4500/60,       3*1000},
  {false, true,   0,      1000,    0,               5*1000}
}; */

/* Step steps[] = {  // TEST BENCH SERVO SPEEDS (200ppr & active low EN pin)
  {true,  true,   400,    400,             960000,          10*1000},
  {true,  true,   0,      400/2,              3*2000/60,       3*1000},
  {true,  true,   800,    800/3,              -540000,         10*1000},
  {true,  true,   0,      800/3,              3*4500/60,       3*1000},
  {false, true,   0,      400,                0,                 5000}
}; */

bool break_loop, pause_requested;
uint32_t sum_time = 0;
elapsedMillis loop_time, hour_timer;

/******* FUNC DECLARATIONS *******/
void display_srcfile_details();
void rising();
void falling();
int pgm_lastIndexOf(uint8_t c, const char * p);

void setup() ////////////////////////////////////////////////////////////Setup ////////////////////////////////////////////////////////////////
{
  Serial.begin(115200);

  pinMode(PRGM_FLAG_BUS_PIN, INPUT_PULLDOWN); // This pin get the signel from the disply to run the programe.
  pinMode(PGM_RESET_BUS_PIN, INPUT_PULLDOWN); // This pin get the signel from the disply to run the programe.
  pinMode(LEDPIN, OUTPUT);               
  pinMode(PRESS_SUPPLY_FLAG_PIN, OUTPUT); 
  pinMode(PRESS_DUMP_FLAG_PIN, OUTPUT);
  pinMode(MOTOR_EN_PIN, OUTPUT);  
  pinMode(MOTOR_STEP_PIN, OUTPUT);  
  pinMode(MOTOR_DIRECTION_PIN, OUTPUT);  
  pinMode(LOOP_BUS_PIN, OUTPUT);  
  pinMode(TORQUE_FLAG_PIN, OUTPUT);  
  pinMode(HEAT_FLAG_BUS_PIN, OUTPUT);

  digitalWrite(HEAT_FLAG_BUS_PIN, HIGH); // always be setup to pre-heat
  digitalWrite(MOTOR_EN_PIN, LOW);
  digitalWrite(PRESS_SUPPLY_FLAG_PIN, LOW);
  digitalWrite(PRESS_DUMP_FLAG_PIN, LOW);
  digitalWrite(LEDPIN, HIGH);
  digitalWrite(LOOP_BUS_PIN, LOW);
  digitalWrite(TORQUE_FLAG_PIN, LOW);

  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
  stepper.setStepsPerRevolution(PPR);


  Wire.begin(); // start the I2C bus
  Wire.setClock(100000); //Optional - set I2C SCL to High Speed Mode of 100kHz

  delay(2000);
  display_srcfile_details();

  attachInterrupt(digitalPinToInterrupt(PGM_RESET_BUS_PIN), rising, RISING); // Enable the interupt.
}

void loop() {
  if (!digitalRead(PRGM_FLAG_BUS_PIN)) {
    digitalWrite (LEDPIN, LOW);
    digitalWrite (MOTOR_EN_PIN, LOW); // note: test bench is active LOW
    hour_timer = 0;
    delay(100);
    if (Resetbool == HIGH) {
      Resetbool = LOW;
      display_srcfile_details();
    }
  } else {
    digitalWrite (LEDPIN, HIGH);
    digitalWrite (MOTOR_EN_PIN, HIGH);
    
    break_loop = false;
    pause_requested = false;

    // loop through test steps array, changing accel, speed each time.
    stepper.setCurrentPositionInRevolutions(0);
    for (uint16_t i = 0; i < sizeof(steps) / sizeof(steps[0]) && !break_loop 
         && digitalRead(PRGM_FLAG_BUS_PIN); i++) {
      loop_time = 0;  // Reset loop_time for each step
      sum_time = 0;   // Reset sum_time for each step
      
      digitalWrite(LEDPIN, !digitalRead(LEDPIN));
      // stepper.setCurrentPositionInSteps(0); // comment out for oscillatory
      // stepper.setTargetPositionInRevolutions(steps[i].revs); // comment out for oscillatory
      stepper.setAccelerationInRevolutionsPerSecondPerSecond(steps[i].accel / 60);
      stepper.setSpeedInRevolutionsPerSecond(steps[i].speed / 60);
      

      // HARLEY OSCILLATORY CODE
      digitalWrite(HEAT_FLAG_BUS_PIN, steps[i].heat);
      stepper.moveToPositionInRevolutions(steps[i].revs);  // BLOCKING CODE!!! WATCH OUT...
      if (!digitalRead(PRGM_FLAG_BUS_PIN)) {
        stepper.setTargetPositionToStop();
        while (!stepper.processMovement());
        i = 0;
        break_loop = true;
        break;
      }

      /* // HARLEY CONSTANT SPEED CODE 
      sum_time += steps[i].time;
      while (loop_time < sum_time) {

        // if stop/reset called, spin down motor and restart loop
        if (!digitalRead(PRGM_FLAG_BUS_PIN)) {
          stepper.setTargetPositionToStop();
          while (!stepper.processMovement());
          i = 0;
          break_loop = true;
          break;
        }

        else if (steps[i].run) {
          digitalWrite(MOTOR_EN_PIN, HIGH);
          stepper.processMovement();
        }
        
        else {
          // stops motor spinning during a dwell step
          stepper.setSpeedInRevolutionsPerSecond(0);
          digitalWrite(MOTOR_EN_PIN, LOW);
        } */
      }
    }

      
   if (hour_timer > 3600000) { // comment out for constant speed oscillatory 
    if (digitalRead(PRGM_FLAG_BUS_PIN)) {
      digitalWrite (LOOP_BUS_PIN, HIGH);
      delay(1);
      digitalWrite (LOOP_BUS_PIN, LOW);
    }
    hour_timer = 0;
   }

  }
//}



/////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////Functions/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
void rising() {
  attachInterrupt(digitalPinToInterrupt(PGM_RESET_BUS_PIN), falling, FALLING);
  start_micros = micros();
}

void falling() {
  end_micros = micros();
  duration = end_micros - start_micros;
  attachInterrupt(digitalPinToInterrupt(PGM_RESET_BUS_PIN), rising, RISING); // Enable the interupt.

  if (duration >= 9900 && duration < 10100) {
    Resetbool = HIGH;
  } else Resetbool = LOW;
}

int pgm_lastIndexOf(uint8_t c, const char * p)// displays at startup the Sketch running in the Arduino
{
  int last_index = -1; // -1 indicates no match
  uint8_t b;
  for (int i = 0; true; i++) {
    b = pgm_read_byte(p++);
    if (b == c)
      last_index = i;
    else if (b == 0) break;
  }
  return last_index;
}

void display_srcfile_details(void) { // displays at startup the Sketch running in the Arduino

  const char *the_path = PSTR(__FILE__);           // save RAM, use flash to hold __FILE__ instead

  int slash_loc = pgm_lastIndexOf('/', the_path); // index of last '/'
  if (slash_loc < 0) slash_loc = pgm_lastIndexOf('\\', the_path); // or last '\' (windows, ugh)

  int dot_loc = pgm_lastIndexOf('.', the_path);  // index of last '.'
  if (dot_loc < 0) dot_loc = pgm_lastIndexOf(0, the_path); // if no dot, return end of string

  Serial.print("\nFirmware Version: ");

  for (int i = slash_loc + 1; i < dot_loc; i++) {
    uint8_t b = pgm_read_byte(&the_path[i]);
    if (b != 0) {
      Serial.print((char) b);
    }
    else break;

  }

  Serial.print(", Compiled on: ");
  Serial.print(__DATE__);
  Serial.print(" at ");
  Serial.print(__TIME__);
  Serial.print("\n");

}

