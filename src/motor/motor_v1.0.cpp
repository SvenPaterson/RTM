#include <Wire.h>
#include <FlexyStepper.h>
#include <avr/pgmspace.h>
#include <Bounce2.h>

//Interupt variables
volatile unsigned long start_micros = 0.0;
volatile unsigned long end_micros = 0;
volatile unsigned long duration = 0;
volatile bool Resetbool = false;

const uint8_t LOOP_BUS_PIN = 1;  // This Pin is used to keeps track of the loops compleated by the motor controller.
const uint8_t MOTOR_STEP_PIN = 3;
const uint8_t MOTOR_DIRECTION_PIN = 4;
const uint8_t MOTOR_ENABLE_PIN = 5;        // This is the pin that enables the motor.
const uint8_t AIR_SUPPLY_BUS_PIN = 14;        // the number of the AIR_SUPPLY_BUS_PIN pin
const uint8_t AIR_DUMP_BUS_PIN = 15 ;        // the number of the AIR_SUPPLY_BUS_PIN pin
const uint8_t PRGM_FLAG_BUS_PIN = 10; //Pin That tell the to spinning the motor program.
const uint8_t TORQUE_FLAG_BUS_PIN = 11; //Pin That tell the to spinning the motor program.
const uint8_t LED_PIN = 13;
const uint8_t PRGM_RESET_BUS_PIN = 20;  // This Pin is used
const uint8_t HEAT_BUS_PIN = 12;

struct Step {
  bool spinning;    // true for motor turning, false for stopped
  bool turnOnHeat;  // true for heat, false for none
  bool spin_dir;    // true for CW from front, false for CCW
  uint16_t speed;   // rpm
  double accel;     // rpm per second
  uint32_t time;    // milliseconds
};

/****** TEST STEP PARAMS ******/

// enter the test stand type, high speed or standard
// and the steps per rev in ClearPath MSP software
bool isHighSpeedGearBox = true;
uint16_t SPR = 200;

Step steps[] = {
// spinning   heating   direction   speed   acceleration    time
  {true,      true,     true,       8000,   8000/5,         5000},
  {true,      true,     true,       8000,   8000/5,         120000},
  {true,      true,     true,       0,      8000/9,         9500},
  {false,     true,     true,       0,      0,              500}
};

bool break_loop, pause_requested;
uint32_t sum_time = 0;
elapsedMillis loop_time;
FlexyStepper stepper;

/******* FUNC DECLARATIONS *******/
void display_srcfile_details();
void rising();
void falling();
int pgm_lastIndexOf(uint8_t c, const char * p);

void setup() ////////////////////////////////////////////////////////////Setup ////////////////////////////////////////////////////////////////
{
  pinMode(PRGM_FLAG_BUS_PIN, INPUT_PULLDOWN);
  pinMode(PRGM_RESET_BUS_PIN, INPUT_PULLDOWN); 
  pinMode(LED_PIN, OUTPUT);
  pinMode(AIR_SUPPLY_BUS_PIN, OUTPUT);  
  pinMode(AIR_DUMP_BUS_PIN, OUTPUT);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_STEP_PIN, OUTPUT);
  pinMode(MOTOR_DIRECTION_PIN, OUTPUT);
  pinMode(LOOP_BUS_PIN, OUTPUT);
  pinMode(TORQUE_FLAG_BUS_PIN, OUTPUT);
  pinMode(HEAT_BUS_PIN, OUTPUT);

  digitalWrite (MOTOR_ENABLE_PIN, LOW);
  digitalWrite (AIR_SUPPLY_BUS_PIN, LOW);
  digitalWrite (AIR_DUMP_BUS_PIN, LOW);
  digitalWrite (LED_PIN, HIGH);
  digitalWrite (LOOP_BUS_PIN, LOW);
  digitalWrite (TORQUE_FLAG_BUS_PIN, LOW);

  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
  stepper.setStepsPerRevolution(isHighSpeedGearBox ? SPR / 3 : SPR);

  Serial.begin(115200); // Initalize UART, I2C bus, and connect to the micropressure sensor

  Wire.begin(); // start the I2C bus
  Wire.setClock(100000); //Optional - set I2C SCL to High Speed Mode of 100kHz

  delay(2000);
  display_srcfile_details();

  attachInterrupt(digitalPinToInterrupt(PRGM_RESET_BUS_PIN), rising, RISING); // Enable the interupt.
}

void loop() {
  if (!digitalRead(PRGM_FLAG_BUS_PIN)) {
    digitalWrite (LED_PIN, LOW);
    digitalWrite (MOTOR_ENABLE_PIN, LOW);
    delay(100);
    if (Resetbool == HIGH) {
      Resetbool = LOW;
      display_srcfile_details();
    }
  } else {
    digitalWrite (LED_PIN, HIGH);
    digitalWrite (MOTOR_ENABLE_PIN, HIGH);
    
    break_loop = false;
    pause_requested = false;
    // loop through test steps array, changing accel, speed each time.
    for (uint16_t i = 0; i < sizeof(steps) / sizeof(steps[0]) && !break_loop 
         && digitalRead(PRGM_FLAG_BUS_PIN); i++) {
      loop_time = 0;  // Reset loop_time for each step
      sum_time = 0;   // Reset sum_time for each step
      
      digitalWrite(HEAT_BUS_PIN, steps[i].turnOnHeat); 
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      stepper.setAccelerationInRevolutionsPerSecondPerSecond(steps[i].accel / 60);
      stepper.setSpeedInRevolutionsPerSecond(steps[i].speed / 60);
      stepper.setTargetPositionInSteps(steps[i].spin_dir ? -2147483647 : 2147483647);

      sum_time += steps[i].time;
      while (loop_time < sum_time) {
        
        // if stop/reset called, spin down motor and restart loop
        if (!digitalRead(PRGM_FLAG_BUS_PIN)) {
          stepper.setAccelerationInRevolutionsPerSecondPerSecond(850 / 60);
          stepper.setTargetPositionToStop();
          while (!stepper.processMovement());
          i = 0;
          break_loop = true;
          break;
        }

        if (steps[i].spinning) {
          stepper.processMovement();
        } 
        else {
          // stops motor spinning during a dwell step
          digitalWrite(MOTOR_ENABLE_PIN, LOW);
        }
      }
    }

    if (digitalRead(PRGM_FLAG_BUS_PIN)) {
      digitalWrite (LOOP_BUS_PIN, HIGH);
      delay(1);
      digitalWrite (LOOP_BUS_PIN, LOW);
    }
  }
}



/////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////Functions/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
void rising() {
  attachInterrupt(digitalPinToInterrupt(PRGM_RESET_BUS_PIN), falling, FALLING);
  start_micros = micros();
}

void falling() {
  end_micros = micros();
  duration = end_micros - start_micros;
  attachInterrupt(digitalPinToInterrupt(PRGM_RESET_BUS_PIN), rising, RISING); // Enable the interupt.

  if (duration >= 9900 && duration < 10100) {
    Resetbool = HIGH;
  } else Resetbool = LOW;
}

int pgm_lastIndexOf(uint8_t c, const char * p)// displays at startup the Sketch spinningning in the Arduino
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

void display_srcfile_details(void) { // displays at startup the Sketch spinningning in the Arduino

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

