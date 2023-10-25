/*
  
*/

#include <Wire.h>
#include <FlexyStepper.h>
#include <avr/pgmspace.h>
#include <Bounce2.h>

//Interupt variables
volatile unsigned long start_micros = 0.0;
volatile unsigned long end_micros = 0;
volatile unsigned long duration = 0;
volatile bool Resetbool = false;

const uint8_t LoopPin = 1;  // This Pin is used to keeps track of the loops compleated by the motor controller.
const uint8_t PWM_TorquePin = 2;   //This pin reads the PWM signal from the motor for torque. ** Not Used at this time**
const uint8_t MOTOR_STEP_PIN = 3;
const uint8_t MOTOR_DIRECTION_PIN = 4;
const uint8_t EnablePin = 5;        // This is the pin that enables the motor.
const uint8_t AirValvePin = 14;        // the number of the AirValvePin pin
const uint8_t AirDumpValvePin = 15 ;        // the number of the AirValvePin pin
const uint8_t ProgramRunPin = 10; //Pin That tell the to run the motor program.
const uint8_t TorqueFLagPin = 11; //Pin That tell the to run the motor program.
const uint8_t LEDPIN = 13;
const uint8_t ResetPin = 20;  // This Pin is used

// Define a stepper and the pins it will use
FlexyStepper stepper;
uint16_t PPR = (200 / 3);  // pulses per rev for motor

/****** TEST STEP PARAMS ******/
struct Step {
  bool run;       // true for motor turning, false for stopped
  uint16_t speed; // rpm
  double accel;   // rpm per second
  uint32_t time;  // milliseconds
};

Step steps[] = {
  {true,  2475,  2475/8,          18000},
  {true,  8500,  (8500-2475)/15,  15000},
  {true,  0,     8500/5,          5000},
  {false, 0,     0,               120000}
};
bool break_loop, pause_requested;
uint32_t sum_time = 0;
elapsedMillis loop_time;

/******* FUNC DECLARATIONS *******/
void display_srcfile_details();
void rising();
void falling();
int pgm_lastIndexOf(uint8_t c, const char * p);

void setup() ////////////////////////////////////////////////////////////Setup ////////////////////////////////////////////////////////////////
{
  pinMode(PWM_TorquePin, INPUT_PULLUP); // Motor torque pin.
  pinMode(ProgramRunPin, INPUT_PULLDOWN); // This pin get the signel from the disply to run the programe.
  pinMode(ResetPin, INPUT_PULLDOWN); // This pin get the signel from the disply to run the programe.
  pinMode(LEDPIN, OUTPUT);  // Make it an OUTput.
  pinMode(AirValvePin, OUTPUT);  // Make it an OUTput.
  pinMode(AirDumpValvePin, OUTPUT);// Make it an OUTput.
  pinMode(EnablePin, OUTPUT);   // Make it an OUTput.
  pinMode(MOTOR_STEP_PIN, OUTPUT);   // Make it an OUTput.
  pinMode(MOTOR_DIRECTION_PIN, OUTPUT);   // Make it an OUTput.
  pinMode(LoopPin, OUTPUT);   // Make it an OUTput.
  pinMode(TorqueFLagPin, OUTPUT);   // Make it an OUTput.

  digitalWrite (EnablePin, LOW);
  digitalWrite (AirValvePin, LOW);
  digitalWrite (AirDumpValvePin, LOW);
  digitalWrite (LEDPIN, HIGH);
  digitalWrite (LoopPin, LOW);
  digitalWrite (TorqueFLagPin, LOW);

  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
  stepper.setStepsPerRevolution(PPR);

  Serial.begin(115200); // Initalize UART, I2C bus, and connect to the micropressure sensor

  Wire.begin(); // start the I2C bus
  Wire.setClock(100000); //Optional - set I2C SCL to High Speed Mode of 100kHz

  delay(2000);
  display_srcfile_details();

  attachInterrupt(digitalPinToInterrupt(ResetPin), rising, RISING); // Enable the interupt.
}

void loop() {
  if (!digitalRead(ProgramRunPin)) {
    digitalWrite (LEDPIN, LOW);
    digitalWrite (EnablePin, LOW);
    delay(100);
    if (Resetbool == HIGH) {
      Resetbool = LOW;
      display_srcfile_details();
    }
  } else {
    digitalWrite (LEDPIN, HIGH);
    digitalWrite (EnablePin, HIGH);
    
    break_loop = false;
    pause_requested = false;
    // loop through test steps array, changing accel, speed each time.
    for (uint16_t i = 0; i < sizeof(steps) / sizeof(steps[0]) && !break_loop && digitalRead(ProgramRunPin); i++) {
      loop_time = 0;  // Reset loop_time for each step
      sum_time = 0;   // Reset sum_time for each step
      
      digitalWrite(LEDPIN, !digitalRead(LEDPIN));
      stepper.setAccelerationInRevolutionsPerSecondPerSecond(steps[i].accel / 60);
      stepper.setSpeedInRevolutionsPerSecond(steps[i].speed / 60);
      stepper.setTargetPositionInSteps(2147483647);

      sum_time += steps[i].time;
      while (loop_time < sum_time) {
        
        // if stop/reset called, spin down motor and restart loop
        if (!digitalRead(ProgramRunPin)) {
          stepper.setAccelerationInRevolutionsPerSecondPerSecond(850 / 60);
          stepper.setTargetPositionToStop();
          while (!stepper.processMovement());
          i = 0;
          break_loop = true;
          break;
        }

        if (steps[i].run) {
          stepper.processMovement();
        } 
        else {
          // stops motor spinning during a dwell step
          digitalWrite(EnablePin, LOW);
        }
      }
    }

    if (digitalRead(ProgramRunPin)) {
      digitalWrite (LoopPin, HIGH);
      delay(1);
      digitalWrite (LoopPin, LOW);
    }
  }
}



/////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////Functions/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
void rising() {
  attachInterrupt(digitalPinToInterrupt(ResetPin), falling, FALLING);
  start_micros = micros();
}

void falling() {
  end_micros = micros();
  duration = end_micros - start_micros;
  attachInterrupt(digitalPinToInterrupt(ResetPin), rising, RISING); // Enable the interupt.

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

